#!/usr/bin/env python3
"""Entry point: JSON-driven mission simulation."""

from __future__ import annotations

import argparse
import json
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from src.core.drone import Drone
from src.core.environment import Environment
from src.core.mission_manager import MissionManager
from src.decision.fsm import AutonomyFSM
from src.faults.fault_detector import FaultDetector
from src.faults.fault_injector import FaultInjector
from src.planners.a_star import AStarPlanner
from src.planners.base_planner import BasePlanner
from src.planners.d_star_lite import DStarLitePlanner
from src.viz.dashboard import Dashboard
from src.viz import json_export as json_export_helpers


def load_config(path: Path) -> dict[str, Any]:
    """Load a mission JSON file from disk."""
    with path.open(encoding="utf-8") as f:
        return json.load(f)


def _print_segment_paths(config: dict[str, Any], environment: Environment, planner: BasePlanner) -> None:
    """Print full path for each waypoint-to-waypoint segment (initial map, no dynamic hazards)."""
    wps = config["mission"]["waypoints"]
    start = (int(config["drone"]["start_position"][0]), int(config["drone"]["start_position"][1]))
    prev = start
    label = planner.get_name().replace("_", " ")
    print(f"Waypoint segment paths ({label}, static map only):")
    for i, wp in enumerate(wps):
        goal = (int(wp[0]), int(wp[1]))
        path = planner.plan(prev, goal, environment)
        print(f"  [{i}] {prev} -> {goal}  ({len(path)} cells)")
        print(f"      {path}")
        prev = goal


def _make_planner(config: dict[str, Any]) -> BasePlanner:
    """Instantiate the configured primary planner (``algorithm.primary_planner`` in JSON)."""
    primary = config["algorithm"]["primary_planner"]
    if primary == "d_star_lite":
        return DStarLitePlanner()
    if primary == "a_star":
        return AStarPlanner()
    raise NotImplementedError(f"Planner not implemented: {primary}")


def _safe_plan(
    planner: BasePlanner,
    config: dict[str, Any],
    start: tuple[int, int],
    goal: tuple[int, int],
    environment: Environment,
) -> BasePlanner:
    """Like JS ``safePlan``: avoid crashing when start or goal is blocked (e.g. after hazard spawn)."""
    sx, sy = int(start[0]), int(start[1])
    exempt_start = environment.is_blocked(sx, sy)
    try:
        planner.plan(start, goal, environment, exempt_start=exempt_start)
        return planner
    except Exception:
        p2 = _make_planner(config)
        try:
            p2.plan(start, goal, environment, exempt_start=exempt_start)
            return p2
        except Exception:
            return _make_planner(config)


def _faults_payload_for_export(active: list[dict[str, Any]], step_events: list[dict[str, Any]]) -> dict[str, Any]:
    """Shape `faults` block for one timeline frame."""
    injected: dict[str, Any] | None = None
    for ev in step_events:
        if ev["type"] == "fault_injected":
            injected = {
                "fault_type": ev["fault_type"],
                "severity": ev.get("severity"),
            }
            break
        if ev["type"] == "hazard_spawned":
            injected = {
                "fault_type": "environmental_hazard",
                "hazard_id": ev["id"],
                "description": ev.get("description", ""),
            }
            break
    return {"active": active, "injected_this_step": injected}


def _fsm_transition_for_timeline(new_entries: list[dict[str, Any]]) -> dict[str, Any] | None:
    if not new_entries:
        return None
    e = new_entries[-1]
    return {"from": e["from_state"], "to": e["to_state"], "trigger": e["trigger"]}


def _planning_position(drone: Drone, environment: Environment, fsm: AutonomyFSM) -> tuple[int, int]:
    """Position passed to the planner: noisy when FSM indicates degraded sensing."""
    if fsm.get_state() in ("DEGRADED", "REPLANNING", "SAFE_MODE"):
        return drone.get_position_estimate(environment)
    return (int(drone.position[0]), int(drone.position[1]))


def _skip_blocked_waypoints(
    environment: Environment,
    mission_manager: MissionManager,
    fsm: AutonomyFSM,
    waypoint_count: int,
    timestep: int,
) -> None:
    """Advance past waypoints whose target cell is blocked (e.g. inside a hazard)."""
    while mission_manager.mission_status == "in_progress" and mission_manager.current_waypoint_index < waypoint_count:
        tx, ty = mission_manager.get_current_target()
        if not environment.is_blocked(tx, ty):
            break
        fsm.record_waypoint_skipped(timestep, "target_cell_blocked")
        mission_manager.skip_waypoint()


def run_simulation(
    config: dict[str, Any],
    max_steps: int,
    verbose: bool = True,
    *,
    stop_on_mission_complete: bool = True,
    no_viz: bool = False,
    export_json_path: Path | None = None,
) -> dict[str, Any]:
    """Main loop (plan.md): fault injector → env → sense → detect → FSM → replan → move → mission."""
    if export_json_path is not None:
        no_viz = True
    environment = Environment(config)
    drone = Drone(config, environment=environment)
    mission_manager = MissionManager(config)
    viz_cfg = dict(config["visualization"])
    if no_viz:
        viz_cfg["dashboard_enabled"] = False
    waypoints_xy = [(int(w[0]), int(w[1])) for w in config["mission"]["waypoints"]]
    dashboard = Dashboard(
        viz_cfg,
        environment,
        config["algorithm"],
        config["drone"],
        waypoints_xy,
    )
    fault_injector = FaultInjector(config["fault_injection"])
    fault_detector = FaultDetector(config["algorithm"])
    fsm = AutonomyFSM(config["mission"], config["algorithm"])
    waypoint_count = len(config["mission"]["waypoints"])
    planner = _make_planner(config)
    _skip_blocked_waypoints(environment, mission_manager, fsm, waypoint_count, 0)
    if verbose:
        _print_segment_paths(config, environment, planner)
    goal = mission_manager.get_current_target()
    planner = _safe_plan(planner, config, (int(drone.position[0]), int(drone.position[1])), goal, environment)
    last_goal: tuple[int, int] = goal

    timestep = 0
    replan_events: list[dict[str, Any]] = []
    timeline: list[dict[str, Any]] | None = [] if export_json_path is not None else None
    injector_events_accum: list[dict[str, Any]] = []
    dashboard_closed = False
    fsm_log_cursor = 0
    bat_cap = float(config["drone"]["battery_capacity"])
    if verbose:
        print("Resilient Navigator — simulation with faults + FSM")
        print(f"Grid {environment.width}x{environment.height}, waypoints: {config['mission']['waypoints']}")

    while timestep < max_steps:
        fsm_log_before = len(fsm.get_transition_log())
        fault_injector.update(timestep, drone, environment)
        step_inj = fault_injector.get_last_step_events()
        injector_events_accum.extend(step_inj)
        environment.update(timestep)
        _skip_blocked_waypoints(environment, mission_manager, fsm, waypoint_count, timestep)
        sense_data = drone.sense(environment)

        if mission_manager.get_current_target() != last_goal:
            last_goal = mission_manager.get_current_target()
            planner = _safe_plan(
                planner,
                config,
                _planning_position(drone, environment, fsm),
                last_goal,
                environment,
            )

        fault_status = fault_detector.evaluate(drone, environment, planner)
        fsm.update(timestep, fault_status, drone, mission_manager)
        planning_pos = _planning_position(drone, environment, fsm)

        replanned_this_step = False
        path_before_replan: list[tuple[int, int]] | None = None
        if fsm.requires_replan:
            path_before = list(planner.get_full_path())
            path_before_replan = path_before
            replanned_this_step = True
            exempt_start = fsm.emergency_escape
            try:
                planner.replan(
                    planning_pos,
                    mission_manager.get_current_target(),
                    environment,
                    exempt_start=exempt_start,
                )
                path_ok = bool(planner.get_full_path())
                path_after = list(planner.get_full_path())
                # Re-evaluate faults after the map/path update so replan_succeeded uses current reality,
                # not the pre-replan path_blocked status from this same timestep.
                fault_status_after_replan = fault_detector.evaluate(drone, environment, planner)
                replan_events.append(
                    {
                        "timestep": timestep,
                        "path_before": path_before,
                        "path_after": path_after,
                    }
                )
                if verbose:
                    print(f"\n*** REPLAN @ t={timestep} ***")
                    print(f"    path_before ({len(path_before)}): {path_before}")
                    print(f"    path_after  ({len(path_after)}): {path_after}")
                if not path_ok:
                    fsm.replan_failed(drone, mission_manager)
                else:
                    fsm.replan_succeeded(fault_status_after_replan, drone, mission_manager)
            except Exception:
                replan_events.append({"timestep": timestep, "path_before": path_before, "path_after": [], "error": True})
                if verbose:
                    print(f"\n*** REPLAN FAILED @ t={timestep} (exception) ***")
                fsm.replan_failed(drone, mission_manager)
            finally:
                fsm.emergency_escape = False

        elif fsm.get_state() in ("DEGRADED", "REPLANNING", "SAFE_MODE"):
            # Keep planner aligned with noisy planning pose (not only on requires_replan).
            try:
                px_, py_ = planning_pos[0], planning_pos[1]
                planner.replan(
                    planning_pos,
                    mission_manager.get_current_target(),
                    environment,
                    exempt_start=environment.is_blocked(px_, py_),
                )
            except Exception:
                pass

        if fsm.get_state() == "SAFE_MODE" and mission_manager.mission_status == "aborted":
            fsm.check_safe_mode_abort(timestep, drone, mission_manager, planner)

        try:
            next_move = planner.get_next_step(planning_pos)
        except RuntimeError:
            px, py = int(drone.position[0]), int(drone.position[1])
            if environment.is_blocked(px, py):
                neighbors = environment.get_neighbors(px, py)
                if not neighbors:
                    neighbors = environment.get_neighbors(px, py, ignore_hazard_cells=True)
                if neighbors:
                    next_move = neighbors[0]
                else:
                    next_move = drone.position
            else:
                if fsm.get_state() == "REPLANNING":
                    fsm.replan_failed(drone, mission_manager)
                next_move = drone.position

        drone.move(next_move)
        mission_manager.update(drone.position)
        fsm.check_depleted_battery_abort(timestep, drone, mission_manager)

        if timeline is not None:
            log_full = fsm.get_transition_log()
            new_fsm = log_full[fsm_log_cursor:]
            fsm_log_cursor = len(log_full)
            wp = mission_manager.get_progress()
            if fsm.get_state() in ("DEGRADED", "REPLANNING", "SAFE_MODE"):
                pe = [float(planning_pos[0]), float(planning_pos[1])]
            else:
                pos_est = sense_data.get("position_estimate", (float(drone.position[0]), float(drone.position[1])))
                if isinstance(pos_est, tuple):
                    pe = [float(pos_est[0]), float(pos_est[1])]
                else:
                    pe = [float(pos_est[0]), float(pos_est[1])]
            planned = [[int(p[0]), int(p[1])] for p in planner.get_full_path()]
            timeline.append(
                {
                    "timestep": timestep,
                    "drone": {
                        "position": [int(drone.position[0]), int(drone.position[1])],
                        "position_estimate": pe,
                        "planning_position": [int(planning_pos[0]), int(planning_pos[1])],
                        "heading": float(drone.heading),
                        "battery": float(drone.battery),
                        "battery_pct": round(100.0 * float(drone.battery) / bat_cap, 4) if bat_cap > 0 else 0.0,
                        "sensor_noise_level": float(drone.sensor_noise_level),
                        "speed_scale": float(drone.get_telemetry()["speed_scale"]),
                    },
                    "fsm": {
                        "state": fsm.get_state(),
                        "transition": _fsm_transition_for_timeline(new_fsm),
                    },
                    "mission": {
                        "current_waypoint_index": int(wp["current_waypoint_index"]),
                        "current_target": [int(wp["current_target"][0]), int(wp["current_target"][1])],
                        "waypoints_completed": int(wp["waypoints_completed"]),
                        "total_waypoints": int(wp["waypoints_total"]),
                        "status": str(wp["mission_status"]),
                    },
                    "planner": {
                        "name": json_export_helpers.planner_display_name(planner),
                        "planned_path": planned,
                        "replanned_this_step": replanned_this_step,
                        "path_before_replan": (
                            [[int(x), int(y)] for x, y in path_before_replan] if path_before_replan else None
                        ),
                    },
                    "faults": _faults_payload_for_export(fault_injector.get_active_faults(), step_inj),
                }
            )

        dashboard.render(
            drone,
            environment,
            planner,
            fsm,
            mission_manager,
            timestep,
            fault_injector=fault_injector,
            planning_position=planning_pos,
        )
        if dashboard.is_closed:
            dashboard_closed = True
            if verbose:
                print("Dashboard window closed; stopping simulation.")
            break

        if verbose and (timestep % 25 == 0 or timestep < 5):
            print(
                f"t={timestep:4d}  FSM={fsm.get_state():12s}  pos={drone.position}  "
                f"noise={drone.sensor_noise_level:.3f}  fault={fault_status.type}"
            )

        if stop_on_mission_complete and mission_manager.mission_status == "completed":
            if verbose:
                print("Mission completed.")
            break
        if fsm.get_state() == "ABORT":
            if verbose:
                print("Mission aborted (FSM ABORT).")
            break
        if drone.battery <= 0:
            if verbose:
                print("Battery depleted.")
            break

        timestep += 1

    dashboard.finalize()

    transition_log = fsm.get_transition_log()
    export_payload: dict[str, Any] | None = None
    if export_json_path is not None and timeline is not None:
        merged_events = json_export_helpers.merge_export_events(transition_log, injector_events_accum)
        mstatus = mission_manager.mission_status
        mresult = json_export_helpers.mission_result_string(
            mstatus, fsm.get_state(), float(drone.battery), dashboard_closed
        )
        export_payload = {
            "metadata": {
                "config": json_export_helpers.snapshot_config(config),
                "total_timesteps": len(timeline),
                "mission_result": mresult,
                "planner_used": json_export_helpers.planner_display_name(planner),
                "total_replans": len(replan_events),
                "export_timestamp": datetime.now(timezone.utc).replace(microsecond=0).isoformat().replace(
                    "+00:00", "Z"
                ),
            },
            "environment": json_export_helpers.build_environment_export(config),
            "timeline": timeline,
            "events": merged_events,
            "fsm_transition_log": transition_log,
        }
        export_json_path.parent.mkdir(parents=True, exist_ok=True)
        export_json_path.write_text(json.dumps(export_payload, indent=2), encoding="utf-8")
        if verbose:
            print(f"Wrote mission export JSON to {export_json_path}")

    if verbose:
        print(f"\nFinished after timestep {timestep}.")
        print("FSM transition log (full):")
        for entry in transition_log:
            print(f"  {entry}")
        print("Replan events (path before / after):")
        for ev in replan_events:
            print(f"  {ev}")
    out: dict[str, Any] = {
        "final_timestep": timestep,
        "fsm_transition_log": transition_log,
        "replan_events": replan_events,
    }
    if export_payload is not None:
        out["export"] = export_payload
    return out


def main() -> None:
    """CLI entry: load config and run :func:`run_simulation`."""
    parser = argparse.ArgumentParser(description="Resilient Navigator mission simulator")
    default_config = Path(__file__).resolve().parent / "config" / "mission_01.json"
    parser.add_argument(
        "--config",
        type=Path,
        default=default_config,
        help="Path to mission JSON (default: config/mission_01.json)",
    )
    parser.add_argument(
        "--max-steps",
        type=int,
        default=200,
        help="Maximum timesteps (default: 200; use >=150 for fault/FSM demo)",
    )
    parser.add_argument("--quiet", action="store_true", help="Less per-timestep output")
    parser.add_argument(
        "--run-full-horizon",
        action="store_true",
        help="Do not stop when mission completes (for integration demos / long fault schedules)",
    )
    parser.add_argument(
        "--no-viz",
        action="store_true",
        help="Disable matplotlib dashboard (headless / CI)",
    )
    parser.add_argument(
        "--export-json",
        type=Path,
        default=None,
        metavar="FILE",
        help="Write full mission replay JSON to FILE and run headless (no Matplotlib)",
    )
    args = parser.parse_args()

    config = load_config(args.config)
    export_path = args.export_json
    if export_path is not None:
        export_path = export_path.resolve()
    _ = run_simulation(
        config,
        args.max_steps,
        verbose=not args.quiet,
        stop_on_mission_complete=not args.run_full_horizon,
        no_viz=args.no_viz or export_path is not None,
        export_json_path=export_path,
    )


if __name__ == "__main__":
    main()
