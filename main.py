#!/usr/bin/env python3
"""Entry point: JSON-driven mission simulation."""

from __future__ import annotations

import argparse
import json
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
) -> dict[str, Any]:
    """Main loop (plan.md): fault injector → env → sense → detect → FSM → replan → move → mission."""
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
    planner.plan(drone.position, goal, environment)
    last_goal: tuple[int, int] = goal

    timestep = 0
    replan_events: list[dict[str, Any]] = []
    if verbose:
        print("Resilient Navigator — simulation with faults + FSM")
        print(f"Grid {environment.width}x{environment.height}, waypoints: {config['mission']['waypoints']}")

    while timestep < max_steps:
        fault_injector.update(timestep, drone, environment)
        environment.update(timestep)
        _skip_blocked_waypoints(environment, mission_manager, fsm, waypoint_count, timestep)
        drone.sense(environment)

        if mission_manager.get_current_target() != last_goal:
            last_goal = mission_manager.get_current_target()
            planner.plan(drone.position, last_goal, environment)

        fault_status = fault_detector.evaluate(drone, environment, planner)
        fsm.update(timestep, fault_status, drone, mission_manager)

        if fsm.requires_replan:
            path_before = list(planner.get_full_path())
            try:
                planner.replan(drone.position, mission_manager.get_current_target(), environment)
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

        if fsm.get_state() == "SAFE_MODE" and mission_manager.mission_status == "aborted":
            try:
                planner.replan(drone.position, mission_manager.get_current_target(), environment)
            except Exception:
                pass
            fsm.check_safe_mode_abort(timestep, drone, mission_manager, planner)

        try:
            next_move = planner.get_next_step(drone.position)
        except RuntimeError:
            if fsm.get_state() == "REPLANNING":
                fsm.replan_failed(drone, mission_manager)
            next_move = drone.position

        drone.move(next_move)
        mission_manager.update(drone.position)
        fsm.check_depleted_battery_abort(timestep, drone, mission_manager)

        dashboard.render(
            drone,
            environment,
            planner,
            fsm,
            mission_manager,
            timestep,
            fault_injector=fault_injector,
        )
        if dashboard.is_closed:
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
    if verbose:
        print(f"\nFinished after timestep {timestep}.")
        print("FSM transition log (full):")
        for entry in transition_log:
            print(f"  {entry}")
        print("Replan events (path before / after):")
        for ev in replan_events:
            print(f"  {ev}")
    return {
        "final_timestep": timestep,
        "fsm_transition_log": transition_log,
        "replan_events": replan_events,
    }


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
    args = parser.parse_args()

    config = load_config(args.config)
    _ = run_simulation(
        config,
        args.max_steps,
        verbose=not args.quiet,
        stop_on_mission_complete=not args.run_full_horizon,
        no_viz=args.no_viz,
    )


if __name__ == "__main__":
    main()
