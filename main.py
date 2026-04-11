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
from src.planners.d_star_lite import DStarLitePlanner


def load_config(path: Path) -> dict[str, Any]:
    with path.open(encoding="utf-8") as f:
        return json.load(f)


def _make_planner(config: dict[str, Any]) -> DStarLitePlanner:
    primary = config["algorithm"]["primary_planner"]
    if primary == "d_star_lite":
        return DStarLitePlanner()
    raise NotImplementedError(f"Planner not implemented: {primary}")


def run_simulation(
    config: dict[str, Any],
    max_steps: int,
    verbose: bool = True,
    *,
    stop_on_mission_complete: bool = True,
) -> dict[str, Any]:
    """Main loop (plan.md): fault injector → env → sense → detect → FSM → replan → move → mission."""
    environment = Environment(config)
    drone = Drone(config)
    mission_manager = MissionManager(config)
    fault_injector = FaultInjector(config["fault_injection"])
    fault_detector = FaultDetector(config["algorithm"])
    fsm = AutonomyFSM(config["mission"], config["algorithm"])
    planner = _make_planner(config)

    goal = mission_manager.get_current_target()
    planner.plan(drone.position, goal, environment)
    last_goal: tuple[int, int] = goal

    timestep = 0
    if verbose:
        print("Resilient Navigator — simulation with faults + FSM")
        print(f"Grid {environment.width}x{environment.height}, waypoints: {config['mission']['waypoints']}")

    while timestep < max_steps:
        fault_injector.update(timestep, drone, environment)
        environment.update(timestep)
        drone.sense(environment)

        if mission_manager.get_current_target() != last_goal:
            last_goal = mission_manager.get_current_target()
            planner.plan(drone.position, last_goal, environment)

        fault_status = fault_detector.evaluate(drone, environment, planner)
        fsm.update(timestep, fault_status, drone, mission_manager)

        if fsm.requires_replan:
            try:
                planner.replan(drone.position, mission_manager.get_current_target(), environment)
                path_ok = bool(planner.get_full_path())
                if not path_ok:
                    fsm.replan_failed(drone, mission_manager)
                else:
                    fsm.replan_succeeded(fault_status, drone, mission_manager)
            except Exception:
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

    transition_log = fsm.get_transition_log()
    if verbose:
        print(f"\nFinished after timestep {timestep}.")
        print("FSM transition log:")
        for entry in transition_log:
            print(f"  {entry}")
    return {"final_timestep": timestep, "fsm_transition_log": transition_log}


def main() -> None:
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
    args = parser.parse_args()

    config = load_config(args.config)
    _ = run_simulation(
        config,
        args.max_steps,
        verbose=not args.quiet,
        stop_on_mission_complete=not args.run_full_horizon,
    )


if __name__ == "__main__":
    main()
