#!/usr/bin/env python3
"""Entry point: load config and run simulation (Hour 1 — stdout stub loop)."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

from src.core.drone import Drone
from src.core.environment import Environment
from src.core.mission_manager import MissionManager


def load_config(path: Path) -> dict[str, Any]:
    with path.open(encoding="utf-8") as f:
        return json.load(f)


def run_simulation_stub(config: dict[str, Any], max_steps: int) -> None:
    """
    Hour 1 stub: wires Environment, Drone, MissionManager and prints each step.
    Full loop from plan (faults, FSM, planner, dashboard) is added in later hours.
    """
    environment = Environment(config)
    drone = Drone(config)
    mission_manager = MissionManager(config)

    timestep = 0
    print("Resilient Navigator — Hour 1 simulation stub")
    print(f"Grid {environment.width}x{environment.height}, waypoints: {config['mission']['waypoints']}")

    while timestep < max_steps:
        environment.update(timestep)
        drone.sense(environment)
        mission_manager.update(drone.position)

        target = mission_manager.get_current_target()
        telemetry = drone.get_telemetry()
        progress = mission_manager.get_progress()

        print(
            f"t={timestep:4d}  pos={drone.position}  target={target}  "
            f"battery={telemetry['battery']:.1f}  status={progress['mission_status']}"
        )

        if mission_manager.mission_status == "completed":
            print("Mission completed (stub).")
            break

        drone.move(target)

        if drone.battery <= 0:
            print("Battery depleted (stub).")
            break

        timestep += 1

    print(f"Stub finished after timestep {timestep}.")


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
        help="Maximum timesteps for Hour 1 stub (default: 200)",
    )
    args = parser.parse_args()

    config = load_config(args.config)
    run_simulation_stub(config, args.max_steps)


if __name__ == "__main__":
    main()
