"""Classify faults from telemetry and planned path vs environment (report only — no actions)."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Literal

from src.core.drone import Drone
from src.core.environment import Environment
from src.planners.base_planner import BasePlanner

FaultLevel = Literal["NOMINAL", "WARNING", "CRITICAL"]
FaultType = Literal[
    "none",
    "sensor_degradation",
    "path_blocked",
    "combined",
    "position_compromised",
    "position_trapped",
]


@dataclass(frozen=True, slots=True)
class FaultStatus:
    """Snapshot of detected fault conditions for the FSM."""

    level: FaultLevel
    type: FaultType
    details: str


class FaultDetector:
    """
    Compares sensor noise to thresholds and checks whether the planner's path
    crosses blocked cells. Does **not** choose control actions.
    """

    def __init__(self, algorithm_config: dict[str, Any]) -> None:
        self._warn = float(algorithm_config["sensor_degradation_threshold"])
        self._crit = float(algorithm_config["critical_fault_threshold"])

    def evaluate(
        self,
        drone: Drone,
        environment: Environment,
        planner: BasePlanner,
    ) -> FaultStatus:
        """Produce a :class:`FaultStatus` for the current timestep."""
        px, py = int(drone.position[0]), int(drone.position[1])
        if environment.is_blocked(px, py):
            nbs_egress = environment.get_neighbors(px, py, ignore_hazard_cells=True)
            if not nbs_egress:
                return FaultStatus(
                    level="CRITICAL",
                    type="position_trapped",
                    details="Drone surrounded by hard obstacles — no escape.",
                )
            path = planner.get_full_path()
            escape_planned = False
            if len(path) >= 2:
                n0 = (int(path[0][0]), int(path[0][1]))
                n1 = (int(path[1][0]), int(path[1][1]))
                if (
                    n0 == (px, py)
                    and n1 in nbs_egress
                    and not environment.is_blocked(n1[0], n1[1], ignore_hazard_cells=True)
                ):
                    escape_planned = True
            if not escape_planned:
                return FaultStatus(
                    level="CRITICAL",
                    type="position_compromised",
                    details="Drone is inside a blocked/hazard cell — emergency escape required.",
                )

        noise = float(drone.sensor_noise_level)
        path_blocked = self._planned_path_intersects_blocked(environment, planner, drone)

        if noise >= self._crit:
            sensor_level: FaultLevel = "CRITICAL"
        elif noise >= self._warn:
            sensor_level = "WARNING"
        else:
            sensor_level = "NOMINAL"

        if sensor_level != "NOMINAL" and path_blocked:
            level: FaultLevel = "CRITICAL" if sensor_level == "CRITICAL" else "WARNING"
            details = (
                f"Combined: sensor noise {noise:.3f} (warn={self._warn}, crit={self._crit}) "
                f"and planned path crosses blocked or hazard cells."
            )
            return FaultStatus(level=level, type="combined", details=details)

        if path_blocked:
            return FaultStatus(
                level="WARNING",
                type="path_blocked",
                details="Planned path intersects a blocked or hazard cell.",
            )

        if sensor_level == "CRITICAL":
            return FaultStatus(
                level="CRITICAL",
                type="sensor_degradation",
                details=f"Sensor noise {noise:.3f} >= critical threshold {self._crit}.",
            )
        if sensor_level == "WARNING":
            return FaultStatus(
                level="WARNING",
                type="sensor_degradation",
                details=f"Sensor noise {noise:.3f} >= warning threshold {self._warn}.",
            )

        return FaultStatus(level="NOMINAL", type="none", details="Nominal.")

    def _planned_path_intersects_blocked(
        self,
        environment: Environment,
        planner: BasePlanner,
        drone: Drone,
    ) -> bool:
        path = planner.get_full_path()
        if not path:
            return True
        px, py = int(drone.position[0]), int(drone.position[1])
        for x, y in path:
            ix, iy = int(x), int(y)
            if (ix, iy) == (px, py):
                continue
            if environment.is_blocked(ix, iy):
                return True
        return False
