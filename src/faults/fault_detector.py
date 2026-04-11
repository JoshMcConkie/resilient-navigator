"""Classify faults from telemetry and planned path vs environment (report only — no actions)."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Literal

from src.core.drone import Drone
from src.core.environment import Environment
from src.planners.base_planner import BasePlanner

FaultLevel = Literal["NOMINAL", "WARNING", "CRITICAL"]
FaultType = Literal["none", "sensor_degradation", "path_blocked", "combined"]


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
        noise = float(drone.sensor_noise_level)
        path_blocked = self._planned_path_intersects_blocked(environment, planner)

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

    def _planned_path_intersects_blocked(self, environment: Environment, planner: BasePlanner) -> bool:
        path = planner.get_full_path()
        if not path:
            return True
        for x, y in path:
            if environment.is_blocked(int(x), int(y)):
                return True
        return False
