"""Finite-state autonomy logic driven by fault reports (decision, not detection)."""

from __future__ import annotations

from enum import Enum
from typing import Any, List

from src.core.drone import Drone
from src.core.mission_manager import MissionManager
from src.faults.fault_detector import FaultStatus
from src.planners.base_planner import BasePlanner


class FSMState(str, Enum):
    """High-level autonomy modes."""

    NOMINAL = "NOMINAL"
    DEGRADED = "DEGRADED"
    REPLANNING = "REPLANNING"
    SAFE_MODE = "SAFE_MODE"
    ABORT = "ABORT"


class AutonomyFSM:
    """
    Consumes :class:`FaultStatus` and telemetry, updates discrete state, and sets
    ``requires_replan``. Does **not** implement detection (see :class:`FaultDetector`).
    """

    def __init__(self, mission_config: dict[str, Any], algorithm_config: dict[str, Any]) -> None:
        # Reserved for future tie-breaking / cost shaping (plan.md).
        self._priority_weights = dict(mission_config.get("priority_weights", {}))
        _ = self._priority_weights
        self._sensor_warn = float(algorithm_config["sensor_degradation_threshold"])
        self._sensor_crit = float(algorithm_config["critical_fault_threshold"])
        self._battery_fraction_abort = 0.05

        self._state = FSMState.NOMINAL
        self.requires_replan: bool = False
        self._transition_log: list[dict[str, Any]] = []
        self._prev_noise: float | None = None
        self._last_timestep: int = 0

    def get_state(self) -> str:
        """Current state name (same string values as :class:`FSMState`)."""
        return self._state.value

    def get_transition_log(self) -> List[dict[str, Any]]:
        """Append-only history of state changes."""
        return list(self._transition_log)

    def update(
        self,
        timestep: int,
        fault_status: FaultStatus,
        drone: Drone,
        mission_manager: MissionManager,
    ) -> None:
        """
        Evaluate transitions for this timestep. Does not run replanning itself;
        sets ``requires_replan`` when entering :attr:`FSMState.REPLANNING`.
        """
        self._last_timestep = timestep
        prev = self._state

        if self._battery_low(drone):
            self._transition(timestep, prev, FSMState.ABORT, "battery_below_5pct", drone, mission_manager)
            self._prev_noise = float(drone.sensor_noise_level)
            return

        if self._state == FSMState.ABORT:
            self._prev_noise = float(drone.sensor_noise_level)
            return

        if self._state == FSMState.NOMINAL:
            self._from_nominal(timestep, fault_status, drone, mission_manager, prev)
        elif self._state == FSMState.DEGRADED:
            self._from_degraded(timestep, fault_status, drone, mission_manager, prev)
        elif self._state == FSMState.REPLANNING:
            self._from_replanning(timestep, fault_status, drone, mission_manager, prev)
        elif self._state == FSMState.SAFE_MODE:
            self._from_safe_mode(timestep, fault_status, drone, mission_manager, prev)

        self._prev_noise = float(drone.sensor_noise_level)

    def _battery_low(self, drone: Drone) -> bool:
        tel = drone.get_telemetry()
        cap = float(tel["battery_capacity"])
        return float(tel["battery"]) <= self._battery_fraction_abort * cap

    def _from_nominal(
        self,
        timestep: int,
        fs: FaultStatus,
        drone: Drone,
        mission_manager: MissionManager,
        prev: FSMState,
    ) -> None:
        if fs.type == "path_blocked":
            self._transition(timestep, prev, FSMState.REPLANNING, "path_blocked", drone, mission_manager)
            self.requires_replan = True
            return
        if fs.level == "CRITICAL":
            self._transition(timestep, prev, FSMState.SAFE_MODE, "sensor_critical", drone, mission_manager)
            return
        if fs.level == "WARNING":
            self._transition(timestep, prev, FSMState.DEGRADED, "sensor_warning", drone, mission_manager)

    def _from_degraded(
        self,
        timestep: int,
        fs: FaultStatus,
        drone: Drone,
        mission_manager: MissionManager,
        prev: FSMState,
    ) -> None:
        if fs.type in ("path_blocked", "combined"):
            self._transition(timestep, prev, FSMState.REPLANNING, "path_blocked_or_combined", drone, mission_manager)
            self.requires_replan = True
            return
        if fs.level == "CRITICAL":
            self._transition(timestep, prev, FSMState.SAFE_MODE, "sensor_critical", drone, mission_manager)
            return
        noise = float(drone.sensor_noise_level)
        prev_n = self._prev_noise
        if prev_n is not None and noise > prev_n + 1e-12:
            self._transition(timestep, prev, FSMState.REPLANNING, "sensor_noise_worsened", drone, mission_manager)
            self.requires_replan = True
            return
        if fs.level == "NOMINAL" and noise < self._sensor_warn:
            self._transition(timestep, prev, FSMState.NOMINAL, "sensor_recovered", drone, mission_manager)

    def _from_replanning(
        self,
        timestep: int,
        fs: FaultStatus,
        drone: Drone,
        mission_manager: MissionManager,
        prev: FSMState,
    ) -> None:
        if fs.level == "CRITICAL":
            self._transition(timestep, prev, FSMState.SAFE_MODE, "critical_while_replanning", drone, mission_manager)

    def _from_safe_mode(
        self,
        timestep: int,
        fs: FaultStatus,
        drone: Drone,
        mission_manager: MissionManager,
        prev: FSMState,
    ) -> None:
        _ = (timestep, fs, drone, mission_manager, prev)

    def replan_succeeded(
        self,
        fault_status: FaultStatus,
        drone: Drone,
        mission_manager: MissionManager,
    ) -> None:
        """Call after ``planner.replan`` succeeds with a usable path."""
        if self._state != FSMState.REPLANNING:
            return
        prev = self._state
        ts = self._last_timestep
        if fault_status.level == "CRITICAL":
            self._transition(ts, prev, FSMState.SAFE_MODE, "replan_done_critical_fault", drone, mission_manager)
        elif fault_status.level == "WARNING":
            self._transition(ts, prev, FSMState.DEGRADED, "replan_succeeded_warning", drone, mission_manager)
        else:
            self._transition(ts, prev, FSMState.NOMINAL, "replan_succeeded_nominal", drone, mission_manager)
        self.requires_replan = False

    def replan_failed(self, drone: Drone, mission_manager: MissionManager) -> None:
        """Call when replanning throws or yields no valid path."""
        if self._state != FSMState.REPLANNING:
            return
        prev = self._state
        ts = self._last_timestep
        self._transition(ts, prev, FSMState.SAFE_MODE, "replan_failed", drone, mission_manager)
        self.requires_replan = False

    def check_safe_mode_abort(
        self,
        timestep: int,
        drone: Drone,
        mission_manager: MissionManager,
        planner: BasePlanner,
    ) -> None:
        """If in SAFE_MODE with no route home, transition to ABORT."""
        if self._state != FSMState.SAFE_MODE:
            return
        if mission_manager.mission_status != "aborted":
            return
        if planner.get_full_path():
            return
        prev = self._state
        self._transition(timestep, prev, FSMState.ABORT, "no_path_home", drone, mission_manager)

    def _transition(
        self,
        timestep: int,
        old: FSMState,
        new: FSMState,
        trigger: str,
        drone: Drone | None,
        mission_manager: MissionManager | None,
    ) -> None:
        if old == new:
            return
        self._transition_log.append(
            {
                "timestep": timestep,
                "from_state": old.value,
                "to_state": new.value,
                "trigger": trigger,
            }
        )
        self._state = new
        if new == FSMState.DEGRADED and drone is not None:
            drone.set_speed_scale(0.75)
        if new == FSMState.NOMINAL and drone is not None:
            drone.set_speed_scale(1.0)
        if new == FSMState.SAFE_MODE and mission_manager is not None:
            mission_manager.abort_mission()
