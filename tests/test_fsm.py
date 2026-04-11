"""Unit tests for :class:`AutonomyFSM` transitions and logging."""

from __future__ import annotations

import pytest

from src.core.drone import Drone
from src.core.mission_manager import MissionManager
from src.decision.fsm import AutonomyFSM
from src.faults.fault_detector import FaultStatus


def _algo_cfg() -> dict:
    return {
        "sensor_degradation_threshold": 0.3,
        "critical_fault_threshold": 0.7,
    }


def _mission_cfg() -> dict:
    return {
        "priority_weights": {},
        "home_position": [0, 0],
        "waypoints": [[5, 5]],
    }


def _drone_cfg() -> dict:
    return {
        "environment": {"grid_size": [20, 20]},
        "drone": {
            "start_position": [1, 1],
            "speed": 1.0,
            "sensor_range": 5,
            "sensor_noise_baseline": 0.05,
            "max_sensor_noise": 0.5,
            "battery_capacity": 1000.0,
            "battery_drain_rate": 1.0,
        },
    }


@pytest.fixture
def fsm() -> AutonomyFSM:
    return AutonomyFSM(_mission_cfg(), _algo_cfg())


@pytest.fixture
def drone() -> Drone:
    return Drone(_drone_cfg())


@pytest.fixture
def mission_manager() -> MissionManager:
    return MissionManager(
        {
            "mission": {
                "waypoints": [[5, 5]],
                "home_position": [0, 0],
            }
        }
    )


def test_fsm_starts_nominal(fsm: AutonomyFSM) -> None:
    assert fsm.get_state() == "NOMINAL"


def test_nominal_to_degraded_on_warning(
    fsm: AutonomyFSM, drone: Drone, mission_manager: MissionManager
) -> None:
    drone.sensor_noise_level = 0.35
    fs = FaultStatus(level="WARNING", type="sensor_degradation", details="test")
    fsm.update(0, fs, drone, mission_manager)
    assert fsm.get_state() == "DEGRADED"
    assert any(e["trigger"] == "sensor_warning" for e in fsm.get_transition_log())


def test_nominal_to_replanning_on_path_blocked(
    fsm: AutonomyFSM, drone: Drone, mission_manager: MissionManager
) -> None:
    fs = FaultStatus(level="WARNING", type="path_blocked", details="test")
    fsm.update(1, fs, drone, mission_manager)
    assert fsm.get_state() == "REPLANNING"
    assert fsm.requires_replan is True


def test_degraded_to_safe_mode_on_critical(
    fsm: AutonomyFSM, drone: Drone, mission_manager: MissionManager
) -> None:
    drone.sensor_noise_level = 0.35
    fsm.update(0, FaultStatus(level="WARNING", type="sensor_degradation", details="w"), drone, mission_manager)
    assert fsm.get_state() == "DEGRADED"
    drone.sensor_noise_level = 0.75
    fsm.update(1, FaultStatus(level="CRITICAL", type="sensor_degradation", details="c"), drone, mission_manager)
    assert fsm.get_state() == "SAFE_MODE"


def test_degraded_to_nominal_when_fault_clears(
    fsm: AutonomyFSM, drone: Drone, mission_manager: MissionManager
) -> None:
    drone.sensor_noise_level = 0.35
    fsm.update(0, FaultStatus(level="WARNING", type="sensor_degradation", details="w"), drone, mission_manager)
    assert fsm.get_state() == "DEGRADED"
    drone.sensor_noise_level = 0.05
    fsm.update(1, FaultStatus(level="NOMINAL", type="none", details="ok"), drone, mission_manager)
    assert fsm.get_state() == "NOMINAL"


def test_replanning_to_nominal_after_replan_succeeded_nominal(
    fsm: AutonomyFSM, drone: Drone, mission_manager: MissionManager
) -> None:
    fsm.update(0, FaultStatus(level="WARNING", type="path_blocked", details="pb"), drone, mission_manager)
    assert fsm.get_state() == "REPLANNING"
    fsm.replan_succeeded(FaultStatus(level="NOMINAL", type="none", details="ok"), drone, mission_manager)
    assert fsm.get_state() == "NOMINAL"
    assert fsm.requires_replan is False


def test_replanning_to_safe_mode_after_replan_failed(
    fsm: AutonomyFSM, drone: Drone, mission_manager: MissionManager
) -> None:
    fsm.update(0, FaultStatus(level="WARNING", type="path_blocked", details="pb"), drone, mission_manager)
    assert fsm.get_state() == "REPLANNING"
    fsm.replan_failed(drone, mission_manager)
    assert fsm.get_state() == "SAFE_MODE"


def test_abort_on_critically_low_battery(
    fsm: AutonomyFSM, drone: Drone, mission_manager: MissionManager
) -> None:
    drone.battery = 40.0
    fsm.update(0, FaultStatus(level="NOMINAL", type="none", details="ok"), drone, mission_manager)
    assert fsm.get_state() == "ABORT"
    assert any(e["to_state"] == "ABORT" for e in fsm.get_transition_log())


def test_transition_log_shape(fsm: AutonomyFSM, drone: Drone, mission_manager: MissionManager) -> None:
    fsm.update(0, FaultStatus(level="WARNING", type="sensor_degradation", details="w"), drone, mission_manager)
    for entry in fsm.get_transition_log():
        assert "timestep" in entry
        assert "from_state" in entry
        assert "to_state" in entry
        assert "trigger" in entry


def test_battery_depleted_abort_hook(
    fsm: AutonomyFSM, drone: Drone, mission_manager: MissionManager
) -> None:
    drone.battery = 0.0
    fsm.check_depleted_battery_abort(5, drone, mission_manager)
    assert fsm.get_state() == "ABORT"
    assert any(e.get("trigger") == "battery_depleted" for e in fsm.get_transition_log())
