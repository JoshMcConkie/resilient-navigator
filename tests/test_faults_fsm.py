"""Hour 3 verification: fault injector, detector, FSM."""

from __future__ import annotations

import json
from pathlib import Path

import pytest

from src.core.drone import Drone
from src.core.environment import Environment
from src.core.mission_manager import MissionManager
from src.decision.fsm import AutonomyFSM, FSMState
from src.faults.fault_detector import FaultDetector
from src.faults.fault_injector import FaultInjector
from src.planners.d_star_lite import DStarLitePlanner


@pytest.fixture
def mission_config() -> dict:
    root = Path(__file__).resolve().parents[1]
    return json.loads((root / "config" / "mission_01.json").read_text(encoding="utf-8"))


def _minimal_env() -> Environment:
    return Environment(
        {
            "environment": {
                "grid_size": [40, 40],
                "static_obstacles": [],
                "dynamic_hazards": [],
            }
        }
    )


def test_fault_injector_sensor_at_timestep(mission_config: dict) -> None:
    cfg = mission_config.copy()
    cfg["fault_injection"] = {
        "schedule": [
            {
                "timestep": 10,
                "type": "sensor_degradation",
                "severity": 0.25,
                "duration": 5,
                "description": "test",
            }
        ]
    }
    drone = Drone(cfg)
    env = _minimal_env()
    inj = FaultInjector(cfg["fault_injection"])
    assert abs(drone.sensor_noise_level - 0.05) < 1e-6
    for t in range(10):
        inj.update(t, drone, env)
    assert abs(drone.sensor_noise_level - 0.05) < 1e-6
    inj.update(10, drone, env)
    assert drone.sensor_noise_level > 0.29


def test_fault_injector_sensor_restores_after_duration(mission_config: dict) -> None:
    cfg = mission_config.copy()
    cfg["fault_injection"] = {
        "schedule": [
            {
                "timestep": 0,
                "type": "sensor_degradation",
                "severity": 0.2,
                "duration": 2,
                "description": "test",
            }
        ]
    }
    drone = Drone(cfg)
    env = _minimal_env()
    inj = FaultInjector(cfg["fault_injection"])
    inj.update(0, drone, env)
    assert drone.sensor_noise_level > 0.2
    inj.update(1, drone, env)
    inj.update(2, drone, env)
    assert abs(drone.sensor_noise_level - float(cfg["drone"]["sensor_noise_baseline"])) < 1e-5


def test_fault_injector_environmental_hazard_grid(mission_config: dict) -> None:
    env = Environment(mission_config)
    cfg = mission_config.copy()
    cfg["fault_injection"] = {
        "schedule": [
            {
                "timestep": 5,
                "type": "environmental_hazard",
                "hazard_id": "hazard_01",
                "description": "inject",
            }
        ]
    }
    drone = Drone(cfg)
    inj = FaultInjector(cfg["fault_injection"])
    for t in range(6):
        inj.update(t, drone, env)
        env.update(t)
    assert env.is_blocked(15, 15)


def test_detector_nominal(mission_config: dict) -> None:
    cfg = mission_config.copy()
    drone = Drone(cfg)
    env = _minimal_env()
    p = DStarLitePlanner()
    p.plan((1, 1), (5, 5), env)
    det = FaultDetector(cfg["algorithm"])
    fs = det.evaluate(drone, env, p)
    assert fs.level == "NOMINAL"


def test_detector_warning(mission_config: dict) -> None:
    cfg = mission_config.copy()
    drone = Drone(cfg)
    drone.apply_sensor_degradation(0.35)
    env = _minimal_env()
    p = DStarLitePlanner()
    p.plan((1, 1), (5, 5), env)
    det = FaultDetector(cfg["algorithm"])
    fs = det.evaluate(drone, env, p)
    assert fs.level == "WARNING"


def test_detector_critical(mission_config: dict) -> None:
    cfg = mission_config.copy()
    cfg["algorithm"] = {**mission_config["algorithm"], "critical_fault_threshold": 0.4}
    drone = Drone(cfg)
    drone.apply_sensor_degradation(0.5)
    env = _minimal_env()
    p = DStarLitePlanner()
    p.plan((1, 1), (5, 5), env)
    det = FaultDetector(cfg["algorithm"])
    fs = det.evaluate(drone, env, p)
    assert fs.level == "CRITICAL"


def test_detector_path_blocked(mission_config: dict) -> None:
    cfg = mission_config.copy()
    drone = Drone(cfg)
    env = _minimal_env()
    p = DStarLitePlanner()
    p.plan((1, 1), (10, 1), env)
    env.register_obstacle(5, 1)
    det = FaultDetector(cfg["algorithm"])
    fs = det.evaluate(drone, env, p)
    assert fs.type == "path_blocked"


def test_fsm_nominal_start(mission_config: dict) -> None:
    fsm = AutonomyFSM(mission_config["mission"], mission_config["algorithm"])
    assert fsm.get_state() == FSMState.NOMINAL.value


def test_fsm_nominal_to_degraded(mission_config: dict) -> None:
    from src.faults.fault_detector import FaultStatus

    fsm = AutonomyFSM(mission_config["mission"], mission_config["algorithm"])
    drone = Drone(mission_config)
    mm = MissionManager(mission_config)
    fs = FaultStatus(level="WARNING", type="sensor_degradation", details="x")
    fsm.update(0, fs, drone, mm)
    assert fsm.get_state() == FSMState.DEGRADED.value


def test_fsm_nominal_to_replanning_path_blocked(mission_config: dict) -> None:
    from src.faults.fault_detector import FaultStatus

    fsm = AutonomyFSM(mission_config["mission"], mission_config["algorithm"])
    drone = Drone(mission_config)
    mm = MissionManager(mission_config)
    fs = FaultStatus(level="WARNING", type="path_blocked", details="x")
    fsm.update(0, fs, drone, mm)
    assert fsm.get_state() == FSMState.REPLANNING.value
    assert fsm.requires_replan is True


def test_fsm_degraded_to_safe_critical(mission_config: dict) -> None:
    from src.faults.fault_detector import FaultStatus

    fsm = AutonomyFSM(mission_config["mission"], mission_config["algorithm"])
    drone = Drone(mission_config)
    mm = MissionManager(mission_config)
    fsm._state = FSMState.DEGRADED  # noqa: SLF001
    fs = FaultStatus(level="CRITICAL", type="sensor_degradation", details="x")
    fsm.update(1, fs, drone, mm)
    assert fsm.get_state() == FSMState.SAFE_MODE.value
    assert mm.mission_status == "aborted"


def test_fsm_replan_sets_flag(mission_config: dict) -> None:
    from src.faults.fault_detector import FaultStatus

    fsm = AutonomyFSM(mission_config["mission"], mission_config["algorithm"])
    drone = Drone(mission_config)
    mm = MissionManager(mission_config)
    fs = FaultStatus(level="WARNING", type="path_blocked", details="x")
    fsm.update(0, fs, drone, mm)
    assert fsm.requires_replan is True


def test_fsm_transition_log(mission_config: dict) -> None:
    from src.faults.fault_detector import FaultStatus

    fsm = AutonomyFSM(mission_config["mission"], mission_config["algorithm"])
    drone = Drone(mission_config)
    mm = MissionManager(mission_config)
    fs = FaultStatus(level="WARNING", type="sensor_degradation", details="x")
    fsm.update(3, fs, drone, mm)
    log = fsm.get_transition_log()
    assert len(log) == 1
    assert log[0]["timestep"] == 3
    assert log[0]["from_state"] == "NOMINAL"
    assert log[0]["to_state"] == "DEGRADED"
    assert "trigger" in log[0]


def test_integration_loop_150_steps_observes_faults(mission_config: dict) -> None:
    """Run the main loop for 150+ steps without early mission exit to exercise schedules."""
    from main import run_simulation

    r = run_simulation(
        mission_config,
        155,
        verbose=False,
        stop_on_mission_complete=False,
        no_viz=True,
    )
    assert r["final_timestep"] == 155
    assert len(r["fsm_transition_log"]) >= 1
    # mission_01: sensor degradation scheduled at t=40 → FSM should log at least that transition.
    assert any(e.get("timestep") == 40 for e in r["fsm_transition_log"])


def test_emergency_escape_hazard_on_drone_completes_mission(mission_config: dict) -> None:
    """Dynamic hazard spawns on the drone cell mid-route; planner must leave the cell and finish."""
    from main import run_simulation

    root = Path(__file__).resolve().parents[1]
    cfg = json.loads((root / "config" / "mission_emergency_escape.json").read_text(encoding="utf-8"))
    for planner_name in ("d_star_lite", "a_star"):
        cfg = cfg.copy()
        cfg["algorithm"] = {**cfg["algorithm"], "primary_planner": planner_name}
        r = run_simulation(cfg, 120, verbose=False, no_viz=True)
        log = r["fsm_transition_log"]
        assert any(
            e.get("trigger") == "emergency_escape" and e.get("to_state") == "REPLANNING" for e in log
        ), f"expected emergency_escape transition for {planner_name}"
        assert r["final_timestep"] < 119, f"mission should finish before horizon ({planner_name})"


def test_detector_position_trapped(mission_config: dict) -> None:
    cfg = mission_config.copy()
    drone = Drone(cfg)
    env = Environment(
        {
            "environment": {
                "grid_size": [5, 5],
                "static_obstacles": [],
                "dynamic_hazards": [],
            }
        }
    )
    for x in range(5):
        for y in range(5):
            env.register_obstacle(x, y)
    drone.position = (2, 2)
    p = DStarLitePlanner()
    det = FaultDetector(cfg["algorithm"])
    fs = det.evaluate(drone, env, p)
    assert fs.type == "position_trapped"


def test_fsm_replan_succeeded_nominal(mission_config: dict) -> None:
    from src.faults.fault_detector import FaultStatus

    fsm = AutonomyFSM(mission_config["mission"], mission_config["algorithm"])
    drone = Drone(mission_config)
    mm = MissionManager(mission_config)
    fsm._state = FSMState.REPLANNING  # noqa: SLF001
    fsm._last_timestep = 5  # noqa: SLF001
    fsm.requires_replan = True
    fs = FaultStatus(level="NOMINAL", type="none", details="ok")
    fsm.replan_succeeded(fs, drone, mm)
    assert fsm.get_state() == FSMState.NOMINAL.value
    assert fsm.requires_replan is False
