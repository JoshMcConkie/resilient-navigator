"""Tests for :class:`Environment` grid, hazards, and change tracking."""

from __future__ import annotations

import pytest

from src.core.drone import Drone
from src.core.environment import HAZARD, OBSTACLE, Environment


def _base_cfg() -> dict:
    return {
        "environment": {
            "grid_size": [30, 30],
            "static_obstacles": [],
            "dynamic_hazards": [],
        }
    }


def test_grid_size_from_config() -> None:
    env = Environment(_base_cfg())
    assert env.width == 30
    assert env.height == 30


def test_static_obstacles_placed_at_init() -> None:
    cfg = _base_cfg()
    cfg["environment"]["static_obstacles"] = [{"x": 4, "y": 7, "width": 3, "height": 2}]
    env = Environment(cfg)
    for x in range(4, 7):
        for y in range(7, 9):
            assert env.is_blocked(x, y)


def test_is_blocked_obstacle_vs_free() -> None:
    cfg = _base_cfg()
    cfg["environment"]["static_obstacles"] = [{"x": 10, "y": 10, "width": 1, "height": 1}]
    env = Environment(cfg)
    assert env.is_blocked(10, 10) is True
    assert env.is_blocked(0, 0) is False


def test_get_neighbors_walkable_eight_connected() -> None:
    env = Environment(_base_cfg())
    n = env.get_neighbors(5, 5)
    assert len(n) == 8
    assert all(not env.is_blocked(x, y) for x, y in n)
    assert (5, 6) in n and (6, 6) in n


def test_ignore_hazard_cells_allows_egress_neighbors() -> None:
    """Emergency planning treats hazards as traversable; obstacles still block."""
    cfg = _base_cfg()
    cfg["environment"]["dynamic_hazards"] = [
        {"id": "nfz", "type": "no_fly_zone", "position": {"x": 10, "y": 10}, "radius": 2}
    ]
    env = Environment(cfg)
    env.trigger_dynamic_hazard_by_id("nfz", event_timestep=0)
    assert env.is_blocked(10, 10)
    assert not env.get_neighbors(10, 10)
    n_esc = env.get_neighbors(10, 10, ignore_hazard_cells=True)
    assert len(n_esc) == 8
    assert not env.is_blocked(10, 11, ignore_hazard_cells=True)


def test_dynamic_hazard_trigger_timestep_and_changes() -> None:
    cfg = _base_cfg()
    cfg["environment"]["dynamic_hazards"] = [
        {
            "id": "nfz_test",
            "type": "no_fly_zone",
            "position": {"x": 12, "y": 12},
            "radius": 2,
        }
    ]
    env = Environment(cfg)
    env.update(4)
    assert env.trigger_dynamic_hazard_by_id("nfz_test", event_timestep=5) is True
    assert env.is_blocked(12, 12)
    changed = env.get_changes_since(4)
    assert changed
    assert (12, 12) in changed


def test_get_changes_since_incremental() -> None:
    env = Environment(_base_cfg())
    env.update(0)
    env.register_obstacle(3, 4)
    env.update(1)
    env.register_obstacle(5, 6)
    c0 = env.get_changes_since(-1)
    assert (3, 4) in c0
    c1 = env.get_changes_since(0)
    assert (5, 6) in c1
    assert (3, 4) not in c1


def test_no_fly_zone_disk_shape() -> None:
    cfg = _base_cfg()
    cfg["environment"]["dynamic_hazards"] = [
        {
            "id": "disk1",
            "type": "no_fly_zone",
            "position": {"x": 10, "y": 10},
            "radius": 1,
        }
    ]
    env = Environment(cfg)
    env.trigger_dynamic_hazard_by_id("disk1", event_timestep=0)
    assert int(env.get_grid()[10, 10]) == HAZARD
    assert int(env.get_grid()[11, 10]) == HAZARD


def test_obstacle_spawn_rectangle() -> None:
    cfg = _base_cfg()
    cfg["environment"]["dynamic_hazards"] = [
        {
            "id": "obs1",
            "type": "obstacle_spawn",
            "position": {"x": 5, "y": 5},
            "width": 2,
            "height": 3,
        }
    ]
    env = Environment(cfg)
    env.trigger_dynamic_hazard_by_id("obs1", event_timestep=0)
    # Grid is indexed [y, x]
    assert int(env.get_grid()[5, 5]) == OBSTACLE
    assert int(env.get_grid()[7, 6]) == OBSTACLE


def test_drone_start_on_blocked_cell_raises() -> None:
    cfg = {
        "environment": {
            "grid_size": [10, 10],
            "static_obstacles": [{"x": 2, "y": 3, "width": 1, "height": 1}],
            "dynamic_hazards": [],
        },
        "drone": {
            "start_position": [2, 3],
            "speed": 1.0,
            "sensor_range": 3,
            "sensor_noise_baseline": 0.05,
            "max_sensor_noise": 0.5,
            "battery_capacity": 100.0,
            "battery_drain_rate": 1.0,
        },
    }
    env = Environment(cfg)
    with pytest.raises(ValueError, match="blocked"):
        Drone(cfg, environment=env)
