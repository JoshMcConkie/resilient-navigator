"""Planner verification (Hour 2 acceptance criteria)."""

from __future__ import annotations

import json
from pathlib import Path

import pytest

from src.core.environment import Environment
from src.planners.base_planner import BasePlanner
from src.planners.d_star_lite import DStarLitePlanner


def _empty_grid(w: int, h: int) -> dict:
    return {
        "environment": {
            "grid_size": [w, h],
            "static_obstacles": [],
            "dynamic_hazards": [],
        }
    }


def test_base_planner_is_abstract() -> None:
    with pytest.raises(TypeError):
        BasePlanner()  # type: ignore[abstract,misc]


def test_d_star_initial_path_avoids_static_obstacles() -> None:
    cfg = _empty_grid(30, 30)
    cfg["environment"]["static_obstacles"] = [{"x": 5, "y": 5, "width": 10, "height": 1}]
    env = Environment(cfg)
    planner = DStarLitePlanner()
    path = planner.plan((0, 0), (15, 10), env)
    assert len(path) >= 2
    for x, y in path:
        assert not env.is_blocked(x, y)


def test_mission_01_first_waypoint_path_valid() -> None:
    root = Path(__file__).resolve().parents[1]
    cfg = json.loads((root / "config" / "mission_01.json").read_text(encoding="utf-8"))
    env = Environment(cfg)
    start = tuple(cfg["drone"]["start_position"])
    wp0 = tuple(cfg["mission"]["waypoints"][0])
    planner = DStarLitePlanner()
    path = planner.plan(start, wp0, env)
    assert len(path) >= 2
    assert path[0] == start
    assert path[-1] == wp0
    for cell in path:
        assert isinstance(cell, tuple)
        assert len(cell) == 2
        assert all(isinstance(v, int) for v in cell)
        assert not env.is_blocked(cell[0], cell[1])


def test_replan_avoids_new_obstacle_without_full_reinit() -> None:
    env = Environment(_empty_grid(25, 25))
    planner = DStarLitePlanner()
    path_before = planner.plan((0, 0), (12, 0), env)
    assert len(path_before) >= 2
    mid = path_before[len(path_before) // 2]
    assert mid not in ((0, 0), (12, 0))

    inits_after_plan = planner.full_initialize_calls
    env._current_timestep = 7
    env.register_obstacle(mid[0], mid[1])

    planner.replan((0, 0), (12, 0), env)
    assert planner.full_initialize_calls == inits_after_plan, "replan must not run full Initialize()"

    path_after = planner.get_full_path()
    assert isinstance(path_after, list)
    assert all(isinstance(c, tuple) and len(c) == 2 for c in path_after)
    assert mid not in path_after
    for x, y in path_after:
        assert not env.is_blocked(x, y)


def test_get_next_step_along_path() -> None:
    env = Environment(_empty_grid(15, 15))
    planner = DStarLitePlanner()
    planner.plan((1, 1), (8, 8), env)
    nxt = planner.get_next_step((1, 1))
    assert isinstance(nxt, tuple)
    assert len(nxt) == 2
    assert nxt in env.get_neighbors(1, 1)


def test_get_full_path_returns_tuples() -> None:
    env = Environment(_empty_grid(10, 10))
    planner = DStarLitePlanner()
    planner.plan((0, 0), (3, 3), env)
    fp = planner.get_full_path()
    assert fp
    assert all(type(p) is tuple for p in fp)


def test_blocked_start_raises() -> None:
    cfg = _empty_grid(5, 5)
    cfg["environment"]["static_obstacles"] = [{"x": 0, "y": 0, "width": 1, "height": 1}]
    env = Environment(cfg)
    planner = DStarLitePlanner()
    with pytest.raises(ValueError):
        planner.plan((0, 0), (3, 3), env)


def test_planner_name() -> None:
    assert DStarLitePlanner().get_name() == "d_star_lite"
