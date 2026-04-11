"""Planner verification (Hour 2 acceptance criteria)."""

from __future__ import annotations

import json
import math
from pathlib import Path

import pytest

from main import _make_planner
from src.core.environment import Environment
from src.planners.a_star import AStarPlanner
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
    assert AStarPlanner().get_name() == "a_star"


def _path_cost(path: list[tuple[int, int]], env: Environment) -> float:
    if len(path) < 2:
        return 0.0
    total = 0.0
    for i in range(len(path) - 1):
        u, v = path[i], path[i + 1]
        dx, dy = abs(u[0] - v[0]), abs(u[1] - v[1])
        if dx + dy == 1:
            total += 1.0
        elif dx == 1 and dy == 1:
            total += math.sqrt(2.0)
        else:
            pytest.fail("non-adjacent step in path")
        assert not env.is_blocked(v[0], v[1])
    return total


def test_a_star_finds_valid_path_with_obstacles() -> None:
    cfg = _empty_grid(20, 20)
    cfg["environment"]["static_obstacles"] = [{"x": 8, "y": 5, "width": 4, "height": 1}]
    env = Environment(cfg)
    planner = AStarPlanner()
    path = planner.plan((2, 2), (15, 12), env)
    assert len(path) >= 2
    assert path[0] == (2, 2)
    assert path[-1] == (15, 12)
    for x, y in path:
        assert not env.is_blocked(x, y)


def test_a_star_and_d_star_similar_cost_on_mission_segment() -> None:
    root = Path(__file__).resolve().parents[1]
    cfg = json.loads((root / "config" / "mission_01.json").read_text(encoding="utf-8"))
    env = Environment(cfg)
    start = tuple(cfg["drone"]["start_position"])
    wp0 = tuple(cfg["mission"]["waypoints"][0])
    pa = AStarPlanner().plan(start, wp0, env)
    pd = DStarLitePlanner().plan(start, wp0, env)
    assert pa and pd
    ca, cd = _path_cost(pa, env), _path_cost(pd, env)
    assert abs(ca - cd) < 1e-5
    assert pa[-1] == wp0 == pd[-1]


def test_a_star_replan_after_obstacle_valid() -> None:
    env = Environment(_empty_grid(22, 22))
    planner = AStarPlanner()
    planner.plan((0, 0), (10, 0), env)
    mid = planner.get_full_path()[len(planner.get_full_path()) // 2]
    env._current_timestep = 3
    env.register_obstacle(mid[0], mid[1])
    planner.replan((0, 0), (10, 0), env)
    path_after = planner.get_full_path()
    assert path_after
    assert mid not in path_after
    for x, y in path_after:
        assert not env.is_blocked(x, y)


def test_factory_dispatch_returns_expected_planner() -> None:
    assert isinstance(_make_planner({"algorithm": {"primary_planner": "d_star_lite"}}), DStarLitePlanner)
    assert isinstance(_make_planner({"algorithm": {"primary_planner": "a_star"}}), AStarPlanner)
    with pytest.raises(NotImplementedError):
        _make_planner({"algorithm": {"primary_planner": "unknown_planner"}})


def test_a_star_unreachable_returns_empty_path() -> None:
    cfg = _empty_grid(3, 3)
    cfg["environment"]["static_obstacles"] = [{"x": 1, "y": 0, "width": 1, "height": 3}]
    env = Environment(cfg)
    planner = AStarPlanner()
    assert planner.plan((0, 1), (2, 1), env) == []
    assert planner.get_full_path() == []
