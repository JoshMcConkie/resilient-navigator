"""
A* search on an 8-connected grid with Euclidean heuristic.

Full replan on each :meth:`replan` call (no incremental updates), matching the
same movement costs as :class:`DStarLitePlanner`.
"""

from __future__ import annotations

import heapq
import math
from typing import Dict, List, Tuple

from src.core.environment import Environment
from src.planners.base_planner import BasePlanner


def _h(a: Tuple[int, int], b: Tuple[int, int]) -> float:
    """Euclidean heuristic (consistent with octile edge costs)."""
    return float(math.hypot(a[0] - b[0], a[1] - b[1]))


def _edge_cost(u: Tuple[int, int], v: Tuple[int, int], environment: Environment) -> float:
    """Cost of traversing edge u -> v (8-connected); matches D* Lite."""
    ux, uy = u
    vx, vy = v
    if environment.is_blocked(vx, vy) or environment.is_blocked(ux, uy):
        return float("inf")
    if abs(ux - vx) + abs(uy - vy) == 1:
        return 1.0
    if abs(ux - vx) == 1 and abs(uy - vy) == 1:
        return math.sqrt(2.0)
    return float("inf")


class AStarPlanner(BasePlanner):
    """Standard A*; :meth:`replan` is a full :meth:`plan` recomputation."""

    def __init__(self) -> None:
        self._env: Environment | None = None
        self._goal: Tuple[int, int] = (0, 0)
        self._path: List[Tuple[int, int]] = []

    def get_name(self) -> str:
        return "a_star"

    def plan(
        self,
        start: Tuple[int, int],
        goal: Tuple[int, int],
        environment: Environment,
    ) -> List[Tuple[int, int]]:
        """Compute a shortest path with A*; return ``[]`` if no path exists."""
        sx, sy = int(start[0]), int(start[1])
        gx, gy = int(goal[0]), int(goal[1])
        if environment.is_blocked(sx, sy) or environment.is_blocked(gx, gy):
            raise ValueError("start and goal must lie on free cells")

        self._env = environment
        self._goal = (gx, gy)

        start_c = (sx, sy)
        goal_c = (gx, gy)
        if start_c == goal_c:
            self._path = [start_c]
            return list(self._path)

        tie = 0
        open_heap: list[tuple[float, int, float, Tuple[int, int]]] = []
        g_score: Dict[Tuple[int, int], float] = {}
        came_from: Dict[Tuple[int, int], Tuple[int, int]] = {}

        g0 = 0.0
        g_score[start_c] = g0
        heapq.heappush(
            open_heap,
            (g0 + _h(start_c, goal_c), tie, g0, start_c),
        )
        tie += 1

        while open_heap:
            _f, _tie, g_u, u = heapq.heappop(open_heap)
            if g_u > g_score.get(u, float("inf")) + 1e-9:
                continue

            if u == goal_c:
                self._path = self._reconstruct(came_from, u, start_c)
                return list(self._path)

            for v in environment.get_neighbors(u[0], u[1]):
                c = _edge_cost(u, v, environment)
                if math.isinf(c):
                    continue
                tentative = g_u + c
                gv = g_score.get(v)
                if gv is None or tentative < gv - 1e-12:
                    g_score[v] = tentative
                    came_from[v] = u
                    heapq.heappush(
                        open_heap,
                        (tentative + _h(v, goal_c), tie, tentative, v),
                    )
                    tie += 1

        self._path = []
        return []

    def _reconstruct(
        self,
        came_from: Dict[Tuple[int, int], Tuple[int, int]],
        goal_cell: Tuple[int, int],
        start_cell: Tuple[int, int],
    ) -> List[Tuple[int, int]]:
        path_rev: List[Tuple[int, int]] = []
        cur: Tuple[int, int] | None = goal_cell
        guard = (len(came_from) + 2) * 4
        while cur is not None and guard > 0:
            guard -= 1
            path_rev.append(cur)
            if cur == start_cell:
                break
            cur = came_from.get(cur)
        if not path_rev or path_rev[-1] != start_cell:
            return []
        path_rev.reverse()
        return path_rev

    def replan(
        self,
        current_position: Tuple[int, int],
        goal: Tuple[int, int],
        environment: Environment,
    ) -> None:
        """Full recomputation (no incremental repair)."""
        self.plan(current_position, goal, environment)

    def get_next_step(self, current_position: Tuple[int, int]) -> Tuple[int, int]:
        """Return the next cell along the stored path toward the goal."""
        if not self._path or self._env is None:
            raise RuntimeError("call plan() or replan() before get_next_step()")
        cx, cy = int(current_position[0]), int(current_position[1])
        cur = (cx, cy)
        try:
            i = self._path.index(cur)
        except ValueError as exc:
            raise RuntimeError("current position not on planned path") from exc
        gx, gy = self._goal
        if cur == (gx, gy):
            return cur
        if i + 1 < len(self._path):
            return self._path[i + 1]
        raise RuntimeError("no valid successor toward the goal")

    def get_full_path(self) -> List[Tuple[int, int]]:
        """Return the last computed path (empty if planning failed)."""
        return list(self._path)
