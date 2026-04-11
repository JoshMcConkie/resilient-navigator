"""
D* Lite (Koenig & Likhachev, 2002) — final version (Figure 5).

Search runs backward from goal; g-values are shortest path costs to the goal.
"""

from __future__ import annotations

import heapq
import math
from typing import List, Set, Tuple

import numpy as np

from src.core.environment import Environment
from src.planners.base_planner import BasePlanner


def _h(a: Tuple[int, int], b: Tuple[int, int]) -> float:
    """Heuristic: Euclidean distance on the grid (consistent with octile costs)."""
    return float(math.hypot(a[0] - b[0], a[1] - b[1]))


def _leq_key(k1: Tuple[float, float], k2: Tuple[float, float]) -> bool:
    """Lexicographic k1 <= k2 (paper notation <=_)."""
    return k1[0] < k2[0] or (k1[0] == k2[0] and k1[1] <= k2[1])


def _lt_key(k1: Tuple[float, float], k2: Tuple[float, float]) -> bool:
    """Strict lexicographic k1 < k2."""
    return k1[0] < k2[0] or (k1[0] == k2[0] and k1[1] < k2[1])


class DStarLitePlanner(BasePlanner):
    """
    D* Lite incremental search on an 8-connected grid.

    Implements Figure 5 (final version) from Koenig & Likhachev (2002), including
    ``km`` accumulation when the start vertex moves.
    """

    def __init__(self) -> None:
        self._env: Environment | None = None
        self._w = 0
        self._hgt = 0
        self._g: np.ndarray | None = None
        self._rhs: np.ndarray | None = None
        self._s_start: Tuple[int, int] = (0, 0)
        self._s_goal: Tuple[int, int] = (0, 0)
        self._s_last: Tuple[int, int] = (0, 0)
        self._km = 0.0

        self._heap: list[tuple[float, float, int, Tuple[int, int]]] = []
        self._heap_seq = 0
        self._heap_best: dict[Tuple[int, int], Tuple[float, float]] = {}

        self._last_processed_timestep = -1
        self._path: List[Tuple[int, int]] = []
        self._full_initialize_calls = 0
        self._exempt_cell: Tuple[int, int] | None = None
        self._ignore_hazard_cells: bool = False

    @property
    def full_initialize_calls(self) -> int:
        """How many times :meth:`_initialize` (paper ``Initialize()``) has run; ``replan`` does not reset g/rhs."""

        return self._full_initialize_calls

    def get_name(self) -> str:
        return "d_star_lite"

    # --- grid indexing: (x, y) -> [y, x] ---

    def _ig(self, x: int, y: int) -> float:
        assert self._g is not None
        return float(self._g[y, x])

    def _irhs(self, x: int, y: int) -> float:
        assert self._rhs is not None
        return float(self._rhs[y, x])

    def _set_g(self, x: int, y: int, v: float) -> None:
        assert self._g is not None
        self._g[y, x] = v

    def _set_rhs(self, x: int, y: int, v: float) -> None:
        assert self._rhs is not None
        self._rhs[y, x] = v

    def calculate_key(self, s: Tuple[int, int]) -> Tuple[float, float]:
        """
        Paper Figure 5, line 01''':
        [min(g(s), rhs(s)) + h(s_start, s) + km; min(g(s), rhs(s))]
        """
        x, y = s
        gx, rhsx = self._ig(x, y), self._irhs(x, y)
        m = min(gx, rhsx)
        return (m + _h(self._s_start, s) + self._km, m)

    def _edge_cost(self, u: Tuple[int, int], v: Tuple[int, int]) -> float:
        """Cost of traversing edge u -> v (8-connected)."""
        assert self._env is not None
        ux, uy = u
        vx, vy = v
        ex = self._exempt_cell
        ign = self._ignore_hazard_cells
        if self._env.is_blocked(vx, vy, exempt=ex, ignore_hazard_cells=ign) or self._env.is_blocked(
            ux, uy, exempt=ex, ignore_hazard_cells=ign
        ):
            return float("inf")
        if abs(ux - vx) + abs(uy - vy) == 1:
            return 1.0
        if abs(ux - vx) == 1 and abs(uy - vy) == 1:
            return math.sqrt(2.0)
        return float("inf")

    def _succ(self, u: Tuple[int, int]) -> List[Tuple[int, int]]:
        """Succ(u): valid moves from u toward the goal (paper Figure 3)."""
        assert self._env is not None
        return list(
            self._env.get_neighbors(u[0], u[1], ignore_hazard_cells=self._ignore_hazard_cells)
        )

    def _pred(self, u: Tuple[int, int]) -> List[Tuple[int, int]]:
        """Pred(u): all vertices that have an edge into u (8-neighborhood on grid)."""
        ux, uy = u
        out: list[Tuple[int, int]] = []
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                nx, ny = ux + dx, uy + dy
                if 0 <= nx < self._w and 0 <= ny < self._hgt:
                    out.append((nx, ny))
        return out

    def update_vertex(self, u: Tuple[int, int]) -> None:
        """Paper Figure 5, lines 07''–09'' (UpdateVertex for final D* Lite)."""
        assert self._env is not None
        gx, gy = self._s_goal
        if u != (gx, gy):
            best = float("inf")
            for sp in self._succ(u):
                c = self._edge_cost(u, sp)
                gsp = self._ig(sp[0], sp[1])
                best = min(best, c + gsp)
            self._set_rhs(u[0], u[1], best)

        ux, uy = u
        if (ux, uy) in self._heap_best:
            del self._heap_best[(ux, uy)]

        gxy, rhsxy = self._ig(ux, uy), self._irhs(ux, uy)
        if gxy != rhsxy:
            k = self.calculate_key(u)
            self._heap_best[u] = k
            self._heap_seq += 1
            heapq.heappush(self._heap, (k[0], k[1], self._heap_seq, u))

    def _top_key(self) -> Tuple[float, float]:
        """U.TopKey(): smallest key in U, or [inf, inf] if empty (paper)."""
        while self._heap:
            k0, k1, _seq, u = self._heap[0]
            if u not in self._heap_best:
                heapq.heappop(self._heap)
                continue
            kb = self._heap_best[u]
            if (k0, k1) != kb:
                heapq.heappop(self._heap)
                continue
            return (k0, k1)
        return (float("inf"), float("inf"))

    def _pop(self) -> Tuple[Tuple[float, float], Tuple[int, int]] | None:
        while self._heap:
            k0, k1, _seq, u = heapq.heappop(self._heap)
            if u not in self._heap_best:
                continue
            kb = self._heap_best[u]
            if (k0, k1) != kb:
                continue
            del self._heap_best[u]
            return ((k0, k1), u)
        return None

    def _consistent_start(self) -> bool:
        sx, sy = self._s_start
        return self._ig(sx, sy) == self._irhs(sx, sy)

    def compute_shortest_path(self) -> None:
        """Paper Figure 5, lines 10''–20'' (ComputeShortestPath, final version)."""
        sx, sy = self._s_start
        ck = self.calculate_key((sx, sy))
        while _leq_key(self._top_key(), ck) or not self._consistent_start():
            k_old = self._top_key()
            popped = self._pop()
            if popped is None:
                break
            (k0, k1), u = popped
            k_old_t = (k0, k1)
            k_new = self.calculate_key(u)
            if _lt_key(k_old_t, k_new):
                self._heap_best[u] = k_new
                self._heap_seq += 1
                heapq.heappush(self._heap, (k_new[0], k_new[1], self._heap_seq, u))
            else:
                ux, uy = u
                gu, rhsu = self._ig(ux, uy), self._irhs(ux, uy)
                if gu > rhsu:
                    self._set_g(ux, uy, rhsu)
                    for s in self._pred(u):
                        self.update_vertex(s)
                else:
                    self._set_g(ux, uy, float("inf"))
                    for s in set(self._pred(u)) | {u}:
                        self.update_vertex(s)
            ck = self.calculate_key((sx, sy))

    def _initialize_arrays(self) -> None:
        assert self._env is not None
        self._w = self._env.width
        self._hgt = self._env.height
        inf = float("inf")
        self._g = np.full((self._hgt, self._w), inf, dtype=np.float64)
        self._rhs = np.full((self._hgt, self._w), inf, dtype=np.float64)

    def _initialize(self) -> None:
        """Paper Figure 5, lines 02''–06'' (Initialize)."""
        self._full_initialize_calls += 1
        self._heap.clear()
        self._heap_best.clear()
        self._initialize_arrays()
        assert self._g is not None and self._rhs is not None
        gx, gy = self._s_goal
        self._set_rhs(gx, gy, 0.0)
        self.update_vertex((gx, gy))

    def plan(
        self,
        start: Tuple[int, int],
        goal: Tuple[int, int],
        environment: Environment,
        *,
        exempt_start: bool = False,
    ) -> List[Tuple[int, int]]:
        self._exempt_cell = None
        self._ignore_hazard_cells = False
        sx, sy = int(start[0]), int(start[1])
        gx, gy = int(goal[0]), int(goal[1])
        if not exempt_start:
            if environment.is_blocked(sx, sy) or environment.is_blocked(gx, gy):
                raise ValueError("start and goal must lie on free cells")
        else:
            if environment.is_blocked(gx, gy):
                raise ValueError("goal must lie on a free cell")
            self._exempt_cell = (sx, sy)
            self._ignore_hazard_cells = True
        self._env = environment
        self._s_start = (sx, sy)
        self._s_goal = (gx, gy)
        self._s_last = self._s_start
        self._km = 0.0
        self._initialize()
        self.compute_shortest_path()
        self._last_processed_timestep = max(environment.current_timestep, 0)
        self._path = self._extract_path()
        return list(self._path)

    def replan(
        self,
        current_position: Tuple[int, int],
        goal: Tuple[int, int],
        environment: Environment,
        *,
        exempt_start: bool = False,
    ) -> None:
        """
        Incremental replan: update ``km`` when the start moves, apply
        :meth:`Environment.get_changes_since` to touch only affected vertices,
        then :meth:`compute_shortest_path`. Does **not** reset ``g``/``rhs``.
        """
        if (int(goal[0]), int(goal[1])) != self._s_goal:
            self.plan(current_position, goal, environment, exempt_start=exempt_start)
            return

        self._exempt_cell = None
        self._ignore_hazard_cells = False
        gx, gy = int(goal[0]), int(goal[1])
        cur = (int(current_position[0]), int(current_position[1]))
        if not exempt_start:
            if environment.is_blocked(cur[0], cur[1]) or environment.is_blocked(gx, gy):
                raise ValueError("current position and goal must lie on free cells")
        else:
            if environment.is_blocked(gx, gy):
                raise ValueError("goal must lie on a free cell")
            self._exempt_cell = cur
            self._ignore_hazard_cells = True

        self._env = environment
        if cur != self._s_last:
            self._km += _h(self._s_last, cur)
            self._s_last = cur
        self._s_start = cur

        changed = environment.get_changes_since(self._last_processed_timestep)
        self._last_processed_timestep = environment.current_timestep

        affected: Set[Tuple[int, int]] = set()
        for cx, cy in changed:
            affected.add((cx, cy))
            for px, py in self._pred((cx, cy)):
                affected.add((px, py))

        for u in affected:
            self.update_vertex(u)

        self.compute_shortest_path()
        self._path = self._extract_path()

    def _extract_path(self) -> List[Tuple[int, int]]:
        """Follow greedy steps: argmin_{s in Succ(u)} c(u,s) + g(s)."""
        if self._g is None or self._env is None:
            return []
        sx, sy = self._s_start
        gx, gy = self._s_goal
        if math.isinf(self._ig(sx, sy)):
            return []

        path: List[Tuple[int, int]] = [(sx, sy)]
        cur = (sx, sy)
        guard = self._w * self._hgt + 5
        while cur != (gx, gy) and guard > 0:
            guard -= 1
            best: Tuple[int, int] | None = None
            best_c = float("inf")
            for sp in self._succ(cur):
                c = self._edge_cost(cur, sp) + self._ig(sp[0], sp[1])
                if c < best_c or (math.isclose(c, best_c) and (best is None or sp < best)):
                    best_c = c
                    best = sp
            if best is None or math.isinf(best_c):
                break
            path.append(best)
            cur = best
        return path

    def get_next_step(self, current_position: Tuple[int, int]) -> Tuple[int, int]:
        cx, cy = int(current_position[0]), int(current_position[1])
        if self._g is None or self._env is None:
            raise RuntimeError("call plan() or replan() before get_next_step()")
        if math.isinf(self._ig(cx, cy)):
            raise RuntimeError("no known path to the goal from the current cell")
        gx, gy = self._s_goal
        if (cx, cy) == (gx, gy):
            return (cx, cy)
        best: Tuple[int, int] | None = None
        best_c = float("inf")
        for sp in self._succ((cx, cy)):
            c = self._edge_cost((cx, cy), sp) + self._ig(sp[0], sp[1])
            if c < best_c or (math.isclose(c, best_c) and (best is None or sp < best)):
                best_c = c
                best = sp
        if best is None or math.isinf(best_c):
            raise RuntimeError("no valid successor toward the goal")
        return best

    def get_full_path(self) -> List[Tuple[int, int]]:
        return list(self._path)
