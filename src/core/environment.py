"""2D grid environment with static obstacles and scheduled dynamic hazards."""

from __future__ import annotations

import math
from typing import Any, List, Tuple

import numpy as np

# Occupancy: 0 = free, 1 = obstacle, 2 = hazard
FREE = 0
OBSTACLE = 1
HAZARD = 2


class Environment:
    """Represents the 2D grid world driven by JSON config."""

    def __init__(self, config: dict[str, Any]) -> None:
        env_cfg = config["environment"]
        gw, gh = int(env_cfg["grid_size"][0]), int(env_cfg["grid_size"][1])
        self._width, self._height = gw, gh
        self._grid = np.zeros((gh, gw), dtype=np.int8)
        # Last timestep passed to update(); used with get_changes_since for incremental planners.
        self._current_timestep: int = -1
        self._dynamic_hazards: list[dict[str, Any]] = list(env_cfg.get("dynamic_hazards", []))
        self._triggered_ids: set[str] = set()
        self._change_events: list[tuple[int, list[tuple[int, int]]]] = []

        static_cells: list[tuple[int, int]] = []
        for ob in env_cfg.get("static_obstacles", []):
            static_cells.extend(self._fill_rectangle_cells(ob))
        self._apply_cells(static_cells, OBSTACLE)
        if static_cells:
            self._change_events.append((0, static_cells))

    def _in_bounds(self, x: int, y: int) -> bool:
        return 0 <= x < self._width and 0 <= y < self._height

    def _fill_rectangle_cells(self, ob: dict[str, Any]) -> list[tuple[int, int]]:
        x0, y0 = int(ob["x"]), int(ob["y"])
        w, h = int(ob["width"]), int(ob["height"])
        out: list[tuple[int, int]] = []
        for x in range(x0, x0 + w):
            for y in range(y0, y0 + h):
                if self._in_bounds(x, y):
                    out.append((x, y))
        return out

    def _disk_cells(self, cx: int, cy: int, radius: float) -> list[tuple[int, int]]:
        r = float(radius)
        out: list[tuple[int, int]] = []
        r_int = int(math.ceil(r))
        for x in range(cx - r_int, cx + r_int + 1):
            for y in range(cy - r_int, cy + r_int + 1):
                if not self._in_bounds(x, y):
                    continue
                if math.hypot(x - cx, y - cy) <= r + 1e-9:
                    out.append((x, y))
        return out

    def _apply_cells(self, cells: list[tuple[int, int]], value: int) -> None:
        for x, y in cells:
            self._grid[y, x] = value

    def _apply_dynamic_hazard(self, hz: dict[str, Any]) -> list[tuple[int, int]]:
        """Paint grid cells for one hazard entry; returns changed cell list."""
        htype = hz["type"]
        pos = hz["position"]
        px, py = int(pos["x"]), int(pos["y"])
        changed: list[tuple[int, int]] = []
        if htype == "no_fly_zone":
            cells = self._disk_cells(px, py, float(hz["radius"]))
            for x, y in cells:
                self._grid[y, x] = HAZARD
            changed.extend(cells)
        elif htype == "obstacle_spawn":
            rect = {"x": px, "y": py, "width": int(hz["width"]), "height": int(hz["height"])}
            cells = self._fill_rectangle_cells(rect)
            for x, y in cells:
                self._grid[y, x] = OBSTACLE
            changed.extend(cells)
        else:
            raise ValueError(f"Unknown dynamic hazard type: {htype}")
        return changed

    def update(self, timestep: int) -> None:
        """
        Advance simulation time. Dynamic hazards are **not** applied here — they are
        placed exactly once via :meth:`trigger_dynamic_hazard_by_id` (called from the
        fault-injection schedule) so grid changes, change events, and active-fault
        tracking stay synchronized.
        """
        self._current_timestep = timestep

    def trigger_dynamic_hazard_by_id(
        self,
        hazard_id: str,
        *,
        event_timestep: int | None = None,
    ) -> bool:
        """
        Activate a configured dynamic hazard by id (e.g. fault injection schedule).
        Idempotent: returns False if already triggered or id not found.
        ``event_timestep`` is used for :meth:`get_changes_since` when the caller runs
        before :meth:`update` (e.g. fault injector on the same simulation tick).
        """
        for hz in self._dynamic_hazards:
            if str(hz["id"]) != str(hazard_id):
                continue
            hid = str(hz["id"])
            if hid in self._triggered_ids:
                return False
            self._triggered_ids.add(hid)
            cells = self._apply_dynamic_hazard(hz)
            if event_timestep is not None:
                t = int(event_timestep)
            else:
                t = self._current_timestep if self._current_timestep >= 0 else 0
            if cells:
                self._change_events.append((t, cells))
            return True
        return False

    def is_blocked(
        self,
        x: int,
        y: int,
        *,
        exempt: tuple[int, int] | None = None,
        ignore_hazard_cells: bool = False,
    ) -> bool:
        """
        Blocked if out of bounds, obstacle, or hazard (unless ``ignore_hazard_cells``).
        If ``exempt`` is set, that cell is treated as traversable for planning queries only.
        """
        if exempt is not None and (int(x), int(y)) == (int(exempt[0]), int(exempt[1])):
            return False
        if not self._in_bounds(x, y):
            return True
        val = int(self._grid[y, x])
        if ignore_hazard_cells:
            return val == OBSTACLE
        return val != FREE

    def get_neighbors(
        self,
        x: int,
        y: int,
        *,
        ignore_hazard_cells: bool = False,
    ) -> List[Tuple[int, int]]:
        """8-connected walkable neighbors (in bounds); hazards may be ignored for emergency egress."""
        out: list[tuple[int, int]] = []
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                nx, ny = x + dx, y + dy
                if not self.is_blocked(nx, ny, ignore_hazard_cells=ignore_hazard_cells):
                    out.append((nx, ny))
        return out

    def get_grid(self) -> np.ndarray:
        return self._grid

    def get_changes_since(self, last_timestep: int) -> List[Tuple[int, int]]:
        """Cells whose occupancy changed at timesteps strictly after last_timestep."""
        cells: set[tuple[int, int]] = set()
        for t, lst in self._change_events:
            if t > last_timestep:
                cells.update(lst)
        return sorted(cells)

    def register_obstacle(self, x: int, y: int) -> None:
        """
        Mark a cell as a static obstacle and log it for :meth:`get_changes_since`.
        Used when the map changes mid-simulation (tests, fault injection, editor tools).
        """
        if not self._in_bounds(x, y):
            return
        self._grid[y, x] = OBSTACLE
        t = self._current_timestep if self._current_timestep >= 0 else 0
        self._change_events.append((t, [(x, y)]))

    @property
    def current_timestep(self) -> int:
        """Most recent timestep passed to ``update()`` (or -1 before any update)."""

        return self._current_timestep

    @property
    def width(self) -> int:
        return self._width

    @property
    def height(self) -> int:
        return self._height
