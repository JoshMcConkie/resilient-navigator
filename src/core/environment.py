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

    def update(self, timestep: int) -> None:
        """Activate dynamic hazards whose trigger_timestep matches."""
        changed: list[tuple[int, int]] = []
        for hz in self._dynamic_hazards:
            hid = str(hz["id"])
            if hid in self._triggered_ids:
                continue
            if int(hz["trigger_timestep"]) != timestep:
                continue
            self._triggered_ids.add(hid)
            htype = hz["type"]
            pos = hz["position"]
            px, py = int(pos["x"]), int(pos["y"])
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

        if changed:
            self._change_events.append((timestep, changed))

    def is_blocked(self, x: int, y: int) -> bool:
        if not self._in_bounds(x, y):
            return True
        return int(self._grid[y, x]) != FREE

    def get_neighbors(self, x: int, y: int) -> List[Tuple[int, int]]:
        """8-connected walkable neighbors (not blocked, in bounds)."""
        out: list[tuple[int, int]] = []
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                nx, ny = x + dx, y + dy
                if not self.is_blocked(nx, ny):
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

    @property
    def width(self) -> int:
        return self._width

    @property
    def height(self) -> int:
        return self._height
