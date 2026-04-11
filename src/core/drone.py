"""Drone dynamics, sensing, and telemetry."""

from __future__ import annotations

import math
from typing import Any

import numpy as np

from src.core.environment import Environment, FREE


class Drone:
    """Drone state and motion; parameters from JSON config."""

    def __init__(self, config: dict[str, Any]) -> None:
        d = config["drone"]
        sx, sy = int(d["start_position"][0]), int(d["start_position"][1])
        self.position: tuple[int, int] = (sx, sy)
        self.velocity: tuple[float, float] = (0.0, 0.0)
        self.battery: float = float(d["battery_capacity"])
        self.sensor_noise_level: float = float(d["sensor_noise_baseline"])
        self.sensor_range: int = int(d["sensor_range"])
        self.heading: float = 0.0
        self._speed: float = float(d["speed"])
        self._battery_capacity: float = float(d["battery_capacity"])
        self._battery_drain_rate: float = float(d["battery_drain_rate"])
        self._max_sensor_noise: float = float(d["max_sensor_noise"])
        self._sensor_noise_baseline: float = float(d["sensor_noise_baseline"])
        gw, gh = config["environment"]["grid_size"]
        self._grid_width: int = int(gw)
        self._grid_height: int = int(gh)

    def sense(self, environment: Environment) -> dict[str, Any]:
        """Local observation within sensor_range, corrupted by sensor_noise_level."""
        x, y = self.position
        obs_cells: list[tuple[int, int]] = []
        r = float(self.sensor_range)
        r_int = int(math.ceil(r))
        grid = environment.get_grid()
        for cx in range(x - r_int, x + r_int + 1):
            for cy in range(y - r_int, y + r_int + 1):
                if not (0 <= cx < environment.width and 0 <= cy < environment.height):
                    continue
                if math.hypot(cx - x, cy - y) > r + 1e-9:
                    continue
                if int(grid[cy, cx]) != FREE:
                    obs_cells.append((cx, cy))

        sigma = max(self.sensor_noise_level, 0.0) * 0.5
        noise_x = float(np.random.normal(0.0, sigma)) if sigma > 0 else 0.0
        noise_y = float(np.random.normal(0.0, sigma)) if sigma > 0 else 0.0
        position_estimate = (float(x) + noise_x, float(y) + noise_y)

        return {
            "observed_obstacles": obs_cells,
            "position_estimate": position_estimate,
        }

    def move(self, target_cell: tuple[int, int]) -> None:
        """Move one grid step toward target_cell; drain battery; update heading."""
        x, y = self.position
        tx, ty = int(target_cell[0]), int(target_cell[1])
        if (x, y) == (tx, ty):
            self.velocity = (0.0, 0.0)
            return

        best: tuple[int, int] | None = None
        best_d = math.inf
        for nx in range(x - 1, x + 2):
            for ny in range(y - 1, y + 2):
                if nx == x and ny == y:
                    continue
                if not (0 <= nx < self._grid_width and 0 <= ny < self._grid_height):
                    continue
                d = math.hypot(tx - nx, ty - ny)
                if d < best_d or (math.isclose(d, best_d) and (best is None or (nx, ny) < best)):
                    best_d = d
                    best = (nx, ny)

        if best is None:
            return

        dx, dy = best[0] - x, best[1] - y
        self.position = best
        self.velocity = (float(dx), float(dy))
        self.heading = math.atan2(dy, dx)
        self.battery = max(0.0, self.battery - self._battery_drain_rate)

    def apply_sensor_degradation(self, severity: float) -> None:
        self.sensor_noise_level = min(
            self._max_sensor_noise,
            self.sensor_noise_level + float(severity),
        )

    def restore_sensor(self, amount: float) -> None:
        self.sensor_noise_level = max(
            self._sensor_noise_baseline,
            self.sensor_noise_level - float(amount),
        )

    def get_telemetry(self) -> dict[str, Any]:
        return {
            "position": self.position,
            "velocity": self.velocity,
            "battery": self.battery,
            "sensor_noise_level": self.sensor_noise_level,
            "sensor_range": self.sensor_range,
            "heading": self.heading,
            "speed": self._speed,
            "battery_capacity": self._battery_capacity,
        }
