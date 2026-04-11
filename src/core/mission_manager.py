"""Waypoint sequencing and mission status."""

from __future__ import annotations

from typing import Any, Literal, Tuple

MissionStatus = Literal["in_progress", "completed", "aborted"]


class MissionManager:
    """Tracks waypoints and mission lifecycle from JSON config."""

    def __init__(self, config: dict[str, Any]) -> None:
        m = config["mission"]
        self._waypoints: list[tuple[int, int]] = [
            (int(w[0]), int(w[1])) for w in m["waypoints"]
        ]
        self._home: tuple[int, int] = (int(m["home_position"][0]), int(m["home_position"][1]))
        self._waypoint_tolerance: int = int(m.get("waypoint_tolerance", 1))
        self.current_waypoint_index: int = 0
        self.waypoints_completed: int = 0
        self.mission_status: MissionStatus = "in_progress"
        self._aborted_target: tuple[int, int] | None = None

    def _chebyshev(self, a: tuple[int, int], b: tuple[int, int]) -> int:
        return max(abs(a[0] - b[0]), abs(a[1] - b[1]))

    def update(self, drone_position: tuple[int, int]) -> None:
        """Advance waypoint index when the drone reaches the current target within tolerance."""
        if self.mission_status != "in_progress":
            return
        if self.current_waypoint_index >= len(self._waypoints):
            self.mission_status = "completed"
            return

        target = self._waypoints[self.current_waypoint_index]
        if self._chebyshev(drone_position, target) <= self._waypoint_tolerance:
            self.waypoints_completed += 1
            self.current_waypoint_index += 1
            if self.current_waypoint_index >= len(self._waypoints):
                self.mission_status = "completed"

    def get_current_target(self) -> Tuple[int, int]:
        if self.mission_status == "aborted" and self._aborted_target is not None:
            return self._aborted_target
        if self.current_waypoint_index >= len(self._waypoints):
            return self._waypoints[-1] if self._waypoints else self._home
        return self._waypoints[self.current_waypoint_index]

    def abort_mission(self) -> None:
        self.mission_status = "aborted"
        self._aborted_target = self._home

    def skip_waypoint(self) -> None:
        if self.mission_status != "in_progress":
            return
        if self.current_waypoint_index >= len(self._waypoints):
            return
        self.current_waypoint_index += 1
        if self.current_waypoint_index >= len(self._waypoints):
            self.mission_status = "completed"

    def get_progress(self) -> dict[str, Any]:
        total = len(self._waypoints)
        idx = min(self.current_waypoint_index, total)
        return {
            "mission_status": self.mission_status,
            "current_waypoint_index": self.current_waypoint_index,
            "waypoints_total": total,
            "waypoints_completed": self.waypoints_completed,
            "current_target": self.get_current_target(),
            "home_position": self._home,
        }
