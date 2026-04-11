"""Abstract interface for path planners."""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import List, Tuple

from src.core.environment import Environment


class BasePlanner(ABC):
    """Abstract base for grid planners (D* Lite, A*, etc.)."""

    @abstractmethod
    def plan(
        self,
        start: Tuple[int, int],
        goal: Tuple[int, int],
        environment: Environment,
    ) -> List[Tuple[int, int]]:
        """Compute a full path from start to goal."""

    @abstractmethod
    def replan(
        self,
        current_position: Tuple[int, int],
        goal: Tuple[int, int],
        environment: Environment,
    ) -> None:
        """Recompute the path after the robot moves and/or the map changes."""

    @abstractmethod
    def get_next_step(self, current_position: Tuple[int, int]) -> Tuple[int, int]:
        """Return the next grid cell to move toward along the current plan."""

    @abstractmethod
    def get_full_path(self) -> List[Tuple[int, int]]:
        """Return the entire planned path from current start context to goal (for visualization)."""

    @abstractmethod
    def get_name(self) -> str:
        """Human-readable planner name."""
