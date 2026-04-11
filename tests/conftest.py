"""Pytest hooks — deterministic RNG for tests that touch stochastic drone / planner behavior."""

from __future__ import annotations

import numpy as np
import pytest


@pytest.fixture(autouse=True)
def _seed_numpy_random() -> None:
    np.random.seed(42)
