"""Scheduled fault injection coordinated with the simulation clock."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, List

from src.core.drone import Drone
from src.core.environment import Environment


@dataclass
class _SensorFaultState:
    """Tracks one active sensor degradation episode."""

    applied_severity: float
    expire_at_timestep: int
    description: str
    activated_at: int


@dataclass
class _InjectorState:
    fired_schedule_indices: set[int] = field(default_factory=set)
    sensor_episode: _SensorFaultState | None = None


class FaultInjector:
    """
    Reads ``fault_injection.schedule`` from config and activates entries when
    ``update(timestep, ...)`` reaches the configured timestep.
    """

    def __init__(self, fault_config: dict[str, Any]) -> None:
        self._schedule: list[dict[str, Any]] = list(fault_config.get("schedule", []))
        self._state = _InjectorState()
        self._active_faults: list[dict[str, Any]] = []

    def update(self, timestep: int, drone: Drone, environment: Environment) -> None:
        """Fire new schedule entries, expire timed faults, refresh ``active_faults``."""
        self._expire_sensor_fault_if_needed(timestep, drone)

        for i, entry in enumerate(self._schedule):
            if i in self._state.fired_schedule_indices:
                continue
            if int(entry["timestep"]) != timestep:
                continue
            self._state.fired_schedule_indices.add(i)
            etype = entry["type"]
            if etype == "sensor_degradation":
                self._activate_sensor_degradation(timestep, entry, drone)
            elif etype == "environmental_hazard":
                self._activate_environmental_hazard(timestep, entry, environment)
            else:
                raise ValueError(f"Unknown fault schedule type: {etype}")

        self._refresh_active_fault_list(timestep)

    def _activate_sensor_degradation(self, timestep: int, entry: dict[str, Any], drone: Drone) -> None:
        severity = float(entry["severity"])
        duration = int(entry["duration"])
        drone.apply_sensor_degradation(severity)
        desc = str(entry.get("description", "sensor_degradation"))
        self._state.sensor_episode = _SensorFaultState(
            applied_severity=severity,
            expire_at_timestep=timestep + duration,
            description=desc,
            activated_at=timestep,
        )

    def _expire_sensor_fault_if_needed(self, timestep: int, drone: Drone) -> None:
        ep = self._state.sensor_episode
        if ep is None:
            return
        if timestep < ep.expire_at_timestep:
            return
        drone.restore_sensor(ep.applied_severity)
        self._state.sensor_episode = None

    def _activate_environmental_hazard(
        self,
        timestep: int,
        entry: dict[str, Any],
        environment: Environment,
    ) -> None:
        hid = str(entry["hazard_id"])
        environment.trigger_dynamic_hazard_by_id(hid, event_timestep=timestep)

    def _refresh_active_fault_list(self, timestep: int) -> None:
        out: list[dict[str, Any]] = []

        ep = self._state.sensor_episode
        if ep is not None:
            remaining = max(0, ep.expire_at_timestep - timestep)
            out.append(
                {
                    "type": "sensor_degradation",
                    "severity": ep.applied_severity,
                    "remaining_duration": remaining,
                    "description": ep.description,
                    "timestep_activated": ep.activated_at,
                }
            )

        for i, entry in enumerate(self._schedule):
            if i not in self._state.fired_schedule_indices:
                continue
            if entry["type"] != "environmental_hazard":
                continue
            desc = str(entry.get("description", "environmental_hazard"))
            out.append(
                {
                    "type": "environmental_hazard",
                    "severity": None,
                    "remaining_duration": None,
                    "description": desc,
                    "hazard_id": str(entry["hazard_id"]),
                    "timestep_activated": int(entry["timestep"]),
                }
            )

        self._active_faults = out

    def get_active_faults(self) -> List[dict[str, Any]]:
        """Currently active faults for dashboard / detector (copy of internal list)."""
        return list(self._active_faults)
