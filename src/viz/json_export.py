"""Build JSON structures for mission replay export (browser dashboard)."""

from __future__ import annotations

import copy
import math
from datetime import datetime, timezone
from typing import Any

from src.planners.base_planner import BasePlanner


def planner_display_name(planner: BasePlanner) -> str:
    raw = planner.get_name()
    if raw == "d_star_lite":
        return "D* Lite"
    if raw == "a_star":
        return "A*"
    return raw.replace("_", " ").title()


def static_obstacle_cells(config: dict[str, Any]) -> list[list[int]]:
    """All [x, y] cells occupied by static obstacles at t=0."""
    env_cfg = config["environment"]
    gw, gh = int(env_cfg["grid_size"][0]), int(env_cfg["grid_size"][1])
    out: list[list[int]] = []

    def in_bounds(x: int, y: int) -> bool:
        return 0 <= x < gw and 0 <= y < gh

    for ob in env_cfg.get("static_obstacles", []):
        x0, y0 = int(ob["x"]), int(ob["y"])
        w, h = int(ob["width"]), int(ob["height"])
        for x in range(x0, x0 + w):
            for y in range(y0, y0 + h):
                if in_bounds(x, y):
                    out.append([x, y])
    return out


def _disk_cells(cx: int, cy: int, radius: float, gw: int, gh: int) -> list[list[int]]:
    r = float(radius)
    r_int = int(math.ceil(r))
    out: list[list[int]] = []
    for x in range(cx - r_int, cx + r_int + 1):
        for y in range(cy - r_int, cy + r_int + 1):
            if not (0 <= x < gw and 0 <= y < gh):
                continue
            if math.hypot(x - cx, y - cy) <= r + 1e-9:
                out.append([x, y])
    return out


def _rect_cells(x0: int, y0: int, w: int, h: int, gw: int, gh: int) -> list[list[int]]:
    out: list[list[int]] = []
    for x in range(x0, x0 + w):
        for y in range(y0, y0 + h):
            if 0 <= x < gw and 0 <= y < gh:
                out.append([x, y])
    return out


def hazard_cells_for_definition(hz: dict[str, Any], grid_size: tuple[int, int]) -> list[list[int]]:
    """Cells a dynamic hazard occupies once triggered (for export)."""
    gw, gh = grid_size
    htype = hz["type"]
    pos = hz["position"]
    px, py = int(pos["x"]), int(pos["y"])
    if htype == "no_fly_zone":
        return _disk_cells(px, py, float(hz["radius"]), gw, gh)
    if htype == "obstacle_spawn":
        return _rect_cells(px, py, int(hz["width"]), int(hz["height"]), gw, gh)
    raise ValueError(f"Unknown hazard type: {htype}")


def hazard_trigger_timestep(config: dict[str, Any], hazard_id: str) -> int | None:
    """Scheduled timestep for this hazard id, if any."""
    for entry in config.get("fault_injection", {}).get("schedule", []):
        if entry.get("type") == "environmental_hazard" and str(entry.get("hazard_id")) == str(hazard_id):
            return int(entry["timestep"])
    return None


def build_environment_export(config: dict[str, Any]) -> dict[str, Any]:
    env_cfg = config["environment"]
    gw, gh = int(env_cfg["grid_size"][0]), int(env_cfg["grid_size"][1])
    dynamic: list[dict[str, Any]] = []
    for hz in env_cfg.get("dynamic_hazards", []):
        hid = str(hz["id"])
        trig = hazard_trigger_timestep(config, hid)
        cells = hazard_cells_for_definition(hz, (gw, gh))
        dynamic.append(
            {
                "id": hid,
                "type": hz["type"],
                "trigger_timestep": trig,
                "cells": cells,
            }
        )
    return {
        "grid_size": [gw, gh],
        "static_obstacles": static_obstacle_cells(config),
        "dynamic_hazards": dynamic,
    }


def merge_export_events(
    fsm_log: list[dict[str, Any]],
    injector_events: list[dict[str, Any]],
) -> list[dict[str, Any]]:
    """Single chronological list for the replay UI."""
    events: list[dict[str, Any]] = []

    for e in fsm_log:
        events.append(
            {
                "timestep": int(e["timestep"]),
                "type": "fsm_transition",
                "from": e["from_state"],
                "to": e["to_state"],
                "trigger": e["trigger"],
            }
        )

    for e in injector_events:
        et = e["type"]
        t = int(e["timestep"])
        if et == "hazard_spawned":
            events.append(
                {
                    "timestep": t,
                    "type": "hazard_spawned",
                    "id": e["id"],
                    "description": e.get("description", ""),
                }
            )
        elif et == "fault_injected":
            events.append(
                {
                    "timestep": t,
                    "type": "fault_injected",
                    "fault_type": e["fault_type"],
                    "severity": e.get("severity"),
                }
            )
        elif et == "fault_expired":
            events.append(
                {
                    "timestep": t,
                    "type": "fault_expired",
                    "fault_type": e["fault_type"],
                }
            )

    events.sort(key=lambda x: (x["timestep"], _event_sort_key(x["type"])))
    return events


def _event_sort_key(etype: str) -> int:
    order = {"hazard_spawned": 0, "fault_injected": 1, "fault_expired": 2, "fsm_transition": 3}
    return order.get(etype, 9)


def mission_result_string(
    mission_status: str,
    fsm_state: str,
    battery: float,
    dashboard_closed: bool,
) -> str:
    if mission_status == "completed":
        return "completed"
    if battery <= 0.0 and mission_status != "completed":
        return "abort_emergency"
    if fsm_state == "ABORT":
        return "aborted"
    if dashboard_closed:
        return "aborted"
    return "aborted"


def snapshot_config(config: dict[str, Any]) -> dict[str, Any]:
    """Deep copy of config safe for JSON (already JSON-serializable)."""
    return copy.deepcopy(config)


def export_timestamp_iso() -> str:
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat().replace("+00:00", "Z")

