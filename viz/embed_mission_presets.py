#!/usr/bin/env python3
"""Rewrite MISSION_PRESETS in mission_replay.html from config/mission_*.json files."""

from __future__ import annotations

import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "config"
HTML = ROOT / "viz" / "mission_replay.html"

META: dict[str, tuple[str, str]] = {
    "mission_01": ("Original", "Balanced baseline — replanning + sensor degradation"),
    "mission_02": ("The Gauntlet", "Dense obstacles and narrow corridors"),
    "mission_03": ("Sensor Blackout", "Escalating sensor faults toward SAFE_MODE"),
    "mission_04": ("Battery Race", "Long route, tight battery (100×100 grid)"),
    "mission_05": ("Cascade Failure", "Rapid, overlapping faults"),
    "mission_06": ("Return to Base", "Abort-to-home through hazards"),
}

FILES = {
    "mission_01": "mission_01.json",
    "mission_02": "mission_02_gauntlet.json",
    "mission_03": "mission_03_sensor_blackout.json",
    "mission_04": "mission_04_battery_race.json",
    "mission_05": "mission_05_cascade_failure.json",
    "mission_06": "mission_06_return_to_base.json",
}


def main() -> None:
    out: dict[str, dict] = {}
    for key, fn in FILES.items():
        cfg = json.loads((CONFIG / fn).read_text(encoding="utf-8"))
        name, subtitle = META[key]
        out[key] = {"name": name, "subtitle": subtitle, "config": cfg}
    blob = json.dumps(out, separators=(",", ":"), ensure_ascii=True)
    text = HTML.read_text(encoding="utf-8")
    start = "  const MISSION_PRESETS = "
    end = ";\n\n  var PRESET_ORDER"
    i0 = text.find(start)
    i1 = text.find(end, i0)
    if i0 < 0 or i1 < 0:
        raise SystemExit("Could not find MISSION_PRESETS splice markers in mission_replay.html")
    new_text = text[: i0 + len(start)] + blob + text[i1:]
    HTML.write_text(new_text, encoding="utf-8")
    print(f"Updated {HTML.relative_to(ROOT)} ({len(blob)} bytes JSON)")


if __name__ == "__main__":
    main()
