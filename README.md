# Resilient Navigator

A modular, fault-tolerant autonomous drone mission simulator demonstrating dynamic replanning, FSM-based decision logic, and configurable fault injection.

The project models a 2D grid world with static obstacles and scheduled dynamic hazards. A drone follows JSON-defined waypoints while a fault injector perturbs sensors and the map; a detector classifies conditions, and a finite-state machine chooses when to replan, slow down, return home, or abort. Two swappable planners—**D\* Lite** (incremental) and **A\*** (full recomputation)—implement the same octile movement costs, so you can compare incremental repair against a classical baseline.

The codebase is intentionally layered (environment, sensing, planning, faults, decision, visualization) with strict typing and tests, suitable as a teaching or research scaffold for resilient autonomy.

## Architecture

```
┌─────────────────────────────────────────────────┐
│ main.py                                         │
│ (11-step sim loop)                              │
├──────┬──────┬──────┬──────┬──────┬──────────────┤
│ Env  │Drone │Mission│Fault │Fault │ Autonomy   │
│      │      │Mgr    │Inject│Detect│ FSM        │
├──────┴──────┴──────┬┴──────┴──────┴──────────────┤
│                    │ Planners (swappable)       │
│                    ├─────────────┬──────────────┤
│                    │ D* Lite     │ A*           │
├────────────────────┴─────────────┴────────────┤
│ Dashboard (Matplotlib)                          │
└─────────────────────────────────────────────────┘
```

## Features

- **JSON-driven missions**: grid size, obstacles, dynamic hazards, waypoints, home, algorithm thresholds, fault schedules, and visualization options in one file.
- **D\* Lite** incremental replanning after map changes (Koenig & Likhachev, 2002).
- **A\*** fallback with full replan each time (`replan` calls `plan`).
- **Fault injection**: sensor degradation and environmental hazards on a timestep schedule.
- **Fault detection**: WARNING / CRITICAL levels and path–obstacle consistency checks.
- **FSM**: NOMINAL, DEGRADED, REPLANNING, SAFE_MODE, ABORT with a full transition log.
- **Live dashboard**: map, trail, planned path, sensor disk, and telemetry (optional via `--no-viz`).

## Quick Start

```bash
git clone <repository-url>
cd resilient-navigator
python -m venv .venv
source .venv/bin/activate   # Windows: .venv\Scripts\activate
pip install -e ".[dev]"
```

Run the default mission:

```bash
PYTHONPATH=. python main.py --max-steps 200
```

Headless / CI (no Matplotlib window):

```bash
PYTHONPATH=. python main.py --max-steps 160 --no-viz
```

Export a full mission history as JSON (no Matplotlib), then open `viz/mission_replay.html` in a browser and load the file for an HTML replay:

```bash
PYTHONPATH=. python main.py --export-json output/sim_run.json --max-steps 160 --run-full-horizon
```

Use **A\*** as the primary planner by setting `"primary_planner": "a_star"` under `algorithm` in your mission JSON (same schema as D\* Lite).

## Configuration

The simulator reads a single JSON file (default `config/mission_01.json`). Top-level sections:

| Section | Purpose |
| -------- | -------- |
| `environment` | `grid_size`, `static_obstacles`, `dynamic_hazards` (definitions only; activation times come from fault schedule) |
| `drone` | Start pose, speed, sensor range, noise limits, battery |
| `mission` | `waypoints`, `home_position`, optional `waypoint_tolerance`, `priority_weights` |
| `algorithm` | `primary_planner` (`d_star_lite` or `a_star`), sensor thresholds, replan settings |
| `fault_injection` | `schedule` of timed sensor or environmental events |
| `visualization` | Dashboard FPS, trail length, toggles |

**Example** (`algorithm`):

```json
"algorithm": {
  "primary_planner": "d_star_lite",
  "sensor_degradation_threshold": 0.3,
  "critical_fault_threshold": 0.7
}
```

Full field descriptions: **`config/schema.md`**.

## How It Works

Each timestep:

1. **Fault injector** applies scheduled events (sensor noise, hazard activation by id).
2. **Environment** advances time; hazards already triggered update occupancy.
3. **Drone** senses local obstacles (noise-corrupted).
4. **Fault detector** compares noise to thresholds and checks the planner’s path against blocked cells.
5. **FSM** updates mode (e.g. path blocked → REPLANNING) and may set `requires_replan`.
6. **Planner** `replan` runs when required; empty path leads to SAFE_MODE handling in the FSM.
7. **Drone** moves one grid step along the planner’s next cell; **mission manager** advances waypoints.
8. **Dashboard** redraws if enabled.

Waypoints whose target cell is blocked (e.g. inside a hazard) are **skipped** and logged; start positions on blocked cells raise at drone construction.

## Planners

| Planner | `primary_planner` | Behavior |
| -------- | ------------------- | -------- |
| D\* Lite | `d_star_lite` | Incremental search; updates only affected vertices after map changes. |
| A\* | `a_star` | Full A\* from current pose to goal each `plan` / `replan`. |

Both use an **8-connected** grid: cardinal step cost `1`, diagonal `√2`, with a **Euclidean** heuristic. For background on D\* Lite, see:

- Koenig, S., & Likhachev, M. (2002). *D\* Lite.* AAAI.

A\* trades CPU for simplicity: every replan rebuilds the open set from scratch, which is easier to audit than incremental repair.

## FSM States

| State | Meaning |
| ------- | -------- |
| NOMINAL | Healthy sensing and a feasible plan. |
| DEGRADED | Elevated sensor noise; speed may be reduced. |
| REPLANNING | Path invalid or noise worsened; replan requested. |
| SAFE_MODE | Critical fault or failed replan; mission may abort to **home**. |
| ABORT | Battery critically low / depleted, or no route home in safe mode. |

Typical transitions: WARNING → DEGRADED; path blocked → REPLANNING; successful replan → NOMINAL or DEGRADED depending on fault level; failed replan → SAFE_MODE; low battery → ABORT.

## Running Tests

```bash
PYTHONPATH=. python -m pytest tests/ -v
```

## References

- Koenig, S., & Likhachev, M. (2002). *D\* Lite.* AAAI.
- LaValle, S. M. (1998). *Rapidly-Exploring Random Trees.* (possible future extension for sampling-based planning.)

## License

MIT
