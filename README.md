# Resilient Navigator - AI-coding Hackathon
Hackathon context: This repository was built during an AI-coding hackathon focused on using coding agents to rapidly develop software. I defined and iterated on the project concept, architecture, behavior, and evaluation, while AI coding tools generated the large majority of the implementation. I’m publishing it as an example of AI-assisted prototyping rather than as a claim of hand-authoring the full codebase.

A modular, fault-tolerant autonomous drone mission simulator demonstrating dynamic replanning, FSM-based decision logic, and configurable fault injection.

The project models a 2D grid world with static obstacles and scheduled dynamic hazards. A drone follows JSON-defined waypoints while a fault injector perturbs sensors and the map; a detector classifies conditions, and a finite-state machine chooses when to replan, slow down, return home, or abort. Two swappable planners—**D\* Lite** (incremental) and **A\*** (full recomputation)—share the same octile movement costs, so you can compare incremental repair against a classical baseline.

The codebase is layered (environment, sensing, planning, faults, decision, visualization) with strict typing and tests, suitable as a teaching or research scaffold for resilient autonomy.

## Architecture

```
┌─────────────────────────────────────────────────┐
│ main.py                                         │
│ (simulation loop)                               │
├──────┬──────┬──────┬──────┬──────┬──────────────┤
│ Env  │Drone │Mission│Fault │Fault │ Autonomy   │
│      │      │Mgr    │Inject│Detect│ FSM        │
├──────┴──────┴──────┬┴──────┴──────┴──────────────┤
│                    │ Planners (swappable)       │
│                    ├─────────────┬──────────────┤
│                    │ D* Lite     │ A*           │
├────────────────────┴─────────────┴──────────────┤
│ Live dashboard (Matplotlib)  │  HTML replay    │
│ (optional)                   │  (viz/, browser)│
└─────────────────────────────────────────────────┘
```

## Features

- **JSON-driven missions**: grid size, obstacles, dynamic hazards, waypoints, home, algorithm thresholds, fault schedules, and visualization options in one file.
- **D\* Lite** incremental replanning after map changes (Koenig & Likhachev, 2002).
- **A\*** with full replan each time (`replan` calls `plan`).
- **Fault injection**: sensor degradation and environmental hazards on a timestep schedule.
- **Fault detection**: WARNING / CRITICAL levels and path–obstacle consistency checks.
- **FSM**: NOMINAL, DEGRADED, REPLANNING, SAFE_MODE, ABORT with a full transition log.
- **Live dashboard**: map, trail, planned path, sensor disk, and telemetry (optional via `--no-viz`).
- **HTML mission replay** (`viz/mission_replay.html`): self-contained browser dashboard with optional in-browser simulation (no Python required for demos), six embedded mission presets, and side-by-side D\* Lite vs A\* comparison when both runs are cached.

## Quick start

```bash
git clone <repository-url>
cd resilient-navigator
python -m venv .venv
source .venv/bin/activate   # Windows: .venv\Scripts\activate
pip install -e ".[dev]"
```

Using [uv](https://github.com/astral-sh/uv) (optional):

```bash
uv sync --all-extras
```

Run the default mission (`config/mission_01.json`):

```bash
PYTHONPATH=. python main.py --max-steps 200
```

Headless / CI (no Matplotlib window):

```bash
PYTHONPATH=. python main.py --max-steps 160 --no-viz
```

## Mission replay dashboard (HTML)

Open **`viz/mission_replay.html`** in a modern browser (double-click or “Open file”). Everything is self-contained in that file; no build step.

| Mode | What it does |
| ------ | -------------- |
| **Easy Open** | Pick one of **six embedded presets** (same JSON as under `config/`). Runs **D\* Lite** and **A\*** in the browser and caches both; switch algorithms instantly from the title bar. |
| **Load exported JSON** | Load a file produced by **`python main.py --export-json …`** for a single recorded run (the planner matches that export). |
| **Load custom config** | Load any mission JSON matching the schema; both planners are simulated in-browser; you can switch between D\* Lite and A\*. |

Export from Python for a faithful recording of the simulator (then load it in the HTML file picker):

```bash
PYTHONPATH=. python main.py --export-json output/sim_run.json --max-steps 160 --run-full-horizon
```

Bundled example missions (also embedded in the HTML presets):

| File | Theme |
| ------ | ------ |
| `config/mission_01.json` | Balanced baseline — replanning and sensor degradation |
| `config/mission_02_gauntlet.json` | Dense obstacles and narrow corridors |
| `config/mission_03_sensor_blackout.json` | Escalating sensor faults toward SAFE_MODE |
| `config/mission_04_battery_race.json` | Long route, tight battery (100×100 grid) |
| `config/mission_05_cascade_failure.json` | Rapid, overlapping faults |
| `config/mission_06_return_to_base.json` | Abort-to-home through hazards |

To drive the **Python** simulator with a specific file:

```bash
PYTHONPATH=. python main.py --config config/mission_04_battery_race.json --max-steps 400 --no-viz
```

Use **A\*** as the primary planner by setting `"primary_planner": "a_star"` under `algorithm` in your mission JSON (same schema as D\* Lite).

## Configuration

The simulator reads a single JSON file (default `config/mission_01.json`). Top-level sections:

| Section | Purpose |
| -------- | -------- |
| `environment` | `grid_size`, `static_obstacles`, `dynamic_hazards` (definitions; activation times come from the fault schedule) |
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

## How it works

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

## FSM states

| State | Meaning |
| ------- | -------- |
| NOMINAL | Healthy sensing and a feasible plan. |
| DEGRADED | Elevated sensor noise; speed may be reduced. |
| REPLANNING | Path invalid or noise worsened; replan requested. |
| SAFE_MODE | Critical fault or failed replan; mission may abort to **home**. |
| ABORT | Battery critically low / depleted, or no route home in safe mode. |

Typical transitions: WARNING → DEGRADED; path blocked → REPLANNING; successful replan → NOMINAL or DEGRADED depending on fault level; failed replan → SAFE_MODE; low battery → ABORT.

## Running tests

```bash
PYTHONPATH=. python -m pytest tests/ -v
```

With uv:

```bash
uv run pytest tests/ -v
```

## References

- Koenig, S., & Likhachev, M. (2002). *D\* Lite.* AAAI.
- LaValle, S. M. (1998). *Rapidly-Exploring Random Trees.* (possible future extension for sampling-based planning.)

## License

MIT
