# Resilient Navigator — Project Plan

## Overview
A modular, JSON-configurable 2D drone mission simulator that demonstrates autonomous fault-tolerant navigation. The drone follows waypoints using D* Lite as the primary path planner, dynamically re-tasks when environmental hazards appear or sensors degrade, and makes priority-based decisions via a Finite State Machine (FSM). A* serves as the secondary/fallback planner. Visualization is handled via Matplotlib with a real-time telemetry dashboard.

This project targets skills required by autonomy engineering roles at True Anomaly, Boeing, and Skydio — specifically: dynamic replanning, fault detection & response, mission planning, and clean modular architecture.

---

## Tech Stack
- **Language:** Python 3.11+
- **Core Libraries:** NumPy, Matplotlib (animation + dashboard)
- **Config:** JSON (mission/environment configs), pyproject.toml (project metadata/dependencies)
- **No hard-coded behavior.** All parameters are driven by JSON configuration files.

---

## Project Structure

```
resilient-navigator/
├── pyproject.toml
├── README.md
├── main.py
├── config/
│   ├── mission_01.json
│   └── schema.md              # Documents the JSON config schema
├── src/
│   ├── __init__.py
│   ├── core/
│   │   ├── __init__.py
│   │   ├── drone.py
│   │   ├── environment.py
│   │   └── mission_manager.py
│   ├── planners/
│   │   ├── __init__.py
│   │   ├── base_planner.py
│   │   ├── d_star_lite.py
│   │   └── a_star.py
│   ├── faults/
│   │   ├── __init__.py
│   │   ├── fault_injector.py
│   │   └── fault_detector.py
│   ├── decision/
│   │   ├── __init__.py
│   │   └── fsm.py
│   └── viz/
│       ├── __init__.py
│       └── dashboard.py
└── tests/
    ├── __init__.py
    ├── test_planners.py
    ├── test_fsm.py
    └── test_environment.py
```

---

## JSON Configuration Schema

The simulation is entirely driven by a single JSON config file. Here is the full schema for `config/mission_01.json`:

```json
{
  "environment": {
    "grid_size": [80, 80],
    "static_obstacles": [
      {"x": 10, "y": 10, "width": 5, "height": 3},
      {"x": 30, "y": 25, "width": 4, "height": 6}
    ],
    "dynamic_hazards": [
      {
        "id": "hazard_01",
        "type": "no_fly_zone",
        "trigger_timestep": 50,
        "position": {"x": 20, "y": 15},
        "radius": 5
      },
      {
        "id": "hazard_02",
        "type": "obstacle_spawn",
        "trigger_timestep": 100,
        "position": {"x": 45, "y": 40},
        "width": 3,
        "height": 3
      }
    ]
  },
  "drone": {
    "start_position": [0, 0],
    "speed": 1.0,
    "sensor_range": 10,
    "sensor_noise_baseline": 0.05,
    "max_sensor_noise": 0.5,
    "battery_capacity": 1000,
    "battery_drain_rate": 1.0
  },
  "mission": {
    "waypoints": [
      [10, 5],
      [25, 30],
      [50, 50],
      [70, 70]
    ],
    "priority_weights": {
      "mission_completion": 0.6,
      "safety": 0.3,
      "energy_conservation": 0.1
    },
    "home_position": [0, 0]
  },
  "algorithm": {
    "primary_planner": "d_star_lite",
    "secondary_planner": "a_star",
    "replanning_trigger": "on_fault_or_hazard",
    "replan_cooldown_steps": 5,
    "sensor_degradation_threshold": 0.3,
    "critical_fault_threshold": 0.7
  },
  "fault_injection": {
    "schedule": [
      {
        "timestep": 50,
        "type": "sensor_degradation",
        "severity": 0.4,
        "duration": 30,
        "description": "GPS noise increases by 40%"
      },
      {
        "timestep": 120,
        "type": "environmental_hazard",
        "hazard_id": "hazard_01",
        "description": "No-fly zone appears near waypoint 2"
      }
    ]
  },
  "visualization": {
    "fps": 10,
    "trail_length": 50,
    "show_sensor_range": true,
    "show_planned_path": true,
    "dashboard_enabled": true
  }
}
```

---

## Module Specifications

### 1. `main.py` — Entry Point
- Loads the JSON config file (path passed via CLI argument or defaults to `config/mission_01.json`).
- Instantiates all modules: Environment, Drone, MissionManager, FaultInjector, FaultDetector, FSM, selected Planner, and Dashboard.
- Runs the main simulation loop:
  ```
  while mission_not_complete and drone_has_battery:
      1. fault_injector.update(timestep)
      2. environment.update(timestep)
      3. drone.sense(environment)
      4. fault_detector.evaluate(drone)
      5. fsm.update(fault_detector.status, drone, mission_manager)
      6. if fsm.requires_replan:
             planner.replan(drone.position, mission_manager.current_target, environment)
      7. next_move = planner.get_next_step(drone.position)
      8. drone.move(next_move)
      9. mission_manager.update(drone.position)
      10. dashboard.render(drone, environment, planner, fsm, mission_manager, timestep)
      11. timestep += 1
  ```
- Handles graceful shutdown and final summary output.

### 2. `src/core/environment.py` — Environment Model
- **Class: `Environment`**
- Represents the 2D grid world.
- Loads static obstacles from config at init.
- Maintains a 2D occupancy grid (numpy array): 0 = free, 1 = obstacle, 2 = hazard.
- `update(timestep)`: Checks `dynamic_hazards` config. If a hazard's `trigger_timestep` matches, adds it to the grid.
- `is_blocked(x, y) -> bool`: Returns whether a cell is occupied.
- `get_neighbors(x, y) -> List[Tuple[int, int]]`: Returns walkable 8-connected neighbors.
- `get_grid() -> np.ndarray`: Returns the current occupancy grid.
- `get_changes_since(last_timestep) -> List[Tuple[int, int]]`: Returns cells that changed since a given timestep (used by D* Lite for incremental updates).

### 3. `src/core/drone.py` — Drone Model
- **Class: `Drone`**
- Holds state: `position`, `velocity`, `battery`, `sensor_noise_level`, `sensor_range`, `heading`.
- `sense(environment) -> dict`: Returns a local observation of the environment within `sensor_range`, corrupted by `sensor_noise_level`. Returns dict with `observed_obstacles`, `position_estimate` (true position + Gaussian noise scaled by `sensor_noise_level`).
- `move(target_cell)`: Moves one step toward `target_cell`. Decrements battery by `battery_drain_rate`. Updates position and heading.
- `apply_sensor_degradation(severity)`: Increases `sensor_noise_level` by `severity` (clamped to `max_sensor_noise`).
- `restore_sensor(amount)`: Decreases `sensor_noise_level` (for when a fault duration expires).
- `get_telemetry() -> dict`: Returns a snapshot of all drone state for the dashboard.

### 4. `src/core/mission_manager.py` — Mission Manager
- **Class: `MissionManager`**
- Holds the ordered list of waypoints from config.
- Tracks `current_waypoint_index`, `waypoints_completed`, `mission_status` (in_progress, completed, aborted).
- `update(drone_position)`: Checks if the drone has reached the current waypoint (within a configurable tolerance). If so, advances to the next waypoint.
- `get_current_target() -> Tuple[int, int]`: Returns the current target waypoint.
- `abort_mission()`: Sets status to aborted, sets current target to `home_position`.
- `skip_waypoint()`: Skips the current waypoint (used when a waypoint is inside a hazard zone).
- `get_progress() -> dict`: Returns mission progress data for the dashboard.

### 5. `src/planners/base_planner.py` — Abstract Planner Interface
- **Abstract Class: `BasePlanner`**
- Defines the interface that all planners must implement:
  - `plan(start, goal, environment) -> List[Tuple[int, int]]`: Computes a full path.
  - `replan(current_position, goal, environment)`: Recomputes the path (may be incremental or full depending on implementation).
  - `get_next_step(current_position) -> Tuple[int, int]`: Returns the next cell to move to.
  - `get_full_path() -> List[Tuple[int, int]]`: Returns the entire planned path (for visualization).
  - `get_name() -> str`: Returns the planner name string.

### 6. `src/planners/d_star_lite.py` — D* Lite Planner
- **Class: `DStarLitePlanner(BasePlanner)`**
- Implements the D* Lite algorithm (Koenig & Likhachev, 2002).
- Key features:
  - Maintains `g` and `rhs` value grids.
  - Uses a priority queue with key calculation: `[min(g(s), rhs(s)) + h(s, start) + km; min(g(s), rhs(s))]`.
  - `km` accumulates as the drone moves (accounts for start position changes).
  - `replan()` performs incremental updates: only reprocesses cells affected by environment changes (retrieved via `environment.get_changes_since()`). This is the key advantage over A* — it does not recompute from scratch.
  - `plan()` performs initial full computation.
- Handles 8-connected grid movement with diagonal cost = sqrt(2), cardinal cost = 1.
- Edge costs: infinite for blocked cells, normal cost for free cells.
- Reference: Koenig, S., & Likhachev, M. (2002). "D* Lite." Proceedings of the AAAI Conference on Artificial Intelligence.

### 7. `src/planners/a_star.py` — A* Planner (Secondary/Fallback)
- **Class: `AStarPlanner(BasePlanner)`**
- Standard A* implementation with Euclidean heuristic.
- `replan()` simply calls `plan()` again (full recompute — this is the trade-off vs. D* Lite).
- Used as a fallback if D* Lite encounters an error or if selected via config.
- 8-connected grid, same cost model as D* Lite for consistency.

### 8. `src/faults/fault_injector.py` — Fault Injector
- **Class: `FaultInjector`**
- Reads the `fault_injection.schedule` from config.
- `update(timestep, drone, environment)`: At each timestep, checks if any scheduled fault should activate.
  - `sensor_degradation`: Calls `drone.apply_sensor_degradation(severity)`. Tracks duration; calls `drone.restore_sensor()` when duration expires.
  - `environmental_hazard`: Calls `environment.add_hazard()` to place the hazard on the grid.
- Maintains a list of `active_faults` with remaining durations.
- `get_active_faults() -> List[dict]`: Returns currently active faults for the dashboard and fault detector.

### 9. `src/faults/fault_detector.py` — Fault Detector
- **Class: `FaultDetector`**
- Analyzes drone telemetry and environment state to detect and classify faults.
- `evaluate(drone, environment, active_faults) -> FaultStatus`:
  - Compares `drone.sensor_noise_level` against `sensor_degradation_threshold` and `critical_fault_threshold` from config.
  - Checks if the current planned path intersects any newly appeared hazards.
  - Returns a `FaultStatus` object with:
    - `level`: `NOMINAL`, `WARNING`, `CRITICAL`
    - `type`: `none`, `sensor_degradation`, `path_blocked`, `combined`
    - `details`: Human-readable description.
- This module does NOT decide what to do — it only reports. The FSM decides.

### 10. `src/decision/fsm.py` — Finite State Machine
- **Class: `AutonomyFSM`**
- **States:**
  - `NOMINAL` — Normal operation. Follow the planned path.
  - `DEGRADED` — Sensor noise is above warning threshold but below critical. Continue mission with increased caution (slower speed or tighter replanning). Trigger a replan if path may be suboptimal.
  - `REPLANNING` — Active replanning in progress. Triggered when path is blocked or environment changed significantly. Transitions to NOMINAL or DEGRADED once replan completes.
  - `SAFE_MODE` — Critical fault detected. Abandon current waypoint, replan to home position.
  - `ABORT` — Unrecoverable situation (e.g., no valid path to home, battery critical). Drone stops.
- **Transition Logic (driven by `priority_weights` from config):**
  - `NOMINAL` -> `DEGRADED`: FaultDetector reports WARNING level.
  - `NOMINAL` -> `REPLANNING`: FaultDetector reports path_blocked.
  - `DEGRADED` -> `REPLANNING`: FaultDetector reports path_blocked OR sensor noise worsens.
  - `DEGRADED` -> `NOMINAL`: Fault clears, sensor noise returns below threshold.
  - `DEGRADED` -> `SAFE_MODE`: FaultDetector reports CRITICAL level.
  - `REPLANNING` -> `NOMINAL`: Replan succeeds, fault level is NOMINAL.
  - `REPLANNING` -> `DEGRADED`: Replan succeeds, but fault level is still WARNING.
  - `REPLANNING` -> `SAFE_MODE`: Replan fails or fault level is CRITICAL.
  - `SAFE_MODE` -> `ABORT`: No valid path to home or battery insufficient.
  - Any state -> `ABORT`: Battery falls below minimum threshold.
- `update(fault_status, drone, mission_manager) -> None`: Evaluates transitions, updates current state, sets `requires_replan` flag, may call `mission_manager.skip_waypoint()` or `mission_manager.abort_mission()`.
- `get_state() -> str`: Returns current state name.
- `get_transition_log() -> List[dict]`: Returns history of all state transitions with timestamps (for dashboard and post-mission analysis).

### 11. `src/viz/dashboard.py` — Visualization Dashboard
- **Class: `Dashboard`**
- Uses `matplotlib.animation.FuncAnimation` for real-time animated visualization.
- **Layout: 2-panel figure**
  - **Left panel (large):** The 2D grid map.
    - Static obstacles rendered as dark gray cells.
    - Dynamic hazards rendered as red cells (appear when triggered).
    - Waypoints rendered as numbered blue diamonds.
    - Current waypoint target highlighted with a ring.
    - Drone rendered as a green triangle (oriented by heading).
    - Planned path rendered as a dotted cyan line.
    - Drone trail rendered as a fading green line (last N positions from config `trail_length`).
    - Sensor range rendered as a translucent green circle around the drone (if `show_sensor_range` is true).
    - Sensed obstacles (within sensor range) rendered with a distinct color to show what the drone "sees."
  - **Right panel (narrow, stacked text/indicators):** Telemetry dashboard.
    - **FSM State:** Large colored text. Green=NOMINAL, Yellow=DEGRADED, Orange=REPLANNING, Red=SAFE_MODE, Black=ABORT.
    - **Sensor Health:** Bar indicator showing `sensor_noise_level` vs. thresholds.
    - **Battery:** Bar indicator showing remaining battery.
    - **Active Planner:** Text showing "D* Lite" or "A*".
    - **Mission Progress:** "Waypoint 2/4" style indicator.
    - **Active Faults:** List of currently active fault descriptions.
    - **Timestep:** Current simulation timestep.
- `render(drone, environment, planner, fsm, mission_manager, timestep)`: Updates all visual elements for the current frame.
- Supports saving the animation as a GIF or MP4 via config flag (stretch goal).

---

## pyproject.toml

```toml
[project]
name = "resilient-navigator"
version = "0.1.0"
description = "A modular, fault-tolerant autonomous drone mission simulator demonstrating dynamic replanning with D* Lite, FSM-based decision logic, and configurable fault injection."
readme = "README.md"
requires-python = ">=3.11"
license = {text = "MIT"}
authors = [
    {name = "Josh"}
]

[project.dependencies]
dependencies = [
    "numpy>=1.24.0",
    "matplotlib>=3.7.0",
]

[project.optional-dependencies]
dev = [
    "pytest>=7.0.0",
    "pytest-cov>=4.0.0",
]

[project.scripts]
resilient-navigator = "main:main"
```

---

## Implementation Order (5-Hour Sprint)

### Hour 1 — Foundation (Project Scaffolding + Core Models)
**Files to create:**
- `pyproject.toml`
- `config/mission_01.json` (full config as specified above)
- `config/schema.md`
- `src/core/environment.py`
- `src/core/drone.py`
- `src/core/mission_manager.py`
- `main.py` (skeleton with config loading and simulation loop stub)

**Acceptance criteria:**
- [ ] `Environment` loads config, renders static obstacles on a numpy grid, and can add dynamic hazards.
- [ ] `Drone` initializes from config, can move, sense, and report telemetry.
- [ ] `MissionManager` tracks waypoints and detects arrival.
- [ ] `main.py` loads config and prints a basic simulation loop to stdout.

### Hour 2 — Path Planning (D* Lite + Abstract Interface)
**Files to create:**
- `src/planners/base_planner.py`
- `src/planners/d_star_lite.py`

**Acceptance criteria:**
- [ ] `BasePlanner` abstract class is defined with all required methods.
- [ ] `DStarLitePlanner` computes an initial path from start to first waypoint on a grid with obstacles.
- [ ] `DStarLitePlanner.replan()` performs incremental update when environment changes (not full recompute).
- [ ] Path is valid (does not pass through obstacles).
- [ ] Basic test: plan a path, add an obstacle on the path, replan, verify new path avoids it.

### Hour 3 — Fault System + FSM
**Files to create:**
- `src/faults/fault_injector.py`
- `src/faults/fault_detector.py`
- `src/decision/fsm.py`

**Acceptance criteria:**
- [ ] `FaultInjector` fires faults at configured timesteps and manages durations.
- [ ] `FaultDetector` correctly classifies NOMINAL / WARNING / CRITICAL from drone telemetry.
- [ ] FSM transitions correctly through all states based on fault status.
- [ ] FSM sets `requires_replan` flag when transitioning to REPLANNING.
- [ ] FSM triggers `mission_manager.abort_mission()` when entering SAFE_MODE.
- [ ] Integration: faults inject -> detector classifies -> FSM transitions -> replan triggers.

### Hour 4 — Visualization + Integration
**Files to create:**
- `src/viz/dashboard.py`
- Wire everything together in `main.py`

**Acceptance criteria:**
- [ ] Full simulation loop runs end-to-end with all modules connected.
- [ ] Left panel shows grid, obstacles, hazards, waypoints, drone, path, trail, sensor range.
- [ ] Right panel shows FSM state, sensor health, battery, planner name, mission progress, active faults, timestep.
- [ ] Dynamic hazards visually appear at the correct timestep.
- [ ] Replanning is visually apparent (path changes on screen).
- [ ] FSM state color changes are visible in real time.

### Hour 5 — A* Fallback + Polish + Testing
**Files to create:**
- `src/planners/a_star.py`
- `tests/test_planners.py`
- `tests/test_fsm.py`
- `tests/test_environment.py`
- `README.md`

**Acceptance criteria:**
- [ ] `AStarPlanner` implements `BasePlanner` and produces valid paths.
- [ ] Switching `primary_planner` to `a_star` in config works without code changes.
- [ ] Unit tests pass for planners (path validity, obstacle avoidance), FSM (state transitions), and environment (hazard spawning).
- [ ] README includes: project overview, setup instructions, how to run, how to configure, architecture diagram (ASCII), and references to D* Lite paper.
- [ ] Code is clean, well-commented, and has docstrings on all public methods.
- [ ] Edge cases handled: no valid path exists, drone starts inside obstacle, waypoint inside hazard zone.

---

## Key Design Principles
1. **No hard-coded behavior.** Every parameter comes from the JSON config.
2. **Modularity.** Planners, fault types, and decision logic are all swappable via config or by implementing the abstract interface.
3. **Separation of concerns.** Detection is separate from decision-making. The FaultDetector reports; the FSM decides.
4. **Traceability.** The FSM logs all transitions. The dashboard shows real-time state. Post-mission analysis is possible.
5. **Extensibility.** Adding a new planner = implement `BasePlanner`. Adding a new fault type = add to the JSON schedule and handle in `FaultInjector`. Adding a new FSM state = add to the state transition table.

---

## References
- Koenig, S., & Likhachev, M. (2002). *D* Lite.* Proceedings of the AAAI Conference on Artificial Intelligence (AAAI), 476-483.
- LaValle, S. M. (1998). *Rapidly-Exploring Random Trees: A New Tool for Path Planning.* (For future RRT* extension)

---

## Notes for AI-Assisted Development
- When implementing D* Lite, follow the pseudocode from the original Koenig & Likhachev 2002 paper exactly. Do not simplify or skip the `km` accumulation — it is essential for correctness when the start node moves.
- All grid coordinates are (x, y) where x is column and y is row. Be consistent.
- Use type hints throughout. Target Python 3.11+ syntax.
- Prefer composition over inheritance except for the `BasePlanner` hierarchy.
- The simulation loop in `main.py` is the single source of truth for execution order. Modules should not call each other directly except through the loop.
