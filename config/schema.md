# Mission configuration schema

The simulator reads a single JSON file (default `config/mission_01.json`). All keys below are expected unless noted as optional.

## Top-level keys


| Key               | Type   | Description                                       |
| ----------------- | ------ | ------------------------------------------------- |
| `environment`     | object | Grid world and hazards                            |
| `drone`           | object | Initial drone state and sensor/battery parameters |
| `mission`         | object | Waypoints, priorities, home                       |
| `algorithm`       | object | Planners, replanning, fault thresholds            |
| `fault_injection` | object | Scheduled faults                                  |
| `visualization`   | object | Dashboard and animation settings                  |


## `environment`


| Field              | Type         | Description                                                                                             |
| ------------------ | ------------ | ------------------------------------------------------------------------------------------------------- |
| `grid_size`        | `[int, int]` | Grid width and height in cells; indices use `x` (column) ∈ `[0, width-1]`, `y` (row) ∈ `[0, height-1]`. |
| `static_obstacles` | array        | Axis-aligned rectangles: `{ "x", "y", "width", "height" }` (integer cells, inclusive origin).           |
| `dynamic_hazards`  | array        | Hazards applied when `trigger_timestep` is reached in `Environment.update`.                             |


### Dynamic hazard entries

- `**no_fly_zone**`: `id`, `type`, `trigger_timestep`, `position` `{x,y}`, `radius` (cells; filled disk on grid).
- `**obstacle_spawn**`: `id`, `type`, `trigger_timestep`, `position` `{x,y}`, `width`, `height`.

Occupancy grid cell values: `0` free, `1` static/dynamic obstacle, `2` hazard (no-fly treated as hazard value).

## `drone`


| Field                   | Type         | Description                                                                                     |
| ----------------------- | ------------ | ----------------------------------------------------------------------------------------------- |
| `start_position`        | `[int, int]` | `[x, y]`                                                                                        |
| `speed`                 | number       | Mission speed scalar (used by motion / FSM later).                                              |
| `sensor_range`          | int          | Sensing radius in grid cells (Chebyshev/diamond or Euclidean—see `Drone.sense` implementation). |
| `sensor_noise_baseline` | number       | Initial noise level.                                                                            |
| `max_sensor_noise`      | number       | Upper clamp for sensor noise.                                                                   |
| `battery_capacity`      | number       | Starting battery.                                                                               |
| `battery_drain_rate`    | number       | Per-move drain.                                                                                 |


## `mission`


| Field                | Type             | Description                                                                     |
| -------------------- | ---------------- | ------------------------------------------------------------------------------- |
| `waypoints`          | array of `[x,y]` | Ordered targets.                                                                |
| `priority_weights`   | object           | `mission_completion`, `safety`, `energy_conservation` (FSM / decision).         |
| `home_position`      | `[int, int]`     | Return point for abort / safe mode.                                             |
| `waypoint_tolerance` | int              | **Optional.** Chebyshev distance to count as “arrived”; default `1` if omitted. |


## `algorithm`


| Field                          | Type   | Description                    |
| ------------------------------ | ------ | ------------------------------ |
| `primary_planner`              | string | e.g. `d_star_lite`, `a_star`.  |
| `secondary_planner`            | string | Fallback planner id.           |
| `replanning_trigger`           | string | When replanning is considered. |
| `replan_cooldown_steps`        | int    | Minimum steps between replans. |
| `sensor_degradation_threshold` | number | Fault detector warning band.   |
| `critical_fault_threshold`     | number | Fault detector critical band.  |


## `fault_injection`


| Field      | Type  | Description                 |
| ---------- | ----- | --------------------------- |
| `schedule` | array | Events keyed by `timestep`. |


### Schedule entry types

- `**sensor_degradation`**: `severity`, `duration` (steps), optional `description`.
- `**environmental_hazard**`: `hazard_id` referencing a `dynamic_hazards[].id`, optional `description`.

## `visualization`


| Field               | Type   | Description                  |
| ------------------- | ------ | ---------------------------- |
| `fps`               | number | Animation frame rate target. |
| `trail_length`      | int    | Past positions drawn on map. |
| `show_sensor_range` | bool   | Draw sensor disc.            |
| `show_planned_path` | bool   | Draw planner polyline.       |
| `dashboard_enabled` | bool   | Enable full dashboard.       |


