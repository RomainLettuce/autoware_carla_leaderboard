# Autoware per-cycle branch coverage — interface for the scenario runner

This documents a capability on the Autoware side (`~/autoware`, repos `autoware_universe` /
`autoware_core` / `autoware_utils` / `autoware_cmake` / `autoware_launch`) and how
`run_scenario.py` attributes recorded coverage to the right scenario.

**Status**: implemented and wired into `run_scenario.py` (`_set_cycle_coverage_scenario`, called
automatically from `main()` unless `--no_cycle_coverage` is passed). Autoware-side build has been
verified to compile with `-DAUTOWARE_ENABLE_CYCLE_COVERAGE=ON`; end-to-end scenario capture is
still being smoke-tested.

## What it does

`autoware_utils_debug::CycleCoverageRecorder` is hooked into 5 planning nodes
(`behavior_path_planner`, `motion_velocity_planner`, `behavior_velocity_planner` — including the
`experimental` variant actually used by `autoware_launch`, `planning_validator`,
`freespace_planner`). When enabled, each node dumps an isolated gcov branch-coverage snapshot at
the end of every planning cycle (not one cumulative number for the whole process) into:

```
<output_dir>/<scenario_name>/<node_name>/
  cycle_index.csv            # cycle_index, ros_time_sec, wall_clock_iso8601, processing_time_ms
  cycle_000001/gcda/...
  cycle_000002/gcda/...
  ...
```

This only does anything if Autoware was rebuilt with `-DAUTOWARE_ENABLE_CYCLE_COVERAGE=ON`; a
normal build is unaffected. Build command used so far:

```bash
colcon build --packages-select autoware_utils_debug autoware_behavior_path_planner \
  autoware_motion_velocity_planner autoware_behavior_velocity_planner \
  autoware_planning_validator autoware_freespace_planner \
  --cmake-args -DAUTOWARE_ENABLE_CYCLE_COVERAGE=ON -DCMAKE_BUILD_TYPE=Debug
```

## Why this matters for run_scenario.py

Autoware is kept running as one long-lived process while `run_scenario.py` is invoked once per
scenario against it (no relaunch between runs). `scenario_name` is a ROS parameter that can be
changed at runtime without restarting the node — changing it rolls the recorder over to a fresh
output directory and resets that node's cycle counter to 1. `run_scenario.py` pushes a fresh
`cycle_coverage.scenario_name` (and `.enable`/`.output_dir`) to all 5 nodes at the start of every
`main()` call via `_set_cycle_coverage_scenario()`.

## The parameter interface

Each of the 5 nodes exposes these ROS parameters (all settable at runtime, no restart needed):

| Parameter                     | Type   | Notes                                                                                          |
| ------------------------------ | ------ | ------------------------------------------------------------------------------------------------ |
| `cycle_coverage.enable`        | bool   | Master on/off. `run_scenario.py` sets this to `true` on every run (idempotent).                 |
| `cycle_coverage.output_dir`    | string | Root output dir. `run_scenario.py` sets this to `<repo_root>/cycle_coverage_results` every run. |
| `cycle_coverage.scenario_name` | string | Set fresh on every scenario run. Changing it rolls over to a new output directory + resets the cycle counter to 1. |

Re-using the same `scenario_name` later in the same process *appends* to that scenario's existing
files and restarts its cycle counter at 1 — `run_scenario.py` suffixes with `int(time.time())` to
avoid collisions between repeat runs of the same NPC config.

## Node names (confirmed, not guessed)

Traced through the actual `push-ros-namespace` chain in `autoware_launch`
(`planning_simulator.launch.xml` → `autoware.launch.xml` → `tier4_planning_component.launch.xml`
→ `tier4_planning_launch/planning.launch.xml` → `scenario_planning.launch.xml` →
`lane_driving.launch.xml` → `behavior_planning.launch.xml` / `motion_planning.launch.xml` /
`parking.launch.xml`):

```
/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner
/planning/scenario_planning/lane_driving/behavior_planning/behavior_velocity_planner
/planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner
/planning/scenario_planning/parking/freespace_planner
/planning/planning_validator
```

These are hardcoded into `CYCLE_COVERAGE_NODES` in `run_scenario.py`. If this project ever
launches Autoware differently (custom launch file, different `vehicle_model`/`sensor_model`
overrides don't affect this, but a forked/renamed launch tree would), re-verify with
`ros2 node list` and update that list.

## Important: behavior_path_planner / behavior_velocity_planner share a process by default

`behavior_path_planner` and `behavior_velocity_planner` normally run in the same OS process
(`behavior_planning_container`). gcov counters are process-global, so both having
`cycle_coverage.enable=true` at once means each node's `__gcov_dump`/`__gcov_reset` call disturbs
the other's in-flight cycle boundary.

**Fix**: launch Autoware with `cycle_coverage_split_containers:=true`, e.g.:

```bash
ros2 launch autoware_launch planning_simulator.launch.xml \
  vehicle_model:=carla_audi_etron_vehicle sensor_model:=carla_audi_etron_sensor_kit map_path:=<map_path> \
  cycle_coverage_split_containers:=true
```

This puts `behavior_path_planner` in its own dedicated container so all 5 nodes can be measured
simultaneously without interference. Default is `false` (no behavior/topology change) — this is
Autoware's launch-time concern, not something `run_scenario.py` needs to pass; it just needs the
person starting Autoware to add this flag when coverage runs are wanted. The other 3 nodes
(`motion_velocity_planner`, `freespace_planner`, `planning_validator`) already run in their own
processes, no flag needed for those.

Confirmed **not** a functional/behavior change to the planners either way: both nodes already run
with `use_intra_process_comms="false"`, so they always communicate over regular DDS pub/sub
regardless of process boundary; splitting them only adds one more OS process, no change to
message-passing semantics or planning logic.

## Current implementation in run_scenario.py

- `CYCLE_COVERAGE_NODES`: the 5 fully-qualified node names above.
- `CYCLE_COVERAGE_OUTPUT_DIR`: `<repo_root>/cycle_coverage_results`.
- `_ros2_param_set(node, param, value)`: one `ros2 param set` call via `subprocess`, 5s timeout,
  warns (doesn't raise) on failure so a node being down/not coverage-built never blocks a run.
- `_set_cycle_coverage_scenario(scenario_name)`: calls `_ros2_param_set` 3x per node
  (enable/output_dir/scenario_name).
- Called from `main()` right after the config path is resolved, unless `--no_cycle_coverage` is
  passed. `scenario_name` is `f"{rel_pos}_{motion}_{speed}kmh_{timestamp}"` (or
  `f"no_npc_{timestamp}"` for `--no_npc` runs).

## After a batch of scenarios

Each node's `<output_dir>/<scenario_name>/<node_name>/` directory can be turned into a per-cycle
branch-coverage report with a script on the Autoware side:

```bash
pip install gcovr
ros2 run autoware_utils_debug generate_cycle_coverage_report.py \
  --coverage-root ~/autoware_carla_leaderboard/cycle_coverage_results/<scenario_name>/<node_name> \
  --build-dir ~/autoware/build/<package_name> \
  --html
```

producing `scenario_summary.csv` with one row per cycle (branch_percent, branches_covered/total,
timestamps, processing_time_ms) — not this script's concern, just noting where the output goes.
