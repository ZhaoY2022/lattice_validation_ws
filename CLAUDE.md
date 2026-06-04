# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build and Run

```bash
# Build (from workspace root)
colcon build --symlink-install --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash

# Run scenarios
ros2 launch lattice_standalone_test run_scenario.launch.py scenario:=1    # LCC: lateral recovery
ros2 launch lattice_standalone_test run_scenario.launch.py scenario:=2    # NUDGE: obstacle avoidance
ros2 launch lattice_standalone_test run_scenario.launch.py scenario:=3    # ALC: lane change
ros2 launch lattice_standalone_test run_scenario.launch.py scenario:=5    # 4.2km S-curve highway (interactive LC)
ros2 launch lattice_standalone_test run_scenario.launch.py scenario:=2 persist_history:=true

# Interactive lane change (scenario 5) — use either CLI or the RViz2 Lane Change panel:
ros2 topic pub --once /lattice_test/lane_change_cmd std_msgs/msg/Int8 "data: 1"   # left +3.75m
ros2 topic pub --once /lattice_test/lane_change_cmd std_msgs/msg/Int8 "data: -1"  # right -3.75m
# The RViz2 Lane Change panel loads automatically — look for "Lane Change" in the left dock.
```

**Dependencies**: ROS2 Humble, Eigen3, yaml-cpp (conda version at `/home/zy/miniconda3`, not system — ABI incompatibility).

## Architecture

Two ROS2 packages in a colcon workspace:

- **`pnc_msg`** — Custom ROS2 messages: `TrajectoryPoint` (13 fields including curvature/pinch/jerk) and `TrajPoints` (header + flags + 40-point array). Control interface between planner and downstream modules.
- **`lattice_standalone_test`** — Core planner + simulation harness. Three CMake targets:
  - `lattice_core` (STATIC lib) — All Lattice algorithm code, no ROS deps
  - `scenario_manager` (STATIC lib) — Shared scenario config (reference line, obstacles, vehicle state) for planner and simulator nodes
  - `lattice_test_node` (executable, 10 Hz) — Planning node: reads vehicle pose from TF, runs `LatticePlan()`, publishes visualization markers and `/lattice_trajectory`
  - `lattice_simulator_node` (executable, 100 Hz) — Simulation node: subscribes to `/lattice_trajectory`, integrates motion with 30/70 blend (dead-reckoning / trajectory lookup), broadcasts `map→base_footprint` TF
  - `lane_change_panel` (SHARED lib) — RViz2 native panel plugin with Left/Right buttons, publishes to `/lattice_test/lane_change_cmd`

Launch file starts 5 processes: `lattice_test_node` + `lattice_simulator_node` + `robot_state_publisher` + `joint_state_publisher` + `rviz2`.

## Algorithm Pipeline (per 100ms cycle)

1. **Reference line discretization** → `PathPoint` vector
2. **Match**: `PathMatcher::MatchToPath()` finds ego projection on reference line
3. **Cartesian→Frenet**: Get `init_s = [s, s', s'']`, `init_d = [d, d', d'']`
4. **ST graph**: `PathTimeGraph` projects obstacles onto (s,t) plane for follow/overtake sampling
5. **1D trajectory generation** (Frenet-decoupled):
   - Longitudinal s(t): quartic (cruise) or quintic (follow/stop) polynomials
   - Lateral d(s): quintic polynomials
6. **2D combining**: `TrajectoryCombiner` pairs lon×lat, converts Frenet→Cartesian via reference line
7. **Evaluation**: `TrajectoryEvaluator` scores by priority queue (cost-weighted sum of lon objective, jerk, collision risk, centripetal accel, lateral offset, lateral comfort)
8. **Validation**: `ConstraintChecker` (kinematics) + `CollisionChecker` (AABB with buffer), first valid trajectory wins
9. **State update**: Vehicle "teleports" to `trajectory[1]` (100ms lookahead)

## Key Design Points

- **All parameters in `config/configs.yaml`** — no recompilation needed. Modifying YAML + relaunching is sufficient.
- **Driving modes** defined in `PlanningTarget.h`: `LCC` (lane centering), `ALC` (auto lane change), `NUDGE` (obstacle avoidance). Mode controls lateral sampling distribution AND evaluation weights independently.
- **ALC is target-lat-offset driven**: Reference line is always the current lane centerline; the planner generates lane-change paths via `target_lat_offset` parameter flowing through the entire pipeline (PlanningTarget → LatticePlan → Trajectory1dGenerator → EndConditionSampler → TrajectoryEvaluator).
- **ALC completion state machine** (in `lattice_test_node.cpp` `timer_callback()`): When `driving_mode == ALC`, checks if `|d0 - target_d| < 0.20m` AND `|heading_error| < 0.03 rad` for 5 consecutive frames (debounce). On completion: (1) shifts entire `reference_line_` coordinates by normal direction (nx,ny) to move reference line to target lane, (2) calls `PathMatcher::ComputePathProfile()` to recompute headings/accumulated_s/kappas/dkappas from shifted XY — critical for geometric consistency on curved roads, (3) republishes road boundaries at shifted position, (4) sets `driving_mode = LCC` and `target_lat_offset_ = 0.0`. Reference path and reference line marker are published every frame (10 Hz VOLATILE) and update automatically on the next cycle. Two guards prevent re-triggering: `alc_completed_flag_` blocks both the `target_lat_offset` assignment AND the `driving_mode = ALC` assignment on subsequent frames, so the vehicle remains in LCC mode centered on the target lane.
- **NUDGE post-obstacle return**: Cartesian→Frenet conversion automatically computes actual lateral deviation; switching back to LCC mode lets its strong centering weights (`w_lat_offset=8.0, bound=0.3`) naturally pull the vehicle back — no manual state zeroing.
- **Config path via env var**: `Config.cpp` reads `$LATTICE_CONFIG_PATH` set by the launch file.
- **yaml-cpp must use conda version** at `/home/zy/miniconda3` — system version has old ABI. CMakeLists.txt hardcodes this with `PATHS /home/zy/miniconda3 NO_DEFAULT_PATH`.
- **Trajectory truncation**: Planner generates 8.0s trajectories internally (for smooth polynomial convergence), but only the first 4.0s are consumed per cycle for tighter control.
- **Reference line must be ≥500m**: At 22 m/s, quartic speed profiles can reach s_end ≈ 240m in 8s. Shorter reference lines cause trajectory truncation in `trajectory_combiner.cpp` (`s > s_ref_max` check), freezing the vehicle.
- **Trajectory time uses absolute planner time**: `out.time = sim_time_ + pt.relative_time` in `publish_lattice_trajectory()`. The simulator uses wall-clock elapsed + `traj_t0_` for target time lookup — both advance at 1.0 sim-s / wall-s, keeping planner and simulator synchronized.
- **Simulator is 100% trajectory lookup**: No more 30/70 dead-reckoning/trajectory blend. Vehicle position comes entirely from interpolating the latest trajectory at `target_time`. Freezes before first trajectory arrives. Rejects stale DDS messages (trajectory start position >50m from current tracking position, checked on EVERY message — not just the first).
- **Static RViz2 markers published ONCE**: Road boundaries and obstacle markers use `frame_locked=true`, `lifetime=Duration::max()`, and are never re-published except at ALC completion (road boundaries shift with reference line). Each MarkerArray topic sends per-namespace DELETEALL on first publish to clear stale DDS data from previous runs. **Reference line and reference path are published every frame** (VOLATILE, 10 Hz, 0.5s lifetime with rotating marker IDs) — this matches the green optimal trajectory pattern and ensures RViz2 always renders the current reference line position after ALC-triggered shifts.
- **DDS: CycloneDDS** — 默认使用 CycloneDDS（`rmw_cyclonedds_cpp`），无共享内存残留，大消息（MarkerArray）吞吐更优。Launch 文件自动设置 `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`。Fast-DDS 备用方案（UDPv4-only）已配置于 `config/fastdds_no_shm.xml`，切换方法见 launch 文件注释。
- **Pre-launch cleanup** (`scripts/cleanup_stale.sh`): Each launch automatically kills leftover lattice/robot_state_publisher/joint_state_publisher processes from previous runs and removes Fast-DDS shared-memory files. The cleanup runs immediately; all Node/TimerAction actions are delayed 0.5s so they start after cleanup completes. This prevents zombie processes from hijacking the simulator with stale trajectory messages.
- **Green trajectory starts from `timestep_`**: Both the Path display and LINE_STRIP marker evaluate `opt_traj.Evaluate(t)` from `t=timestep_` (0.1s), not `t=0`, so the green line starts at the vehicle's current position rather than behind it.
- **Interactive lane change**: Scenarios that support lane changes (currently scenario 5) subscribe to `/lattice_test/lane_change_cmd` (`std_msgs/Int8`, values: 1=left+3.75m, -1=right-3.75m). The callback sets `driving_mode_=ALC` and `target_lat_offset_` directly — `driving_mode_` and `target_lat_offset_` are now member variables (not locals) so they persist between timer cycles and across subscription callbacks. Commands are rejected if ALC is already in progress.
- **ALC completion uses normal-direction shift**: Replaced `pt.set_y(pt.get_y() + target_d)` with `nx=-sin(heading), ny=cos(heading)` — the reference line shift is perpendicular to the road direction, working on both straight and curved roads. Backward-compatible: for straight lines (heading=0), `nx=0, ny=1` yields same result as before.
- **Scenario 5: 4.2km S-curve highway**: Reference line built by numerical integration from a piecewise-linear curvature profile (3 bends: 500m left, 700m right, 1000m left radius) with clothoid transitions. Vehicle starts at origin heading=0, LCC mode. Road boundaries rendered using normal-direction offsets so lane lines follow curves correctly. Supports interactive lane change via topic command.
- **Road boundary rendering uses normal-direction offsets**: Instead of `rp.y_ + y_offset`, uses `nx=-sin(heading), ny=cos(heading)` so the white/yellow lane lines stay at the correct lateral offset on curved roads. Both `make_line` and `make_dashed_line` lambdas updated.

## Where to Find Things

| What | Where |
|------|-------|
| Algorithm entry point | `src/Lattice_planner/lattice_planner.cpp` → `LatticePlan()` |
| Lateral sampling | `src/Lattice_planner/end_condition_sampler.cpp` → `SampleLatEndConditions()` |
| Cost evaluation | `src/Lattice_planner/trajectory_evaluator.cpp` → `Evaluate()`, `LatOffsetCost()` |
| Collision checking | `src/Lattice_planner/collision_checker.cpp` → `InCollision()` |
| Cartesian↔Frenet | `src/Frenet/cartesian_frenet_conversion.cpp` |
| Scenario definitions | `src/scenario_manager.cpp` → `init_scenario_1/2/3/5()` |
| Scenario struct | `src/scenario_manager.h` → `ScenarioConfig` |
| Simulator node | `src/lattice_simulator_node.cpp` — subscribes `/lattice_trajectory`, publishes `map→base_footprint` TF |
| Pre-launch cleanup | `scripts/cleanup_stale.sh` — kills zombie processes + cleans DDS SHM |
| Driving mode enum | `include/Lattice_planner/PlanningTarget.h` |
| Trajectory interpolation fix | `src/common/math_utils.cpp` → `InterpolateUsingLinearApproximation()` — must interpolate both PathPoint fields AND direct TrajectoryPoint fields |
| CSV logs | `/tmp/lattice_log_*.csv` (auto-created each run) |
