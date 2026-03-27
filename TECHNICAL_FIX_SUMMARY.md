# TECHNICAL_FIX_SUMMARY

## What was fixed

1. **CTU ROS callback publish behavior fixed in `follow_the_gap_v0/src/main.cpp`**
   - In both callbacks:
     - `Callback(sensor_msgs::msg::LaserScan::ConstSharedPtr lidar_data)`
     - `ObstaclesCallback(obstacle_msgs::msg::ObstaclesStamped::ConstSharedPtr obstacles_data)`
   - `/gap_found` is published every callback cycle via `PublishGapFound(ok);`.
   - `PublishVisualizeLargestGap(...)` is guarded by `gap_borders_out.size() >= 2` to avoid invalid indexing.

2. **Header-global linkage requirement verified in `follow_the_gap_v0/src/follow_the_gap.h`**
   - Mutable globals are declared as `extern`:
     - `AngleFilter::left_index`
     - `AngleFilter::right_index`
     - `kMaxRange`
     - `g_fovAngleMax`
     - `g_goal_angle`
     - `obstacle_nhol_left`
     - `obstacle_nhol_right`

3. **Single definitions requirement verified in `follow_the_gap_v0/src/follow_the_gap.cpp`**
   - Matching single definitions exist exactly once in the `.cpp` for all listed mutable globals.

## Why it mattered

- Publishing `/gap_found` every callback cycle gives deterministic planner-state signaling for downstream nodes.
- Guarding `PublishVisualizeLargestGap(...)` prevents out-of-range access when fewer than two gap-border obstacles are available.
- Using `extern` declarations in the header with single `.cpp` definitions avoids mutable global duplication/linkage hazards and keeps ROS parameter/callback state consistent.

## Architecture consistency re-check (static)

- Final autonomous output remains **`/autonomous/ackermann_cmd`** and is published by `ftg_command_node`.
- No FTG package was changed to publish directly to VESC command topics.
- `ftg_full_system.launch.py` still states `vehicle_control` must be started separately via `manual_control_launch.py` (no duplicate startup introduced).
- CTU core remains C++ (`follow_the_gap_v0`); no Python reimplementation was introduced.
- Package boundaries remain unchanged (planner remains adapter path, control remains final command adapter).

## Validation performed in sandbox

Because this sandbox does not provide a full ROS2 toolchain/runtime, full ROS2 build/test/launch validation was not possible here.

Performed checks that were available:
- `package.xml` syntax check via Python XML parser (`xml.etree.ElementTree`) for all packages.
- Python syntax checks (`python3 -m py_compile`) for relevant launch and FTG Python nodes.
- Static code/topic checks for:
  - `/autonomous/ackermann_cmd` publisher location
  - no direct FTG publishing to VESC control topics
  - no duplicate `vehicle_control` startup in FTG bringup launch files

## Still to validate on real MXCK container / vehicle

1. Full ROS2 build in target container (`colcon build`) with all dependencies sourced.
2. End-to-end launch and topic flow on target:
   - `/scan` -> optional preprocessor -> `/obstacles` -> `follow_the_gap` -> `/final_heading_angle` + `/gap_found` -> adapter -> `/autonomous/ftg/gap_angle` + `/autonomous/ftg/target_speed` -> `ftg_command_node` -> `/autonomous/ackermann_cmd`
3. Runtime behavior under real LiDAR data:
   - `/gap_found` cadence and correctness
   - no visualization crashes when gap borders are incomplete
4. Safety/vehicle-control integration with existing RC/Deadman path in `manual_control_launch.py`.
