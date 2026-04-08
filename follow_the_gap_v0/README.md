# follow_the_gap_v0

Legacy CTU Follow-The-Gap (FTG) package for ROS 2.

This package contains the algorithm core for the older CTU FTG implementation.
It now supports two input modes:

```text
input_mode=scan
/autonomous/ftg/scan_filtered -> follow_the_gap_v0
                               -> /final_heading_angle + /gap_found + debug topics
```

```text
input_mode=obstacles
/obstacles -> follow_the_gap_v0
           -> /final_heading_angle + /gap_found + debug topics
```

The recommended mode for MXCK is `scan`, using the already recentered front scan from
`mxck_ftg_perception/scan_preprocessor_node`.

## Parameters

- `input_mode`: `scan` or `obstacles`
- `scan_topic`: default `/autonomous/ftg/scan_filtered`
- `obstacles_topic`: default `/obstacles`
- `goal_angle_topic`: default `/lsr/angle`

## Publications

These outputs are intentionally unchanged for stack compatibility:

- `/final_heading_angle` (`std_msgs/msg/Float32`) [rad]
- `/gap_found` (`std_msgs/msg/Bool`)
- `/visualize_final_heading_angle` (`geometry_msgs/msg/PoseStamped`)
- `/visualize_obstacles` (`visualization_msgs/msg/Marker`)
- `/visualize_largest_gap` (`geometry_msgs/msg/PointStamped`)

## Recommended MXCK launch

```bash
ros2 launch follow_the_gap_v0 follow_the_gap_v0.launch.py \
  input_mode:=scan \
  scan_topic:=/autonomous/ftg/scan_filtered
```

## Legacy compatibility launch

```bash
ros2 launch follow_the_gap_v0 follow_the_gap_v0.launch.py \
  input_mode:=obstacles \
  obstacles_topic:=/obstacles
```

## Build

```bash
cd /mxck2_ws
source /opt/ros/foxy/setup.bash
colcon build --symlink-install --packages-select follow_the_gap_v0
source install/setup.bash
```
