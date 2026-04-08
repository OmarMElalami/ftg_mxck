# mxck_ftg_perception

`mxck_ftg_perception` is the recommended scan-path frontend for `follow_the_gap_v0`
on MXCarkit.

It has two nodes:

- `scan_preprocessor_node`  
  The runtime node for the FTG scan path.
- `scan_front_window_check`  
  An optional diagnostic node for calibration and debugging.

This package does not produce steering commands. Its job is to transform the
raw LiDAR scan into a front-centered `LaserScan` that is easier and safer for FTG.

## Recommended data flow

```text
/scan
 -> scan_preprocessor_node
 -> /autonomous/ftg/scan_filtered
 -> follow_the_gap_v0 (input_mode=scan)
 -> /final_heading_angle
 -> /gap_found
```

Optional parallel diagnostics:

```text
/scan
 -> scan_front_window_check
 -> /autonomous/ftg/scan_check
 -> /autonomous/ftg/scan_check_markers
```

## Nodes

### `scan_preprocessor_node`

Subscriptions:
- `/scan` (`sensor_msgs/msg/LaserScan`)

Publications:
- `/autonomous/ftg/scan_filtered` (`sensor_msgs/msg/LaserScan`)
- `/autonomous/ftg/front_clearance` (`std_msgs/msg/Float32`)
- `/autonomous/ftg/status` (`std_msgs/msg/String`)

It:
- crops the scan to the front window,
- recenters the window so straight ahead is around `0 rad`,
- clips values to valid ranges,
- optionally smooths the scan,
- publishes minimum front clearance.

### `scan_front_window_check`

Subscriptions:
- `/scan` (`sensor_msgs/msg/LaserScan`)

Publications:
- `/autonomous/ftg/scan_check` (`std_msgs/msg/String`)
- `/autonomous/ftg/scan_check_markers` (`visualization_msgs/msg/MarkerArray`)

It helps validate `front_center_deg` and `front_fov_deg` on the real vehicle.

## Main parameters

From `scan_preprocessor.yaml`:
- `scan_topic`
- `base_frame`
- `filtered_scan_topic`
- `front_clearance_topic`
- `status_topic`
- `front_center_deg`
- `front_fov_deg`
- `clip_min_range_m`
- `clip_max_range_m`
- `enable_moving_average`
- `moving_average_window`

From `scan_front_window_check.yaml`:
- `scan_topic`
- `base_frame`
- `diagnostic_topic`
- `marker_topic`
- `front_center_deg`
- `front_fov_deg`
- `clip_min_range_m`
- `clip_max_range_m`
- `beam_stride`
- `log_every_n_scans`
- `publish_markers`

## Build

```bash
cd /mxck2_ws
source /opt/ros/foxy/setup.bash
colcon build --symlink-install --packages-select mxck_ftg_perception
source install/setup.bash
```

## Launch

Runtime:
```bash
ros2 launch mxck_ftg_perception scan_preprocessor.launch.py
```

Diagnostics:
```bash
ros2 launch mxck_ftg_perception scan_front_window_check.launch.py
```

Combined:
```bash
ros2 launch mxck_ftg_perception stage2_perception.launch.py
```
