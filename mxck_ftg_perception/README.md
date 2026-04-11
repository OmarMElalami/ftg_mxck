# mxck_ftg_perception

`mxck_ftg_perception` provides the LiDAR preprocessing stage for the MXCK FTG stack.

Primary pipeline position:

```text
/scan
  -> scan_preprocessor_node
  -> /autonomous/ftg/scan_filtered (frame: base_link)
  -> follow_the_gap_v0 (input_mode=scan)
```

This package contains two nodes:

- `scan_preprocessor_node` (runtime node)
- `scan_front_window_check` (diagnostic node)

## Node: scan_preprocessor_node

### Subscribes
- `/scan` (`sensor_msgs/msg/LaserScan`)

### Publishes
- `/autonomous/ftg/scan_filtered` (`sensor_msgs/msg/LaserScan`)
- `/autonomous/ftg/front_clearance` (`std_msgs/msg/Float32`)
- `/autonomous/ftg/status` (`std_msgs/msg/String`)

### Runtime behavior
- TF-based recentering (`base_link <- laser`)
- front scan window extraction
- range clipping
- optional moving-average smoothing
- front clearance output

### Current key parameters (`config/scan_preprocessor.yaml`)
- `front_center_deg: 0.0`
- `front_fov_deg: 120.0`
- `clip_min_range_m: 0.25`
- `clip_max_range_m: 10.0`
- `enable_moving_average: true`
- `moving_average_window: 3`

## Node: scan_front_window_check

### Subscribes
- `/scan` (`sensor_msgs/msg/LaserScan`)

### Publishes
- `/autonomous/ftg/scan_check` (`std_msgs/msg/String`)
- `/autonomous/ftg/scan_check_markers` (`visualization_msgs/msg/MarkerArray`)

Used for validating front-window and TF behavior during calibration/debugging.

## Launch

Runtime:
```bash
ros2 launch mxck_ftg_perception scan_preprocessor.launch.py
```

Diagnostic:
```bash
ros2 launch mxck_ftg_perception scan_front_window_check.launch.py
```
