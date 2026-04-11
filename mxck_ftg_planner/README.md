# mxck_ftg_planner

`mxck_ftg_planner` contains exactly one planner node: `ftg_planner_node`.

It converts FTG heading output + clearance into:
- `/autonomous/ftg/gap_angle`
- `/autonomous/ftg/target_speed`
- `/autonomous/ftg/planner_status`

Primary pipeline position:

```text
/final_heading_angle + /gap_found + /autonomous/ftg/front_clearance
  -> ftg_planner_node
  -> /autonomous/ftg/gap_angle + /autonomous/ftg/target_speed
```

> The legacy adapter node is removed; planning is unified in `ftg_planner_node`.

## Node: ftg_planner_node

### Subscribes
- `/final_heading_angle` (`std_msgs/msg/Float32`)
- `/gap_found` (`std_msgs/msg/Bool`)
- `/autonomous/ftg/front_clearance` (`std_msgs/msg/Float32`)

### Publishes
- `/autonomous/ftg/gap_angle` (`std_msgs/msg/Float32`)
- `/autonomous/ftg/target_speed` (`std_msgs/msg/Float32`)
- `/autonomous/ftg/planner_status` (`std_msgs/msg/String`)

### Current key parameters (`config/ftg_planner.yaml`)
- `input_timeout_sec: 0.50`
- `max_abs_gap_angle_rad: 0.45`
- `cruise_speed_mps: 0.45`
- `min_speed_mps: 0.20`
- `stop_clearance_m: 0.25`
- `caution_clearance_m: 0.50`
- `steering_slowdown_start_rad: 0.40`
- `steering_slowdown_full_rad: 0.70`
- `heading_smoothing_alpha: 0.4`
- `speed_smoothing_alpha: 0.3`

## Launch

```bash
ros2 launch mxck_ftg_planner ftg_planner.launch.py
```
