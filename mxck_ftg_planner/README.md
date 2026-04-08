# mxck_ftg_planner

Unified planner node for the MXCK scan-based FTG path.
Consumes `follow_the_gap_v0` outputs and front clearance,
publishes gap angle and target speed for `ftg_command_node`.

## Pipeline position

```text
/final_heading_angle  ──┐
/gap_found            ──┼──▶  ftg_planner_node  ──▶  /autonomous/ftg/gap_angle
/autonomous/ftg/      ──┘                        ──▶  /autonomous/ftg/target_speed
  front_clearance                                ──▶  /autonomous/ftg/planner_status
```

## Node: `ftg_planner_node`

**Inputs:**
- `/final_heading_angle` (`std_msgs/Float32`) — from `follow_the_gap_v0`
- `/gap_found` (`std_msgs/Bool`) — from `follow_the_gap_v0`
- `/autonomous/ftg/front_clearance` (`std_msgs/Float32`) — from `scan_preprocessor_node`

**Outputs:**
- `/autonomous/ftg/gap_angle` (`std_msgs/Float32`)
- `/autonomous/ftg/target_speed` (`std_msgs/Float32`)
- `/autonomous/ftg/planner_status` (`std_msgs/String`)

## Speed policy

| Condition | Result |
|-----------|--------|
| Any input stale (> 0.5 s) | speed = 0, angle = 0 |
| `gap_found` = false | speed = 0, angle = 0 |
| Clearance ≤ `stop_clearance_m` | speed = 0 |
| Clearance between stop and caution | speed scaled linearly |
| Steering angle > `steering_slowdown_start_rad` | speed reduced |
| Clear corridor, small steering | `cruise_speed_mps` |

Speed = `cruise_speed × min(clearance_factor, steering_factor)`,
clamped to `[min_speed_mps, cruise_speed_mps]`.

## Build

```bash
colcon build --symlink-install --packages-select mxck_ftg_planner
source install/setup.bash
```

## Launch

```bash
ros2 launch mxck_ftg_planner ftg_planner.launch.py
```