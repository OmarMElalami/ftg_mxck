# mxck_ftg_control

`mxck_ftg_control` provides `ftg_command_node` and publishes the final autonomous command:

- `/autonomous/ackermann_cmd` (`ackermann_msgs/msg/AckermannDriveStamped`)

Primary pipeline position:

```text
/autonomous/ftg/gap_angle + /autonomous/ftg/target_speed
  -> ftg_command_node
  -> /autonomous/ackermann_cmd
```

## Node: ftg_command_node

### Subscribes
- `/autonomous/ftg/gap_angle` (`std_msgs/msg/Float32`)
- `/autonomous/ftg/target_speed` (`std_msgs/msg/Float32`)

### Publishes
- `/autonomous/ackermann_cmd` (`ackermann_msgs/msg/AckermannDriveStamped`)
- `/autonomous/ftg/control_status` (`std_msgs/msg/String`)

### Current key parameters (`config/ftg_control.yaml`)
- `angle_to_steering_gain: -1.0` (hardware-inverted)
- `max_steering_angle_rad: 0.45`
- `min_speed_mps: 0.18`
- `max_speed_mps: 1.80`
- `input_timeout_sec: 0.50`

## Launch

```bash
ros2 launch mxck_ftg_control ftg_command.launch.py
```
