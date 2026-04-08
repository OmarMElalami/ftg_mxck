# mxck_ftg_control

`mxck_ftg_control` converts FTG planner topics into
`/autonomous/ackermann_cmd`.

Updated default chain:

```text
/autonomous/ftg/gap_angle
    + /autonomous/ftg/target_speed
 -> ftg_command_node
 -> /autonomous/ackermann_cmd
```

## Node

### `ftg_command_node`

Inputs:
- `/autonomous/ftg/gap_angle` (`std_msgs/msg/Float32`)
- `/autonomous/ftg/target_speed` (`std_msgs/msg/Float32`)

Outputs:
- `/autonomous/ackermann_cmd` (`ackermann_msgs/msg/AckermannDriveStamped`)
- `/autonomous/ftg/control_status` (`std_msgs/msg/String`)

## Safety behavior

- stale planner inputs -> publish zero-speed Ackermann command
- steering command is clipped to configured max steering angle
- speed command is clipped to configured min/max limits
- optional `publish_zero_on_stale` remains enabled by default

## Build

```bash
cd /mxck2_ws
source /opt/ros/foxy/setup.bash
colcon build --symlink-install --packages-select mxck_ftg_control
source install/setup.bash
```

## Launch

```bash
ros2 launch mxck_ftg_control ftg_command.launch.py
```
