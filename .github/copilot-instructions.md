# Copilot instructions for ftg_mxck

This repository contains a ROS2-based Follow-The-Gap integration for an MXCarkit platform.

## Platform rules
- The platform is MXCarkit on NVIDIA Jetson, ROS2 in Docker.
- The final autonomous output must be `/autonomous/ackermann_cmd` using `ackermann_msgs/AckermannDriveStamped`.
- Do not bypass the MXCK control chain.
- Do not publish directly to VESC topics from FTG logic.
- `vehicle_control` / `ackermann_to_vesc` remain the existing downstream control path.
- Manual control and safety / deadman remain external to the FTG stack.

## Target pipeline
- `/scan`
- optional scan preprocessor
- `obstacle_substitution`
- `/obstacles`
- `follow_the_gap_v0`
- `/final_heading_angle`
- `/gap_found`
- `ctu_ftg_adapter_node`
- `/autonomous/ftg/gap_angle`
- `/autonomous/ftg/target_speed`
- `ftg_command_node`
- `/autonomous/ackermann_cmd`

## Architectural constraints
- Keep CTU-origin packages separate:
  - `obstacle_msgs`
  - `obstacle_substitution`
  - `follow_the_gap_v0`
- Keep MXCK-origin packages separate:
  - `mxck_ftg_bringup`
  - `mxck_ftg_control`
  - `mxck_ftg_perception`
  - `mxck_ftg_planner`
- Do not rewrite the CTU FTG core to Python.
- `ftg_command_node` is the final MXCK command adapter and should remain the node that publishes `/autonomous/ackermann_cmd`.

## What to verify
- Package structure
- entry points
- ROS2 dependencies in package.xml / setup.py / CMakeLists.txt
- node executable names
- launch file consistency
- topic names, remappings, publishers, subscribers
- whether optional preprocessor logic is actually valid
- whether `vehicle_control` is duplicated anywhere
- whether rosbag topics reflect the final architecture
- whether the FTG stack matches MXCK platform constraints

## Safety and review behavior
- Prefer audit-first.
- If changing code, keep changes minimal and well justified.
- Do not invent files, nodes, or topics that are not present.
- Explicitly flag uncertainties instead of guessing.
- Produce a clear audit report before proposing risky runtime changes.
