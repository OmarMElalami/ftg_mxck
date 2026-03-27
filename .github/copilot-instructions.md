# Copilot instructions for ftg_mxck

This repository implements a ROS2-based Follow-The-Gap integration for the MXCarkit platform.

## Non-negotiable platform rules
- The platform is MXCarkit on NVIDIA Jetson with ROS2 in Docker.
- The final autonomous output must remain `/autonomous/ackermann_cmd` using `ackermann_msgs/AckermannDriveStamped`.
- Do not bypass the MXCK vehicle-control chain.
- Do not publish FTG output directly to VESC topics.
- `vehicle_control` / `ackermann_to_vesc` remain the downstream control path.
- `manual_control_launch.py` is started separately and remains responsible for RC, Deadman, safety, and `ackermann_to_vesc`.
- The FTG stack must integrate into the existing MXCK control flow, not replace it.

## Required target pipeline
`/scan`
-> `[optional] scan_preprocessor_node`
-> `obstacle_substitution`
-> `/obstacles`
-> `follow_the_gap_v0`
-> `/final_heading_angle` + `/gap_found`
-> `ctu_ftg_adapter_node`
-> `/autonomous/ftg/gap_angle` + `/autonomous/ftg/target_speed` + `/autonomous/ftg/planner_status`
-> `ftg_command_node`
-> `/autonomous/ackermann_cmd`

## Package boundaries
- Keep CTU-origin packages separate:
  - `obstacle_msgs`
  - `obstacle_substitution`
  - `follow_the_gap_v0`
- Keep MXCK-origin packages separate:
  - `mxck_ftg_bringup`
  - `mxck_ftg_control`
  - `mxck_ftg_perception`
  - `mxck_ftg_planner`

## Architecture constraints
- Do not rewrite the CTU FTG core in Python.
- `mxck_ftg_planner` should only contain the CTU adapter node, not a Python reimplementation of FTG.
- `ftg_command_node.py` is the final MXCK command adapter and should remain the node that publishes `/autonomous/ackermann_cmd`.
- Prefer minimal, architecture-preserving fixes.
- Avoid duplicated control logic, especially duplicated speed scaling or duplicated startup of vehicle-control components.

## What to verify together
Always verify these together, not in isolation:
- `package.xml`
- `setup.py`
- `CMakeLists.txt`
- launch files
- YAML configs
- executable names
- topic names
- remappings
- publishers/subscribers
- package dependencies
- runtime assumptions

## High-priority audit checks
- Whether topic names and remappings match exactly across code, YAML, and launch files
- Whether optional preprocessor logic is actually valid
- Whether `vehicle_control` is started twice anywhere
- Whether rosbag topics reflect the final architecture
- Whether documentation, Docker/devcontainer setup, and runtime assumptions disagree
- Whether safety-related launch or control mistakes could break first vehicle testing

## Review behavior
- Prefer audit-first before making broad code changes.
- If changing code, keep changes minimal, local, and justified.
- Do not invent files, nodes, topics, or architecture that are not present.
- Explicitly flag uncertainties instead of guessing.
- Preserve compatibility with the MXCK platform and control chain.
