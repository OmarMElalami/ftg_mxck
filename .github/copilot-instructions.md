# Copilot instructions for ftg_mxck

This repository contains a ROS2-based autonomous stack for the MXCarkit platform, focused on Follow-The-Gap (FTG) obstacle avoidance and integration into the existing MXCK control architecture.

A full working `line_tracking` example stack is also present in this repository as a reference example for architecture, package structure, launch consistency, parameter organization, and `/autonomous/ackermann_cmd` integration.

Use the `line_tracking` stack as an architectural reference only.
Do not merge or mix its code directly into the FTG packages unless explicitly requested.

## Non-negotiable platform rules
- The platform is MXCarkit on NVIDIA Jetson with ROS2 in Docker.
- The final autonomous output must remain `/autonomous/ackermann_cmd` using `ackermann_msgs/AckermannDriveStamped`.
- Do not bypass the MXCK vehicle-control chain.
- Do not publish FTG output directly to VESC topics.
- `vehicle_control` / `ackermann_to_vesc` remain the downstream control path.
- `manual_control_launch.py` is started separately and remains responsible for RC, Deadman, safety, and `ackermann_to_vesc`.
- The FTG stack must integrate into the existing MXCK control flow, not replace it.

## Required FTG target pipeline
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

## Important reference example in this repo
A complete working example stack is available in this repository under the uploaded `line_tracking` example path.
Treat it as a reference for:
- clear linear pipeline design
- one clean intermediate interface between stages
- central parameter file organization
- launch consistency
- package.xml / setup.py / executable consistency
- final publication to `/autonomous/ackermann_cmd`

Do not blindly copy it.
Instead, compare the FTG stack against it and adopt only the useful architecture principles.

## Package boundaries
Keep CTU-origin packages separate:
- `obstacle_msgs`
- `obstacle_substitution`
- `follow_the_gap_v0`

Keep MXCK-origin FTG packages separate:
- `mxck_ftg_bringup`
- `mxck_ftg_control`
- `mxck_ftg_perception`
- `mxck_ftg_planner`

Do not rewrite the CTU FTG core in Python.

## Architecture constraints
- `mxck_ftg_planner` must not become a second unrelated planning system without clear justification.
- If both `ftg_planner_node` and `ctu_ftg_adapter_node` exist, clearly define which path is primary and which is legacy or alternate.
- Avoid duplicated control logic, especially:
  - duplicate speed scaling
  - duplicate steering limiting
  - duplicate planner outputs
  - duplicate startup of vehicle-control components
- The repo must have one clearly documented primary FTG path.

## Critical technical concerns to verify
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
- frame / TF assumptions
- scan angle semantics

## High-priority audit and fix checks
- Whether topic names and remappings match exactly across code, YAML, and launch files
- Whether optional preprocessor logic is actually valid
- Whether the active FTG perception path uses a consistent geometry / angle convention
- Whether `scan_filtered` semantics are clearly defined
- Whether `/obstacles` geometry is consistent with vehicle-forward direction
- Whether TF / `base_link` / laser frame assumptions are handled correctly
- Whether multiple planner paths can run simultaneously by mistake
- Whether `vehicle_control` is started twice anywhere
- Whether rosbag topics reflect the final architecture
- Whether documentation, Docker/devcontainer setup, and runtime assumptions disagree
- Whether safety-related launch or control mistakes could break first vehicle testing

## Required behavior
- Prefer audit-first before making broad code changes.
- If changing code, keep changes minimal, local, justified, and architecture-preserving.
- Do not invent files, nodes, topics, or architecture that are not present unless explicitly requested.
- Explicitly flag uncertainties instead of guessing.
- Preserve compatibility with the MXCK platform and control chain.
- When useful, compare the FTG stack against the in-repo `line_tracking` example and explain the differences clearly.
- Produce professional documentation when restructuring or clarifying the stack.
