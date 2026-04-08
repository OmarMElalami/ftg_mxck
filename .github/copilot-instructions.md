# Copilot instructions for ftg_mxck

This repository contains a ROS 2 autonomous stack for the MXCarkit (MXCK)
platform, focused on Follow-The-Gap (FTG) obstacle avoidance using a 2D LiDAR.

## Platform

- NVIDIA Jetson (Tegra, aarch64), Docker-based workflow
- ROS 2 Foxy in `mxck2_development`
- The repository documentation may mention Humble in other MXCK containers, but
  this FTG stack must be documented against the code and launch files that are
  actually present in this repository
- RPLidar 2D LiDAR
- Ackermann steering via VESC
- Host workspace `/home/mxck/mxck2_ws` is volume-mounted into containers

## Non-negotiable rules

- Final autonomous output: `/autonomous/ackermann_cmd` (`AckermannDriveStamped`)
- Do not bypass the MXCK vehicle-control chain
- Do not publish directly to VESC topics
- `manual_control_launch.py` is started separately
- TF and `vehicle_control` are external to this FTG stack unless explicitly added
- Documentation must match the current code and launch files in this repository

## Canonical architecture source order

When there is any conflict, treat the following as the authority in this order:

1. actual package code
2. launch files
3. YAML configs
4. root `README.md`
5. documentation under `docs/`

Do not preserve outdated descriptions just because they existed in an older
document or upstream repo.

## Primary FTG pipeline (single current path)

```text
/scan (LaserScan, frame: laser)
  -> scan_preprocessor_node        [mxck_ftg_perception]
     TF-based recentering, FOV filter, clipping
  -> /autonomous/ftg/scan_filtered (LaserScan, frame: base_link)
  -> /autonomous/ftg/front_clearance (Float32)

/autonomous/ftg/scan_filtered
  -> follow_the_gap_v0             [follow_the_gap_v0, C++, input_mode=scan]
  -> /final_heading_angle (Float32)
  -> /gap_found (Bool)

/final_heading_angle + /gap_found + /autonomous/ftg/front_clearance
  -> ftg_planner_node              [mxck_ftg_planner]
     Speed policy: clearance + steering -> speed
  -> /autonomous/ftg/gap_angle (Float32)
  -> /autonomous/ftg/target_speed (Float32)
  -> /autonomous/ftg/planner_status (String)

/autonomous/ftg/gap_angle + /autonomous/ftg/target_speed
  -> ftg_command_node              [mxck_ftg_control]
  -> /autonomous/ackermann_cmd (AckermannDriveStamped)
