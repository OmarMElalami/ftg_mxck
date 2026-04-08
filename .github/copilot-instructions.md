# Copilot instructions for ftg_mxck

This repository contains a ROS 2 autonomous stack for the MXCarkit (MXCK)
platform, focused on Follow-The-Gap (FTG) obstacle avoidance using a 2D LiDAR.

This repository now also contains project documentation under `docs/`.
When revising documentation, Copilot must treat the **actual repository code and
root README** as the source of truth.

---

## Source of truth priority

When there is any mismatch, use this priority order:

1. Actual code in this repository
2. Launch files and YAML configs in this repository
3. Root `README.md` in this repository
4. Documentation files under `docs/`
5. Older text, diagrams, notes, or imported documentation content

**Do not update code just to match stale documentation.**
Update the documentation to match the current codebase.

---

## Platform

- NVIDIA Jetson (Tegra, aarch64), Ubuntu 22.04, Docker containers
- ROS 2 Foxy (`mxck2_development`) / Humble (`mxck2_control`)
- RPLidar 2D LiDAR, Ackermann steering via VESC
- Host workspace `/home/mxck/mxck2_ws` is volume-mounted into containers

When revising docs, do **not** claim Ubuntu 18.04 unless the text explicitly says
it refers to a historical setup and clearly labels it as such.

---

## Non-negotiable runtime rules

- Final autonomous output: `/autonomous/ackermann_cmd` (`AckermannDriveStamped`)
- Do not bypass the MXCK vehicle-control chain (`vehicle_control` / `ackermann_to_vesc`)
- Do not publish directly to VESC topics
- `manual_control_launch.py` is started separately (RC, Deadman, safety)
- The FTG stack integrates into the existing MXCK control flow; it does not replace it

---

## Primary FTG pipeline (current official path)

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
