# Copilot instructions for ftg_mxck

This repository contains a ROS 2 autonomous stack for the MXCarkit (MXCK)
platform, focused on Follow-The-Gap (FTG) obstacle avoidance using a 2D LiDAR.

## Platform

- NVIDIA Jetson (Tegra, aarch64), Ubuntu 22.04, Docker containers
- ROS 2 Foxy (mxck2_development) / Humble (mxck2_control)
- RPLidar 2D LiDAR, Ackermann steering via VESC
- Host workspace `/home/mxck/mxck2_ws` is volume-mounted into containers

## Non-negotiable rules

- Final autonomous output: `/autonomous/ackermann_cmd` (`AckermannDriveStamped`).
- Do not bypass the MXCK vehicle-control chain (`vehicle_control` / `ackermann_to_vesc`).
- Do not publish directly to VESC topics.
- `manual_control_launch.py` is started separately (RC, Deadman, safety).
- The FTG stack integrates into the existing MXCK control flow, not replaces it.

## Primary FTG pipeline (single path, no alternatives)

```
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
```

There is exactly one planner node (`ftg_planner_node`). The former
`ctu_ftg_adapter_node` has been merged into it and deleted.

## Package structure

### MXCK packages (maintained here)

| Package | Purpose | Nodes |
|---------|---------|-------|
| `mxck_ftg_perception` | LiDAR preprocessing, TF recentering | `scan_preprocessor_node`, `scan_front_window_check` |
| `mxck_ftg_planner` | Heading + clearance -> gap_angle + speed | `ftg_planner_node` |
| `mxck_ftg_control` | Gap angle + speed -> AckermannDriveStamped | `ftg_command_node` |
| `mxck_ftg_bringup` | Launch files for the full stack | (no nodes) |

### CTU-origin packages (upstream, do not rewrite)

| Package | Purpose | Status |
|---------|---------|--------|
| `follow_the_gap_v0` | C++ FTG algorithm | Active (input_mode=scan) |
| `obstacle_msgs` | Custom message definitions | Kept for compatibility |
| `obstacle_substitution` | LaserScan -> Obstacles converter | Legacy, not used in primary path |

## Architecture constraints

- `mxck_ftg_planner` contains exactly one node: `ftg_planner_node`.
- Do not create additional planner or adapter nodes.
- Do not rewrite the CTU FTG core (`follow_the_gap_v0`) in Python.
- `obstacle_substitution` is not part of the primary pipeline.
  The scan-based path feeds `/autonomous/ftg/scan_filtered` directly to
  `follow_the_gap_v0` with `input_mode=scan`.
- Avoid duplicated control logic (speed scaling, steering limits, planner outputs).

## TF and geometry

- `scan_preprocessor_node` uses TF (`base_link <- laser`) for recentering.
- `front_center_deg: 0.0` means "vehicle forward = 0° in base_link frame".
  TF handles the actual laser mounting rotation. Do not add manual offsets.
- Output scan (`/autonomous/ftg/scan_filtered`) is in `base_link` frame
  with 0 rad = vehicle forward.

## Verification checklist

When modifying any file, verify consistency across all of these together:

- `package.xml` dependencies
- `setup.py` entry points and data_files
- `setup.cfg` script directories
- Launch files (node names, executables, parameter files)
- YAML configs (topic names, parameter names)
- Python/C++ code (declared parameters, publishers, subscribers)
- Topic names and types across all nodes
- TF frame assumptions

## Launch files

| File | Purpose |
|------|---------|
| `ftg_full_system.launch.py` | Recommended. Starts all 4 stages with YAML configs. |
| `ftg_scan_path.launch.py` | Simplified variant, same pipeline. |

Both load YAML configs via `FindPackageShare`. Neither starts TF or
vehicle_control — those are started separately.

## Required behavior for Copilot

- Audit before changing. Do not make broad refactors without justification.
- Keep changes minimal, local, and architecture-preserving.
- Do not invent nodes, topics, or files that do not exist in the codebase.
- Flag uncertainties explicitly instead of guessing.
- Verify that documentation matches actual code, not the other way around.