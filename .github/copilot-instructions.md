# Copilot instructions for ftg_mxck

This repository contains a ROS 2 autonomous stack for the MXCarkit (MXCK)
platform, focused on Follow-The-Gap (FTG) obstacle avoidance using a 2D LiDAR.

## Platform

- NVIDIA Jetson (Tegra, aarch64), Ubuntu 22.04, Docker containers
- ROS 2 Foxy (mxck2_development) / Humble (mxck2_control)
- RPLidar 2D LiDAR, Ackermann steering via VESC
- Vehicle width: ~30 cm
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
     TF-based recentering, FOV ±60° (120°), clipping 0.25–10.0 m,
     moving average smoothing
  -> /autonomous/ftg/scan_filtered (LaserScan, frame: base_link)
  -> /autonomous/ftg/front_clearance (Float32)

/autonomous/ftg/scan_filtered
  -> follow_the_gap_v0             [follow_the_gap_v0, C++, input_mode=scan]
  -> /final_heading_angle (Float32)
  -> /gap_found (Bool)
  -> /visualize_obstacles (Marker)
  -> /visualize_largest_gap (PointStamped)
  -> /visualize_final_heading_angle (PoseStamped)

/final_heading_angle + /gap_found + /autonomous/ftg/front_clearance
  -> ftg_planner_node              [mxck_ftg_planner]
     Speed policy: clearance + steering -> speed
     Heading smoothing (alpha=0.4) + speed smoothing (alpha=0.3)
  -> /autonomous/ftg/gap_angle (Float32)
  -> /autonomous/ftg/target_speed (Float32)
  -> /autonomous/ftg/planner_status (String)

/autonomous/ftg/gap_angle + /autonomous/ftg/target_speed
  -> ftg_command_node              [mxck_ftg_control]
     angle_to_steering_gain: -1.0 (hardware-inverted)
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

## Current parameter values (ground truth)

### scan_preprocessor.yaml
- `front_center_deg: 0.0`
- `front_fov_deg: 120.0`
- `clip_min_range_m: 0.25`
- `clip_max_range_m: 10.0`
- `enable_moving_average: true`
- `moving_average_window: 3`

### ftg_planner.yaml
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

### ftg_control.yaml
- `angle_to_steering_gain: -1.00` (hardware-inverted)
- `max_steering_angle_rad: 0.45`
- `min_speed_mps: 0.0`
- `max_speed_mps: 1.80`
- `input_timeout_sec: 0.50`
- `target_speed=0.0` is an explicit stop command and must not be clamped up to `min_speed_mps`.

### follow_the_gap_v0 internal constants (C++, not YAML)
- `kCarRadius = 0.4` (should be ~0.20 for 30cm vehicle)
- `kTurnRadius = 0.3`
- `kTrackMinWidth = 0.35`
- `kDistanceToCorner = 0.22`

## Architecture constraints

- `mxck_ftg_planner` contains exactly one node: `ftg_planner_node`.
- Do not create additional planner or adapter nodes.
- Do not rewrite the CTU FTG core (`follow_the_gap_v0`) in Python.
- `obstacle_substitution` is not part of the primary pipeline.
- Avoid duplicated control logic (speed scaling, steering limits, planner outputs).
- `steering_slowdown_full_rad` must always be > `max_abs_gap_angle_rad`,
  otherwise the vehicle stops when it needs to steer most.

## TF and geometry

- `scan_preprocessor_node` uses TF (`base_link <- laser`) for recentering.
- TF shows yaw ≈ -135° for the MXCK laser mounting.
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
- YAML configs (topic names, parameter names, actual values)
- Python/C++ code (declared parameters, publishers, subscribers)
- Topic names and types across all nodes
- TF frame assumptions
- Per-package README.md and CHANGELOG.md accuracy
- Root README.md section 5 (Konfiguration) parameter values

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
- When reviewing any README.md or CHANGELOG.md, compare against the parameter
  values listed in this file. Flag any mismatch.

## LaTeX-Dokumentation (docs/)

The `docs/Projektarbeit_MXCK_FTG/` directory contains a LaTeX project
documenting this FTG stack as a university project report.

### Rules for reviewing/editing the documentation

- The documentation must reflect the **current** code architecture, not any
  previous version.
- The current primary pipeline is:
  `scan_preprocessor_node -> follow_the_gap_v0(scan) -> ftg_planner_node -> ftg_command_node`
- There is NO `ctu_ftg_adapter_node` — it was merged into `ftg_planner_node`.
- There is NO `obstacle_substitution` in the primary path — the scan-based
  path feeds `/autonomous/ftg/scan_filtered` directly to `follow_the_gap_v0`.
- `front_center_deg` is `0.0` (TF-based), not `135.0`.
- `ftg_planner_node` includes heading smoothing and speed smoothing.
- When reviewing `.tex` files, flag any reference to deleted nodes, old
  pipeline steps, or outdated parameter values.
- The generated PDF is at `docs/pdf/dokumentation.pdf` and is auto-built
  by GitHub Actions when `.tex` files change on `main`.
