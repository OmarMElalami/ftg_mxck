# mxck_ftg_bringup

`mxck_ftg_bringup` starts the FTG stack on MXCarkit.

This updated version makes the scan-based FTG path the default runtime path.

## Standard data flow

```text
/scan
 -> mxck_ftg_perception/scan_preprocessor_node
 -> /autonomous/ftg/scan_filtered
 -> follow_the_gap_v0 (input_mode=scan)
 -> /final_heading_angle + /gap_found
 -> mxck_ftg_planner/_ftg_planner_node
 -> /autonomous/ftg/gap_angle + /autonomous/ftg/target_speed
 -> mxck_ftg_control/ftg_command_node
 -> /autonomous/ackermann_cmd
```

Optional parallel diagnostics:

```text
/scan
 -> mxck_ftg_perception/scan_front_window_check
 -> /autonomous/ftg/scan_check
 -> /autonomous/ftg/scan_check_markers
```

## What changed

Old default path:
```text
/scan -> obstacle_substitution -> /obstacles -> follow_the_gap_v0
```

New default path:
```text
/scan -> scan_preprocessor_node -> /autonomous/ftg/scan_filtered -> follow_the_gap_v0
```

The obstacle path is not deleted. It is simply no longer started by the standard bringup.

## Main launch files

### `ftg_full_system.launch.py`
Main launch file for the scan-based FTG stack.

### `ftg_scan_path.launch.py`
A second launch file with the same runtime chain but a shorter name for development use.

## Main launch arguments

- `run_scan_check`
- `scan_topic`
- `filtered_scan_topic`
- `goal_angle_topic`
- `use_vehicle_control`
- `use_tf`
- `record_bag`

The last three are kept for compatibility with the previous bringup command style.

## Launch

Standard:
```bash
ros2 launch mxck_ftg_bringup ftg_full_system.launch.py
```

With diagnostics:
```bash
ros2 launch mxck_ftg_bringup ftg_full_system.launch.py run_scan_check:=true
```

Legacy-compatible argument pattern:
```bash
ros2 launch mxck_ftg_bringup ftg_full_system.launch.py \
  use_vehicle_control:=false \
  use_tf:=false \
  record_bag:=false
```

## Notes

- `use_vehicle_control`, `use_tf`, and `record_bag` are intentionally still present
  so you do not have to change your old launch command style immediately.
- This package does not itself start the raw MXCK platform stack (`run_lidar`,
  `manual_control_launch.py`, etc.). It only starts the FTG-specific path.
