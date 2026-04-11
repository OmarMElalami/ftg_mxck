# mxck_ftg_bringup

`mxck_ftg_bringup` starts the MXCK FTG runtime path.

Primary pipeline:

```text
scan_preprocessor_node
  -> follow_the_gap_v0 (scan)
  -> ftg_planner_node
  -> ftg_command_node
```

Notes:
- `obstacle_substitution` is not part of the primary path.
- Vehicle control (`manual_control_launch.py`) is started separately.

## Launch files

### `ftg_full_system.launch.py`
Recommended launch for the full FTG stack with package configs.

```bash
ros2 launch mxck_ftg_bringup ftg_full_system.launch.py
```

### `ftg_scan_path.launch.py`
Alternative launch for the same scan-based FTG pipeline.

```bash
ros2 launch mxck_ftg_bringup ftg_scan_path.launch.py
```
