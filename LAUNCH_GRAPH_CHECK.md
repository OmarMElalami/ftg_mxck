# LAUNCH_GRAPH_CHECK

## Primary launch files

- `mxck_ftg_bringup/launch/ftg_full_system.launch.py`
- `mxck_ftg_bringup/launch/ftg_stack.launch.py`

## Verified launch behavior

- [x] `ftg_full_system` includes `ftg_stack`
- [x] `ftg_full_system` does not start `vehicle_control`; states external startup requirement
- [x] Optional TF include is supported
- [x] Optional scan preprocessor include path is supported through conditional node setup
- [x] Obstacle substitution remapping to filtered scan is active when preprocessor is enabled
- [x] Planner stage selection is explicit:
  - `use_ftg_planner=false` → `ctu_ftg_adapter_node`
  - `use_ftg_planner=true`  → `ftg_planner_node`

## Launch interfaces

### `ftg_stack.launch.py` args

- `start_tf`
- `use_scan_preprocessor`
- `start_ctu_ftg`
- `start_adapter`
- `use_ftg_planner`
- `start_control`

### `ftg_full_system.launch.py` args

- `use_tf`
- `use_ftg_stack`
- `use_scan_preprocessor`
- `use_ftg_planner`
- `record_bag`
- `bag_dir`
- `bag_name`
