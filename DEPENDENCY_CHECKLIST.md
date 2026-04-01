# DEPENDENCY_CHECKLIST

## Package manifests and setup consistency (static)

- [x] `mxck_ftg_bringup`: package.xml + setup.py consistent for launch installation
- [x] `mxck_ftg_control`: package.xml + setup.py include `ftg_command_node`
- [x] `mxck_ftg_perception`: package.xml + setup.py include scan nodes
- [x] `mxck_ftg_planner`: package.xml + setup.py include `ctu_ftg_adapter_node` and `ftg_planner_node`
- [x] `obstacle_substitution`: package.xml + setup.py include obstacle node
- [x] `follow_the_gap_v0`: package.xml + CMake target `follow_the_gap` present
- [x] `obstacle_msgs`: message package manifest present

## Static validation run in sandbox

- [x] XML parsing check passed for all `package.xml`
- [x] Python syntax check passed for launch and FTG Python files
- [ ] Full `colcon build` (not available in sandbox: `colcon: command not found`)
