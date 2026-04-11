# Changelog
All notable changes to this project will be documented in this file.

## 0.3.0 - 2026-04-09
### Changed
- Removed the legacy adapter stage from architecture references and unified planning in `ftg_planner_node`.
- Documented heading/speed smoothing behavior in the planner path.
- Updated FTG parameter references for the ~30 cm MXCK vehicle setup.
- Confirmed TF-based recentering with `front_center_deg=0.0`.

## Unreleased
### Added
- Added dual input support via `input_mode`.
- Added configurable `scan_topic`, `obstacles_topic`, and `goal_angle_topic`.
- Reactivated LaserScan callback as a supported runtime path.
- Added launch and config support for scan/obstacles selection.

### Changed
- `scan` is now the recommended runtime mode for MXCK.
- Outputs remain unchanged for compatibility.
