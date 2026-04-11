# Changelog
All notable changes to this project will be documented in this file.

## 0.3.0 - 2026-04-09
### Changed
- Removed the legacy adapter stage from architecture references and unified planning in `ftg_planner_node`.
- Documented heading/speed smoothing behavior in the planner path.
- Updated FTG parameter references for the ~30 cm MXCK vehicle setup.
- Confirmed TF-based recentering with `front_center_deg=0.0`.

## 0.2.0 - 2026-04-07
### Added
- Cleaned and documented package structure.
- Added package README and LICENSE installation.
- Added combined launch file with optional diagnostic node.
- Added clearer comments in all Python nodes.

### Changed
- Clarified that `scan_preprocessor_node` is the main runtime node.
- Clarified that `scan_front_window_check` is an optional diagnostic node.
- Kept topic names and core runtime behavior unchanged.
