# Changelog
All notable changes to this project will be documented in this file.

## 0.3.0 - 2026-04-09
### Changed
- Clarified package-specific documentation to match the current MXCK FTG stack description.
- Limited architecture notes in this package to `follow_the_gap_v0`-relevant references.
- No runtime behavior changes were made in `follow_the_gap_v0` as part of this release.

## Unreleased
### Added
- Added dual input support via `input_mode`.
- Added configurable `scan_topic`, `obstacles_topic`, and `goal_angle_topic`.
- Reactivated LaserScan callback as a supported runtime path.
- Added launch and config support for scan/obstacles selection.

### Changed
- `scan` is now the recommended runtime mode for MXCK.
- Outputs remain unchanged for compatibility.
