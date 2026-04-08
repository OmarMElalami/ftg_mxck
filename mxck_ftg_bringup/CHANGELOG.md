# Changelog
All notable changes to this project will be documented in this file.

## 0.2.0 - 2026-04-07
### Added
- Updated bringup to the scan-based FTG path.
- Added optional scan-window diagnostic node.
- Added compatibility launch arguments for older launch commands.
- Added package README, LICENSE, config and tests.

### Changed
- The default FTG input is now `/autonomous/ftg/scan_filtered`.
- `follow_the_gap_v0` is launched with `input_mode=scan`.

### Deprecated
- The obstacle-substitution path is no longer started by the default bringup.
