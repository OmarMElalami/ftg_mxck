# Changelog
All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](http://keepachangelog.com/).

## 0.3.0 - 2026-04-09
### Changed
- Removed the legacy adapter stage from architecture references and unified planning in `ftg_planner_node`.
- Documented heading/speed smoothing behavior in the planner path.
- Updated FTG parameter references for the ~30 cm MXCK vehicle setup.
- Confirmed TF-based recentering with `front_center_deg=0.0`.

## Unreleased

## 0.1.3 - 2021-01-13
### Added
- Data delay measurement.

### Changed
- Timing is now handled by `rosmeasure`.

## 0.1.2 - 2021-01-11
### Added
- Timing measurement of the callback.

## 0.1.1 - 2020-06-01
### Changed
- Values outside the LiDAR range are not republished.

### Fixed
- Computing the angle of obstacles (as it was probably bad).

## 0.1.0 - 2019-03-17
### Changed
- The node is working with `obstacle_msgs/ObstaclesStamped` instead of `obstacle_msgs/Obstacles`.

## 0.0.1 - 2019-02-15
### Added
- First version of obstacle substitution package.
