# Changelog
All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](http://keepachangelog.com/).

## Unreleased
### Changed
- Introduced a proper `include/follow_the_gap_v0/` header structure.
- Added a ROS 2 launch file.
- Updated README to describe the current `/obstacles`-based data path.
- Aligned `package.xml` license with the source headers and LICENSE file.

### Fixed
- Removed pedantic warnings caused by extra semicolons.
- Removed warnings caused by intentionally unused parameters and variables.
- Added clearer comments to explain legacy code paths.

### Deprecated
- Legacy LaserScan-only path remains in source as commented-out code.
- Legacy angle-filter ROS callbacks remain commented-out because they are not part of the active ROS 2 path.

## 0.2.4 - 2021-01-11
### Added
- Added `/obstacles` remaps to the launch file.

### Fixed
- Sort received obstacles to avoid jumping between them (which is something that FTG cannot handle).
- Disable filtering of lone obstacle groups as it does not work with extrapolation.
- Use obstacle radius instead of car radius parameter.

## 0.2.3 - 2020-10-27
### Fixed
- Convert the borders of phases from dynamic reconfigure from degrees to radians.

## 0.2.2 - 2020-10-24
### Fixed
- Publish the same frame_id as source data instead of using a static value.

## 0.2.1 - 2020-06-02
### Added
- Added callback for `/obstacles` topic, along with the `obstacle_msgs` support.
- Added publisher for expressing that no gap was found.

## 0.2.0 - 2020-04-15
### Added
- Added Makefile to compile the FTG without ROS.
- Added license GPLv3.

### Changed
- Reordered the code and split the ROS compatible layer to a separate file.

### Removed
- Removed unnecessary and obsolete comments.

### Fixed
- Added variable initialization to avoid segfaults.
