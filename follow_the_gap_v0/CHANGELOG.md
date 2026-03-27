# Changelog
All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](http://keepachangelog.com/).

## Unreleased
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

## 0.1.0 - 2019-12-10
### Added
- Added dynamic reconfigure support for FTGR.
- Added commentary for FTGR node.

### Changed
- Changed the name of the package from `follow_the_gap` to `follow_the_gap_v0` to make it clear that this is an older implementation.
- Changed topics and message types for publishing internal parameters.
- Split the parameters for left / right turning, as they were different for a long time anyway.

### Removed
- Removed VESC support.
- Removed architecture dependent constants.
- Removed unused publishers.
- Removed integer publishers of internal parameters (as Drive-API is rather float-based.)

## 0.0.0 - 2019-05-06
### Added
- Ported Follow The Gap algorithm from repository v0.
- Added support for Drive-API.
- Added launch file for FTG+FTGR.

### Changed
- Commented out publishing messages for VESC as it is not currently supported.
- Changed method for retrieving number of measurements from the LiDAR.

### Removed
- Removed old `CarControlData` message which is not supported in newer repository versions. (It was substituted by multiple messages on multiple topics using basic message types.)
- Removed direct control of Teensy board. (It is substituted by using Drive-API.)

### Fixed
- Fixed published steering direction.
