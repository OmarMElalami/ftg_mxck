# ftg_mxck — Follow-The-Gap for MXCarkit (ROS 2 Humble)

This repository contains a complete ROS 2 integration of the **Follow-The-Gap (FTG)** algorithm for the autonomous vehicle **MXCarkit (MXCK)**. The stack runs on an **NVIDIA Jetson Xavier NX** under **Ubuntu 20.04 / JetPack 5.1.5** inside a Docker workspace (`mxck2_ws`) with ROS 2 Humble.

The CTU FTG core (`follow_the_gap_v0`) is a C++ implementation by the Czech Technical University in Prague. The MXCK-specific packages provide scan preprocessing, an adapter layer, and the final Ackermann command node.

---

## Table of Contents

1. [System Overview](#1-system-overview)
2. [Repository Structure](#2-repository-structure)
3. [Package Descriptions](#3-package-descriptions)
4. [Nodes, Topics and Connections](#4-nodes-topics-and-connections)
5. [Configuration Parameters](#5-configuration-parameters)
6. [Dependencies](#6-dependencies)
7. [Installation and Build](#7-installation-and-build)
8. [Deployment to the Jetson](#8-deployment-to-the-jetson)
9. [Starting the System](#9-starting-the-system)
10. [Launching Individual Packages](#10-launching-individual-packages)
11. [Diagnostics and Debugging](#11-diagnostics-and-debugging)
12. [Recording a Rosbag](#12-recording-a-rosbag)
13. [Algorithm — How FTG Works](#13-algorithm--how-ftg-works)
14. [Architecture Decisions](#14-architecture-decisions)
15. [Pre-Test Checklist](#15-pre-test-checklist)

---

## 1. System Overview

The FTG stack is composed of several layers implemented as separate ROS 2 packages:

```
[LiDAR]
   │  /scan  (sensor_msgs/LaserScan)
   ▼
┌──────────────────────────────────────────────┐
│  mxck_ftg_perception          [OPTIONAL]     │
│  scan_preprocessor_node                      │
│  pub: /autonomous/ftg/scan_filtered          │
└──────────────────────────────────────────────┘
           │ /autonomous/ftg/scan_filtered (if used)
           │ or /scan (directly, if preprocessor disabled)
           ▼
┌──────────────────────────────────────────────┐
│  obstacle_substitution                       │  CTU package
│  obstacle_substitution_node                  │
│  sub: /scan  (or remapped scan_filtered)     │
│  pub: /obstacles  (obstacle_msgs/ObstaclesStamped) │
└──────────────────────────────────────────────┘
           │  /obstacles
           ▼
┌──────────────────────────────────────────────┐
│  follow_the_gap_v0                           │  CTU C++ package
│  follow_the_gap                              │
│  pub: /final_heading_angle (std_msgs/Float32)│
│  pub: /gap_found           (std_msgs/Bool)   │
└──────────────────────────────────────────────┘
           │  /final_heading_angle + /gap_found
           ▼
┌──────────────────────────────────────────────┐
│  mxck_ftg_planner                            │
│  ctu_ftg_adapter_node                        │
│  pub: /autonomous/ftg/gap_angle              │
│  pub: /autonomous/ftg/target_speed           │
│  pub: /autonomous/ftg/planner_status         │
└──────────────────────────────────────────────┘
           │  /autonomous/ftg/gap_angle + target_speed
           ▼
┌──────────────────────────────────────────────┐
│  mxck_ftg_control                            │
│  ftg_command_node                            │
│  pub: /autonomous/ackermann_cmd ◄── VEHICLE  │
│  pub: /autonomous/ftg/control_status         │
└──────────────────────────────────────────────┘
           │
           ▼ /autonomous/ackermann_cmd (ackermann_msgs/AckermannDriveStamped)
        [vehicle_control / ackermann_to_vesc / VESC]
```

The vehicle backend (VESC driver, RC switch, TF publisher) is provided by the existing `mxck2_ws` system (`mxck_run`, `vehicle_control`). `manual_control_launch.py` must be started **separately** — it handles RC, Deadman, safety, and `ackermann_to_vesc`. The FTG stack outputs its final drive command on `/autonomous/ackermann_cmd`, which the existing vehicle control chain picks up in Autonomous mode.

---

## 2. Repository Structure

```
ftg_mxck/
├── follow_the_gap_v0/         # CTU C++ FTG algorithm (upstream, unchanged)
├── obstacle_msgs/             # CTU message definitions (CircleObstacle, ObstaclesStamped)
├── obstacle_substitution/     # CTU Python node: /scan → /obstacles
├── mxck_ftg_perception/       # Optional scan preprocessor (FOV filter, range clip)
├── mxck_ftg_planner/          # Adapter: CTU outputs → MXCK topics
├── mxck_ftg_control/          # Final command node → /autonomous/ackermann_cmd
└── mxck_ftg_bringup/          # Launch files for the full stack
    └── launch/
        ├── ftg_stack.launch.py          # Compact stack launch
        └── ftg_full_system.launch.py    # Full system with rosbag support
```

All packages must be placed in `mxck2_ws/src/` (flat — not inside a subdirectory).

---

## 3. Package Descriptions

### 3.1 `follow_the_gap_v0` — CTU C++ FTG Core

The upstream C++ implementation of the Follow-The-Gap algorithm by the Czech Technical University in Prague.

- Subscribes to `/obstacles` (`obstacle_msgs/ObstaclesStamped`)
- Publishes `/final_heading_angle` (`std_msgs/Float32`) — target steering angle in radians
- Publishes `/gap_found` (`std_msgs/Bool`) — whether a drivable gap was found
- Also publishes visualization topics for RViz debugging

**Do not modify this package.** It is treated as an upstream dependency.

---

### 3.2 `obstacle_msgs` — CTU Message Definitions

Defines `CircleObstacle` and `ObstaclesStamped` message types used between `obstacle_substitution` and `follow_the_gap_v0`.

---

### 3.3 `obstacle_substitution` — CTU Scan-to-Obstacle Converter

Converts a raw `LaserScan` into a list of circular obstacles.

- Subscribes to `/scan` (hardcoded — use remapping if the preprocessor is active)
- Publishes `/obstacles` (`obstacle_msgs/ObstaclesStamped`)

> **Important:** When `use_scan_preprocessor=true`, the launch file remaps `/scan` → `/autonomous/ftg/scan_filtered` for this node automatically.

---

### 3.4 `mxck_ftg_perception` — Optional Scan Preprocessor

Pre-processes the raw LiDAR scan before it enters the CTU pipeline.

#### `scan_preprocessor_node`

- Subscribes to `/scan`
- Applies a configurable front FOV window filter
- Clips range values to `[clip_min_range_m, clip_max_range_m]`
- Optionally applies a moving-average noise filter
- Publishes `/autonomous/ftg/scan_filtered` (filtered scan passed to `obstacle_substitution`)

#### `scan_front_window_check` (diagnostic only)

- Reads `/scan`, finds the nearest point in the front window
- Logs direction (LEFT / CENTER / RIGHT) every N scans
- Optionally publishes RViz markers

---

### 3.5 `mxck_ftg_planner` — CTU Adapter Node

Bridges the CTU FTG outputs to the MXCK topic namespace.

#### `ctu_ftg_adapter_node` *(active default)*

- Subscribes to `/final_heading_angle`, `/gap_found`, and optionally `/scan` (for speed policy)
- Applies heading sign/offset correction and angle clamping
- Computes a clearance-based target speed from a front scan window
- Publishes `/autonomous/ftg/gap_angle`, `/autonomous/ftg/target_speed`, `/autonomous/ftg/planner_status`

> **Note:** `ftg_planner_node.py` also exists in this package as a standalone Python FTG reimplementation. It is **not** used in the standard launch pipeline. Do not start it alongside `ctu_ftg_adapter_node`.

---

### 3.6 `mxck_ftg_control` — Final Command Node

Converts planner outputs into a safe, rate-limited `AckermannDriveStamped` command.

#### `ftg_command_node`

- Subscribes to `/autonomous/ftg/gap_angle` and `/autonomous/ftg/target_speed`
- Applies timeout safety (stops if no fresh data within `command_timeout_sec`)
- Applies steering gain, inversion, clamp, and optional IIR smoothing
- Applies optional turn-speed scaling (reduces speed during sharp turns)
- Publishes at a fixed rate (`publish_rate_hz`) to `/autonomous/ackermann_cmd`
- Publishes status on `/autonomous/ftg/control_status`

---

### 3.7 `mxck_ftg_bringup` — Launch Coordinator

Contains launch files that start all components together.

#### `ftg_stack.launch.py`

The primary launch file. Starts:
- (Optional) TF broadcast via `mxck_run/broadcast_tf_launch.py`
- (Optional) `scan_preprocessor_node` (enabled by `use_scan_preprocessor:=true`)
- `obstacle_substitution_node` (with remapping if preprocessor is active)
- `follow_the_gap` (CTU C++ node)
- `ctu_ftg_adapter_node`
- `ftg_command_node`

#### `ftg_full_system.launch.py`

Wraps `ftg_stack.launch.py` and adds optional rosbag recording. Does **not** start `vehicle_control` — that must be started separately.

---

## 4. Nodes, Topics and Connections

### Full Topic Table

| Topic | Type | Publisher | Subscriber | Description |
|-------|------|-----------|------------|-------------|
| `/scan` | `sensor_msgs/LaserScan` | LiDAR driver (`mxck_run`) | `scan_preprocessor_node`, `obstacle_substitution_node`, `scan_front_window_check`, `ctu_ftg_adapter_node` | Raw LiDAR scan |
| `/tf` / `/tf_static` | TF tree | `mxck_run/broadcast_tf_launch.py` | All nodes (via `tf2_ros`) | Coordinate frame transforms |
| `/autonomous/ftg/scan_filtered` | `sensor_msgs/LaserScan` | `scan_preprocessor_node` | `obstacle_substitution_node` (remapped `/scan`) | FOV-filtered, clipped scan |
| `/obstacles` | `obstacle_msgs/ObstaclesStamped` | `obstacle_substitution_node` | `follow_the_gap` | Circle-obstacle list |
| `/final_heading_angle` | `std_msgs/Float32` | `follow_the_gap` | `ctu_ftg_adapter_node` | Target heading angle from CTU FTG [rad] |
| `/gap_found` | `std_msgs/Bool` | `follow_the_gap` | `ctu_ftg_adapter_node` | Whether a drivable gap was found |
| `/autonomous/ftg/gap_angle` | `std_msgs/Float32` | `ctu_ftg_adapter_node` | `ftg_command_node` | Adjusted target steering angle [rad] |
| `/autonomous/ftg/target_speed` | `std_msgs/Float32` | `ctu_ftg_adapter_node` | `ftg_command_node` | Target speed [m/s] |
| `/autonomous/ftg/planner_status` | `std_msgs/String` | `ctu_ftg_adapter_node` | — | Planner status string |
| `/autonomous/ackermann_cmd` | `ackermann_msgs/AckermannDriveStamped` | `ftg_command_node` | Vehicle control (VESC) | **Final autonomous drive command** |
| `/autonomous/ftg/control_status` | `std_msgs/String` | `ftg_command_node` | — | Control node status string |

### Node Connection Diagram

```
mxck_run (LiDAR)
   └─[/scan]──────────────────────────────────────┐
                                                   │
                              ┌────────────────────▼───────────────────────┐
                              │      scan_preprocessor_node  [OPTIONAL]    │
                              │  sub: /scan                                │
                              │  pub: /autonomous/ftg/scan_filtered        │
                              └──────────────────┬─────────────────────────┘
                                                 │ /autonomous/ftg/scan_filtered
                              ┌──────────────────▼─────────────────────────┐
                              │      obstacle_substitution_node             │
                              │  sub: /scan  (remapped when preprocessor on)│
                              │  pub: /obstacles                            │
                              └──────────────────┬─────────────────────────┘
                                                 │ /obstacles
                              ┌──────────────────▼─────────────────────────┐
                              │      follow_the_gap  (CTU C++)              │
                              │  sub: /obstacles                            │
                              │  pub: /final_heading_angle                  │
                              │  pub: /gap_found                            │
                              └──────┬──────────────────┬───────────────────┘
                       [heading]     │        [gap_found]│
                              ┌──────▼──────────────────▼───────────────────┐
                              │      ctu_ftg_adapter_node                   │
                              │  sub: /final_heading_angle                  │
                              │  sub: /gap_found                            │
                              │  sub: /scan  (for speed policy)             │
                              │  pub: /autonomous/ftg/gap_angle             │
                              │  pub: /autonomous/ftg/target_speed          │
                              │  pub: /autonomous/ftg/planner_status        │
                              └──────┬──────────────────┬───────────────────┘
                   [gap_angle]       │    [target_speed] │
                              ┌──────▼──────────────────▼───────────────────┐
                              │      ftg_command_node                       │
                              │  sub: /autonomous/ftg/gap_angle             │
                              │  sub: /autonomous/ftg/target_speed          │
                              │  pub: /autonomous/ackermann_cmd  ◄─ VEHICLE │
                              │  pub: /autonomous/ftg/control_status        │
                              └─────────────────────────────────────────────┘
```

---

## 5. Configuration Parameters

### 5.1 `mxck_ftg_perception` — `scan_preprocessor.yaml`

```yaml
scan_topic: "/scan"
base_frame: "base_link"
filtered_scan_topic: "/autonomous/ftg/scan_filtered"
front_clearance_topic: "/autonomous/ftg/front_clearance"
status_topic: "/autonomous/ftg/status"

front_center_deg: 0.0          # Center of front FOV window [deg]
front_fov_deg: 140.0           # Total width of front FOV window [deg]

clip_min_range_m: 0.05         # Minimum valid range [m]
clip_max_range_m: 8.0          # Maximum valid range [m]
outside_window_as_obstacle: true  # Set rays outside FOV to clip_min (= obstacle)

enable_moving_average: false   # Enable moving-average noise filter
moving_average_window: 3       # Window size for moving average
```

### 5.2 `mxck_ftg_planner` — `ctu_ftg_adapter.yaml`

```yaml
ctu_ftg_adapter_node:
  ros__parameters:
    final_heading_topic: /final_heading_angle
    gap_found_topic: /gap_found
    scan_topic: /scan            # Used for speed policy (front window clearance)

    gap_angle_topic: /autonomous/ftg/gap_angle
    target_speed_topic: /autonomous/ftg/target_speed
    planner_status_topic: /autonomous/ftg/planner_status

    publish_rate_hz: 15.0
    message_timeout_sec: 0.50

    use_scan_for_speed: true     # Derive speed from /scan front-window clearance
    front_window_deg: 20.0       # Half-angle of speed-policy front window [deg]
    default_front_clearance_m: 10.0  # Fallback when scan unavailable

    heading_sign: 1.0            # Set to -1.0 if steering direction is inverted
    heading_offset_rad: 0.0
    gap_angle_limit_rad: 0.60    # Clamp output angle to ±this value [rad]

    # Speed policy thresholds
    stop_distance_m: 0.30
    slow_distance_m: 0.55
    clear_distance_m: 0.90

    stop_speed_mps: 0.0
    crawl_speed_mps: 0.10
    slow_speed_mps: 0.18
    cruise_speed_mps: 0.25
```

### 5.3 `mxck_ftg_control` — `ftg_control.yaml`

```yaml
gap_angle_topic: "/autonomous/ftg/gap_angle"
target_speed_topic: "/autonomous/ftg/target_speed"
ackermann_topic: "/autonomous/ackermann_cmd"
status_topic: "/autonomous/ftg/control_status"
frame_id: "base_link"

publish_rate_hz: 15.0
command_timeout_sec: 0.5       # Stop if no fresh planner data for this long

# Steering
invert_steering: false         # Invert steering direction if needed
steering_gain: 1.0             # Steering angle multiplier
steering_limit_deg: 22.0       # Clamp output to ±this value [deg]
enable_steering_smoothing: true
steering_smoothing_alpha: 0.35 # IIR coefficient (0=frozen, 1=no filter)

# Speed
speed_limit_mps: 0.35          # Absolute speed cap [m/s]
stop_on_timeout: true          # Issue zero-speed command on timeout

# Turn-speed scaling
enable_turn_speed_scaling: true
slowdown_start_deg: 10.0       # Scaling begins at this steering angle [deg]
slowdown_full_deg: 22.0        # Maximum reduction applied at this angle [deg]
min_turn_speed_mps: 0.15       # Minimum speed during sharp turns [m/s]
```

---

## 6. Dependencies

| Package | Type | ROS 2 Dependencies |
|---------|------|--------------------|
| `obstacle_msgs` | C++ (ament_cmake) | `std_msgs`, `geometry_msgs` |
| `obstacle_substitution` | Python | `rclpy`, `sensor_msgs`, `obstacle_msgs` |
| `follow_the_gap_v0` | C++ (ament_cmake) | `rclcpp`, `sensor_msgs`, `std_msgs`, `visualization_msgs`, `geometry_msgs`, `obstacle_msgs`, `tf2_geometry_msgs` |
| `mxck_ftg_perception` | Python | `rclpy`, `sensor_msgs`, `std_msgs`, `geometry_msgs`, `visualization_msgs`, `tf2_ros` |
| `mxck_ftg_planner` | Python | `rclpy`, `sensor_msgs`, `std_msgs`, `geometry_msgs`, `visualization_msgs`, `tf2_ros` |
| `mxck_ftg_control` | Python | `rclpy`, `std_msgs`, `ackermann_msgs` |
| `mxck_ftg_bringup` | Python | `launch`, `launch_ros`, `ament_index_python`, `mxck_run`*, `vehicle_control`* |

> **\*Note:** `mxck_run` and `vehicle_control` are packages from the existing `mxck2_ws` system and must already be installed.

**System requirements:**
- Ubuntu 20.04 (on Jetson: JetPack 5.1.5)
- ROS 2 Humble
- Python 3.10+
- `ros-humble-ackermann-msgs`
- `ros-humble-tf2-ros`
- `ros-humble-visualization-msgs`
- C++ build tools for `follow_the_gap_v0` and `obstacle_msgs` (`ament_cmake`, `g++`)

---

## 7. Installation and Build

### 7.1 Place packages in the workspace

All packages must be placed directly inside `mxck2_ws/src/` (not in a subdirectory):

```bash
# Check that the packages are in the right location:
ls /mxck2_ws/src/follow_the_gap_v0
ls /mxck2_ws/src/obstacle_msgs
ls /mxck2_ws/src/obstacle_substitution
ls /mxck2_ws/src/mxck_ftg_perception
ls /mxck2_ws/src/mxck_ftg_planner
ls /mxck2_ws/src/mxck_ftg_control
ls /mxck2_ws/src/mxck_ftg_bringup
```

### 7.2 Log into the container

```bash
sudo docker exec -it mxck2_control bash
```

### 7.3 Source the ROS environment

```bash
source /opt/ros/humble/setup.bash
source /mxck2_ws/install/setup.bash
```

### 7.4 Build all FTG packages

Build the CTU message and C++ packages first, then the Python packages:

```bash
cd /mxck2_ws
colcon build --symlink-install --packages-select \
    obstacle_msgs \
    obstacle_substitution \
    follow_the_gap_v0 \
    mxck_ftg_perception \
    mxck_ftg_planner \
    mxck_ftg_control \
    mxck_ftg_bringup
```

### 7.5 Re-source the workspace after building

```bash
source /mxck2_ws/install/setup.bash
```

### 7.6 Verify the build

```bash
# Check that all executables are registered:
ros2 pkg executables mxck_ftg_perception
ros2 pkg executables mxck_ftg_planner
ros2 pkg executables mxck_ftg_control
ros2 pkg executables follow_the_gap_v0
ros2 pkg executables obstacle_substitution

# Expected output:
# mxck_ftg_perception scan_front_window_check
# mxck_ftg_perception scan_preprocessor_node
# mxck_ftg_planner ctu_ftg_adapter_node
# mxck_ftg_planner ftg_planner_node
# mxck_ftg_control ftg_command_node
# follow_the_gap_v0 follow_the_gap
# obstacle_substitution obstacle_substitution_node
```

> **Warning:** Do not run `ftg_planner_node` and `ctu_ftg_adapter_node` at the same time. Only `ctu_ftg_adapter_node` is used in the standard pipeline.

---

## 8. Deployment to the Jetson

### 8.1 Copy all packages at once (from development PC)

```bash
# Copy the entire repository folder:
scp -r ftg_mxck/* mxck@192.168.0.100:/home/mxck/mxck2_ws/src/

# Or copy individual packages:
scp -r follow_the_gap_v0    mxck@192.168.0.100:/home/mxck/mxck2_ws/src/
scp -r obstacle_msgs        mxck@192.168.0.100:/home/mxck/mxck2_ws/src/
scp -r obstacle_substitution mxck@192.168.0.100:/home/mxck/mxck2_ws/src/
scp -r mxck_ftg_perception  mxck@192.168.0.100:/home/mxck/mxck2_ws/src/
scp -r mxck_ftg_planner     mxck@192.168.0.100:/home/mxck/mxck2_ws/src/
scp -r mxck_ftg_control     mxck@192.168.0.100:/home/mxck/mxck2_ws/src/
scp -r mxck_ftg_bringup     mxck@192.168.0.100:/home/mxck/mxck2_ws/src/
```

### 8.2 Remove conflicting old packages (if present)

If your workspace contains older versions of the CTU packages from a previous setup, remove them first:

```bash
# Move old packages out of the source tree:
mv /home/mxck/mxck2_ws/src/follow_the_gap_v0_ride /home/mxck/backup_old_ftg/ 2>/dev/null || true
mv /home/mxck/mxck2_ws/src/auto                   /home/mxck/backup_old_ftg/ 2>/dev/null || true
mv /home/mxck/mxck2_ws/src/ftg_scan_filter        /home/mxck/backup_old_ftg/ 2>/dev/null || true

# Remove old build artifacts:
rm -rf /mxck2_ws/build/follow_the_gap_v0_ride
rm -rf /mxck2_ws/build/auto
rm -rf /mxck2_ws/build/ftg_scan_filter
rm -rf /mxck2_ws/install/follow_the_gap_v0_ride
rm -rf /mxck2_ws/install/auto
rm -rf /mxck2_ws/install/ftg_scan_filter
```

### 8.3 Start the Docker containers on the Jetson

```bash
cd ~/mxck2_ws/.devcontainer
sudo docker compose -f docker-compose.all.yml up -d lidar foxglove control kickstart
```

### 8.4 Build on the Jetson (inside container)

```bash
sudo docker exec -it mxck2_control bash -lc "
    source /opt/ros/humble/setup.bash &&
    source /mxck2_ws/install/setup.bash &&
    cd /mxck2_ws &&
    colcon build --symlink-install --packages-select \
        obstacle_msgs \
        obstacle_substitution \
        follow_the_gap_v0 \
        mxck_ftg_perception \
        mxck_ftg_planner \
        mxck_ftg_control \
        mxck_ftg_bringup &&
    source /mxck2_ws/install/setup.bash
"
```

### 8.5 Check TF availability (run only once)

```bash
sudo docker exec -it mxck2_control bash -lc "
    source /opt/ros/humble/setup.bash &&
    source /mxck2_ws/install/setup.bash &&
    ros2 node list | grep robot_state_publisher
"
# If a robot_state_publisher is already running, do NOT start a second one.
# Use start_tf:=false in ftg_stack.launch.py in that case.
```

---

## 9. Starting the System

### Step 0 — Start vehicle control separately (mandatory first step)

The FTG stack does **not** start vehicle control. It must be running before you launch the FTG stack:

```bash
sudo docker exec -it mxck2_control bash -lc "
    source /opt/ros/humble/setup.bash &&
    source /mxck2_ws/install/setup.bash &&
    ros2 launch vehicle_control manual_control_launch.py
"
```

### Step 1 — Start the FTG stack

#### Recommended: full stack with scan preprocessor (default)

```bash
sudo docker exec -it mxck2_control bash -lc "
    source /opt/ros/humble/setup.bash &&
    source /mxck2_ws/install/setup.bash &&
    ros2 launch mxck_ftg_bringup ftg_stack.launch.py
"
```

#### Without the optional scan preprocessor (direct /scan → obstacle_substitution)

```bash
ros2 launch mxck_ftg_bringup ftg_stack.launch.py use_scan_preprocessor:=false
```

#### Without TF broadcast (when TF is already running)

```bash
ros2 launch mxck_ftg_bringup ftg_stack.launch.py start_tf:=false
```

### Step 2 — Full system with rosbag recording

```bash
sudo docker exec -it mxck2_control bash -lc "
    source /opt/ros/humble/setup.bash &&
    source /mxck2_ws/install/setup.bash &&
    ros2 launch mxck_ftg_bringup ftg_full_system.launch.py \
        record_bag:=true \
        bag_dir:=/mxck2_ws/bags \
        bag_name:=ftg_run_$(date +%Y%m%d_%H%M%S)
"
```

### Launch Arguments — `ftg_stack.launch.py`

| Argument | Default | Description |
|----------|---------|-------------|
| `start_tf` | `true` | Start TF broadcast via `mxck_run/broadcast_tf_launch.py` |
| `use_scan_preprocessor` | `true` | Run `scan_preprocessor_node` before `obstacle_substitution` |
| `start_ctu_ftg` | `true` | Start `follow_the_gap_v0` C++ node |
| `start_adapter` | `true` | Start `ctu_ftg_adapter_node` |
| `start_control` | `true` | Start `ftg_command_node` |

### Launch Arguments — `ftg_full_system.launch.py`

| Argument | Default | Description |
|----------|---------|-------------|
| `use_tf` | `true` | Start TF broadcast |
| `use_ftg_stack` | `true` | Start the full FTG stack |
| `use_scan_preprocessor` | `true` | Pass to inner `ftg_stack.launch.py` |
| `record_bag` | `false` | Enable rosbag2 MCAP recording |
| `bag_dir` | `/mxck2_ws/bags` | Output directory for bag files |
| `bag_name` | `ctu_ftg_run` | Bag session name |

---

## 10. Launching Individual Packages

For step-by-step testing you can launch each package independently.

### Scan Preprocessor only

```bash
ros2 launch mxck_ftg_perception scan_preprocessor.launch.py
```

### Diagnostic scan check only

```bash
ros2 launch mxck_ftg_perception scan_front_window_check.launch.py
```

### Adapter node only

```bash
# Use the adapter launch from mxck_ftg_planner
ros2 run mxck_ftg_planner ctu_ftg_adapter_node \
    --ros-args --params-file $(ros2 pkg prefix mxck_ftg_planner)/share/mxck_ftg_planner/config/ctu_ftg_adapter.yaml
```

### Control node only

```bash
ros2 launch mxck_ftg_control ftg_command.launch.py
```

### Manual startup order (4 terminals)

```bash
# Terminal A — TF
ros2 launch mxck_run broadcast_tf_launch.py

# Terminal B — Scan preprocessor
ros2 launch mxck_ftg_perception scan_preprocessor.launch.py

# Terminal C — CTU obstacle pipeline
ros2 run obstacle_substitution obstacle_substitution_node \
    --ros-args --remap /scan:=/autonomous/ftg/scan_filtered

# Terminal D — CTU FTG C++ node
ros2 run follow_the_gap_v0 follow_the_gap

# Terminal E — Adapter
ros2 run mxck_ftg_planner ctu_ftg_adapter_node \
    --ros-args --params-file $(ros2 pkg prefix mxck_ftg_planner)/share/mxck_ftg_planner/config/ctu_ftg_adapter.yaml

# Terminal F — Control
ros2 launch mxck_ftg_control ftg_command.launch.py
```

---

## 11. Diagnostics and Debugging

### 11.1 Check running nodes

```bash
sudo docker exec -it mxck2_control bash -lc "
    source /opt/ros/humble/setup.bash &&
    source /mxck2_ws/install/setup.bash &&
    ros2 node list | sort
"
# Expected nodes (minimal):
# /ctu_ftg_adapter_node
# /follow_the_gap
# /ftg_command_node
# /obstacle_substitution
# /scan_preprocessor_node  (if use_scan_preprocessor=true)
```

### 11.2 Verify publisher counts (critical safety check)

Each of these topics must have exactly **1 publisher**:

```bash
ros2 topic info /autonomous/ftg/gap_angle -v
ros2 topic info /autonomous/ftg/target_speed -v
ros2 topic info /autonomous/ackermann_cmd -v
```

### 11.3 Check LiDAR scan

```bash
# Print one scan:
ros2 topic echo /scan --once

# Measure scan frequency:
ros2 topic hz /scan
```

### 11.4 Check CTU pipeline output

```bash
# Obstacle list:
ros2 topic hz /obstacles

# CTU heading output (should be a radian value):
ros2 topic echo /final_heading_angle

# Gap found flag:
ros2 topic echo /gap_found
```

### 11.5 Check adapter output

```bash
# Target steering angle:
ros2 topic echo /autonomous/ftg/gap_angle

# Target speed:
ros2 topic echo /autonomous/ftg/target_speed

# Planner status string:
ros2 topic echo /autonomous/ftg/planner_status
```

### 11.6 Check final drive command

```bash
# AckermannDriveStamped output:
ros2 topic echo /autonomous/ackermann_cmd

# Output frequency (should match publish_rate_hz = 15 Hz):
ros2 topic hz /autonomous/ackermann_cmd

# Control node status:
ros2 topic echo /autonomous/ftg/control_status
```

### 11.7 Check preprocessed scan

```bash
# Filtered scan frequency:
ros2 topic hz /autonomous/ftg/scan_filtered
```

### 11.8 Check TF transforms

```bash
# All static transforms:
ros2 topic echo /tf_static --qos-durability transient_local --once

# View TF tree:
ros2 run tf2_tools view_frames
```

### 11.9 Inspect and change parameters at runtime

```bash
# List all parameters:
ros2 param list /ctu_ftg_adapter_node
ros2 param list /ftg_command_node

# Read a parameter:
ros2 param get /ftg_command_node speed_limit_mps

# Change a parameter at runtime:
ros2 param set /ftg_command_node speed_limit_mps 0.30
ros2 param set /ctu_ftg_adapter_node heading_sign -1.0
```

---

## 12. Recording a Rosbag

### 12.1 Manual recording of all relevant topics

```bash
sudo docker exec -it mxck2_control bash -lc "
    source /opt/ros/humble/setup.bash &&
    source /mxck2_ws/install/setup.bash &&
    mkdir -p /mxck2_ws/bags &&
    ros2 bag record -s mcap -o /mxck2_ws/bags/ftg_run \
        /scan \
        /tf \
        /tf_static \
        /autonomous/ftg/scan_filtered \
        /obstacles \
        /final_heading_angle \
        /gap_found \
        /autonomous/ftg/gap_angle \
        /autonomous/ftg/target_speed \
        /autonomous/ftg/planner_status \
        /autonomous/ackermann_cmd \
        /autonomous/ftg/control_status \
        /rc/ackermann_cmd \
        /commands/servo/position \
        /commands/motor/speed \
        /commands/motor/brake
"
```

### 12.2 Play back a bag for offline testing (without a vehicle)

```bash
# Play back the bag:
ros2 bag play /mxck2_ws/bags/ftg_run --clock

# Run the FTG stack against the bag data (in a second terminal):
ros2 launch mxck_ftg_bringup ftg_stack.launch.py start_tf:=false
```

### 12.3 Inspect bag contents

```bash
ros2 bag info /mxck2_ws/bags/ftg_run
```

---

## 13. Algorithm — How FTG Works

The **Follow-The-Gap** algorithm is a reactive local planning method. The implementation follows the paper by Sezer & Gökasan (2012). The MXCK stack feeds it through the CTU packages as follows:

### Step 1 — Scan preprocessing (optional)

`scan_preprocessor_node` restricts the raw `/scan` to a configurable front FOV window, clips out-of-range values, and optionally smooths with a moving average. The result is `/autonomous/ftg/scan_filtered`.

### Step 2 — Obstacle conversion

`obstacle_substitution_node` converts each valid LiDAR beam into a `CircleObstacle` message and publishes the full list as `/obstacles`.

### Step 3 — CTU FTG algorithm (C++)

`follow_the_gap` receives `/obstacles` and:
1. Finds the nearest obstacle
2. Applies a **safety bubble** around it (sets nearby beams to zero distance)
3. Identifies **free gaps** (consecutive beams above a distance threshold)
4. Selects the best gap (widest, closest to center)
5. Computes the **final heading angle** toward the best gap

Outputs `/final_heading_angle` (rad) and `/gap_found` (bool) every callback cycle.

### Step 4 — Adapter (MXCK bridge)

`ctu_ftg_adapter_node` receives the CTU outputs and:
- Applies heading sign correction (`heading_sign`) and offset
- Clamps the angle to `±gap_angle_limit_rad`
- Computes front clearance from a narrow forward `/scan` window
- Derives a target speed from the clearance thresholds:

| Condition | Speed |
|-----------|-------|
| `clearance < stop_distance_m` (0.30 m) | 0.0 m/s (stop) |
| `clearance < slow_distance_m` (0.55 m) | 0.10 m/s (crawl) |
| `clearance < clear_distance_m` (0.90 m) | 0.18 m/s (slow) |
| `clearance >= clear_distance_m` | 0.25 m/s (cruise) |

Outputs `/autonomous/ftg/gap_angle` and `/autonomous/ftg/target_speed`.

### Step 5 — Command node (safety layer)

`ftg_command_node` applies a final safety layer before outputting to the vehicle:
- **Timeout check**: If no fresh data for `command_timeout_sec`, speed is forced to 0
- **Steering gain and clamp**: `steering_gain × angle`, clamped to `±steering_limit_deg`
- **IIR smoothing**: `filtered = α × new + (1−α) × old`
- **Turn-speed scaling**: Reduces speed linearly between `slowdown_start_deg` and `slowdown_full_deg`
- Publishes at `publish_rate_hz` Hz to `/autonomous/ackermann_cmd`

---

## 14. Architecture Decisions

### Why separate CTU and MXCK packages?

The CTU packages (`follow_the_gap_v0`, `obstacle_substitution`, `obstacle_msgs`) are upstream code maintained by Czech Technical University in Prague. They are kept unchanged to allow clean upstream updates. The MXCK-specific packages add the adapter and safety layers needed for the MXCarkit platform.

### Why `/autonomous/ackermann_cmd`?

This topic is the defined input of the autonomous vehicle control chain in `mxck2_ws`. The RC switch and VESC driver consume commands from this topic in Autonomous mode. Publishing here integrates the FTG stack transparently into the existing vehicle architecture without modifying any downstream components.

### Why is vehicle_control started separately?

`vehicle_control/manual_control_launch.py` owns RC control, Deadman, VESC driver setup, and the safety switch. It must be running before the FTG stack so that the safety interlock is always active. Starting it from the FTG launch would risk duplicate startup or loss of the safety chain.

### Why an optional scan preprocessor?

The CTU FTG algorithm is sensitive to wide-angle noise and out-of-range artifacts. Restricting input to the front FOV and clipping extremes reduces false gaps and improves gap detection reliability in confined spaces. The preprocessor can be disabled with `use_scan_preprocessor:=false` for testing with raw scan data.

### Why does `ftg_command_node` apply turn-speed scaling if the adapter already has a speed policy?

The adapter's speed policy is based on **front clearance** (obstacle proximity). The command node's turn-speed scaling is based on **steering angle** (cornering dynamics). These are orthogonal concerns. The command node always applies a final hard speed cap and timeout safety, regardless of what the adapter computes.

---

## 15. Pre-Test Checklist

Run through this checklist before the first autonomous movement on the vehicle:

- [ ] `vehicle_control` / `manual_control_launch.py` is running separately before the FTG stack is started
- [ ] TF is available (`base_link` ↔ LiDAR frame) — verify with `ros2 run tf2_tools view_frames`
- [ ] Exactly one publisher on `/autonomous/ackermann_cmd` — verify with `ros2 topic info /autonomous/ackermann_cmd -v`
- [ ] All expected FTG nodes are up: `obstacle_substitution`, `follow_the_gap`, `ctu_ftg_adapter_node`, `ftg_command_node`
- [ ] `/scan` is live and stable (`ros2 topic hz /scan`)
- [ ] `/obstacles` is being published (`ros2 topic hz /obstacles`)
- [ ] `/final_heading_angle` and `/gap_found` are published every scan cycle
- [ ] `/autonomous/ftg/gap_angle` and `/autonomous/ftg/target_speed` are published
- [ ] `/autonomous/ackermann_cmd` carries finite `steering_angle` and `speed` values
- [ ] Timeout behavior verified: interrupting upstream briefly causes speed to fall to 0
- [ ] Steering sign verified in a low-speed test (obstacle offset left → steers right, and vice versa)
- [ ] Speed stays within the configured limit (`<= 0.35 m/s` in default config)
- [ ] Emergency abort path (deadman / RC override) tested before first autonomous movement
- [ ] **Abort immediately** if:
  - No TF or repeated TF lookup failures
  - No data on `/gap_found` or `/final_heading_angle`
  - More than one publisher on `/autonomous/ackermann_cmd`
  - Non-zero speed command persists after upstream disconnect
  - Oscillatory or inverted steering at low speed

---

*This repository is part of an integration project for the MXCarkit autonomous vehicle platform. The CTU FTG core (`follow_the_gap_v0`) is © Czech Technical University in Prague, licensed under GPL-3.0.*
