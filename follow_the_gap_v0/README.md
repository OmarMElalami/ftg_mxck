# follow_the_gap_v0

Legacy CTU Follow-The-Gap (FTG) package for ROS 2.

This package contains the **algorithm core** for the older CTU FTG implementation.
In the current MXCK stack it is best treated as a **planner kernel**:

```text
/scan -> obstacle_substitution -> /obstacles -> follow_the_gap_v0
      -> /final_heading_angle + /gap_found + debug topics
```

It does **not** publish Ackermann steering commands directly.

---

## 1. Role in the MXCK stack

`follow_the_gap_v0` is the package that chooses a **safe heading angle** through
the currently visible gap between obstacles.

### Current active input
- `/obstacles` (`obstacle_msgs/msg/ObstaclesStamped`)

### Current main outputs
- `/final_heading_angle` (`std_msgs/msg/Float32`)
- `/gap_found` (`std_msgs/msg/Bool`)

### Debug / visualization outputs
- `/visualize_final_heading_angle` (`geometry_msgs/msg/PoseStamped`)
- `/visualize_obstacles` (`visualization_msgs/msg/Marker`)
- `/visualize_largest_gap` (`geometry_msgs/msg/PointStamped`)

---

## 2. Node

### Executable
- `follow_the_gap`

### ROS node name
- `follow_the_gap`

This package currently builds **one executable** and runs **one node**.

---

## 3. Active topic interface

## Subscriptions

### `/obstacles`
- **Type:** `obstacle_msgs/msg/ObstaclesStamped`
- **Meaning:** Circular obstacle representation already prepared by upstream perception.
- **Expected frame:** Same frame that should be used for debug visualization.
- **Units:** meters for positions and radii.

### `/lsr/angle`
- **Type:** `std_msgs/msg/Float64`
- **Meaning:** Goal heading bias.
- **Units:** radians.

> Note: This topic is optional for the algorithm, but currently kept to preserve
> existing behavior.

## Publications

### `/final_heading_angle`
- **Type:** `std_msgs/msg/Float32`
- **Meaning:** Final selected FTG heading angle.
- **Units:** radians.

### `/gap_found`
- **Type:** `std_msgs/msg/Bool`
- **Meaning:** Indicates whether the planner found a valid gap in the current cycle.

### `/visualize_final_heading_angle`
- **Type:** `geometry_msgs/msg/PoseStamped`
- **Meaning:** Orientation-only pose showing the selected heading angle.
- **Units:** orientation quaternion derived from radians.
- **Frame:** copied from input obstacle message header.

### `/visualize_obstacles`
- **Type:** `visualization_msgs/msg/Marker`
- **Meaning:** Points representing the obstacle set that was actually used by the planner.
- **Units:** meters.
- **Frame:** copied from input obstacle message header.

### `/visualize_largest_gap`
- **Type:** `geometry_msgs/msg/PointStamped`
- **Meaning:** Three consecutive published points:
  1. robot origin
  2. left border of selected gap
  3. right border of selected gap
- **Units:** meters.
- **Frame:** copied from input obstacle message header.

> This topic is kept unchanged to avoid behavior changes.
> For future cleanup, a `Marker` or `MarkerArray` would be easier to visualize.

---

## 4. Algorithm overview

The algorithm works on a set of circular obstacles.

1. Convert input circles into internal `Obstacle` objects.
2. Filter obstacles by field-of-view and current maximum relevant range.
3. Build candidate `Gap` objects between non-overlapping neighboring obstacles.
4. Select the largest valid gap.
5. Compute the gap center angle.
6. Fuse gap center angle with goal angle.
7. If no gap can be found, try fallback logic such as corner-following.

The current implementation contains several legacy FTG variants, but the active
ROS path uses the central callback in `follow_the_gap.cpp`.

---

## 5. File-by-file overview

### `include/follow_the_gap_v0/obstacle.hpp`
Obstacle representation used by the FTG algorithm.

### `include/follow_the_gap_v0/gap.hpp`
Represents a candidate gap between two obstacles.

### `include/follow_the_gap_v0/corner.hpp`
Represents a corner as a specialized gap.

### `include/follow_the_gap_v0/lidar_data.hpp`
Small helper container for LiDAR metadata.
Kept for compatibility even though the active ROS path currently uses `/obstacles`.

### `include/follow_the_gap_v0/follow_the_gap.hpp`
Public declarations for the algorithm layer.

### `src/obstacle.cpp`
Obstacle geometry computations.

### `src/gap.cpp`
Gap construction and debug string formatting.

### `src/corner.cpp`
Translation unit for `Corner`.
The class logic lives in the header and this file is intentionally minimal.

### `src/lidar_data.cpp`
Implements the metadata helper container.

### `src/follow_the_gap.cpp`
Core FTG algorithm and fallback logic.

### `src/main.cpp`
ROS 2 wrapper:
- subscriptions
- publications
- conversion from ROS messages to internal objects
- debug publishing

### `launch/follow_the_gap_v0.launch.py`
Minimal ROS 2 launch file with remappable topic names.

---

## 6. Why the debug topics are useful

These topics are very valuable for Foxglove and rosbag-based debugging.

### `/visualize_obstacles`
Use this to verify:
- Are the right obstacles arriving from `obstacle_substitution`?
- Are obstacle positions plausible?
- Is the frame correct?

### `/visualize_final_heading_angle`
Use this to verify:
- Is the selected angle stable?
- Does the heading point into the expected gap?

### `/visualize_largest_gap`
Use this to verify:
- Which gap was chosen?
- Are the left and right gap borders correct?

### `/gap_found`
Use this to detect:
- planner failures
- narrow passages
- poor obstacle preprocessing

---

## 7. Foxglove usage

Recommended panels:

### 3D panel
Show:
- `/visualize_obstacles`
- `/visualize_final_heading_angle`
- `/visualize_largest_gap`

### Plot panel
Show:
- `/final_heading_angle.data`
- `/gap_found.data`

### Raw messages panel
Show:
- `/obstacles`
- `/final_heading_angle`
- `/gap_found`

---

## 8. Logging with rosbag2

A practical debug recording set is:

```bash
ros2 bag record -o ftg_debug \
  /obstacles \
  /final_heading_angle \
  /gap_found \
  /visualize_obstacles \
  /visualize_final_heading_angle \
  /visualize_largest_gap \
  /lsr/angle
```

If you also want the full chain, add upstream topics such as `/scan`.

---

## 9. Build

```bash
cd /mxck2_ws
source /opt/ros/foxy/setup.bash
colcon build --symlink-install --packages-select follow_the_gap_v0
source install/setup.bash
```

---

## 10. Launch

```bash
ros2 launch follow_the_gap_v0 follow_the_gap_v0.launch.py
```

Or with remapping:

```bash
ros2 launch follow_the_gap_v0 follow_the_gap_v0.launch.py \
  obstacles_topic:=/autonomous/ftg/obstacles \
  goal_angle_topic:=/autonomous/ftg/goal_angle
```

---

## 11. Legacy notes

This package still contains some legacy concepts from older ROS / LiDAR-only
variants. In this cleanup pass these paths were **commented out** instead of
physically deleted so that the historical behavior can still be inspected later.

The active, supported ROS 2 path is the `/obstacles` input path.
