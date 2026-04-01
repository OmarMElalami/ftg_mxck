# JETSON_TEST_RUNBOOK.md

Practical FTG bringup and first-motion runbook for MXCK on Jetson (ROS 2 in Docker).

---

## 1) Required assumptions

- You are on the target Jetson with the expected MXCK ROS 2 Docker environment.
- LiDAR is connected and expected to publish `/scan`.
- TF chain includes laser frame to `base_link` transform.
- `manual_control_launch.py` remains the owner of RC / Deadman / safety / `ackermann_to_vesc`.
- FTG stack will only publish `/autonomous/ackermann_cmd`.
- Vehicle is on stands or in a controlled low-risk area for first motion.

---

## 2) Build steps (inside the correct container)

```bash
cd /mxck2_ws
source /opt/ros/$ROS_DISTRO/setup.bash

colcon build --packages-select \
  obstacle_msgs obstacle_substitution follow_the_gap_v0 \
  mxck_ftg_perception mxck_ftg_planner mxck_ftg_control mxck_ftg_bringup

source /mxck2_ws/install/setup.bash
```

If build fails, fix build issues first before proceeding.

---

## 3) Start order (4 terminals)

Use 4 terminals in the same sourced container environment.

### Terminal 1 — vehicle control / manual stack

```bash
cd /mxck2_ws
source /opt/ros/$ROS_DISTRO/setup.bash
source /mxck2_ws/install/setup.bash

ros2 launch vehicle_control manual_control_launch.py
```

Expected:
- RC/deadman chain starts
- `ackermann_to_vesc` is active

### Terminal 2 — sensor availability checks

```bash
cd /mxck2_ws
source /opt/ros/$ROS_DISTRO/setup.bash
source /mxck2_ws/install/setup.bash

ros2 topic hz /scan
ros2 topic echo /scan --once
ros2 topic echo /tf_static --once
ros2 run tf2_ros tf2_echo base_link laser
```

Expected:
- `/scan` has stable frequency and valid ranges
- fixed `base_link`/laser transform is available (typically from `/tf_static`)

### Terminal 3 — FTG full system launch (primary path)

```bash
cd /mxck2_ws
source /opt/ros/$ROS_DISTRO/setup.bash
source /mxck2_ws/install/setup.bash

ros2 launch mxck_ftg_bringup ftg_full_system.launch.py \
  use_ftg_planner:=true start_ctu_ftg:=false use_scan_preprocessor:=true
```

Expected:
- `scan_preprocessor_node` running
- `ftg_planner_node` running
- `ftg_command_node` running
- no CTU adapter path nodes by default

### Terminal 4 — debug/inspection

```bash
cd /mxck2_ws
source /opt/ros/$ROS_DISTRO/setup.bash
source /mxck2_ws/install/setup.bash

ros2 node list
ros2 topic list
ros2 topic echo /autonomous/ftg/planner_status
ros2 topic echo /autonomous/ftg/control_status
ros2 topic echo /autonomous/ackermann_cmd
ros2 topic hz /autonomous/ackermann_cmd
```

---

## 4) Topics to inspect and expected behavior

### Core topics
- `/scan`
- `/autonomous/ftg/scan_filtered`
- `/autonomous/ftg/front_clearance`
- `/autonomous/ftg/gap_angle`
- `/autonomous/ftg/target_speed`
- `/autonomous/ftg/planner_status`
- `/autonomous/ftg/control_status`
- `/autonomous/ackermann_cmd`

### Good output looks like
- `/scan` active and stable
- `/autonomous/ftg/scan_filtered` active
- planner status messages update continuously without TF errors
- `/autonomous/ackermann_cmd` updates continuously when FTG active
- in clear straight corridor, steering angle trends near 0 with small corrections

### Bad output looks like
- no `/scan` or very unstable scan rate
- repeated TF lookup failure warnings in preprocessor/planner status
- `/autonomous/ackermann_cmd` missing or stale
- large constant steering bias in straight corridor
- frequent timeout stop messages without intentional input loss

---

## 5) Pre-test checklist (before moving vehicle)

- [ ] RC/deadman function verified
- [ ] emergency stop procedure verified
- [ ] no unexpected duplicate publishers on planner topics
- [ ] `/autonomous/ackermann_cmd` observed before enabling motion
- [ ] low speed limits configured
- [ ] steering direction sanity-check completed (left obstacle => steering away)

Useful checks:

```bash
ros2 topic info /autonomous/ftg/gap_angle
ros2 topic info /autonomous/ftg/target_speed
ros2 topic info /autonomous/ackermann_cmd
```

---

## 6) Low-speed first-motion procedure

1. Keep speed limits conservative.
2. Start in open space with clear escape zone.
3. Enable autonomous mode only after command topics are healthy.
4. Allow short straight roll first; verify steering sign and smoothness.
5. Introduce mild obstacle geometry and confirm avoidance direction.
6. Abort immediately on any unsafe behavior.

---

## 7) Abort / safety criteria

Abort test immediately if any occur:
- deadman/RC behavior is unclear or inconsistent
- no operator with immediate override present
- unexpected acceleration or no braking response
- steering sign appears inverted
- `/autonomous/ackermann_cmd` freezes or oscillates aggressively
- repeated TF or planner timeout faults during motion

---

## 8) Troubleshooting

### A) No `/scan`
Checks:
```bash
ros2 topic list | grep /scan
ros2 topic hz /scan
```
Actions:
- verify LiDAR driver/node is running
- verify sensor cable/power
- verify correct container/device access

### B) Wrong TF
Checks:
```bash
ros2 topic echo /tf_static --once
ros2 run tf2_ros tf2_echo base_link laser
```
Actions:
- verify TF broadcaster launch
- verify frame names match runtime expectations
- fix static transform yaw/sign before motion testing

### C) No `/autonomous/ackermann_cmd`
Checks:
```bash
ros2 node list
ros2 topic info /autonomous/ackermann_cmd
ros2 topic echo /autonomous/ftg/planner_status
ros2 topic echo /autonomous/ftg/control_status
```
Actions:
- ensure `ftg_command_node` running
- ensure planner publishes gap/speed topics
- check timeout behavior and input topic freshness

### D) Wrong steering direction
Checks:
```bash
ros2 topic echo /autonomous/ftg/gap_angle
ros2 topic echo /autonomous/ackermann_cmd
```
Actions:
- validate front-angle convention (`front_center_deg`) using front-window check
- verify steering inversion/gain settings in `ftg_control.yaml`
- run very low-speed sign test in controlled area

### E) No motor speed output
Checks:
```bash
ros2 topic echo /autonomous/ackermann_cmd
ros2 topic echo /commands/motor/speed
```
Actions:
- verify manual control stack and mode switching
- verify deadman and safety interlocks are satisfied
- verify downstream `ackermann_to_vesc` path is running

### F) Planner timeout
Checks:
```bash
ros2 topic echo /autonomous/ftg/control_status
ros2 topic hz /autonomous/ftg/gap_angle
ros2 topic hz /autonomous/ftg/target_speed
```
Actions:
- check planner input freshness (`scan_filtered`, front clearance)
- increase signal stability first; do not mask timeout without root cause
- keep `stop_on_timeout` safety behavior enabled for testing

---

## 9) Optional legacy CTU path launch

Use only when intentionally testing alternate path:

```bash
ros2 launch mxck_ftg_bringup ftg_full_system.launch.py \
  use_ftg_planner:=false start_ctu_ftg:=true
```

Do not run legacy CTU path in parallel with primary planner path during normal testing.
