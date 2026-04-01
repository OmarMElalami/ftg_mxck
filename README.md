# ftg_mxck

ROS 2 Follow-The-Gap (FTG) integration for MXCarkit that preserves the existing MXCK control chain.

Final autonomous output stays:
- Topic: `/autonomous/ackermann_cmd`
- Type: `ackermann_msgs/AckermannDriveStamped`

---

## 1) Repository purpose

This repository integrates FTG into MXCK without bypassing existing safety and actuation architecture:
- `manual_control_launch.py` remains external and owns RC / Deadman / safety / `ackermann_to_vesc`
- FTG stack runs in addition to manual stack
- In autonomous mode, MXCK vehicle-control consumes `/autonomous/ackermann_cmd`

---

## 2) Package overview

### CTU-origin packages
- `obstacle_msgs`: obstacle message definitions
- `obstacle_substitution`: LaserScan to `/obstacles` conversion
- `follow_the_gap_v0`: C++ FTG core publishing `/final_heading_angle` and `/gap_found`

### MXCK-origin FTG packages
- `mxck_ftg_perception`: scan preprocessing + perception diagnostics
- `mxck_ftg_planner`: `ftg_planner_node` (primary planner) + `ctu_ftg_adapter_node` (legacy adapter)
- `mxck_ftg_control`: `ftg_command_node` that publishes `/autonomous/ackermann_cmd`
- `mxck_ftg_bringup`: launch orchestration and optional rosbag recording

### Reference-only stack
- `line_tracking-Stack_Beispiel Code/line_tracking`
- Used only as architectural reference; not the FTG runtime path.

---

## 3) Node overview

- `scan_preprocessor_node` (`mxck_ftg_perception`)
  - Uses TF (`base_link <- scan frame`) once to define front window and recenter scan
  - Publishes `/autonomous/ftg/scan_filtered` and `/autonomous/ftg/front_clearance`
- `ftg_planner_node` (`mxck_ftg_planner`)
  - Primary planner consuming recentered/base-relative scan semantics
  - Publishes `/autonomous/ftg/gap_angle`, `/autonomous/ftg/target_speed`, `/autonomous/ftg/planner_status`
- `obstacle_substitution_node` + `follow_the_gap_v0` + `ctu_ftg_adapter_node`
  - Legacy/alternate CTU path
- `ftg_command_node` (`mxck_ftg_control`)
  - Converts planner outputs into `/autonomous/ackermann_cmd`

---

## 4) Topic / data-flow overview

### Primary production path (default)
`/scan`
-> `scan_preprocessor_node`
-> `/autonomous/ftg/scan_filtered` + `/autonomous/ftg/front_clearance`
-> `ftg_planner_node`
-> `/autonomous/ftg/gap_angle` + `/autonomous/ftg/target_speed` + `/autonomous/ftg/planner_status`
-> `ftg_command_node`
-> `/autonomous/ackermann_cmd`

### Legacy / alternate CTU path
`/scan`
-> `[optional] scan_preprocessor_node`
-> `obstacle_substitution`
-> `/obstacles`
-> `follow_the_gap_v0`
-> `/final_heading_angle` + `/gap_found`
-> `ctu_ftg_adapter_node`
-> `/autonomous/ftg/gap_angle` + `/autonomous/ftg/target_speed`
-> `ftg_command_node`
-> `/autonomous/ackermann_cmd`

---

## 5) Build commands

From your ROS 2 workspace root:

```bash
colcon build --packages-select \
  obstacle_msgs obstacle_substitution follow_the_gap_v0 \
  mxck_ftg_perception mxck_ftg_planner mxck_ftg_control mxck_ftg_bringup
source install/setup.bash
```

---

## 6) Launch commands

### Primary path (default)
```bash
ros2 launch mxck_ftg_bringup ftg_full_system.launch.py
```

### Primary path explicit
```bash
ros2 launch mxck_ftg_bringup ftg_full_system.launch.py \
  use_ftg_planner:=true start_ctu_ftg:=false use_scan_preprocessor:=true
```

### Legacy CTU path explicit
```bash
ros2 launch mxck_ftg_bringup ftg_full_system.launch.py \
  use_ftg_planner:=false start_ctu_ftg:=true
```

### Core stack only
```bash
ros2 launch mxck_ftg_bringup ftg_stack.launch.py
```

---

## 7) Debug commands

```bash
ros2 node list
ros2 topic list
ros2 topic info /autonomous/ftg/gap_angle
ros2 topic info /autonomous/ackermann_cmd
ros2 topic echo /autonomous/ftg/planner_status
ros2 topic echo /autonomous/ftg/control_status
ros2 topic echo /autonomous/ackermann_cmd
ros2 topic hz /autonomous/ackermann_cmd
ros2 run rqt_graph rqt_graph
```

Front-window diagnostics:
```bash
ros2 launch mxck_ftg_perception scan_front_window_check.launch.py
```

---

## 8) Safety notes

- Keep `manual_control_launch.py` and safety chain active separately.
- Do not bypass `ackermann_to_vesc`.
- Do not publish FTG output directly to VESC topics.
- Keep first tests low-speed, supervised, and with immediate abort readiness.
- Confirm timeout behavior (`stop_on_timeout`) before moving tests.

---

## 9) FTG integration into MXCK vehicle control

Integration boundary:
- FTG stack ends at `/autonomous/ackermann_cmd`
- Existing MXCK vehicle-control remains responsible for RC arbitration, Deadman/safety logic, and VESC actuation

This preserves MXCK operating model while allowing FTG planner experimentation upstream of the control chain.
