# ftg_mxck

ROS 2 Follow-The-Gap (FTG) integration for MXCarkit, preserving the MXCK control chain and publishing final autonomous commands to:

- **Topic:** `/autonomous/ackermann_cmd`
- **Type:** `ackermann_msgs/AckermannDriveStamped`

This repository contains:

- Production FTG packages for MXCK integration
- CTU-origin FTG core packages
- A separate in-repo `line_tracking` reference stack for architecture guidance

---

## 1) Project purpose

This project integrates CTU Follow-The-Gap with MXCarkit in a way that is compatible with the existing vehicle-control architecture:

- `manual_control_launch.py` is started separately (RC / Deadman / safety / ackermann_to_vesc)
- FTG stack is started additionally
- In autonomous mode, existing downstream control consumes `/autonomous/ackermann_cmd`

The FTG stack **does not** bypass the MXCK control chain and **does not** publish FTG output directly to VESC command topics.

---

## 2) Repository/package overview

### Production FTG packages

- `mxck_ftg_bringup`  
  Launch orchestration for the FTG stack and optional rosbag recording.
- `mxck_ftg_perception`  
  Optional scan preprocessor and scan/front-window diagnostics.
- `mxck_ftg_planner`  
  CTU adapter node (primary path) and legacy alternate planner node.
- `mxck_ftg_control`  
  Final command adapter to `/autonomous/ackermann_cmd`.
- `obstacle_substitution` (CTU-origin Python)
  Converts LaserScan to `/obstacles`.
- `obstacle_msgs` (CTU-origin messages)
  Message definitions for obstacle representation.
- `follow_the_gap_v0` (CTU-origin C++)
  Core FTG algorithm publishing `/final_heading_angle` and `/gap_found`.

### Reference example package set (not production FTG path)

- `line_tracking-Stack_Beispiel Code/line_tracking`  
  Used as architectural reference only (clean launch structure, conditional paths, central params, stable `/autonomous/ackermann_cmd` integration principles).

### Legacy/alternate FTG path (explicitly marked)

- `mxck_ftg_planner/ftg_planner_node.py` and `mxck_ftg_planner/launch/ftg_planner.launch.py`  
  Alternate TF-aware planner path. Not the default production FTG path in bringup.

---

## 3) Primary active FTG pipeline

The default production path launched by bringup is:

`/scan`
→ `[optional] scan_preprocessor_node`
→ `obstacle_substitution_node`
→ `/obstacles`
→ `follow_the_gap_v0`
→ `/final_heading_angle` + `/gap_found`
→ `ctu_ftg_adapter_node`
→ `/autonomous/ftg/gap_angle` + `/autonomous/ftg/target_speed` + `/autonomous/ftg/planner_status`
→ `ftg_command_node`
→ `/autonomous/ackermann_cmd`

---

## 4) Alternate / legacy paths

### Alternate planner path

- Node: `ftg_planner_node`
- Launch: `mxck_ftg_planner/launch/ftg_planner.launch.py`
- Role: standalone TF-aware planner consuming preprocessed scan and publishing `/autonomous/ftg/gap_angle` + `/autonomous/ftg/target_speed`

This path is kept for experimentation and comparison.  
The default bringup path remains CTU core + CTU adapter.

### Diagnostics path

- `scan_front_window_check` for front-window orientation checks

---

## 5) Node overview

- `scan_preprocessor_node` (`mxck_ftg_perception`)  
  Optional TF-aware front-window scan recentering/cropping.
- `obstacle_substitution_node` (`obstacle_substitution`)  
  Converts LaserScan beams to obstacle messages (`/obstacles`).
- `follow_the_gap` (`follow_the_gap_v0`)  
  C++ FTG core.
- `ctu_ftg_adapter_node` (`mxck_ftg_planner`)  
  Converts CTU outputs into MXCK FTG planner interface topics.
- `ftg_command_node` (`mxck_ftg_control`)  
  Converts FTG planner interface to final Ackermann command.

---

## 6) Topic graph / data flow

Key production topics:

- Inputs:
  - `/scan`
  - `/tf`, `/tf_static` (when TF-aware components enabled)
- Intermediate:
  - `/autonomous/ftg/scan_filtered`
  - `/obstacles`
  - `/final_heading_angle`
  - `/gap_found`
  - `/autonomous/ftg/gap_angle`
  - `/autonomous/ftg/target_speed`
  - `/autonomous/ftg/planner_status`
  - `/autonomous/ftg/control_status`
- Final output:
  - `/autonomous/ackermann_cmd`

---

## 7) Architecture decision (primary vs alternate)

### Primary production decision

Primary remains:

- CTU C++ core (`follow_the_gap_v0`) + `ctu_ftg_adapter_node`

Reason:

- Preserves separation of CTU core package as C++
- Matches required target pipeline
- Already integrated by `mxck_ftg_bringup/launch/ftg_stack.launch.py`

### Alternate path status

- `ftg_planner_node` remains available but is documented as alternate/legacy.
- It should not be launched in parallel with the CTU adapter path unless intentionally testing alternatives.

---

## 8) Launch usage

### A) Main FTG stack

```bash
ros2 launch mxck_ftg_bringup ftg_full_system.launch.py
```

Important: `vehicle_control` remains external and should be started separately as usual.

### B) Core stack only (without full wrapper)

```bash
ros2 launch mxck_ftg_bringup ftg_stack.launch.py
```

Useful launch arguments:

- `start_tf:=true|false`
- `use_scan_preprocessor:=true|false`
- `start_ctu_ftg:=true|false`
- `start_adapter:=true|false`
- `start_control:=true|false`

### C) Alternate planner only (non-default path)

```bash
ros2 launch mxck_ftg_planner ftg_planner.launch.py
```

---

## 9) Parameter/config overview

- `mxck_ftg_perception/config/scan_preprocessor.yaml`
  - front window definition (`front_center_deg`, `front_fov_deg`)
  - range clipping and filtering
- `mxck_ftg_planner/config/ctu_ftg_adapter.yaml`
  - CTU output mapping and clearance-based speed policy
- `mxck_ftg_control/config/ftg_control.yaml`
  - steering limits, smoothing, command timeout, output limits
- `mxck_ftg_planner/config/ftg_planner.yaml`
  - alternate planner parameters (legacy/alternate path)

---

## 10) Build commands

In your ROS 2 workspace:

```bash
colcon build --packages-select \
  obstacle_msgs obstacle_substitution follow_the_gap_v0 \
  mxck_ftg_perception mxck_ftg_planner mxck_ftg_control mxck_ftg_bringup
source install/setup.bash
```

---

## 11) Runtime commands

Example sequence:

1. Start manual/control chain externally (existing MXCK process)
2. Start FTG stack:

```bash
ros2 launch mxck_ftg_bringup ftg_full_system.launch.py use_scan_preprocessor:=true
```

3. Switch mode using existing RC/autonomous mechanism

---

## 12) Debug commands

```bash
ros2 topic list
ros2 topic echo /autonomous/ftg/planner_status
ros2 topic echo /autonomous/ftg/control_status
ros2 topic echo /autonomous/ackermann_cmd
ros2 topic hz /autonomous/ackermann_cmd
ros2 node list
ros2 run rqt_graph rqt_graph
```

To inspect front window assumptions:

```bash
ros2 launch mxck_ftg_perception scan_front_window_check.launch.py
```

---

## 13) Common failure modes

- Missing TF between laser frame and `base_link` causes preprocessor/planner warnings.
- Front-angle convention mismatch (`front_center_deg`) can rotate perceived forward direction.
- Running alternate planner and CTU adapter path in parallel can create multiple publishers on planner topics.
- If input topics timeout, `ftg_command_node` commands speed 0 (expected safety behavior when `stop_on_timeout=true`).

---

## 14) Pre-test and vehicle-test procedure

Before first controlled vehicle test:

1. Validate topic graph (`ros2 node info`, `rqt_graph`)
2. Confirm only intended publishers exist for:
   - `/autonomous/ftg/gap_angle`
   - `/autonomous/ftg/target_speed`
   - `/autonomous/ackermann_cmd`
3. Confirm steering sign and heading convention in low-speed safe area
4. Validate timeout stop behavior by temporarily pausing planner inputs
5. Record bag for traceability during tests

---

## 15) Safety notes

- Keep RC/Deadman/safety architecture unchanged in manual control stack.
- Do not bypass `ackermann_to_vesc`.
- Keep FTG output limited to `/autonomous/ackermann_cmd`.
- Run initial tests with low speed limits and supervised emergency-stop readiness.

---

## 16) FTG integration into MXCK control chain

Integration boundary is explicit:

- FTG stack responsibility ends at publishing `/autonomous/ackermann_cmd`.
- Existing MXCK vehicle-control stack remains responsible for RC arbitration, deadman logic, and hardware actuation path to VESC.

For full audit findings, architecture checks, and risk classification, see:

- `AUDIT_REPORT.md`
- `ARCHITECTURE.md`
- `ARCHITECTURE_CHECKLIST.md`
- `DEPENDENCY_CHECKLIST.md`
- `LAUNCH_GRAPH_CHECK.md`
- `TOPIC_INTERFACE_CHECK.md`
- `RISK_REGISTER.md`
