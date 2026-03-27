# AGENTS.md — ftg_mxck

## Mission

This repository contains a Follow-The-Gap integration for the MXCarkit platform.

Your primary goal is to improve or audit this repository **without breaking the MXCK vehicle-control architecture**.

The most important requirement is:

- Final autonomous output must remain **`/autonomous/ackermann_cmd`**
- The FTG stack must integrate into the existing MXCK control chain
- Do **not** bypass `ackermann_to_vesc`
- Do **not** redesign the safety / RC / Deadman architecture unless explicitly asked

---

## Platform and runtime assumptions

This project targets **MXCarkit on NVIDIA Jetson running ROS2 in Docker**.

Expected real-world operating model:

1. `manual_control_launch.py` is started separately
2. Deadman / calibration / RC mode handling stay in the existing vehicle_control chain
3. The FTG stack is started additionally
4. When RC mode switches to Autonomous, the existing MXCK control chain consumes `/autonomous/ackermann_cmd`

You must preserve this operating model unless a task explicitly asks to change it.

---

## Target runtime pipeline

The intended FTG dataflow is:

`/scan`
-> `[optional] scan_preprocessor_node`
-> `obstacle_substitution_node`
-> `/obstacles`
-> `follow_the_gap_v0`
-> `/final_heading_angle` + `/gap_found`
-> `ctu_ftg_adapter_node`
-> `/autonomous/ftg/gap_angle` + `/autonomous/ftg/target_speed` + `/autonomous/ftg/planner_status`
-> `ftg_command_node`
-> `/autonomous/ackermann_cmd`

If the repository differs from this target, report the mismatch clearly.

---

## Repository structure assumptions

Main packages expected in this repo:

- `follow_the_gap_v0`
- `obstacle_substitution`
- `obstacle_msgs`
- `mxck_ftg_bringup`
- `mxck_ftg_control`
- `mxck_ftg_perception`
- `mxck_ftg_planner`

Expected package roles:

- `follow_the_gap_v0` stays a **C++ package**
- `obstacle_substitution` stays a **Python package**
- `mxck_ftg_planner` should only contain the **adapter node**, not a Python reimplementation of CTU FTG
- `ftg_command_node.py` is the final MXCK command adapter and should not be redesigned unless necessary

---

## Hard rules

1. Do not change the final output topic away from `/autonomous/ackermann_cmd`
2. Do not bypass MXCK vehicle control
3. Do not silently introduce new runtime architecture
4. Prefer minimal, architecture-preserving fixes
5. Be conservative with topic renames
6. Do not invent undocumented nodes or topics
7. If a fix is uncertain, document it instead of guessing
8. Always explain safety impact for launch/control changes
9. Treat duplicated speed-scaling or duplicated control logic as a serious issue
10. Treat mismatches between README, Docker/devcontainer, launch files, and runtime assumptions as first-class audit items

---

## What to audit first

Always inspect these first:

1. Topic graph consistency
   - publishers/subscribers
   - exact topic names
   - remappings
   - final control path

2. Launch consistency
   - executable names
   - package names
   - parameters
   - remappings
   - optional nodes
   - duplicated node startup
   - duplicate vehicle_control startup

3. Package/build consistency
   - `package.xml`
   - `setup.py`
   - `CMakeLists.txt`
   - build type
   - runtime dependencies
   - missing message dependencies

4. FTG algorithm integration consistency
   - `/obstacles`
   - `/final_heading_angle`
   - `/gap_found`
   - adapter output topics
   - command node expectations

5. Safety and platform consistency
   - RC / Deadman assumptions
   - autonomous output path
   - timeout behavior
   - speed scaling ownership
   - test readiness

---

## Known integration traps to check explicitly

You must explicitly verify whether the repository currently has any of these problems:

- `follow_the_gap_v0/main.cpp` only publishes `/gap_found` on failure instead of always
- mutable `static` variables are defined directly in `follow_the_gap.h`
- `ctu_ftg_adapter_node.py` duplicates turn-based speed reduction even though `ftg_command_node.py` already handles steering-based speed scaling
- adapter YAML still contains obsolete turn-speed parameters
- `obstacle_substitution` always expects `/scan` and therefore needs launch-aware remapping if a preprocessor is used
- `ftg_stack.launch.py` breaks when the preprocessor is disabled
- `ftg_full_system.launch.py` incorrectly restarts `vehicle_control`
- rosbag topics are incomplete for FTG debugging
- ROS distro / Docker / README assumptions do not match

---

## Required output style for audits

For repo-wide audits, do not start with large silent code edits.

First produce:
- `AUDIT_REPORT.md`
- `ARCHITECTURE_CHECKLIST.md`
- `DEPENDENCY_CHECKLIST.md`
- `LAUNCH_GRAPH_CHECK.md`
- `TOPIC_INTERFACE_CHECK.md`
- `RISK_REGISTER.md`

Each finding must include:
- severity: `P0`, `P1`, or `P2`
- exact file path
- exact reason
- exact fix proposal
- impact category:
  - architecture
  - dependency
  - launch
  - topic
  - build
  - runtime
  - safety

Only apply code changes automatically when the fix is:
- small
- local
- clearly correct
- architecture-preserving

If not, document first.

---

## Expected engineering behavior

- Prefer correctness over speed
- Prefer explicitness over assumptions
- Prefer small isolated commits
- Do not change many files at once without a clear reason
- If multiple alternative fixes exist, explain which one best fits MXCK and why
- For runtime-sensitive files, always mention how the change affects real-vehicle testing

---

## Definition of success

A successful audit answers:

1. Is the repository internally consistent?
2. Does it match the intended MXCK FTG architecture?
3. Is it safe enough for a first controlled integration test?
4. If not, what exact P0/P1 issues must be fixed first?
