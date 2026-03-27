# AUDIT_MXCK_FTG

## Scope and verification limits

Repository audited: `/home/runner/work/ftg_mxck/ftg_mxck`

Repository instruction file check:

- `AGENTS.md` was not found in this repository at audit time.
- Platform/architecture constraints were taken from the repository Copilot instructions and issue requirements.

This audit is static (code + launch + config inspection). Runtime build/test verification in this sandbox was limited because required tools were unavailable:

- `colcon` not available (`colcon: command not found`)
- `pytest` not available (`pytest: command not found`)

So this report explicitly distinguishes **verified-from-repo** findings vs **runtime assumptions that must be validated on vehicle**.

## Release/readiness status

This PR should be treated as an **audit/documentation PR with small mechanical fixes**, **not** as a full “system ready for vehicle test” release.

In particular, this PR intentionally does **not** claim closure of the following runtime/architecture items:

- potential layered speed-scaling behavior across `ctu_ftg_adapter_node` and `ftg_command_node`
- operator misuse risk of parallel planner publishers
- no functional changes in `follow_the_gap_v0/main.cpp` for `/gap_found`
- no functional changes in `follow_the_gap.h/.cpp` static/extern structure

---

## 1) Architecture summary (as implemented)

Implemented FTG integration is consistent with the MXCK architecture goal:

- Final output is produced by `mxck_ftg_control/ftg_command_node`
- Final topic is `/autonomous/ackermann_cmd`
- Message type is `ackermann_msgs/AckermannDriveStamped`
- No FTG package publishes directly to VESC topics (`/commands/*`) as control output
- `vehicle_control` is **not** launched by FTG bringup; it is expected externally

Primary launch path:

- `/home/runner/work/ftg_mxck/ftg_mxck/mxck_ftg_bringup/launch/ftg_full_system.launch.py`
  - Includes TF broadcast via `mxck_run` (optional)
  - Includes FTG stack via `ftg_stack.launch.py`
  - Logs that vehicle_control must be started separately

---

## 2) Package-level consistency audit

### 2.1 Package/folder/build consistency

All required package folders exist and package names match `package.xml` names:

1. `/home/runner/work/ftg_mxck/ftg_mxck/obstacle_msgs` → `obstacle_msgs` (ament_cmake)
2. `/home/runner/work/ftg_mxck/ftg_mxck/obstacle_substitution` → `obstacle_substitution` (ament_python)
3. `/home/runner/work/ftg_mxck/ftg_mxck/follow_the_gap_v0` → `follow_the_gap_v0` (ament_cmake)
4. `/home/runner/work/ftg_mxck/ftg_mxck/mxck_ftg_perception` → `mxck_ftg_perception` (ament_python)
5. `/home/runner/work/ftg_mxck/ftg_mxck/mxck_ftg_planner` → `mxck_ftg_planner` (ament_python)
6. `/home/runner/work/ftg_mxck/ftg_mxck/mxck_ftg_control` → `mxck_ftg_control` (ament_python)
7. `/home/runner/work/ftg_mxck/ftg_mxck/mxck_ftg_bringup` → `mxck_ftg_bringup` (ament_python)

### 2.2 Entry points and executables

Verified launched executables exist:

- `obstacle_substitution_node` → `/home/runner/work/ftg_mxck/ftg_mxck/obstacle_substitution/obstacle_substitution/obstacle_substitution_node.py`
- `follow_the_gap` target → `/home/runner/work/ftg_mxck/ftg_mxck/follow_the_gap_v0/CMakeLists.txt`
- `scan_preprocessor_node` → `/home/runner/work/ftg_mxck/ftg_mxck/mxck_ftg_perception/mxck_ftg_perception/scan_preprocessor_node.py`
- `ctu_ftg_adapter_node` → `/home/runner/work/ftg_mxck/ftg_mxck/mxck_ftg_planner/mxck_ftg_planner/ctu_ftg_adapter_node.py`
- `ftg_command_node` → `/home/runner/work/ftg_mxck/ftg_mxck/mxck_ftg_control/mxck_ftg_control/ftg_command_node.py`

Also present but not used in main stack:

- `ftg_planner_node` (legacy/alternate planner path)
- `scan_front_window_check` (diagnostic tool)

### 2.3 Dependency findings

#### Confirmed good

- `mxck_ftg_bringup/package.xml` correctly includes `mxck_run` exec dependency
  - `/home/runner/work/ftg_mxck/ftg_mxck/mxck_ftg_bringup/package.xml:17`

#### Risk / inconsistency

- `mxck_ftg_planner` Python code imports `visualization_msgs` and `tf2_ros` in `ftg_planner_node.py`, but `package.xml` only declares `rclpy`, `std_msgs`, `sensor_msgs`
  - File: `/home/runner/work/ftg_mxck/ftg_mxck/mxck_ftg_planner/mxck_ftg_planner/ftg_planner_node.py:11-12`
  - File: `/home/runner/work/ftg_mxck/ftg_mxck/mxck_ftg_planner/package.xml:13-15`
  - Impact: standalone use of `ftg_planner_node` may fail on missing runtime deps.

---

## 3) Node-level and topic-level consistency

## 3.1 Intended chain verification (explicit)

Requested chain:

`/scan -> optional preprocessing -> /obstacles -> /final_heading_angle + /gap_found -> /autonomous/ftg/gap_angle + /autonomous/ftg/target_speed -> /autonomous/ackermann_cmd`

### Implemented chain (verified)

1. `/scan` (`sensor_msgs/LaserScan`)
   - Publisher: external LiDAR stack (`mxck_run`, outside repo)

2. optional preprocessing:
   - `scan_preprocessor_node` input `/scan`
   - output `/autonomous/ftg/scan_filtered`
   - File: `/home/runner/work/ftg_mxck/ftg_mxck/mxck_ftg_perception/mxck_ftg_perception/scan_preprocessor_node.py:24-27`

3. `/obstacles` generation:
   - `obstacle_substitution_node` subscribes `/scan` and publishes `/obstacles`
   - In bringup, when preprocessor is enabled, `/scan` is remapped to `/autonomous/ftg/scan_filtered`
   - Files:
     - `/home/runner/work/ftg_mxck/ftg_mxck/obstacle_substitution/obstacle_substitution/obstacle_substitution_node.py:107-115`
     - `/home/runner/work/ftg_mxck/ftg_mxck/mxck_ftg_bringup/launch/ftg_stack.launch.py:63-72`

4. CTU FTG core:
   - `follow_the_gap` subscribes `/obstacles`
   - publishes `/final_heading_angle` (`std_msgs/Float32`) and `/gap_found` (`std_msgs/Bool`)
   - File: `/home/runner/work/ftg_mxck/ftg_mxck/follow_the_gap_v0/src/main.cpp:381-415`

5. Adapter:
   - `ctu_ftg_adapter_node` subscribes `/final_heading_angle`, `/gap_found`, `/scan`
   - publishes `/autonomous/ftg/gap_angle`, `/autonomous/ftg/target_speed`
   - File: `/home/runner/work/ftg_mxck/ftg_mxck/mxck_ftg_planner/mxck_ftg_planner/ctu_ftg_adapter_node.py:20-26, 97-118, 240-242`

6. Control:
   - `ftg_command_node` subscribes `/autonomous/ftg/gap_angle`, `/autonomous/ftg/target_speed`
   - publishes `/autonomous/ackermann_cmd` (`ackermann_msgs/AckermannDriveStamped`)
   - File: `/home/runner/work/ftg_mxck/ftg_mxck/mxck_ftg_control/mxck_ftg_control/ftg_command_node.py:15-19, 63-68, 137-146`

### Conclusion on chain

**Yes** — the repository implements the requested chain end-to-end, with one nuance:

- when preprocessor is enabled, filtered scan is only remapped into obstacle substitution
- `ctu_ftg_adapter_node` speed estimation still defaults to raw `/scan` (configurable)
  - `/home/runner/work/ftg_mxck/ftg_mxck/mxck_ftg_planner/config/ctu_ftg_adapter.yaml:5`

This is not structurally broken, but it is a behavior choice to verify during test.

---

## 4) Launch-level consistency audit

### 4.1 Main launch behavior

- `ftg_full_system.launch.py` does **not** launch `vehicle_control`
  - Explicit log statement confirms external startup expectation
  - File: `/home/runner/work/ftg_mxck/ftg_mxck/mxck_ftg_bringup/launch/ftg_full_system.launch.py:110`

- `ftg_stack.launch.py` conditionally launches exactly one obstacle_substitution node:
  - direct mode (`UnlessCondition(use_scan_preprocessor)`)
  - remapped mode (`IfCondition(use_scan_preprocessor)`)
  - same node name, but conditions are mutually exclusive
  - File: `/home/runner/work/ftg_mxck/ftg_mxck/mxck_ftg_bringup/launch/ftg_stack.launch.py:55-72`

### 4.2 Launch findings

#### Fixed in this PR

- `mxck_ftg_planner/launch/ctu_adapter.launch.py` was empty (0 bytes)
- Added proper launch wrapper for `ctu_ftg_adapter_node`
- File fixed: `/home/runner/work/ftg_mxck/ftg_mxck/mxck_ftg_planner/launch/ctu_adapter.launch.py`

#### Remaining caution

- `ftg_planner.launch.py` launches `ftg_planner_node` (alternate planner path) while main stack uses `ctu_ftg_adapter_node`
  - If an operator launches both accidentally, they can publish duplicate commands to `/autonomous/ftg/gap_angle` and `/autonomous/ftg/target_speed`
  - File: `/home/runner/work/ftg_mxck/ftg_mxck/mxck_ftg_planner/launch/ftg_planner.launch.py`

---

## 5) MXCK platform compatibility

## 5.1 Final command topic and type

Verified:

- topic: `/autonomous/ackermann_cmd`
- type: `ackermann_msgs/AckermannDriveStamped`
- set in both defaults and config
  - `/home/runner/work/ftg_mxck/ftg_mxck/mxck_ftg_control/mxck_ftg_control/ftg_command_node.py:18`
  - `/home/runner/work/ftg_mxck/ftg_mxck/mxck_ftg_control/config/ftg_control.yaml:6`

## 5.2 No bypass to VESC from FTG logic

Verified static code in FTG packages only publishes:

- `/autonomous/ackermann_cmd`
- FTG internal topics

No FTG node publishes `/commands/motor/*` or `/commands/servo/*`. Those appear only in rosbag recording list:

- `/home/runner/work/ftg_mxck/ftg_mxck/mxck_ftg_bringup/launch/ftg_full_system.launch.py:68-70`

## 5.3 vehicle_control duplication risk

- Main launch does not start `vehicle_control`
- Risk is operational (human): starting multiple command producers externally
- Mitigation: verify single publisher on `/autonomous/ackermann_cmd` before enabling autonomous mode.

---

## 6) Risk detection (pre-physical-test)

### P0 / P1 classification for first vehicle testing

#### P0 (must be resolved or explicitly accepted before first autonomous motion)

1. **Potential double speed scaling**
   - `ctu_ftg_adapter_node` computes clearance-based target speed:
     - `/home/runner/work/ftg_mxck/ftg_mxck/mxck_ftg_planner/mxck_ftg_planner/ctu_ftg_adapter_node.py:210-222`
   - `ftg_command_node` applies additional turn-based speed scaling (enabled by config):
     - `/home/runner/work/ftg_mxck/ftg_mxck/mxck_ftg_control/mxck_ftg_control/ftg_command_node.py:97-111`
     - `/home/runner/work/ftg_mxck/ftg_mxck/mxck_ftg_control/config/ftg_control.yaml:22`
   - Why P0: first on-vehicle speed behavior can be harder to predict/tune due to compounded reductions.

2. **Operational risk of duplicate FTG planner publishers if wrong launch combination is used**
   - `ftg_planner_node` and `ctu_ftg_adapter_node` can both publish `/autonomous/ftg/gap_angle` and `/autonomous/ftg/target_speed` if launched together.
   - Why P0: multiple publishers on command-producing topics can produce unstable/ambiguous behavior.

#### P1 (should be fixed before broader validation, but not necessarily a hard blocker for first controlled low-speed run)

1. **`mxck_ftg_planner/package.xml` dependency gap for alternate planner path**
   - `ftg_planner_node.py` imports `visualization_msgs` and `tf2_ros`, but package manifest does not declare them.
   - Files:
     - `/home/runner/work/ftg_mxck/ftg_mxck/mxck_ftg_planner/mxck_ftg_planner/ftg_planner_node.py:11-12`
     - `/home/runner/work/ftg_mxck/ftg_mxck/mxck_ftg_planner/package.xml:13-15`
   - Why P1: primarily affects alternate path and strict deployment/build consistency.

## High priority

1. **Potential double speed scaling**
   - `ctu_ftg_adapter_node` already computes speed from clearance thresholds
     - `/home/runner/work/ftg_mxck/ftg_mxck/mxck_ftg_planner/mxck_ftg_planner/ctu_ftg_adapter_node.py:210-222`
   - `ftg_command_node` also applies turn-based speed scaling (enabled in config)
     - `/home/runner/work/ftg_mxck/ftg_mxck/mxck_ftg_control/mxck_ftg_control/ftg_command_node.py:97-111`
     - `/home/runner/work/ftg_mxck/ftg_mxck/mxck_ftg_control/config/ftg_control.yaml:22`
   - Risk: compounded slowdown and non-obvious tuning response during test.

2. **Alternate planner path can create duplicate publishers if misused**
   - `ftg_planner_node` publishes same command topics as adapter path
     - `/home/runner/work/ftg_mxck/ftg_mxck/mxck_ftg_planner/mxck_ftg_planner/ftg_planner_node.py:71-73`
   - If launched in parallel with adapter path, behavior conflicts.

## Medium priority

3. **Dependency gap in `mxck_ftg_planner/package.xml` for legacy node path**
   - Missing `visualization_msgs`, `tf2_ros` declarations for `ftg_planner_node`
   - Impact mainly when using alternate planner launch or strict dependency checks.

4. **Startup order assumption for TF availability**
   - preprocessor/planner/diagnostic nodes use TF lookup and emit warnings if transform unavailable at startup
   - This is expected behavior but must be considered in bringup sequence.

## Low priority

5. **Stale/legacy topics in CTU core**
   - `follow_the_gap_v0` still subscribes to `/right_constraint_index`, `/left_constraint_index`, `/lsr/angle`
   - Not used in main MXCK FTG chain; mostly harmless but can confuse diagnostics.
   - File: `/home/runner/work/ftg_mxck/ftg_mxck/follow_the_gap_v0/src/main.cpp:387-403`

---

## 7) Mismatches vs intended architecture

1. Documentation in top-level README still describes an architecture where `ftg_planner_node` is core producer on `/autonomous/ftg/gap_angle` and `/autonomous/ftg/target_speed`, but launch stack uses `ctu_ftg_adapter_node`.
   - Runtime chain in code is CTU-core + adapter path.

2. `ctu_ftg_adapter_node` speed input scan topic defaults to `/scan`, not filtered scan.
   - This diverges from a strict “all downstream stages consume preprocessed scan” interpretation.
   - Not inherently incorrect, but should be treated as an explicit tuning decision.

---

## 8) Minimal safe fixes applied in this PR

1. **Added missing launch wrapper** for adapter node
   - File: `/home/runner/work/ftg_mxck/ftg_mxck/mxck_ftg_planner/launch/ctu_adapter.launch.py`
   - Change: implemented `generate_launch_description()` launching `ctu_ftg_adapter_node` with `ctu_ftg_adapter.yaml`
   - Risk: very low (mechanical consistency fix)

No other runtime behavior changes were made.

---

## 9) Recommended fixes by priority (post-audit)

1. **Before physical testing:** choose one speed-scaling stage only (adapter or control) and document the choice.
2. Add missing planner package dependencies (`visualization_msgs`, `tf2_ros`) if `ftg_planner_node` remains in package.
3. Harden operator procedure to avoid launching both planner paths simultaneously.
4. Align README architecture section with actual launched stack (`ctu_ftg_adapter_node`).
5. (Optional cleanup) remove or clearly label legacy CTU auxiliary subscriptions/topics not used in MXCK pipeline.

---

## 10) What could not be fully verified from repository alone

- Actual runtime presence/behavior of external `mxck_run`, `vehicle_control`, and `ackermann_to_vesc` in target deployment
- Real hardware steering sign/scale correctness
- Effective safety/deadman interaction under RC mode switching
- Actual topic rates and timing stability under load

These must be validated on target platform before autonomous driving.

---

## 11) Second-pass strict review (PR comment follow-up)

This section is a strict re-check focused on:

1. ROS2 package/dependency correctness
2. launch-file consistency
3. topic graph consistency
4. executable-name correctness
5. YAML parameter mismatches
6. MXCK vehicle-control integration
7. duplicate control logic
8. duplicate turn-speed scaling
9. duplicate `vehicle_control` startup
10. safety blockers for first vehicle test

### 11.1 Results summary

- **Dependency correctness:** one confirmed packaging issue in `mxck_ftg_planner` (now fixed in this PR): missing runtime deps for imported message/TF modules.
- **Launch consistency:** main FTG launch wiring is internally consistent; node names/executables resolve.
- **Topic graph:** requested chain is consistent in default stack (`follow_the_gap` → `ctu_ftg_adapter_node` → `ftg_command_node`).
- **Executable names:** launch executables match `setup.py` / CMake targets.
- **YAML params:** no key mismatches found for active stack configs.
- **MXCK integration:** FTG stack outputs only `/autonomous/ackermann_cmd`; `vehicle_control` remains external as required.
- **Duplicate control logic / turn-speed scaling:** no duplicate implementation of turn-scaling logic; there is layered speed policy (adapter clearance policy + control-node turn scaling) that remains a tuning/safety consideration.
- **Duplicate `vehicle_control` startup:** not present in FTG launches.

### 11.2 Confirmed local fix applied from second pass

- Added missing `mxck_ftg_planner/package.xml` runtime deps:
  - `geometry_msgs`
  - `visualization_msgs`
  - `tf2_ros`

This aligns package metadata with actual imports in:

- `mxck_ftg_planner/mxck_ftg_planner/ftg_planner_node.py`
- `mxck_ftg_planner/mxck_ftg_planner/common.py`

### 11.3 Conservative safety note

No new architecture changes were applied in second pass. The previously flagged P0/P1 operational checks still apply and should be enforced during first-vehicle bringup (single command publisher verification, timeout-stop behavior, and steering-direction sanity check at low speed).

### 11.4 Raw package.xml integrity check

`mxck_ftg_planner/package.xml` was re-checked in raw form after dependency edits:

- XML parse result: valid
- closing `</package>` tag count: exactly 1

No duplicate trailing closing tag is present.
