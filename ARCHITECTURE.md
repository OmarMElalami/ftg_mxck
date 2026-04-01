# FTG Stack Architecture Analysis and Fix Record

## Scope

Repository: `/home/runner/work/ftg_mxck/ftg_mxck`  
Reference stack: `/home/runner/work/ftg_mxck/ftg_mxck/line_tracking-Stack_Beispiel Code/line_tracking`

This document captures:

- old repository state
- detected architecture/package/launch/topic issues
- applied corrections
- final intended architecture
- what still requires validation on Jetson + ROS 2 runtime

---

## Old state (before this cleanup)

1. Root README described a planner-centric path (`ftg_planner_node`) as if it were the primary stack, while bringup launched CTU core + adapter by default.
2. FTG bringup allowed implicit ambiguity between:
   - CTU path (`follow_the_gap_v0` + `ctu_ftg_adapter_node`)
   - alternate path (`ftg_planner_node`)
3. Control defaults enabled turn-based speed scaling even when CTU adapter already performed clearance-based speed selection, causing layered speed policy behavior by default.
4. Structured audit/checklist artifacts were missing.
5. Documentation did not clearly separate:
   - production FTG packages
   - reference `line_tracking` packages
   - alternate/legacy FTG paths

---

## Important detected problems and why they matter

### P0 — architecture/runtime clarity

- **Potential dual planner outputs**
  - File: `mxck_ftg_bringup/launch/ftg_stack.launch.py`
  - Why it matters: if operator mixes planner paths, same planner interface topics can receive multiple publishers, leading to unstable behavior.

- **Layered speed policy by default**
  - Files:
    - `mxck_ftg_planner/mxck_ftg_planner/ctu_ftg_adapter_node.py`
    - `mxck_ftg_control/config/ftg_control.yaml`
  - Why it matters: speed can be reduced twice (clearance policy + turn policy), which complicates tuning and first-vehicle bringup.

### P1 — documentation-to-runtime mismatch

- **README did not match actual active bringup path**
  - File: `README.md` (old)
  - Why it matters: operators may launch/interpret wrong stack behavior.

### P1 — geometry assumptions require explicit operator validation

- Files:
  - `mxck_ftg_perception/config/scan_preprocessor.yaml`
  - `mxck_ftg_perception/mxck_ftg_perception/scan_preprocessor_node.py`
- Why it matters: incorrect front-angle convention can rotate perceived forward direction and produce unsafe steering choices.

---

## What was fixed

1. **Primary vs alternate planner path made explicit in launch**
   - Updated `mxck_ftg_bringup/launch/ftg_stack.launch.py`
   - Added argument `use_ftg_planner` (default `false`)
   - Enforced mutual exclusivity:
     - `ctu_ftg_adapter_node` when `start_adapter=true` and `use_ftg_planner=false`
     - `ftg_planner_node` when `start_adapter=true` and `use_ftg_planner=true`

2. **Propagated planner-path switch in full-system launch**
   - Updated `mxck_ftg_bringup/launch/ftg_full_system.launch.py`
   - Added `use_ftg_planner` launch argument and forwarded it to `ftg_stack.launch.py`

3. **Default control policy clarified to avoid stacked speed scaling**
   - Updated `mxck_ftg_control/config/ftg_control.yaml`
   - Set `enable_turn_speed_scaling: false` by default with explicit comment about avoiding double scaling in primary CTU path.

4. **Root documentation rewritten professionally**
   - Replaced root `README.md`
   - Added clear:
     - purpose
     - package roles
     - primary pipeline
     - alternate/legacy path status
     - launch/build/debug/test guidance
     - safety and MXCK integration boundaries

5. **Structured audit/report artifacts added**
   - `AUDIT_REPORT.md`
   - `ARCHITECTURE_CHECKLIST.md`
   - `DEPENDENCY_CHECKLIST.md`
   - `LAUNCH_GRAPH_CHECK.md`
   - `TOPIC_INTERFACE_CHECK.md`
   - `RISK_REGISTER.md`

---

## Final intended architecture (after cleanup)

### Primary production pipeline (default)

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

### Alternate path (non-default)

- `ftg_planner_node` in place of CTU adapter planner stage
- selectable only via `use_ftg_planner:=true`

### MXCK control-chain boundary (preserved)

- FTG publishes only `/autonomous/ackermann_cmd` for autonomous driving output.
- Existing vehicle-control path remains downstream and external.

---

## Comparison against in-repo line_tracking reference

Adopted principles from `line_tracking`:

1. **Explicit launch toggles for alternative paths**
2. **Clear single primary runtime entrypoint**
3. **Stronger documentation of node roles and interfaces**
4. **Consistent package/launch/config presentation**

Not copied blindly:

- vision/lane-specific nodes and logic
- line-tracking parameter semantics
- perception algorithm implementation details

---

## What still must be validated on target container/vehicle

Static checks were performed in this sandbox (XML/Python syntax and config/launch consistency).  
Full ROS runtime checks still required on target Jetson container:

1. `colcon build` with full runtime dependencies
2. end-to-end topic flow under real LiDAR data
3. front-angle convention validation (`front_center_deg`)
4. steering sign validation in safe low-speed test
5. timeout/safety behavior with RC/Deadman workflow from external manual control stack

