# FTG Geometry / TF Analysis (Focused Correction PR)

Repository: `ftg_mxck`  
Scope: perception-to-planner geometry/frame consistency only

## Problem summary

The unstable FTG behavior (large near-constant steering and oscillation) is most likely caused by mixing scan-angle conventions and frame assumptions across the CTU and MXCK stages.

The critical mismatch was:

- raw `/scan` angles are in LiDAR frame,
- while multiple downstream stages implicitly assume vehicle-forward (`base_link`) angle semantics.

This PR makes one explicit choice and applies minimal corrective changes.

## Explicit architecture decision

**Chosen option: Option B**

Make the `scan_preprocessor_node` + `ftg_planner_node` path the **primary production path**.

Primary pipeline is now:

`/scan`
-> `scan_preprocessor_node`
-> `/autonomous/ftg/scan_filtered` + `/autonomous/ftg/front_clearance`
-> `ftg_planner_node`
-> `/autonomous/ftg/gap_angle` + `/autonomous/ftg/target_speed` + `/autonomous/ftg/planner_status`
-> `ftg_command_node`
-> `/autonomous/ackermann_cmd`

Legacy/alternate path kept available:

- `obstacle_substitution` -> `/obstacles` -> `follow_the_gap_v0` -> `ctu_ftg_adapter_node`

## Required analysis results

1. **How “front” is defined**
   - `scan_preprocessor.yaml` defines front via `front_center_deg` in base-frame semantics.
   - Current MXCK assumption is `front_center_deg: 135.0` and must be validated on vehicle.

2. **Mask vs recenter**
   - `scan_preprocessor_node` does true recentering (not only masking):
     - selects front window around TF-corrected front center,
     - publishes scan with angles relative to front window.

3. **`/autonomous/ftg/scan_filtered` convention**
   - It changes convention to front-relative angles.
   - This PR also sets published `frame_id` to `base_link` for consistency with recentered geometry.

4. **`obstacle_substitution` assumption**
   - It converts polar to Cartesian directly and assumes angle basis is already vehicle-consistent.
   - It does not apply TF correction internally.

5. **CTU path geometric validity**
   - Valid only if fed geometrically consistent scan assumptions end-to-end.
   - As primary path for MXCK, it is less robust to LiDAR mounting/frame mismatches.

6. **LiDAR mounting / TF handling**
   - In primary path, TF mounting-yaw correction is applied in `scan_preprocessor_node` (`base_link <- scan frame`) before publishing `/autonomous/ftg/scan_filtered`.
   - `ftg_planner_node` consumes base-relative/recentered scan semantics (`frame_id=base_link`) and does not re-apply mounting-yaw correction in this mode.

7. **Planner robustness contract**
   - Primary-path robustness to mounting yaw comes from preprocessor TF correction + recentering.
   - Planner scoring then operates directly on base-relative/front-relative scan angles.

8. **Primary path decision**
   - Switched primary to `scan_preprocessor_node` + `ftg_planner_node` (`use_ftg_planner:=true` by default).

9. **Duplicate speed/steering semantics**
   - Primary path avoids CTU adapter speed logic entirely.
   - Turn-based speed scaling in `ftg_command_node` remains configurable and should stay disabled unless explicitly re-tuned.

10. **Likely root cause of unstable steering**
    - Frame-angle mismatch (laser-frame angle interpreted as vehicle-forward angle), producing constant steering bias and unstable corrections.

## What was changed in code

1. `mxck_ftg_bringup/launch/ftg_stack.launch.py`
   - Default `use_ftg_planner` -> `true`
   - Default `start_ctu_ftg` -> `false`
   - `obstacle_substitution` starts only for CTU path (`use_ftg_planner=false`)

2. `mxck_ftg_bringup/launch/ftg_full_system.launch.py`
   - Added `start_ctu_ftg` launch argument pass-through
   - Default `use_ftg_planner` -> `true`
   - Default `start_ctu_ftg` -> `false`

3. `mxck_ftg_perception/mxck_ftg_perception/scan_preprocessor_node.py`
   - Published filtered scan frame set to `base_link` (matches recentered geometry semantics)
   - Status message now includes `published_frame`

4. `mxck_ftg_perception/config/scan_preprocessor.yaml`
   - Clarified MXCK-specific `front_center_deg` assumption and required on-vehicle validation

5. `mxck_ftg_planner/config/ftg_planner.yaml`
   - Added concise comment documenting primary-path geometry assumptions

## What was wrong before

- Primary bringup default path relied on CTU chain assumptions that are sensitive to laser-vs-base angle interpretation.
- Recentered scan semantics and message frame metadata were inconsistent (`scan_filtered` remained in laser frame id).
- This mismatch can produce large constant steering commands.

## Why this correction is correct

- It preserves required MXCK control chain and final output topic/type.
- It selects a path with explicit single-stage TF correction in perception, followed by planner operation in base-relative scan semantics.
- It aligns filtered-scan metadata with actual geometric interpretation.
- It is minimal and local (launch defaults + frame metadata + documentation).

## Still required on Jetson/container/vehicle

Runtime validation is required outside this sandbox:

1. Verify TF chain (`base_link` ↔ laser frame) is stable and correct.
2. Validate `front_center_deg` against real mounting.
3. Run low-speed steering sign check in controlled space.
4. Confirm `/autonomous/ackermann_cmd` behavior through existing RC/Deadman flow.
