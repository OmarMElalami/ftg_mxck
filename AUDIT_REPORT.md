# AUDIT_REPORT

## Summary

This audit reviewed the full FTG repository and compared it with the in-repo `line_tracking` reference stack.

Primary conclusions:

- Required MXCK output contract is preserved: `/autonomous/ackermann_cmd` (`AckermannDriveStamped`).
- CTU core remains separate C++ package.
- `obstacle_substitution` remains separate Python package.
- Main inconsistencies were architecture clarity and launch-time ambiguity (primary vs alternate planner path), plus layered speed-scaling defaults.

## Severity-coded findings

### P0

1. **Planner-path ambiguity risk**  
   - Impact: architecture/runtime/safety  
   - File: `mxck_ftg_bringup/launch/ftg_stack.launch.py`  
   - Reason: alternate planner path existed without explicit mutual-exclusion switch in the main stack launch.  
   - Fix: added `use_ftg_planner` switch with mutually exclusive conditions.

2. **Stacked speed policies by default**  
   - Impact: runtime/safety  
   - Files:
     - `mxck_ftg_planner/mxck_ftg_planner/ctu_ftg_adapter_node.py`
     - `mxck_ftg_control/config/ftg_control.yaml`  
   - Reason: CTU adapter computes clearance-based speed while control turn-scaling was enabled by default.  
   - Fix: default `enable_turn_speed_scaling` set to `false` for primary CTU path.

### P1

1. **Documentation mismatch with runtime architecture**  
   - Impact: architecture/launch/topic  
   - File: `README.md` (old)  
   - Reason: legacy/alternate path described as primary, causing operator confusion.  
   - Fix: README rewritten to reflect actual primary and alternate paths.

2. **Geometry assumptions need explicit validation guidance**  
   - Impact: runtime/safety  
   - Files:
     - `mxck_ftg_perception/config/scan_preprocessor.yaml`
     - `mxck_ftg_perception/mxck_ftg_perception/scan_preprocessor_node.py`  
   - Reason: front-angle convention is vehicle/setup dependent.  
   - Fix: clarified in README/architecture docs and pre-test checklist.

### P2

1. **Legacy/diagnostic components not clearly marked in docs**  
   - Impact: architecture/docs  
   - Files: multiple docs  
   - Fix: explicit separation in rewritten documentation.

## Comparison with line_tracking reference

Useful principles adopted:

- Explicit launch toggles
- One clear primary runtime path
- Better node/topic documentation
- Stronger config and usage clarity

Not copied:

- line-tracking implementation logic (vision/CNN/controller details)

## Validation done in this environment

- `package.xml` XML syntax check for all packages
- Python syntax check for launch files and FTG Python nodes
- Static launch/topic/executable cross-checks

Not available in sandbox:

- `colcon` runtime build/test toolchain
- Full ROS2 runtime graph/launch execution

