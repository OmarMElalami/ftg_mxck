# RISK_REGISTER

| Severity | Category | File(s) | Risk | Mitigation |
|---|---|---|---|---|
| P0 | runtime/safety | `mxck_ftg_bringup/launch/ftg_stack.launch.py` | Accidental dual planner path usage can create unstable planner interface publishing. | Added explicit `use_ftg_planner` switch and mutually exclusive planner node conditions. |
| P0 | runtime/safety | `mxck_ftg_control/config/ftg_control.yaml` + adapter node | Stacked speed scaling can cause confusing/tight speed behavior for first tests. | Disabled turn speed scaling by default for primary CTU path. |
| P1 | geometry/runtime | `mxck_ftg_perception/config/scan_preprocessor.yaml` | Incorrect front-angle convention can rotate perceived forward direction. | Keep explicit config and require low-speed validation procedure before vehicle test. |
| P1 | architecture/docs | `README.md` (previous state) | Documentation mismatch can lead to wrong launch/use decisions. | Rewrote README and added architecture report/checklists. |
| P2 | ops/testing | runtime process usage | Launching alternate path without intent can still happen via manual commands. | Keep alternate path clearly marked as non-default and verify publishers before test. |

