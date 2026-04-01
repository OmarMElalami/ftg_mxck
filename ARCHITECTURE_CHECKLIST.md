# ARCHITECTURE_CHECKLIST

- [x] Final autonomous output remains `/autonomous/ackermann_cmd`
- [x] Final message type remains `ackermann_msgs/AckermannDriveStamped`
- [x] FTG stack does not publish directly to VESC command topics
- [x] CTU core (`follow_the_gap_v0`) remains separate C++ package
- [x] `obstacle_substitution` remains separate Python package
- [x] Primary FTG path explicitly defined (CTU core + adapter + control)
- [x] Alternate planner path clearly marked and launch-selectable (`use_ftg_planner`)
- [x] `vehicle_control` startup remains external to FTG bringup
- [x] Documentation separates production packages vs reference stack vs legacy/alternate path
- [x] Safety-critical architecture assumptions documented
