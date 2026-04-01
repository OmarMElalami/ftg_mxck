# TOPIC_INTERFACE_CHECK

## Primary pipeline interfaces

1. Input scan:
   - Topic: `/scan`
   - Type: `sensor_msgs/LaserScan`

2. Optional preprocessing:
   - Topic out: `/autonomous/ftg/scan_filtered`
   - Type: `sensor_msgs/LaserScan`

3. Obstacle generation:
   - Topic out: `/obstacles`
   - Type: `obstacle_msgs/ObstaclesStamped`

4. CTU core output:
   - `/final_heading_angle` (`std_msgs/Float32`)
   - `/gap_found` (`std_msgs/Bool`)

5. Adapter/planner interface:
   - `/autonomous/ftg/gap_angle` (`std_msgs/Float32`)
   - `/autonomous/ftg/target_speed` (`std_msgs/Float32`)
   - `/autonomous/ftg/planner_status` (`std_msgs/String`)

6. Final command output:
   - `/autonomous/ackermann_cmd`
   - `ackermann_msgs/AckermannDriveStamped`

## Cross-check results

- [x] Final autonomous output topic/type is correct
- [x] No FTG node publishes directly to `/commands/*` VESC topics
- [x] `obstacle_substitution` expects `/scan`; preprocessor path supplies remap in bringup
- [x] Topic names in launch/config/node defaults are internally consistent for primary path
- [x] Alternate planner path is explicitly marked as non-default
