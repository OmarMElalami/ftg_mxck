#ifndef FOLLOW_THE_GAP_V0__LIDAR_DATA_HPP_
#define FOLLOW_THE_GAP_V0__LIDAR_DATA_HPP_

/**
 * @brief Lightweight container for LiDAR sweep metadata.
 *
 * This helper is kept for compatibility with older FTG variants that operate
 * directly on LaserScan-derived obstacle points. The current ROS 2 path uses
 * /obstacles as input, but the type is still part of the algorithm API.
 */
class LidarData
{
public:
  float range_min;
  float range_max;

  float angle_min;
  float angle_max;

  float angle_increment;

  LidarData(
    float range_min_in,
    float range_max_in,
    float angle_min_in,
    float angle_max_in,
    float angle_increment_in);
};

#endif  // FOLLOW_THE_GAP_V0__LIDAR_DATA_HPP_
