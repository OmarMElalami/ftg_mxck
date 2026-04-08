#ifndef FOLLOW_THE_GAP_V0__LIDAR_DATA_HPP_
#define FOLLOW_THE_GAP_V0__LIDAR_DATA_HPP_

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
