#include "follow_the_gap_v0/lidar_data.hpp"

LidarData::LidarData(
  float range_min_in,
  float range_max_in,
  float angle_min_in,
  float angle_max_in,
  float angle_increment_in)
: range_min(range_min_in),
  range_max(range_max_in),
  angle_min(angle_min_in),
  angle_max(angle_max_in),
  angle_increment(angle_increment_in)
{
}
