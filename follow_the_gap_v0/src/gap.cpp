#include "follow_the_gap_v0/gap.hpp"

#include <cmath>

namespace FollowTheGap
{

Gap::Gap(const Obstacle & o1, const Obstacle & o2)
: angle_left(o1.angle_right),
  angle_right(o2.angle_left),
  gap_size(std::abs(angle_left - angle_right)),
  gap_distance(o1.DistanceBetweenObstacleCentres(o2)),
  obstacle_left(&o1),
  obstacle_right(&o2)
{
}

std::ostream & operator<<(std::ostream & os, const Gap & g)
{
  os << "angle_left: " << g.angle_left << " ";
  os << "angle_right: " << g.angle_right << " ";
  os << "gap size: " << g.gap_size;
  return os;
}

}  // namespace FollowTheGap
