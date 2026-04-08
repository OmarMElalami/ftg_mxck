#ifndef FOLLOW_THE_GAP_V0__GAP_HPP_
#define FOLLOW_THE_GAP_V0__GAP_HPP_

#include "follow_the_gap_v0/obstacle.hpp"

#include <iostream>

namespace FollowTheGap
{

class Gap
{
public:
  Gap(const Obstacle & o1, const Obstacle & o2);

  float angle_left;
  float angle_right;
  float gap_size;
  float gap_distance;
  const Obstacle * obstacle_left;
  const Obstacle * obstacle_right;
};

std::ostream & operator<<(std::ostream & os, const Gap & g);

}  // namespace FollowTheGap

#endif  // FOLLOW_THE_GAP_V0__GAP_HPP_
