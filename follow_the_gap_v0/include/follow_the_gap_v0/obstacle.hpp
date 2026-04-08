#ifndef FOLLOW_THE_GAP_V0__OBSTACLE_HPP_
#define FOLLOW_THE_GAP_V0__OBSTACLE_HPP_

#include <iostream>

namespace FollowTheGap
{

struct Obstacle
{
  Obstacle(float x, float y, float radius, float car_radius);
  Obstacle(float distance, float angle, float radius);

  float DistanceBetweenObstacleEdges(const Obstacle & o) const;
  float DistanceBetweenObstacleCentres(const Obstacle & o) const;
  bool Overlaps(const Obstacle & o) const;

  float distance_to_center;
  float angle;
  float angle_left;
  float angle_right;
  float radius;
  float x;
  float y;
  float distance;

  bool operator<(const Obstacle & obs) const
  {
    return this->angle > obs.angle;
  }
};

std::ostream & operator<<(std::ostream & os, const Obstacle & o);

}  // namespace FollowTheGap

#endif  // FOLLOW_THE_GAP_V0__OBSTACLE_HPP_
