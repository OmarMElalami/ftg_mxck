#ifndef FOLLOW_THE_GAP_V0__OBSTACLE_HPP_
#define FOLLOW_THE_GAP_V0__OBSTACLE_HPP_

#include <iostream>

namespace FollowTheGap
{

/**
 * @brief Circular obstacle used by the FTG algorithm.
 *
 * The obstacle can be constructed from:
 * - Cartesian coordinates and radius, or
 * - Polar coordinates (distance, angle) and radius.
 *
 * Important note about car_radius:
 * Historically one constructor accepted a `car_radius` argument. The current
 * algorithm behavior does not use that parameter anymore. We keep the signature
 * for compatibility and document the unused argument in the implementation.
 */
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
    // Reverse sort by angle so that higher angle (more left) comes first.
    return this->angle > obs.angle;
  }
};

std::ostream & operator<<(std::ostream & os, const Obstacle & o);

}  // namespace FollowTheGap

#endif  // FOLLOW_THE_GAP_V0__OBSTACLE_HPP_
