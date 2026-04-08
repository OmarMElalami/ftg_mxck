#include "follow_the_gap_v0/obstacle.hpp"

#include <cmath>

namespace FollowTheGap
{

Obstacle::Obstacle(float x_in, float y_in, float radius_in, float car_radius)
: distance_to_center(std::hypot(x_in, y_in)),
  angle(std::atan2(y_in, x_in)),
  x(x_in),
  y(y_in),
  distance(distance_to_center)
{
  (void)car_radius;

  if (radius_in > (distance - 0.01f)) {
    radius = distance - 0.01f;
  } else {
    radius = radius_in;
  }

  const float theta_d = std::asin(radius / distance);
  angle_left = angle + theta_d;
  angle_right = angle - theta_d;
  distance = std::sqrt(distance_to_center * distance_to_center - radius * radius);
}

Obstacle::Obstacle(float distance_in, float angle_in, float radius_in)
: distance_to_center(distance_in),
  angle(angle_in),
  x(distance_in * std::cos(angle)),
  y(distance_in * std::sin(angle))
{
  if (radius_in > (distance_in - 0.01f)) {
    radius = distance_in - 0.01f;
  } else {
    radius = radius_in;
  }

  const float theta_d = std::asin(radius / distance_in);
  angle_left = angle + theta_d;
  angle_right = angle - theta_d;
  distance = std::sqrt(distance_to_center * distance_to_center - radius * radius);
}

float Obstacle::DistanceBetweenObstacleEdges(const Obstacle & o) const
{
  return DistanceBetweenObstacleCentres(o) - radius - o.radius;
}

float Obstacle::DistanceBetweenObstacleCentres(const Obstacle & o) const
{
  return std::hypot(x - o.x, y - o.y);
}

bool Obstacle::Overlaps(const Obstacle & o) const
{
  return DistanceBetweenObstacleEdges(o) < 0.0f;
}

std::ostream & operator<<(std::ostream & os, const Obstacle & o)
{
  os << "angle: " << o.angle << " ";
  os << "distance: " << o.distance << " ";
  os << "x: " << o.x << " ";
  os << "y: " << o.y << " ";
  os << "radius: " << o.radius;
  return os;
}

}  // namespace FollowTheGap
