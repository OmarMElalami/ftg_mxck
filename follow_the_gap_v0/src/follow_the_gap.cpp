#include "follow_the_gap_v0/follow_the_gap.hpp"

namespace FollowTheGap
{

namespace AngleFilter
{
size_t left_index = 0;
size_t right_index = 0;
}

float kMaxRange = 0.9f;
float g_fovAngleMax = static_cast<float>(M_PI / 2.0 + M_PI / 16.0);
float g_goal_angle = 0.0f;

Obstacle obstacle_nhol_left(kTurnRadius, static_cast<float>(M_PI / 2.0), kTurnRadius);
Obstacle obstacle_nhol_right(kTurnRadius, static_cast<float>(-M_PI / 2.0), kTurnRadius);

}  // namespace FollowTheGap

float last_final_heading_angle = 0.0f;

using namespace FollowTheGap;

static const Obstacle & FindNearestObstacle(const std::vector<Obstacle> & obstacles)
{
  return *std::min_element(
    obstacles.cbegin(), obstacles.cend(),
    [](const Obstacle & a, const Obstacle & b) {
      return a.distance < b.distance;
    });
}

float FollowTheGap::FollowTheGapMethod(
  std::vector<Obstacle> obstacles,
  LidarData * lidar_data,
  std::vector<Obstacle> & gap_borders_out)
{
  (void)lidar_data;

  std::vector<Gap> gaps;
  float final_heading_angle = 0.0f;

  std::vector<Gap>::const_iterator largest_gap;
  try {
    gaps = FindGapsAngle(obstacles);
    largest_gap = std::max_element(
      gaps.cbegin(), gaps.cend(),
      [](const Gap & a, const Gap & b) {
        return a.gap_size < b.gap_size;
      });
  } catch (const InvalidAngleException & e) {
    std::cerr << "ERROR: Invalid angle encountered when creating gap array:\n";
    std::cerr << e.what() << std::endl;
    throw NoGapFoundException("Found invalid angle.");
  }

  if (largest_gap == gaps.cend()) {
    throw NoGapFoundException("No gap found");
  }

  float gap_center_angle = 0.0f;
  try {
    gap_center_angle = CalculateGapCenterAngle(*largest_gap);
  } catch (const InvalidAngleException & e) {
    std::cerr << "ERROR: Exception occurred when calculating gap centre angle\n";
    std::cerr << e.what() << std::endl;
    throw NoGapFoundException("Found invalid gap center angle");
  } catch (const CenterOutsideGapException &) {
    std::cerr << "Centre angle was outside gap. Falling back to CalculateGapCenterAngleBasic"
              << std::endl;
    gap_center_angle = CalculateGapCenterAngleBasic(*largest_gap);
  }

  const Obstacle & nearest_obstacle = FindNearestObstacle(obstacles);
  final_heading_angle = CalculateFinalHeadingAngle(
    g_goal_angle,
    gap_center_angle,
    nearest_obstacle.distance,
    kGapWeightCoefficient);

  if (std::isnan(final_heading_angle)) {
    throw NoGapFoundException("Final heading angle was nan");
  }

  gap_borders_out.emplace_back(
    largest_gap->obstacle_left->distance_to_center,
    largest_gap->obstacle_left->angle,
    largest_gap->obstacle_left->radius);

  gap_borders_out.emplace_back(
    largest_gap->obstacle_right->distance_to_center,
    largest_gap->obstacle_right->angle,
    largest_gap->obstacle_right->radius);

  return final_heading_angle;
}

float FollowTheGap::FtgWithAngleFilter(
  std::vector<Obstacle> obstacles,
  LidarData * lidar_data,
  std::vector<Obstacle> & gap_borders_out)
{
  if (obstacles.empty()) {
    std::cerr << "No obstacles found" << std::endl;
    throw NoGapFoundException("No gap found");
  }

  std::vector<Gap> gaps;
  std::vector<Gap>::const_iterator largest_gap;
  float final_heading_angle = 0.0f;
  float gap_angle = 0.0f;

  const float filter_angle_left =
    lidar_data->angle_min + AngleFilter::left_index * lidar_data->angle_increment;
  const float filter_angle_right =
    lidar_data->angle_min + AngleFilter::right_index * lidar_data->angle_increment;

  try {
    gaps = FindGapsAngle(obstacles);
    bool ok = false;
    while (!ok && !gaps.empty()) {
      largest_gap = std::max_element(
        gaps.cbegin(), gaps.cend(),
        [](const Gap & a, const Gap & b) {
          return a.gap_size < b.gap_size;
        });

      gap_angle = CalculateGapCenterAngle(*largest_gap);
      if ((gap_angle > filter_angle_right) && (gap_angle < filter_angle_left)) {
        ok = true;
      } else {
        gaps.erase(largest_gap);
      }
    }
  } catch (const InvalidAngleException & e) {
    std::cerr << "ERROR: Invalid angle encountered when creating gap array:\n";
    std::cerr << e.what() << std::endl;
    throw NoGapFoundException("Found invalid angle.");
  }

  if ((largest_gap == gaps.cend()) || gaps.empty()) {
    throw NoGapFoundException("No gap found");
  }

  const Obstacle & nearest_obstacle = FindNearestObstacle(obstacles);
  final_heading_angle = CalculateFinalHeadingAngle(
    g_goal_angle,
    gap_angle,
    nearest_obstacle.distance,
    kGapWeightCoefficient);

  if (std::isnan(final_heading_angle)) {
    throw NoGapFoundException("Final heading angle was nan");
  }

  gap_borders_out.emplace_back(
    largest_gap->obstacle_left->distance_to_center,
    largest_gap->obstacle_left->angle,
    largest_gap->obstacle_left->radius);

  gap_borders_out.emplace_back(
    largest_gap->obstacle_right->distance_to_center,
    largest_gap->obstacle_right->angle,
    largest_gap->obstacle_right->radius);

  return final_heading_angle;
}

float FollowTheGap::FtgWeightedAverageMethod(
  std::vector<Obstacle> obstacles,
  LidarData * lidar_data)
{
  const float max_range = lidar_data->range_max;
  const float min_range = std::max(lidar_data->range_min, 2.0f);
  const float range_increment = 2.0f;

  const unsigned int num_its =
    static_cast<unsigned int>((max_range - min_range) / range_increment);
  unsigned int s = 0;
  float final_heading_angle = 0.0f;

  std::vector<Obstacle> obs_out;
  std::vector<Obstacle> gap_borders_out;

  for (unsigned int i = 1; i <= num_its; ++i) {
    kMaxRange = max_range - i * range_increment;
    float angle = 0.0f;
    try {
      FilterObstacles(obs_out, obstacles);
      angle = FollowTheGapMethod(obs_out, lidar_data, gap_borders_out);
      final_heading_angle += angle * (static_cast<float>(i) / num_its);
      s += i;
    } catch (const NoGapFoundException &) {
    }
  }

  if (s == 0U) {
    throw NoGapFoundException("");
  }

  final_heading_angle /= s;
  return final_heading_angle;
}

float FollowTheGap::FollowTheCornerMethod(
  std::vector<Obstacle> obstacles,
  std::vector<Obstacle> & gap_borders_out)
{
  Corner corner = FindCorner(obstacles);

  const float safe_corner_angle = FindSafeCornerAngle(corner);
  const Obstacle & nearest_obstacle = FindNearestObstacle(obstacles);
  const float final_heading_angle = CalculateFinalHeadingAngle(
    g_goal_angle,
    safe_corner_angle,
    nearest_obstacle.distance,
    kCornerWeightCoefficient);

  gap_borders_out.emplace_back(
    corner.obstacle_left->distance_to_center,
    corner.obstacle_left->angle,
    corner.obstacle_left->radius);

  gap_borders_out.emplace_back(
    corner.obstacle_right->distance_to_center,
    corner.obstacle_right->angle,
    corner.obstacle_right->radius);

  return final_heading_angle;
}

float FollowTheGap::FollowLargestVerticalGapMethod(
  std::vector<Obstacle> obstacles,
  std::vector<Obstacle> & gap_borders_out)
{
  std::vector<Gap> gaps = FindGapsVerticalDistance(obstacles);
  auto largest_gap = std::max_element(
    gaps.cbegin(), gaps.cend(),
    [](const Gap & a, const Gap & b) {
      return a.gap_distance < b.gap_distance;
    });

  if (largest_gap == gaps.cend()) {
    throw NoGapFoundException("");
  }

  const float final_heading_angle = FindVerticalGapSafeAngle(*largest_gap);

  gap_borders_out.emplace_back(
    largest_gap->obstacle_left->distance_to_center,
    largest_gap->obstacle_left->angle,
    largest_gap->obstacle_left->radius);

  gap_borders_out.emplace_back(
    largest_gap->obstacle_right->distance_to_center,
    largest_gap->obstacle_right->angle,
    largest_gap->obstacle_right->radius);

  return final_heading_angle;
}

std::tuple<bool, float> FollowTheGap::Callback(
  const std::vector<Obstacle> & obstacles_in,
  LidarData * lidar_data,
  std::vector<Obstacle> & obstacles_out,
  std::vector<Obstacle> & gap_borders_out)
{
  float final_heading_angle = 0.0f;
  bool ok = false;

  g_fovAngleMax = static_cast<float>(M_PI / 2.0);

  kMaxRange = 4.0f;
  while ((!ok) && (kMaxRange >= 2.0f)) {
    try {
      FilterObstacles(obstacles_out, obstacles_in);
      final_heading_angle = FollowTheGapMethod(obstacles_out, lidar_data, gap_borders_out);
      ok = true;
    } catch (const NoGapFoundException &) {
      kMaxRange = kMaxRange - 0.5f;
    }
  }

  g_fovAngleMax += static_cast<float>(M_PI / 16.0);

  if (!ok) {
    kMaxRange = 2.0f;
    g_fovAngleMax = static_cast<float>(M_PI / 2.0 + M_PI / 8.0);
    while ((!ok) && (kMaxRange >= 1.5f)) {
      try {
        FilterObstacles(obstacles_out, obstacles_in);
        final_heading_angle = FollowTheGapMethod(obstacles_out, lidar_data, gap_borders_out);
        ok = true;
      } catch (const NoGapFoundException &) {
        kMaxRange = kMaxRange - 0.5f;
      }
    }
  }

  if (!ok) {
    g_fovAngleMax = static_cast<float>(M_PI / 2.0 - M_PI / 16.0);
    while ((!ok) && (g_fovAngleMax < static_cast<float>(M_PI))) {
      kMaxRange = 3.0f;
      while ((!ok) && (kMaxRange >= 0.5f)) {
        try {
          FilterObstacles(obstacles_out, obstacles_in);
          final_heading_angle = FollowTheCornerMethod(obstacles_out, gap_borders_out);
          ok = true;
        } catch (const NoGapFoundException &) {
          kMaxRange = kMaxRange - 0.5f;
        }
      }
      g_fovAngleMax += static_cast<float>(M_PI / 32.0);
    }
  }

  if (ok) {
    last_final_heading_angle = final_heading_angle;
  } else {
    std::cerr << "No gaps found" << std::endl;
  }

  return std::make_tuple(ok, final_heading_angle);
}

std::vector<Gap> FollowTheGap::FindGapsAngle(std::vector<Obstacle> & obstacles)
{
  std::vector<Gap> gaps;

  for (size_t i = 1; i < obstacles.size(); ++i) {
    if (obstacles[i - 1].Overlaps(obstacles[i])) {
      continue;
    } else if (obstacles[i - 1].angle_right < obstacles[i].angle_left) {
      continue;
    }

    gaps.emplace_back(obstacles[i - 1], obstacles[i]);
  }

  return gaps;
}

Corner FollowTheGap::FindCorner(std::vector<Obstacle> & obstacles)
{
  std::vector<Corner> corners;

  for (size_t i = 1; i < obstacles.size(); ++i) {
    Obstacle & obstacle_left = obstacles[i - 1];
    Obstacle & obstacle_right = obstacles[i];
    const float obstacles_distance =
      obstacle_left.DistanceBetweenObstacleCentres(obstacle_right);

    if (obstacle_left.x > obstacle_right.x && (obstacles_distance > kTrackMinWidth)) {
      corners.emplace_back(obstacle_left, obstacle_right, Corner::CornerTypes::kRight);
    }

    if (obstacle_right.x > obstacle_left.x && (obstacles_distance > kTrackMinWidth)) {
      corners.emplace_back(obstacle_left, obstacle_right, Corner::CornerTypes::kLeft);
    }
  }

  if (corners.size() == 1U) {
    auto largest_corner = std::max_element(
      corners.cbegin(), corners.cend(),
      [](const Corner & a, const Corner & b) {
        return a.gap_size < b.gap_size;
      });
    return *largest_corner;
  }

  throw NoGapFoundException("No corner found");
}

float FollowTheGap::FindSafeCornerAngle(Corner & corner)
{
  float angle = 0.0f;
  float theta_d = 0.0f;

  if (corner.CornerType() == Corner::CornerTypes::kRight) {
    if (kDistanceToCorner < corner.obstacle_right->distance_to_center) {
      theta_d = std::asin(kDistanceToCorner / corner.obstacle_right->distance_to_center);
    } else {
      theta_d = static_cast<float>(M_PI / 2.0);
    }
    angle = corner.obstacle_right->angle + theta_d;
  } else if (corner.CornerType() == Corner::CornerTypes::kLeft) {
    if (kDistanceToCorner < corner.obstacle_left->distance_to_center) {
      theta_d = std::asin(kDistanceToCorner / corner.obstacle_left->distance_to_center);
    } else {
      theta_d = static_cast<float>(M_PI / 2.0);
    }
    angle = corner.obstacle_left->angle - theta_d;
  } else {
    throw std::runtime_error("FollowTheGap::FindSafeCorner else case not implemented");
  }

  return angle;
}

float FollowTheGap::FindVerticalGapSafeAngle(const Gap & gap)
{
  const Obstacle * obstacle_left = gap.obstacle_left;
  const Obstacle * obstacle_right = gap.obstacle_right;

  if (obstacle_left->distance_to_center > obstacle_right->distance_to_center) {
    return obstacle_right->angle_left;
  }
  return obstacle_left->angle_right;
}

std::vector<Gap> FollowTheGap::FindGapsVerticalDistance(std::vector<Obstacle> & obstacles)
{
  std::vector<Gap> gaps;

  for (size_t i = 1; i < obstacles.size(); ++i) {
    if (obstacles[i - 1].DistanceBetweenObstacleCentres(obstacles[i]) < kCarRadius) {
      continue;
    }
    gaps.emplace_back(obstacles[i - 1], obstacles[i]);
  }

  return gaps;
}

void FilterLoneObstacleGroups(std::vector<Obstacle> & obstacles)
{
  const float max_distance_center = 0.15f;

  for (auto it = obstacles.begin(); it != obstacles.end();) {
    if (obstacles.begin() == it) {
      if (obstacles.size() > 1U) {
        auto nx_it = std::next(it, 1);
        if (it->DistanceBetweenObstacleCentres(*nx_it) > max_distance_center) {
          it = obstacles.erase(it);
        } else {
          ++it;
        }
      } else {
        ++it;
      }
    } else if (obstacles.end() == it) {
      if (obstacles.size() > 1U) {
        auto pv_it = std::prev(it, 1);
        if (it->DistanceBetweenObstacleCentres(*pv_it) > max_distance_center) {
          it = obstacles.erase(it);
        } else {
          ++it;
        }
      } else {
        ++it;
      }
    } else {
      if (obstacles.size() >= 4U) {
        auto pv_it = std::prev(it, 1);
        auto nx1_it = std::next(it, 1);
        if (obstacles.end() == nx1_it) {
          if ((it->DistanceBetweenObstacleCentres(*pv_it) > max_distance_center) &&
              (it->DistanceBetweenObstacleCentres(*nx1_it) > max_distance_center)) {
            it = obstacles.erase(it);
          } else {
            ++it;
          }
        } else {
          auto nx2_it = std::next(it, 2);
          if ((it->DistanceBetweenObstacleCentres(*pv_it) > max_distance_center) &&
              (it->DistanceBetweenObstacleCentres(*nx1_it) < max_distance_center) &&
              (nx1_it->DistanceBetweenObstacleCentres(*nx2_it) > max_distance_center)) {
            it = obstacles.erase(it, nx1_it + 1);
          } else if ((it->DistanceBetweenObstacleCentres(*pv_it) > max_distance_center) &&
                     (it->DistanceBetweenObstacleCentres(*nx1_it) > max_distance_center)) {
            it = obstacles.erase(it);
          } else {
            ++it;
          }
        }
      } else {
        ++it;
      }
    }
  }
}

void FollowTheGap::FilterObstacles(
  std::vector<Obstacle> & obstacles,
  const std::vector<Obstacle> & obstacles_in)
{
  obstacles.clear();

  for (auto it = obstacles_in.begin(); it != obstacles_in.end(); ++it) {
    if (std::abs(it->angle) > g_fovAngleMax) {
      continue;
    } else if (it->distance_to_center > kMaxRange) {
      continue;
    } else {
      obstacles.emplace_back(*it);
    }
  }

  // FilterLoneObstacleGroups(obstacles);
}

float FollowTheGap::CalculateGapCenterAngle(const Gap & gap)
{
  const double d_1 = gap.obstacle_right->distance;
  const double d_2 = gap.obstacle_left->distance;
  const double theta_1 = std::abs(gap.angle_right);
  const double theta_2 = std::abs(gap.angle_left);
  double theta_gap_c = 0.0;

  if ((gap.angle_left >= 0.0f) && (gap.angle_right <= 0.0f)) {
    theta_gap_c = std::acos(
      (d_1 + d_2 * std::cos(theta_1 + theta_2)) /
      (std::sqrt((d_1 * d_1) + (d_2 * d_2) + 2 * d_1 * d_2 * std::cos(theta_1 + theta_2))))
      - theta_1;

    if (std::isnan(theta_gap_c)) {
      throw InvalidAngleException(
        "Gap centre angle was nan for case gap.angle_right <= 0 && gap.angle_left >= 0");
    }
  } else if (gap.angle_right >= 0.0f) {
    const double l_squared =
      (d_1 * d_1 + d_2 * d_2 - 2 * d_1 * d_2 * std::cos(theta_2 - theta_1)) / 4;
    const double h_squared = (d_1 * d_1 + d_2 * d_2 - 2 * l_squared) / 2;
    const double h = std::sqrt(h_squared);
    const double theta_x = std::acos((h_squared + d_1 * d_1 - l_squared) / (2 * h * d_1));
    theta_gap_c = theta_1 + theta_x;

    if (std::isnan(theta_gap_c)) {
      throw InvalidAngleException("Gap centre angle was nan for case gap.angle_right >= 0");
    }
  } else {
    const double l_squared =
      (d_1 * d_1 + d_2 * d_2 - 2 * d_1 * d_2 * std::cos(theta_1 - theta_2)) / 4;
    const double h_squared = (d_1 * d_1 + d_2 * d_2 - 2 * l_squared) / 2;
    const double h = std::sqrt(h_squared);
    const double theta_x = std::acos((h_squared + d_2 * d_2 - l_squared) / (2 * h * d_2));
    theta_gap_c = -(theta_2 + theta_x);

    if (std::isnan(theta_gap_c)) {
      std::stringstream error_msg;
      error_msg << "Gap centre angle was nan for case gap.angle_left <= 0";
      throw InvalidAngleException(error_msg.str());
    }
  }

  if ((theta_gap_c > gap.angle_left) || (theta_gap_c < gap.angle_right)) {
    throw CenterOutsideGapException("The calculated centre of gap was outside the gap");
  }

  if (std::isnan(theta_gap_c)) {
    throw InvalidAngleException("Gap centre angle was nan unknown case");
  }

  return static_cast<float>(theta_gap_c);
}

float FollowTheGap::CalculateGapCenterAngleBasic(const Gap & gap)
{
  return (gap.angle_right + gap.angle_left) / 2.0f;
}

float FollowTheGap::CalculateFinalHeadingAngle(
  float theta_goal,
  float theta_c,
  float d_min,
  float alpha)
{
  return ((alpha / d_min) * theta_c + theta_goal) / ((alpha / d_min) + 1.0f);
}
