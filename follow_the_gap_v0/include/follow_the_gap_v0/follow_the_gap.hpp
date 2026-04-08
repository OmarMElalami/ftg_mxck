#ifndef FOLLOW_THE_GAP_V0__FOLLOW_THE_GAP_HPP_
#define FOLLOW_THE_GAP_V0__FOLLOW_THE_GAP_HPP_

#include "follow_the_gap_v0/corner.hpp"
#include "follow_the_gap_v0/gap.hpp"
#include "follow_the_gap_v0/lidar_data.hpp"
#include "follow_the_gap_v0/obstacle.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

namespace FollowTheGap
{

namespace AngleFilter
{
  extern size_t left_index;
  extern size_t right_index;
}

extern float kMaxRange;
extern float g_fovAngleMax;
extern float g_goal_angle;

static constexpr float kDistanceToCorner = 0.22f;
static constexpr unsigned int kPublishMessageBufferSize = 10;
static constexpr unsigned int kSubscribeMessageBufferSize = 1;
static constexpr float kTurnRadius = 0.3f;
static constexpr float kCarRadius = 0.4f;
static constexpr float kGapWeightCoefficient = 100.0f;
static constexpr float kCornerWeightCoefficient = 100.0f;
static constexpr float kFovAngle = static_cast<float>(M_PI / 4.0);
static constexpr float kTrackMinWidth = 0.35f;

extern Obstacle obstacle_nhol_left;
extern Obstacle obstacle_nhol_right;

float FollowTheGapMethod(
  std::vector<Obstacle> obstacles,
  LidarData * lidar_data,
  std::vector<Obstacle> & gap_borders_out);

float FtgWeightedAverageMethod(std::vector<Obstacle> obstacles, LidarData * lidar_data);

float FollowTheCornerMethod(
  std::vector<Obstacle> obstacles,
  std::vector<Obstacle> & gap_borders_out);

float FollowLargestVerticalGapMethod(
  std::vector<Obstacle> obstacles,
  std::vector<Obstacle> & gap_borders_out);

float FtgWithAngleFilter(
  std::vector<Obstacle> obstacles,
  LidarData * lidar_data,
  std::vector<Obstacle> & gap_borders_out);

std::vector<Gap> FindGapsAngle(std::vector<Obstacle> & obstacles);
std::vector<Gap> FindGapsVerticalDistance(std::vector<Obstacle> & obstacles);

float FindVerticalGapSafeAngle(const Gap & gap);
float CalculateGapCenterAngle(const Gap & gap);
float CalculateGapCenterAngleBasic(const Gap & gap);

float CalculateFinalHeadingAngle(
  float goal_angle,
  float gap_center_angle,
  float d_min,
  float alpha);

std::tuple<bool, float> Callback(
  const std::vector<Obstacle> & obstacles,
  LidarData * lidar_data,
  std::vector<Obstacle> & obstacles_out,
  std::vector<Obstacle> & gap_borders_out);

void FilterObstacles(
  std::vector<Obstacle> & obstacles,
  const std::vector<Obstacle> & obstacles_in);

Corner FindCorner(std::vector<Obstacle> & obstacles);
float FindSafeCornerAngle(Corner & corner);

class InvalidAngleException : public std::logic_error
{
public:
  explicit InvalidAngleException(const std::string & msg) : std::logic_error(msg) {}
};

class CenterOutsideGapException : public std::logic_error
{
public:
  explicit CenterOutsideGapException(const std::string & msg) : std::logic_error(msg) {}
};

class NoGapFoundException : public std::runtime_error
{
public:
  explicit NoGapFoundException(const std::string & msg) : std::runtime_error(msg) {}
};

}  // namespace FollowTheGap

#endif  // FOLLOW_THE_GAP_V0__FOLLOW_THE_GAP_HPP_
