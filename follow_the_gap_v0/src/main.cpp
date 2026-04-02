#include "follow_the_gap_v0/follow_the_gap.hpp"

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int32.hpp"

#include "visualization_msgs/msg/marker.hpp"

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "obstacle_msgs/msg/obstacles_stamped.hpp"
#include "obstacle_msgs/msg/circle_obstacle.hpp"

// -----------------------------------------------------------------------------
// Legacy ROS wrapper style
// -----------------------------------------------------------------------------
// This file intentionally keeps the original free-function style wrapper so the
// cleanup pass does not change runtime behavior. A future refactor can move
// these publishers/subscribers into a dedicated rclcpp::Node subclass.

rclcpp::Node::SharedPtr node;

unsigned int kPublishMessageBufferSize = 10;
unsigned int kSubscribeMessageBufferSize = 1;

rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_final_heading_angle;
rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_gap_found;

// Debug / visualization publishers.
rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr publisher_visualize_largest_gap;
rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_visualize_final_heading_angle;
rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_visualize_obstacles;

// -----------------------------------------------------------------------------
// Utility functions
// -----------------------------------------------------------------------------

std::vector<FollowTheGap::Obstacle> CreateObstacles(
  obstacle_msgs::msg::ObstaclesStamped::ConstSharedPtr obstacles_data)
{
  const size_t data_size = obstacles_data->obstacles.circles.size();
  std::vector<FollowTheGap::Obstacle> obstacles;
  obstacles.reserve(data_size);

  for (size_t i = 0; i < data_size; ++i) {
    obstacles.emplace_back(
      obstacles_data->obstacles.circles[i].center.x,
      obstacles_data->obstacles.circles[i].center.y,
      obstacles_data->obstacles.circles[i].radius,
      FollowTheGap::kCarRadius);
  }

  // Keep sorting identical to the current behavior: leftmost obstacles first.
  std::sort(obstacles.begin(), obstacles.end());

  return obstacles;
}

#if 0
// -----------------------------------------------------------------------------
// Legacy LaserScan path (disabled)
// -----------------------------------------------------------------------------
// The current ROS 2 stack feeds this package with /obstacles, not with /scan.
// We keep the historical code here for reference during migration work.

#include "sensor_msgs/msg/laser_scan.hpp"

std::vector<FollowTheGap::Obstacle> CreateObstacles(
  sensor_msgs::msg::LaserScan::ConstSharedPtr & lidar_data)
{
  size_t const data_size = lidar_data->ranges.size();

  float const & angle_increment = lidar_data->angle_increment;
  float const & range_max = lidar_data->range_max;
  float const & range_min = lidar_data->range_min;
  float angle = lidar_data->angle_min;

  std::vector<FollowTheGap::Obstacle> obstacles;

  for (size_t i = 0; i < data_size; ++i, angle += angle_increment) {
    float const & range = lidar_data->ranges[i];

    if (std::isnan(range)) {
      continue;
    }
    if ((angle < lidar_data->angle_min) || (angle > lidar_data->angle_max)) {
      continue;
    }
    if ((range < range_min) || (range > range_max)) {
      continue;
    }

    float obstacle_radius = FollowTheGap::kCarRadius;
    obstacles.emplace_back(range, angle, obstacle_radius);
  }

  std::reverse(obstacles.begin(), obstacles.end());
  return obstacles;
}

std::vector<FollowTheGap::Obstacle> CreateObstaclesWithAngleFilter(
  sensor_msgs::msg::LaserScan::ConstSharedPtr const & lidar_data)
{
  float const & angle_increment = lidar_data->angle_increment;
  float const & range_max = lidar_data->range_max;
  float const & range_min = lidar_data->range_min;

  float angle =
    lidar_data->angle_min + FollowTheGap::AngleFilter::right_index * angle_increment;

  std::vector<FollowTheGap::Obstacle> obstacles;
  std::cerr << "right_index: " << FollowTheGap::AngleFilter::right_index << std::endl;
  std::cerr << "left_index: " << FollowTheGap::AngleFilter::left_index << std::endl;

  for (size_t i = FollowTheGap::AngleFilter::right_index;
       i <= FollowTheGap::AngleFilter::left_index;
       ++i, angle += angle_increment)
  {
    float const & range = lidar_data->ranges[i];

    if (std::isnan(range)) {
      continue;
    }
    if ((angle < lidar_data->angle_min) || (angle > lidar_data->angle_max)) {
      continue;
    }
    if ((range < range_min) || (range > range_max)) {
      continue;
    }

    float obstacle_radius = FollowTheGap::kCarRadius;
    obstacles.emplace_back(range, angle, obstacle_radius);
  }

  std::reverse(obstacles.begin(), obstacles.end());
  return obstacles;
}
#endif

// -----------------------------------------------------------------------------
// Publishers
// -----------------------------------------------------------------------------

void PublishFinalHeadingAngle(float final_heading_angle)
{
  std_msgs::msg::Float32 message;
  message.data = final_heading_angle;
  publisher_final_heading_angle->publish(message);
}

void PublishVisualizeFinalHeadingAngle(
  float final_heading_angle,
  const std::string & frame_id)
{
  geometry_msgs::msg::PoseStamped angle_message;
  angle_message.header.frame_id = frame_id;

  tf2::Quaternion angle_quaternion;
  angle_quaternion.setEuler(0.0, 0.0, final_heading_angle);
  angle_quaternion.normalize();
  tf2::convert(angle_quaternion, angle_message.pose.orientation);

  publisher_visualize_final_heading_angle->publish(angle_message);
}

void PublishVisualizeLargestGap(
  const FollowTheGap::Obstacle & gap_left,
  const FollowTheGap::Obstacle & gap_right,
  const std::string & frame_id)
{
  // Keep the historical topic format unchanged:
  // publish robot origin, then left border, then right border as PointStamped.
  geometry_msgs::msg::PointStamped robot_point;
  geometry_msgs::msg::PointStamped p0;
  geometry_msgs::msg::PointStamped p1;

  robot_point.header.frame_id = frame_id;
  robot_point.point.x = 0.0;
  robot_point.point.y = 0.0;

  p0.header.frame_id = frame_id;
  p0.point.x = gap_left.distance * std::cos(gap_left.angle_right);
  p0.point.y = gap_left.distance * std::sin(gap_left.angle_right);

  p1.header.frame_id = frame_id;
  p1.point.x = gap_right.distance * std::cos(gap_right.angle_left);
  p1.point.y = gap_right.distance * std::sin(gap_right.angle_left);

  publisher_visualize_largest_gap->publish(robot_point);
  publisher_visualize_largest_gap->publish(p0);
  publisher_visualize_largest_gap->publish(p1);
}

void PublishVisualizeObstacles(
  const std::vector<FollowTheGap::Obstacle> & obstacles,
  const std::string & frame_id)
{
  visualization_msgs::msg::Marker obstacle_points;
  obstacle_points.header.frame_id = frame_id;
  obstacle_points.type = visualization_msgs::msg::Marker::POINTS;

  obstacle_points.scale.x = 0.05;
  obstacle_points.scale.y = 0.05;
  obstacle_points.color.r = 0.0;
  obstacle_points.color.g = 1.0;
  obstacle_points.color.b = 0.0;
  obstacle_points.color.a = 1.0;

  for (const auto & o : obstacles) {
    geometry_msgs::msg::Point p;
    p.x = o.x;
    p.y = o.y;
    obstacle_points.points.push_back(p);
  }

  publisher_visualize_obstacles->publish(obstacle_points);
}

void PublishGapFound(bool gap_found)
{
  std_msgs::msg::Bool message;
  message.data = gap_found;
  publisher_gap_found->publish(message);
}

// -----------------------------------------------------------------------------
// Callbacks
// -----------------------------------------------------------------------------

void ObstaclesCallback(obstacle_msgs::msg::ObstaclesStamped::ConstSharedPtr obstacles_data)
{
  std::vector<FollowTheGap::Obstacle> obstacles = CreateObstacles(obstacles_data);

  bool ok = false;
  float angle = 0.0f;
  std::vector<FollowTheGap::Obstacle> obstacles_out;
  std::vector<FollowTheGap::Obstacle> gap_borders_out;

  // The active ROS 2 path currently uses only /obstacles, so lidar_data is null.
  std::tie(ok, angle) =
    FollowTheGap::Callback(obstacles, nullptr, obstacles_out, gap_borders_out);

  PublishGapFound(ok);

  if (ok) {
    PublishFinalHeadingAngle(angle);
    PublishVisualizeFinalHeadingAngle(angle, obstacles_data->header.frame_id);

    if (gap_borders_out.size() >= 2U) {
      PublishVisualizeLargestGap(
        gap_borders_out.at(0),
        gap_borders_out.at(1),
        obstacles_data->header.frame_id);
    }
  }

  PublishVisualizeObstacles(obstacles_out, obstacles_data->header.frame_id);
}

void GoalAngleCallback(std_msgs::msg::Float64::ConstSharedPtr message)
{
  FollowTheGap::g_goal_angle = static_cast<float>(message->data);
}

#if 0
// -----------------------------------------------------------------------------
// Legacy angle-filter callbacks (disabled)
// -----------------------------------------------------------------------------
// These are not part of the active ROS 2 path because FtgWithAngleFilter is not
// used by the current node wiring.

void AngleFilterLeftCallback(std_msgs::msg::Int32::ConstSharedPtr message) {
  FollowTheGap::AngleFilter::left_index = message->data;
}

void AngleFilterRightCallback(std_msgs::msg::Int32::ConstSharedPtr message) {
  FollowTheGap::AngleFilter::right_index = message->data;
}
#endif

// -----------------------------------------------------------------------------
// Main
// -----------------------------------------------------------------------------

int main(int argc, char ** argv)
{
  std::cout << "Warning: FilterLoneObstacleGroups is disabled." << std::endl;

  rclcpp::init(argc, argv);
  node = rclcpp::Node::make_shared("follow_the_gap");

  // Subscriptions.
  auto subscription_obstacle_data =
    node->create_subscription<obstacle_msgs::msg::ObstaclesStamped>(
      "/obstacles",
      kSubscribeMessageBufferSize,
      ObstaclesCallback);

  (void)subscription_obstacle_data;

  auto goal_angle_subscription =
    node->create_subscription<std_msgs::msg::Float64>(
      "/lsr/angle",
      kSubscribeMessageBufferSize,
      GoalAngleCallback);

  (void)goal_angle_subscription;

#if 0
  // Legacy subscriptions intentionally disabled.
  auto subscription_lidar_data = node->create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan",
    kSubscribeMessageBufferSize,
    Callback
  );

  auto subscription_angle_filter_right =
    node->create_subscription<std_msgs::msg::Int32>(
      "/right_constraint_index",
      kSubscribeMessageBufferSize,
      AngleFilterRightCallback);

  auto subscription_angle_filter_left =
    node->create_subscription<std_msgs::msg::Int32>(
      "/left_constraint_index",
      kSubscribeMessageBufferSize,
      AngleFilterLeftCallback);
#endif

  // Publishers.
  publisher_final_heading_angle =
    node->create_publisher<std_msgs::msg::Float32>(
      "/final_heading_angle",
      kPublishMessageBufferSize);

  publisher_gap_found =
    node->create_publisher<std_msgs::msg::Bool>(
      "/gap_found",
      kPublishMessageBufferSize);

  publisher_visualize_largest_gap =
    node->create_publisher<geometry_msgs::msg::PointStamped>(
      "/visualize_largest_gap",
      kPublishMessageBufferSize);

  publisher_visualize_final_heading_angle =
    node->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/visualize_final_heading_angle",
      kPublishMessageBufferSize);

  publisher_visualize_obstacles =
    node->create_publisher<visualization_msgs::msg::Marker>(
      "/visualize_obstacles",
      kPublishMessageBufferSize);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
