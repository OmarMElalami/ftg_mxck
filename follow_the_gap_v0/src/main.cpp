#include "follow_the_gap_v0/follow_the_gap.hpp"

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float64.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"

#include "visualization_msgs/msg/marker.hpp"

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "obstacle_msgs/msg/obstacles_stamped.hpp"
#include "obstacle_msgs/msg/circle_obstacle.hpp"

#include <algorithm>
#include <cctype>
#include <string>
#include <vector>

rclcpp::Node::SharedPtr node;

unsigned int kPublishMessageBufferSize = 10;
unsigned int kSubscribeMessageBufferSize = 1;

rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_final_heading_angle;
rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_gap_found;
rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr publisher_visualize_largest_gap;
rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_visualize_final_heading_angle;
rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_visualize_obstacles;

std::string g_input_mode = "scan";
std::string g_scan_topic = "/autonomous/ftg/scan_filtered";
std::string g_obstacles_topic = "/obstacles";
std::string g_goal_angle_topic = "/lsr/angle";

namespace
{

std::string ToLowerCopy(const std::string & value)
{
  std::string output = value;
  std::transform(
    output.begin(), output.end(), output.begin(),
    [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return output;
}

}  // namespace

std::vector<FollowTheGap::Obstacle> CreateObstacles(
  sensor_msgs::msg::LaserScan::ConstSharedPtr lidar_data)
{
  const size_t data_size = lidar_data->ranges.size();

  const float & angle_increment = lidar_data->angle_increment;
  const float & range_max = lidar_data->range_max;
  const float & range_min = lidar_data->range_min;
  float angle = lidar_data->angle_min;

  std::vector<FollowTheGap::Obstacle> obstacles;
  obstacles.reserve(data_size);

  for (size_t i = 0; i < data_size; ++i, angle += angle_increment) {
    const float & range = lidar_data->ranges[i];

    if (std::isnan(range)) {
      continue;
    }

    if ((angle < lidar_data->angle_min) || (angle > lidar_data->angle_max)) {
      continue;
    }

    if ((range < range_min) || (range > range_max)) {
      continue;
    }

    const float obstacle_radius = FollowTheGap::kCarRadius;
    obstacles.emplace_back(range, angle, obstacle_radius);
  }

  std::reverse(obstacles.begin(), obstacles.end());
  return obstacles;
}

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

  std::sort(obstacles.begin(), obstacles.end());
  return obstacles;
}

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

void ProcessFtgResult(
  const std::vector<FollowTheGap::Obstacle> & obstacles,
  LidarData * lidar_data,
  const std::string & frame_id)
{
  bool ok = false;
  float angle = 0.0f;
  std::vector<FollowTheGap::Obstacle> obstacles_out;
  std::vector<FollowTheGap::Obstacle> gap_borders_out;

  std::tie(ok, angle) =
    FollowTheGap::Callback(obstacles, lidar_data, obstacles_out, gap_borders_out);

  PublishGapFound(ok);

  if (ok) {
    PublishFinalHeadingAngle(angle);
    PublishVisualizeFinalHeadingAngle(angle, frame_id);

    if (gap_borders_out.size() >= 2U) {
      PublishVisualizeLargestGap(
        gap_borders_out.at(0),
        gap_borders_out.at(1),
        frame_id);
    }
  }

  PublishVisualizeObstacles(obstacles_out, frame_id);
}

void ScanCallback(sensor_msgs::msg::LaserScan::ConstSharedPtr lidar_data)
{
  LidarData ld(
    lidar_data->range_min,
    lidar_data->range_max,
    lidar_data->angle_min,
    lidar_data->angle_max,
    lidar_data->angle_increment);

  std::vector<FollowTheGap::Obstacle> obstacles = CreateObstacles(lidar_data);
  ProcessFtgResult(obstacles, &ld, lidar_data->header.frame_id);
}

void ObstaclesCallback(obstacle_msgs::msg::ObstaclesStamped::ConstSharedPtr obstacles_data)
{
  std::vector<FollowTheGap::Obstacle> obstacles = CreateObstacles(obstacles_data);
  ProcessFtgResult(obstacles, nullptr, obstacles_data->header.frame_id);
}

void GoalAngleCallback(std_msgs::msg::Float64::ConstSharedPtr message)
{
  FollowTheGap::g_goal_angle = static_cast<float>(message->data);
}

int main(int argc, char ** argv)
{
  std::cout << "Warning: FilterLoneObstacleGroups is disabled." << std::endl;

  rclcpp::init(argc, argv);
  node = rclcpp::Node::make_shared("follow_the_gap");

  node->declare_parameter<std::string>("input_mode", "scan");
  node->declare_parameter<std::string>("scan_topic", "/autonomous/ftg/scan_filtered");
  node->declare_parameter<std::string>("obstacles_topic", "/obstacles");
  node->declare_parameter<std::string>("goal_angle_topic", "/lsr/angle");

  g_input_mode = ToLowerCopy(node->get_parameter("input_mode").as_string());
  g_scan_topic = node->get_parameter("scan_topic").as_string();
  g_obstacles_topic = node->get_parameter("obstacles_topic").as_string();
  g_goal_angle_topic = node->get_parameter("goal_angle_topic").as_string();

  if ((g_input_mode != "scan") && (g_input_mode != "obstacles")) {
    RCLCPP_WARN(
      node->get_logger(),
      "Invalid input_mode='%s'. Falling back to 'scan'.",
      g_input_mode.c_str());
    g_input_mode = "scan";
  }

  RCLCPP_INFO(
    node->get_logger(),
    "Starting follow_the_gap with input_mode='%s', scan_topic='%s', obstacles_topic='%s', goal_angle_topic='%s'",
    g_input_mode.c_str(),
    g_scan_topic.c_str(),
    g_obstacles_topic.c_str(),
    g_goal_angle_topic.c_str());

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription;
  rclcpp::Subscription<obstacle_msgs::msg::ObstaclesStamped>::SharedPtr obstacle_subscription;

  if (g_input_mode == "scan") {
    scan_subscription =
      node->create_subscription<sensor_msgs::msg::LaserScan>(
        g_scan_topic,
        kSubscribeMessageBufferSize,
        ScanCallback);
  } else {
    obstacle_subscription =
      node->create_subscription<obstacle_msgs::msg::ObstaclesStamped>(
        g_obstacles_topic,
        kSubscribeMessageBufferSize,
        ObstaclesCallback);
  }

  auto goal_angle_subscription =
    node->create_subscription<std_msgs::msg::Float64>(
      g_goal_angle_topic,
      kSubscribeMessageBufferSize,
      GoalAngleCallback);

  (void)scan_subscription;
  (void)obstacle_subscription;
  (void)goal_angle_subscription;

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
