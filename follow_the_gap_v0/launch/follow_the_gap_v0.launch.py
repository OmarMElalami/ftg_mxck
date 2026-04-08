from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    input_mode = LaunchConfiguration("input_mode")
    scan_topic = LaunchConfiguration("scan_topic")
    obstacles_topic = LaunchConfiguration("obstacles_topic")
    goal_angle_topic = LaunchConfiguration("goal_angle_topic")

    return LaunchDescription([
        DeclareLaunchArgument("input_mode", default_value="scan"),
        DeclareLaunchArgument("scan_topic", default_value="/autonomous/ftg/scan_filtered"),
        DeclareLaunchArgument("obstacles_topic", default_value="/obstacles"),
        DeclareLaunchArgument("goal_angle_topic", default_value="/lsr/angle"),
        Node(
            package="follow_the_gap_v0",
            executable="follow_the_gap",
            name="follow_the_gap",
            output="screen",
            parameters=[{
                "input_mode": input_mode,
                "scan_topic": scan_topic,
                "obstacles_topic": obstacles_topic,
                "goal_angle_topic": goal_angle_topic,
            }],
        ),
    ])
