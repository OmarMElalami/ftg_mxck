from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    obstacles_topic = LaunchConfiguration("obstacles_topic")
    goal_angle_topic = LaunchConfiguration("goal_angle_topic")
    final_heading_topic = LaunchConfiguration("final_heading_topic")
    gap_found_topic = LaunchConfiguration("gap_found_topic")
    visualize_obstacles_topic = LaunchConfiguration("visualize_obstacles_topic")
    visualize_heading_topic = LaunchConfiguration("visualize_heading_topic")
    visualize_gap_topic = LaunchConfiguration("visualize_gap_topic")

    return LaunchDescription([
        DeclareLaunchArgument("obstacles_topic", default_value="/obstacles"),
        DeclareLaunchArgument("goal_angle_topic", default_value="/lsr/angle"),
        DeclareLaunchArgument("final_heading_topic", default_value="/final_heading_angle"),
        DeclareLaunchArgument("gap_found_topic", default_value="/gap_found"),
        DeclareLaunchArgument("visualize_obstacles_topic", default_value="/visualize_obstacles"),
        DeclareLaunchArgument("visualize_heading_topic", default_value="/visualize_final_heading_angle"),
        DeclareLaunchArgument("visualize_gap_topic", default_value="/visualize_largest_gap"),
        Node(
            package="follow_the_gap_v0",
            executable="follow_the_gap",
            name="follow_the_gap",
            output="screen",
            remappings=[
                ("/obstacles", obstacles_topic),
                ("/lsr/angle", goal_angle_topic),
                ("/final_heading_angle", final_heading_topic),
                ("/gap_found", gap_found_topic),
                ("/visualize_obstacles", visualize_obstacles_topic),
                ("/visualize_final_heading_angle", visualize_heading_topic),
                ("/visualize_largest_gap", visualize_gap_topic),
            ],
        ),
    ])
