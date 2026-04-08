from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("mxck_ftg_perception")
    check_cfg = os.path.join(pkg_share, "config", "scan_front_window_check.yaml")
    prep_cfg = os.path.join(pkg_share, "config", "scan_preprocessor.yaml")

    run_scan_check = LaunchConfiguration("run_scan_check")

    return LaunchDescription([
        DeclareLaunchArgument("run_scan_check", default_value="false"),
        Node(
            package="mxck_ftg_perception",
            executable="scan_preprocessor_node",
            name="scan_preprocessor_node",
            output="screen",
            parameters=[prep_cfg],
        ),
        Node(
            package="mxck_ftg_perception",
            executable="scan_front_window_check",
            name="scan_front_window_check",
            output="screen",
            parameters=[check_cfg],
            condition=IfCondition(run_scan_check),
        ),
    ])
