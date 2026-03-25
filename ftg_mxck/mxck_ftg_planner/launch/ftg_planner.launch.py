from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('mxck_ftg_planner')
    cfg = os.path.join(pkg_share, 'config', 'ftg_planner.yaml')

    return LaunchDescription([
        Node(
            package='mxck_ftg_planner',
            executable='ftg_planner_node',
            name='ftg_planner_node',
            output='screen',
            parameters=[cfg],
        )
    ])