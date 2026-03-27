from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('mxck_ftg_planner')
    cfg = os.path.join(pkg_share, 'config', 'ctu_ftg_adapter.yaml')

    return LaunchDescription([
        Node(
            package='mxck_ftg_planner',
            executable='ctu_ftg_adapter_node',
            name='ctu_ftg_adapter_node',
            output='screen',
            parameters=[cfg],
        )
    ])
