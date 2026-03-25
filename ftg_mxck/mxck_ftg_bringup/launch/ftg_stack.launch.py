from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    mxck_run_share = get_package_share_directory('mxck_run')
    perception_share = get_package_share_directory('mxck_ftg_perception')
    planner_share = get_package_share_directory('mxck_ftg_planner')
    control_share = get_package_share_directory('mxck_ftg_control')

    default_tf_launch = os.path.join(mxck_run_share, 'launch', 'broadcast_tf_launch.py')
    default_scan_check_cfg = os.path.join(perception_share, 'config', 'scan_front_window_check.yaml')
    default_preprocessor_cfg = os.path.join(perception_share, 'config', 'scan_preprocessor.yaml')
    default_planner_cfg = os.path.join(planner_share, 'config', 'ftg_planner.yaml')
    default_control_cfg = os.path.join(control_share, 'config', 'ftg_control.yaml')

    start_tf_arg = DeclareLaunchArgument(
        'start_tf',
        default_value='true',
        description='Start robot_state_publisher / TF via mxck_run broadcast_tf_launch.py'
    )

    start_scan_check_arg = DeclareLaunchArgument(
        'start_scan_check',
        default_value='false',
        description='Also start scan_front_window_check debug node'
    )

    scan_check_cfg_arg = DeclareLaunchArgument(
        'scan_check_config',
        default_value=default_scan_check_cfg,
        description='YAML config for scan_front_window_check'
    )

    perception_cfg_arg = DeclareLaunchArgument(
        'perception_config',
        default_value=default_preprocessor_cfg,
        description='YAML config for scan_preprocessor_node'
    )

    planner_cfg_arg = DeclareLaunchArgument(
        'planner_config',
        default_value=default_planner_cfg,
        description='YAML config for ftg_planner_node'
    )

    control_cfg_arg = DeclareLaunchArgument(
        'control_config',
        default_value=default_control_cfg,
        description='YAML config for ftg_command_node'
    )

    tf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(default_tf_launch),
        condition=IfCondition(LaunchConfiguration('start_tf')),
    )

    scan_check_node = Node(
        package='mxck_ftg_perception',
        executable='scan_front_window_check',
        name='scan_front_window_check',
        output='screen',
        parameters=[LaunchConfiguration('scan_check_config')],
        condition=IfCondition(LaunchConfiguration('start_scan_check')),
    )

    preprocessor_node = Node(
        package='mxck_ftg_perception',
        executable='scan_preprocessor_node',
        name='scan_preprocessor_node',
        output='screen',
        parameters=[LaunchConfiguration('perception_config')],
    )

    planner_node = Node(
        package='mxck_ftg_planner',
        executable='ftg_planner_node',
        name='ftg_planner_node',
        output='screen',
        parameters=[LaunchConfiguration('planner_config')],
    )

    control_node = Node(
        package='mxck_ftg_control',
        executable='ftg_command_node',
        name='ftg_command_node',
        output='screen',
        parameters=[LaunchConfiguration('control_config')],
    )

    return LaunchDescription([
        start_tf_arg,
        start_scan_check_arg,
        scan_check_cfg_arg,
        perception_cfg_arg,
        planner_cfg_arg,
        control_cfg_arg,
        tf_launch,
        scan_check_node,
        preprocessor_node,
        planner_node,
        control_node,
    ])
