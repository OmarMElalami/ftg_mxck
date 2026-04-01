from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_tf = LaunchConfiguration('use_tf')
    use_ftg_stack = LaunchConfiguration('use_ftg_stack')
    use_scan_preprocessor = LaunchConfiguration('use_scan_preprocessor')
    start_ctu_ftg = LaunchConfiguration('start_ctu_ftg')
    use_ftg_planner = LaunchConfiguration('use_ftg_planner')
    record_bag = LaunchConfiguration('record_bag')
    bag_dir = LaunchConfiguration('bag_dir')
    bag_name = LaunchConfiguration('bag_name')

    tf_launch = PathJoinSubstitution([
        FindPackageShare('mxck_run'),
        'launch',
        'broadcast_tf_launch.py',
    ])

    ftg_stack_launch = PathJoinSubstitution([
        FindPackageShare('mxck_ftg_bringup'),
        'launch',
        'ftg_stack.launch.py',
    ])

    tf_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(tf_launch),
        condition=IfCondition(use_tf),
    )

    ftg_stack_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ftg_stack_launch),
        condition=IfCondition(use_ftg_stack),
        launch_arguments={
            'start_tf': 'false',
            'use_scan_preprocessor': use_scan_preprocessor,
            'start_ctu_ftg': start_ctu_ftg,
            'start_adapter': 'true',
            'use_ftg_planner': use_ftg_planner,
            'start_control': 'true',
        }.items(),
    )

    rosbag_record = ExecuteProcess(
        condition=IfCondition(record_bag),
        cmd=[
            'bash',
            '-lc',
            [
                'mkdir -p ', bag_dir,
                ' && ros2 bag record -s mcap -o ',
                bag_dir, '/', bag_name,
                ' /scan'
                ' /tf'
                ' /tf_static'
                ' /autonomous/ftg/scan_filtered'
                ' /obstacles'
                ' /final_heading_angle'
                ' /gap_found'
                ' /autonomous/ftg/gap_angle'
                ' /autonomous/ftg/target_speed'
                ' /autonomous/ftg/planner_status'
                ' /autonomous/ackermann_cmd'
                ' /autonomous/ftg/control_status'
                ' /rc/ackermann_cmd'
                ' /commands/servo/position'
                ' /commands/motor/speed'
                ' /commands/motor/brake'
            ],
        ],
        output='screen',
        shell=False,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_tf',
            default_value='true',
            description='Start TF broadcast'
        ),
        DeclareLaunchArgument(
            'use_ftg_stack',
            default_value='true',
            description='Start the CTU FTG stack'
        ),
        DeclareLaunchArgument(
            'use_scan_preprocessor',
            default_value='true',
            description='Use scan_preprocessor_node before obstacle_substitution'
        ),
        DeclareLaunchArgument(
            'start_ctu_ftg',
            default_value='false',
            description='Start follow_the_gap_v0 node (required only for CTU adapter path)'
        ),
        DeclareLaunchArgument(
            'use_ftg_planner',
            default_value='true',
            description='Use primary TF-aware ftg_planner_node path instead of ctu_ftg_adapter_node',
        ),
        DeclareLaunchArgument(
            'record_bag',
            default_value='false',
            description='Record rosbag2 MCAP'
        ),
        DeclareLaunchArgument(
            'bag_dir',
            default_value='/mxck2_ws/bags',
            description='Bag output directory'
        ),
        DeclareLaunchArgument(
            'bag_name',
            default_value='ctu_ftg_run',
            description='Bag session name'
        ),

        LogInfo(msg='Starting MXCK full CTU-FTG system...'),
        LogInfo(msg='vehicle_control must be started separately via manual_control_launch.py'),

        tf_include,
        ftg_stack_include,
        rosbag_record,
    ])
