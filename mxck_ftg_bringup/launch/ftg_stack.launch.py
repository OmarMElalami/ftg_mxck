from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import AndCondition, IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    start_tf = LaunchConfiguration('start_tf')
    use_scan_preprocessor = LaunchConfiguration('use_scan_preprocessor')
    start_ctu_ftg = LaunchConfiguration('start_ctu_ftg')
    start_adapter = LaunchConfiguration('start_adapter')
    use_ftg_planner = LaunchConfiguration('use_ftg_planner')
    start_control = LaunchConfiguration('start_control')

    tf_launch = PathJoinSubstitution([
        FindPackageShare('mxck_run'),
        'launch',
        'broadcast_tf_launch.py',
    ])

    preprocessor_cfg = PathJoinSubstitution([
        FindPackageShare('mxck_ftg_perception'),
        'config',
        'scan_preprocessor.yaml',
    ])

    adapter_cfg = PathJoinSubstitution([
        FindPackageShare('mxck_ftg_planner'),
        'config',
        'ctu_ftg_adapter.yaml',
    ])

    control_cfg = PathJoinSubstitution([
        FindPackageShare('mxck_ftg_control'),
        'config',
        'ftg_control.yaml',
    ])

    tf_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(tf_launch),
        condition=IfCondition(start_tf),
    )

    preprocessor_node = Node(
        package='mxck_ftg_perception',
        executable='scan_preprocessor_node',
        name='scan_preprocessor_node',
        output='screen',
        parameters=[preprocessor_cfg],
        condition=IfCondition(use_scan_preprocessor),
    )

    obstacle_substitution_direct = Node(
        package='obstacle_substitution',
        executable='obstacle_substitution_node',
        name='obstacle_substitution',
        output='screen',
        # CTU path without scan preprocessor
        condition=AndCondition([
            IfCondition(start_ctu_ftg),
            UnlessCondition(use_scan_preprocessor),
            UnlessCondition(use_ftg_planner),
        ]),
    )

    obstacle_substitution_filtered = Node(
        package='obstacle_substitution',
        executable='obstacle_substitution_node',
        name='obstacle_substitution',
        output='screen',
        remappings=[
            ('/scan', '/autonomous/ftg/scan_filtered'),
        ],
        # CTU path with scan preprocessor
        condition=AndCondition([
            IfCondition(start_ctu_ftg),
            IfCondition(use_scan_preprocessor),
            UnlessCondition(use_ftg_planner),
        ]),
    )

    ctu_ftg_node = Node(
        package='follow_the_gap_v0',
        executable='follow_the_gap',
        name='follow_the_gap',
        output='screen',
        condition=AndCondition([
            IfCondition(start_ctu_ftg),
            UnlessCondition(use_ftg_planner),
        ]),
    )

    adapter_node = Node(
        package='mxck_ftg_planner',
        executable='ctu_ftg_adapter_node',
        name='ctu_ftg_adapter_node',
        output='screen',
        parameters=[adapter_cfg],
        condition=AndCondition([
            IfCondition(start_adapter),
            UnlessCondition(use_ftg_planner),
        ]),
    )

    planner_node = Node(
        package='mxck_ftg_planner',
        executable='ftg_planner_node',
        name='ftg_planner_node',
        output='screen',
        parameters=[PathJoinSubstitution([
            FindPackageShare('mxck_ftg_planner'),
            'config',
            'ftg_planner.yaml',
        ])],
        condition=AndCondition([
            IfCondition(start_adapter),
            IfCondition(use_scan_preprocessor),
            IfCondition(use_ftg_planner),
        ]),
    )

    control_node = Node(
        package='mxck_ftg_control',
        executable='ftg_command_node',
        name='ftg_command_node',
        output='screen',
        parameters=[control_cfg],
        condition=IfCondition(start_control),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'start_tf',
            default_value='true',
            description='Start TF via mxck_run/broadcast_tf_launch.py',
        ),
        DeclareLaunchArgument(
            'use_scan_preprocessor',
            default_value='true',
            description='Use scan_preprocessor_node before obstacle_substitution (required for primary ftg_planner_node path)',
        ),
        DeclareLaunchArgument(
            'start_ctu_ftg',
            default_value='false',
            description='Start legacy CTU follow_the_gap_v0 path (use only when use_ftg_planner:=false to avoid running two planners in parallel)',
        ),
        DeclareLaunchArgument(
            'start_adapter',
            default_value='true',
            description='Start selected planner adapter path (CTU adapter or alternate ftg_planner)',
        ),
        DeclareLaunchArgument(
            'use_ftg_planner',
            default_value='true',
            description='Use primary recentered-scan ftg_planner_node instead of ctu_ftg_adapter_node',
        ),
        DeclareLaunchArgument(
            'start_control',
            default_value='true',
            description='Start ftg_command_node',
        ),

        tf_include,
        preprocessor_node,
        obstacle_substitution_direct,
        obstacle_substitution_filtered,
        ctu_ftg_node,
        adapter_node,
        planner_node,
        control_node,
    ])
