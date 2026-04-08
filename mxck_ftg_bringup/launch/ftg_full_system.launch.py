from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    run_scan_check = LaunchConfiguration("run_scan_check")
    scan_topic = LaunchConfiguration("scan_topic")
    filtered_scan_topic = LaunchConfiguration("filtered_scan_topic")
    goal_angle_topic = LaunchConfiguration("goal_angle_topic")

    # Compatibility arguments: kept so old commands still work.
    use_vehicle_control = LaunchConfiguration("use_vehicle_control")
    use_tf = LaunchConfiguration("use_tf")
    record_bag = LaunchConfiguration("record_bag")

    preprocessor_cfg = PathJoinSubstitution([
        FindPackageShare("mxck_ftg_perception"), "config", "scan_preprocessor.yaml"
    ])
    scan_check_cfg = PathJoinSubstitution([
        FindPackageShare("mxck_ftg_perception"), "config", "scan_front_window_check.yaml"
    ])
    planner_cfg = PathJoinSubstitution([
        FindPackageShare("mxck_ftg_planner"), "config", "ftg_planner.yaml"
    ])
    control_cfg = PathJoinSubstitution([
        FindPackageShare("mxck_ftg_control"), "config", "ftg_control.yaml"
    ])

    return LaunchDescription([
        DeclareLaunchArgument("run_scan_check", default_value="false"),
        DeclareLaunchArgument("scan_topic", default_value="/scan"),
        DeclareLaunchArgument("filtered_scan_topic", default_value="/autonomous/ftg/scan_filtered"),
        DeclareLaunchArgument("goal_angle_topic", default_value="/lsr/angle"),

        DeclareLaunchArgument("use_vehicle_control", default_value="false"),
        DeclareLaunchArgument("use_tf", default_value="false"),
        DeclareLaunchArgument("record_bag", default_value="false"),

        LogInfo(msg=["Starting MXCK FTG scan path bringup..."]),
        LogInfo(msg=["Compatibility args: use_vehicle_control=", use_vehicle_control,
                     ", use_tf=", use_tf, ", record_bag=", record_bag]),
        LogInfo(msg=["FTG path: ", scan_topic, " -> ", filtered_scan_topic,
                     " -> follow_the_gap_v0(scan) -> ftg_planner -> ftg_command"]),

        # --- Stage 1: Perception ---
        Node(
            package="mxck_ftg_perception",
            executable="scan_preprocessor_node",
            name="scan_preprocessor_node",
            output="screen",
            parameters=[
                preprocessor_cfg,
                {"scan_topic": scan_topic,
                 "filtered_scan_topic": filtered_scan_topic},
            ],
        ),

        Node(
            package="mxck_ftg_perception",
            executable="scan_front_window_check",
            name="scan_front_window_check",
            output="screen",
            condition=IfCondition(run_scan_check),
            parameters=[
                scan_check_cfg,
                {"scan_topic": scan_topic},
            ],
        ),

        # --- Stage 2: Gap detection (CTU C++ node) ---
        Node(
            package="follow_the_gap_v0",
            executable="follow_the_gap",
            name="follow_the_gap",
            output="screen",
            parameters=[{
                "input_mode": "scan",
                "scan_topic": filtered_scan_topic,
                "goal_angle_topic": goal_angle_topic,
            }],
        ),

        # --- Stage 3: Planner (heading + clearance -> gap_angle + speed) ---
        Node(
            package="mxck_ftg_planner",
            executable="ftg_planner_node",
            name="ftg_planner_node",
            output="screen",
            parameters=[planner_cfg],
        ),

        # --- Stage 4: Control (gap_angle + speed -> AckermannDriveStamped) ---
        Node(
            package="mxck_ftg_control",
            executable="ftg_command_node",
            name="ftg_command_node",
            output="screen",
            parameters=[control_cfg],
        ),
    ])
