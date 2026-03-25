from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_tf = LaunchConfiguration("use_tf")
    use_vehicle_control = LaunchConfiguration("use_vehicle_control")
    use_perception = LaunchConfiguration("use_perception")
    use_planner = LaunchConfiguration("use_planner")
    use_control = LaunchConfiguration("use_control")

    record_bag = LaunchConfiguration("record_bag")
    bag_dir = LaunchConfiguration("bag_dir")
    bag_name = LaunchConfiguration("bag_name")

    tf_launch = PathJoinSubstitution([
        FindPackageShare("mxck_run"),
        "launch",
        "broadcast_tf_launch.py",
    ])

    vehicle_control_launch = PathJoinSubstitution([
        FindPackageShare("vehicle_control"),
        "launch",
        "manual_control_launch.py",
    ])

    perception_launch = PathJoinSubstitution([
        FindPackageShare("mxck_ftg_perception"),
        "launch",
        "scan_preprocessor.launch.py",
    ])

    planner_launch = PathJoinSubstitution([
        FindPackageShare("mxck_ftg_planner"),
        "launch",
        "ftg_planner.launch.py",
    ])

    control_launch = PathJoinSubstitution([
        FindPackageShare("mxck_ftg_control"),
        "launch",
        "ftg_command.launch.py",
    ])

    tf_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(tf_launch),
        condition=IfCondition(use_tf),
    )

    vehicle_control_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(vehicle_control_launch),
        condition=IfCondition(use_vehicle_control),
    )

    perception_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(perception_launch),
        condition=IfCondition(use_perception),
    )

    planner_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(planner_launch),
        condition=IfCondition(use_planner),
    )

    control_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(control_launch),
        condition=IfCondition(use_control),
    )

    rosbag_record = ExecuteProcess(
        condition=IfCondition(record_bag),
        cmd=[
            "bash",
            "-lc",
            [
                "mkdir -p ", bag_dir,
                " && ros2 bag record -s mcap -o ",
                bag_dir, "/", bag_name,
                " /scan"
                " /tf"
                " /tf_static"
                " /autonomous/ftg/scan_filtered"
                " /autonomous/ftg/gap_angle"
                " /autonomous/ftg/target_speed"
                " /autonomous/ackermann_cmd"
                " /autonomous/ftg/control_status"
                " /rc/ackermann_cmd"
                " /commands/servo/position"
                " /commands/motor/speed"
                " /commands/motor/brake"
            ],
        ],
        output="screen",
        shell=False,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_tf",
            default_value="true",
            description="Start TF broadcast via mxck_run/broadcast_tf_launch.py",
        ),
        DeclareLaunchArgument(
            "use_vehicle_control",
            default_value="false",
            description=(
                "Start vehicle_control/manual_control_launch.py. "
                "Set to false if mxck2_control already starts ackermann_to_vesc / joy_to_ackermann / vesc nodes."
            ),
        ),
        DeclareLaunchArgument(
            "use_perception",
            default_value="true",
            description="Start mxck_ftg_perception",
        ),
        DeclareLaunchArgument(
            "use_planner",
            default_value="true",
            description="Start mxck_ftg_planner",
        ),
        DeclareLaunchArgument(
            "use_control",
            default_value="true",
            description="Start mxck_ftg_control",
        ),
        DeclareLaunchArgument(
            "record_bag",
            default_value="false",
            description="Record rosbag2 MCAP with important FTG topics",
        ),
        DeclareLaunchArgument(
            "bag_dir",
            default_value="/mxck2_ws/bags",
            description="Directory where bag folders are stored",
        ),
        DeclareLaunchArgument(
            "bag_name",
            default_value="ftg_run",
            description="Name of this bag session",
        ),

        LogInfo(msg="Starting MXCK FTG full system bringup..."),
        LogInfo(
            msg=(
                "Note: use_vehicle_control defaults to FALSE to avoid duplicate vehicle_control nodes."
            )
        ),

        tf_include,
        vehicle_control_include,
        perception_include,
        planner_include,
        control_include,
        rosbag_record,
    ])
