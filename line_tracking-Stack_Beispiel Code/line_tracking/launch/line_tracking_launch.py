#!/usr/bin/env python3
"""
Launch file pour le système de suivi de voie line_tracking.
Lance les noeuds de détection et de contrôle.

Nouveau : argument 'use_cnn' pour basculer entre le détecteur OpenCV et le CNN.
  ros2 launch line_tracking line_tracking_launch.py use_cnn:=true
  ros2 launch line_tracking line_tracking_launch.py use_cnn:=false   (défaut, OpenCV)
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    package_dir = get_package_share_directory('line_tracking')
    config_file = os.path.join(package_dir, 'config', 'params.yaml')

    # ── Arguments de lancement ──
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Utiliser le temps de simulation')

    debug_arg = DeclareLaunchArgument(
        'debug', default_value='true',
        description='Activer le mode debug avec visualisation')

    enable_control_arg = DeclareLaunchArgument(
        'enable_control', default_value='true',
        description='Activer le contrôle du véhicule')

    enable_recording_arg = DeclareLaunchArgument(
        'enable_recording', default_value='true',
        description="Activer l'enregistrement des images")

    enable_obstacle_detection_arg = DeclareLaunchArgument(
        'enable_obstacle_detection', default_value='true',
        description="Activer l'arrêt sécurité obstacle via profondeur")

    use_cv_bridge_arg = DeclareLaunchArgument(
        'use_cv_bridge', default_value='true',
        description='Utiliser cv_bridge pour la conversion images')

    # ── NOUVEAU : choix OpenCV vs CNN ──
    use_cnn_arg = DeclareLaunchArgument(
        'use_cnn', default_value='false',
        description='true = LaneNet CNN, false = pipeline OpenCV classique')

    # ── Configurations ──
    sim_time = LaunchConfiguration('use_sim_time')
    debug = LaunchConfiguration('debug')
    enable_control = LaunchConfiguration('enable_control')
    enable_recording = LaunchConfiguration('enable_recording')
    enable_obstacle_detection = LaunchConfiguration('enable_obstacle_detection')
    use_cv_bridge = LaunchConfiguration('use_cv_bridge')
    use_cnn = LaunchConfiguration('use_cnn')

    # ══════════════════════════════════════════════════
    # Détecteur OpenCV (lancé si use_cnn:=false)
    # ══════════════════════════════════════════════════
    lane_detector_opencv = Node(
        package='line_tracking',
        executable='lane_detector_node',
        name='lane_detector_node',
        parameters=[
            config_file,
            {
                'use_sim_time': sim_time,
                'publish_debug_image': debug,
                'publish_birdseye': debug,
                'use_cv_bridge': use_cv_bridge,
            }
        ],
        condition=UnlessCondition(use_cnn),
        output='screen',
        emulate_tty=True,
    )

    # ══════════════════════════════════════════════════
    # Détecteur CNN (lancé si use_cnn:=true)
    # ══════════════════════════════════════════════════
    lane_detector_cnn = Node(
        package='line_tracking',
        executable='lane_detector_cnn_node',
        name='lane_detector_node',        # même nom → même topic, le controller ne change pas
        parameters=[
            config_file,
            {
                'use_sim_time': sim_time,
                'publish_debug_image': debug,
                'use_cv_bridge': use_cv_bridge,
            }
        ],
        condition=IfCondition(use_cnn),
        output='screen',
        emulate_tty=True,
    )

    # ── Controller ──
    controller = Node(
        package='line_tracking',
        executable='controller_node',
        name='controller_node',
        parameters=[config_file, {'use_sim_time': sim_time}],
        condition=IfCondition(enable_control),
        output='screen',
        emulate_tty=True,
    )

    # ── Pure Pursuit ──
    pure_pursuit_node = Node(
        package='line_tracking',
        executable='pure_pursuit_node',
        name='pure_pursuit_node',
        parameters=[config_file, {'use_sim_time': sim_time}],
        condition=IfCondition(enable_control),
        output='screen',
        emulate_tty=True,
    )

    # ── Obstacle Detector ──
    obstacle_detector_node = Node(
        package='line_tracking',
        executable='obstacle_detector_node',
        name='obstacle_detector_node',
        parameters=[config_file, {'use_sim_time': sim_time}],
        condition=IfCondition(enable_obstacle_detection),
        output='screen',
        emulate_tty=True,
    )

    # ── Data Recorder ──
    data_recorder_node = Node(
        package='line_tracking',
        executable='data_recorder_node',
        name='data_recorder_node',
        parameters=[
            config_file,
            {
                'use_sim_time': sim_time,
                'recording': enable_recording,
                'use_cv_bridge': use_cv_bridge,
            }
        ],
        condition=IfCondition(enable_recording),
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        use_sim_time_arg,
        debug_arg,
        enable_control_arg,
        enable_recording_arg,
        enable_obstacle_detection_arg,
        use_cv_bridge_arg,
        use_cnn_arg,
        lane_detector_opencv,
        lane_detector_cnn,
        controller,
        pure_pursuit_node,
        obstacle_detector_node,
        data_recorder_node,
    ])
