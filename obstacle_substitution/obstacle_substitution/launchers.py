# NOTE: this is workaround for misbehaviour of symlink install
#       see the project README.md (the top-level one)
#       for more info (section [Launch files are not symlinked])
import launch
import launch.actions
import launch.substitutions
import launch_ros.actions


# see https://docs.ros.org/en/foxy/Tutorials/Launch-system.html
# see https://github.com/ros2/launch/blob/foxy/launch/doc/source/architecture.rst
# see https://docs.ros.org/en/foxy/Guides/Node-arguments.html
def generate_start_launch_description():
    # note: Always use an order collection (e.g. List NOT Set),
    #       because the order here matters!
    return launch.LaunchDescription([

        launch.actions.DeclareLaunchArgument(
            '/scan',
            default_value='/scan',
            description='Topic remapping',
        ),
        launch.actions.DeclareLaunchArgument(
            '/obstacles',
            default_value='/obstacles',
            description='Topic remapping',
        ),
        launch.actions.DeclareLaunchArgument(
            'remap',
            default_value='false',
            description='When true, use arguments above for remapping.',
        ),

        launch.actions.DeclareLaunchArgument(
            'anonymous',
            default_value='false',
            description='When true, run the node as anonymous (generate random name).',
        ),

        launch.actions.SetLaunchConfiguration(
            'node_name',
            value=launch.substitutions.AnonName(name='recognition_obstacle_sub'),
            condition=launch.conditions.IfCondition(
                launch.substitutions.LaunchConfiguration('anonymous')
            ),
        ),
        launch.actions.SetLaunchConfiguration(
            'node_name',
            value='recognition_obstacle_sub',
            condition=launch.conditions.UnlessCondition(
                launch.substitutions.LaunchConfiguration('anonymous')
            ),
        ),

        launch.actions.LogInfo(
            msg=['node_name=', launch.substitutions.LaunchConfiguration('node_name')],
        ),

        launch_ros.actions.Node(
            package='obstacle_substitution',
            executable='obstacle_substitution_node',
            output='screen',
            name=[launch.substitutions.LaunchConfiguration('node_name')],
            remappings=[
                # TODO: only if remap=true
                ('/scan', launch.substitutions.LaunchConfiguration('/scan')),
                ('/obstacles', launch.substitutions.LaunchConfiguration('/obstacles')),
            ]
        ),

    ])
