# Copyright (c) 2023 Open Navigation LLC
# Modified by Wasif Raza

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LoadComposableNodes
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')

    lifecycle_nodes = ['map_server',
                       'amcl',
                       'controller_server',
                       'planner_server',
                       'behavior_server',
                       'bt_navigator',
                       'velocity_smoother',
                       'coverage_server'
                       ]

    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    # Create our own temporary YAML files that include substitutions
    autostart = True
    use_sim_time = True
    param_substitutions = {
        'use_sim_time': str(use_sim_time),
        'autostart': str(autostart)}

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key='',
            param_rewrites=param_substitutions,
            convert_types=True),
            allow_substs=True)

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    declare_params_file_cmd = DeclareLaunchArgument('params_file')

    create_container = Node(
        name='nav2_container',
        package='rclcpp_components',
        executable='component_container_isolated',
        parameters=[configured_params, {'autostart': autostart}],
        remappings=remappings,
        output='screen')

    load_composable_nodes = LoadComposableNodes(
        target_container='nav2_container',
        composable_node_descriptions=[
            ComposableNode(
                package='nav2_map_server',
                plugin='nav2_map_server::MapServer',
                name='map_server',
                parameters=[configured_params],
                remappings=remappings),
            ComposableNode(
                package='nav2_amcl',
                plugin='nav2_amcl::AmclNode',
                name='amcl',
                parameters=[configured_params],
                remappings=remappings),
            ComposableNode(
                package='nav2_controller',
                plugin='nav2_controller::ControllerServer',
                name='controller_server',
                parameters=[configured_params],
                remappings=remappings + [('cmd_vel', 'cmd_vel_nav')]),
            ComposableNode(
                package='opennav_coverage',
                plugin='opennav_coverage::CoverageServer',
                name='coverage_server',
                parameters=[configured_params],
                remappings=remappings),
            ComposableNode(
                package='nav2_planner',
                plugin='nav2_planner::PlannerServer',
                name='planner_server',
                parameters=[configured_params],
                remappings=remappings),
            ComposableNode(
                package='nav2_behaviors',
                plugin='behavior_server::BehaviorServer',
                name='behavior_server',
                parameters=[configured_params],
                remappings=remappings),
            ComposableNode(
                package='backported_bt_navigator',
                plugin='backported_bt_navigator::BtNavigator',
                name='bt_navigator',
                parameters=[configured_params],
                remappings=remappings),
            ComposableNode(
                package='nav2_velocity_smoother',
                plugin='nav2_velocity_smoother::VelocitySmoother',
                name='velocity_smoother',
                parameters=[configured_params],
                remappings=remappings +
                           [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')]),
            ComposableNode(
                package='nav2_lifecycle_manager',
                plugin='nav2_lifecycle_manager::LifecycleManager',
                name='lifecycle_manager_navigation',
                parameters=[{'use_sim_time': use_sim_time,
                             'autostart': autostart,
                             'node_names': lifecycle_nodes}]),
        ],
    )

    # # Create the launch description and populate
    ld = LaunchDescription()
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(create_container)
    ld.add_action(load_composable_nodes)
    return ld
