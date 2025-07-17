# Copyright (c) 2023 Open Navigation LLC
# Modified by Wasif Raza

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    cleanbot_demo_dir = get_package_share_directory('cleanbot')
    mts_dept_dir = get_package_share_directory('mts_department')
    
    simulation = LaunchConfiguration('simulation')
    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')

    declare_simulation_cmd = DeclareLaunchArgument(
        'simulation',
        default_value='true',
        description='Launch in simulation mode (true) or physical robot mode (false)')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value=simulation,
        description='Use simulation (Gazebo) clock if true')

    declare_gui_cmd = DeclareLaunchArgument(
        'gui',
        default_value='false',
        description='Launch GUI coverage (true) or simple coverage (false)')

    # Parameter files based on mode
    sim_params = os.path.join(cleanbot_demo_dir, 'sim_params.yaml')
    physical_params = os.path.join(cleanbot_demo_dir, 'physical_bot_params.yaml')

    param_substitutions = {'use_sim_time': use_sim_time}
    
    # Configured parameters for simulation
    sim_configured_params = ParameterFile(
        RewrittenYaml(
            source_file=sim_params,
            root_key="",
            param_rewrites=param_substitutions,
            convert_types=True),
        allow_substs=True)
    
    # Configured parameters for physical robot
    physical_configured_params = ParameterFile(
        RewrittenYaml(
            source_file=physical_params,
            root_key="",
            param_rewrites=param_substitutions,
            convert_types=True),
        allow_substs=True)

    # Simulation-only components
    world = os.path.join(mts_dept_dir, 'worlds', 'mts_dept.world')
    sdf = os.path.join(nav2_bringup_dir, 'worlds', 'waffle.model')

    start_gazebo_server_cmd = ExecuteProcess(
        condition=IfCondition(simulation),
        cmd=['gzserver', '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so', world],
        cwd=[cleanbot_demo_dir], 
        output='screen')

    start_gazebo_client_cmd = ExecuteProcess(
        condition=IfCondition(simulation),
        cmd=['gzclient'],
        cwd=[cleanbot_demo_dir], 
        output='screen')

    start_gazebo_spawner_cmd = Node(
        condition=IfCondition(simulation),
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-entity', 'tb3',
            '-file', sdf,
            '-x', '0.0', '-y', '0.0', '-z', '0.0',
            '-R', '0.0', '-P', '0.0', '-Y', '0.0'])

    # Robot state publisher - only for simulation mode
    urdf = os.path.join(nav2_bringup_dir, 'urdf', 'turtlebot3_waffle.urdf')
    with open(urdf, 'r') as infp:
        robot_description = infp.read()

    start_robot_state_publisher_cmd = Node(
        condition=IfCondition(simulation),
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True,
                     'robot_description': robot_description}])

    # RViz visualization
    rviz_config = os.path.join(cleanbot_demo_dir, 'cleanbot.rviz')
    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'rviz_launch.py')),
        launch_arguments={'namespace': '', 'rviz_config': rviz_config}.items())

    # Navigation bringup - conditional parameter files
    sim_bringup_cmd = IncludeLaunchDescription(
        condition=IfCondition(simulation),
        launch_description_source=PythonLaunchDescriptionSource(
            os.path.join(cleanbot_demo_dir, 'bringup.launch.py')),
        launch_arguments={'params_file': sim_params}.items())

    physical_bringup_cmd = IncludeLaunchDescription(
        condition=UnlessCondition(simulation),
        launch_description_source=PythonLaunchDescriptionSource(
            os.path.join(cleanbot_demo_dir, 'bringup.launch.py')),
        launch_arguments={'params_file': physical_params}.items())

    # Coverage nodes - conditional based on gui argument
    gui_coverage_cmd = Node(
        condition=IfCondition(gui),
        package='cleanbot',
        executable='gui_coverage',
        emulate_tty=True,
        output='screen')

    simple_coverage_cmd = Node(
        condition=UnlessCondition(gui),
        package='cleanbot',
        executable='coverage',
        emulate_tty=True,
        output='screen')

    ld = LaunchDescription()
    ld.add_action(declare_simulation_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_gui_cmd)
    
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)
    ld.add_action(start_gazebo_spawner_cmd)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(rviz_cmd)
    ld.add_action(sim_bringup_cmd)
    ld.add_action(physical_bringup_cmd)
    ld.add_action(gui_coverage_cmd)
    ld.add_action(simple_coverage_cmd)
    
    return ld