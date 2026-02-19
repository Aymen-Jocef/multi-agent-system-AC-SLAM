"""
Multi-robot bringup launch file.
Spawns multiple robots in Gazebo and launches SLAM, Nav2, and CPP Explorer for each.

Usage:
    ros2 launch robot_sim_gz multi_robot_bringup.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    GroupAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
import xacro


def generate_launch_description():
    # Package directories
    pkg_robot_sim = get_package_share_directory('robot_sim_gz')
    pkg_cpp_explorer = get_package_share_directory('cpp_explorer')

    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Declare arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    # World file path
    world_path = os.path.join(pkg_robot_sim, 'config', 'sem_world.world')

    # Launch Gazebo with the world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            )
        ),
        launch_arguments={'world': world_path}.items(),
    )

    # Robot URDF
    xacro_file = os.path.join(pkg_robot_sim, 'description', 'robot.urdf.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()

    # Robot configurations: name, spawn position, graph file
    robots = [
        {'name': 'robot1', 'x': '0.0',  'y': '0.0',  'graph': 'graph.yaml'},
        {'name': 'robot2', 'x': '-2.0', 'y': '14.0', 'graph': 'graph2.yaml'},
        {'name': 'robot3', 'x': '-4.0', 'y': '0.0',  'graph': 'graph3.yaml'},
    ]

    # Nav2 params file path
    nav2_params_file = os.path.join(pkg_robot_sim, 'config', 'nav2_params.yaml')

    # Build launch actions
    launch_actions = [
        declare_use_sim_time,
        gazebo,
    ]

    # Timing delays (in seconds)
    SLAM_DELAY = 5.0      # Wait for Gazebo to start
    NAV2_DELAY = 8.0      # Wait for SLAM to initialize
    EXPLORER_DELAY = 13.0 # Wait for Nav2 to be ready

    for robot in robots:
        robot_name = robot['name']
        graph_file = os.path.join(pkg_cpp_explorer, 'config', robot['graph'])

        # Robot State Publisher
        rsp_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=robot_name,
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': use_sim_time
            }],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
            output='screen'
        )

        # Spawn robot in Gazebo
        spawn_node = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', robot_name,
                '-topic', f'/{robot_name}/robot_description',
                '-robot_namespace', robot_name,
                '-x', robot['x'],
                '-y', robot['y'],
                '-z', '0.01'
            ],
            output='screen'
        )

        # SLAM Toolbox
        slam_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_robot_sim, 'launch', 'online_async.launch.py')
            ),
            launch_arguments={
                'use_sim_time': 'true',
                'namespace': robot_name,
            }.items()
        )

        # Nav2
        nav2_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_robot_sim, 'launch', 'navigation_launch.py')
            ),
            launch_arguments={
                'use_sim_time': 'true',
                'namespace': robot_name,
                'params_file': nav2_params_file,
            }.items()
        )

        # CPP Explorer
        cpp_explorer_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_cpp_explorer, 'launch', 'cpp_explore.launch.py')
            ),
            launch_arguments={
                'use_sim_time': 'true',
                'namespace': robot_name,
                'graph_yaml_path': graph_file,
            }.items()
        )

        # Add robot spawning immediately
        launch_actions.append(rsp_node)
        launch_actions.append(spawn_node)

        # Add SLAM with delay
        launch_actions.append(
            TimerAction(period=SLAM_DELAY, actions=[slam_launch])
        )

        # Add Nav2 with delay
        launch_actions.append(
            TimerAction(period=NAV2_DELAY, actions=[nav2_launch])
        )

        # Add CPP Explorer with delay
        launch_actions.append(
            TimerAction(period=EXPLORER_DELAY, actions=[cpp_explorer_launch])
        )

    return LaunchDescription(launch_actions)
