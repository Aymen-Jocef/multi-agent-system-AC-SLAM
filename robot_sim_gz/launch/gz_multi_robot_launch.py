import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import xacro


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')

    world_path = os.path.join(
        get_package_share_directory('robot_sim_gz'),
        'config',
        'sem_world.world'
    )

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

    pkg_path = get_package_share_directory('robot_sim_gz')
    xacro_file = os.path.join(pkg_path, 'description', 'robot.urdf.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()
    # I changed here for the third rogot
    robots = [
        {'name': 'robot1', 'x': '0.0',  'y': '0.0'},
        {'name': 'robot2', 'x': '-2.0', 'y': '14.0'},
        {'name': 'robot3', 'x': '-4.0', 'y': '0.0'},
    ]

    launch_actions = [
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true'
        ),
        gazebo
    ]

    for robot in robots:

        rsp = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=robot['name'],
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': use_sim_time
            }],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
            output='screen'
        )

        spawn = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', robot['name'],
                '-topic', f'/{robot["name"]}/robot_description',
                '-robot_namespace', robot['name'],
                '-x', robot['x'],
                '-y', robot['y'],
                '-z', '0.01'
            ],
            output='screen'
        )

        launch_actions.append(rsp)
        launch_actions.append(spawn)

    return LaunchDescription(launch_actions)
