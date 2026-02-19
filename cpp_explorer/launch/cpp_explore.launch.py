from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('cpp_explorer')
    default_graph_path = os.path.join(pkg_share, 'config', 'graph.yaml')

    # Declare launch arguments
    declare_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='robot1',
        description='Robot namespace'
    )
    declare_graph_yaml = DeclareLaunchArgument(
        'graph_yaml_path',
        default_value=default_graph_path,
        description='Path to the graph YAML file'
    )
    declare_start_node = DeclareLaunchArgument(
        'start_node',
        default_value='1',
        description='Starting node ID for CPP route'
    )
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    namespace = LaunchConfiguration('namespace')
    graph_yaml_path = LaunchConfiguration('graph_yaml_path')
    start_node = LaunchConfiguration('start_node')
    use_sim_time = LaunchConfiguration('use_sim_time')

    cpp_explorer_node = Node(
        package='cpp_explorer',
        executable='cpp_optimized',
        name='cpp_explorer_node',
        output='screen',
        namespace=namespace,
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_namespace': namespace,
            'graph_yaml_path': graph_yaml_path,
            'start_node': start_node,
            'map_frame': 'map',
            'odom_frame': 'odom',
            'base_frame': 'base_link'
        }]
    )

    return LaunchDescription([
        declare_namespace,
        declare_graph_yaml,
        declare_start_node,
        declare_use_sim_time,
        cpp_explorer_node
    ])
