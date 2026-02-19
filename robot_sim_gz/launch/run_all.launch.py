# import os
# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch.actions import IncludeLaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch_ros.actions import Node



# def generate_launch_description():




#     package_name='robot_sim_gz' 
#     cpp_package_name = 'cpp_explorer'

#     robot_spwan_gz = IncludeLaunchDescription(
#                 PythonLaunchDescriptionSource([os.path.join(
#                     get_package_share_directory(package_name),'launch','gz_sim.launch.py'
#                 )]), launch_arguments={'use_sim_time': 'true'}.items()
#     )

#     # Include the explorer launch file
    
#     # Include the slam launch file
#     slam_toolbox = IncludeLaunchDescription(
#                 PythonLaunchDescriptionSource([os.path.join(
#                     get_package_share_directory(package_name),'launch','online_async.launch.py'
#                 )]), launch_arguments={'use_sim_time': 'true'}.items()
#     )
#     nav2 = IncludeLaunchDescription(
#                 PythonLaunchDescriptionSource([os.path.join(
#                     get_package_share_directory(package_name),'launch','navigation.launch.py'
#                 )]), launch_arguments={'use_sim_time': 'true'}.items()
#     )

#     cpp_explorer = IncludeLaunchDescription(
#                 PythonLaunchDescriptionSource([os.path.join(
#                     get_package_share_directory(cpp_package_name), 'launch', 'cpp_explore.launch.py')]),
#              )
#     # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    



#     # Launch them all!
#     return LaunchDescription([
#         robot_spwan_gz,
#         cpp_explorer,
#         slam_toolbox,
#         nav2,
#     ])







import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace, SetRemap
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    
    # 1. Configuration
    robot_name_arg = DeclareLaunchArgument('robot_name', default_value='robot1')
    robot_name = LaunchConfiguration('robot_name')
    package_name = 'robot_sim_gz'
    pkg_share = get_package_share_directory(package_name)
    cpp_explorer_share = get_package_share_directory('cpp_explorer')
    nav2_params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')

    # configured_params = RewrittenYaml(
    #     source_file=nav2_params_file,
    #     root_key=robot_name, # Keeps the top-level namespace correct
    #     param_rewrites={
    #         'use_sim_time': LaunchConfiguration('use_sim_time'),
    #         # 'global_frame': 'odom', # If you want local SLAM, or 'map' for global
    #         'robot_base_frame': [robot_name, '/base_link'], # Becomes "robot1/base_link"
    #         # If your SLAM runs in namespace, you might need to rewrite odom too:
    #         'global_frame': [robot_name, '/odom'] 
    #     },
    #     convert_types=True
    # )
    
    
    
    gazebo_sim = GroupAction([
        PushRosNamespace(robot_name),
        IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'use_sim_time': 'true',
                          'robot_name': robot_name}.items()
    )
    ])

    slam = GroupAction([
        PushRosNamespace(robot_name),
        SetRemap(src='/map', dst='map'),
        SetRemap(src='/scan', dst='scan'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'online_async.launch.py')),
            launch_arguments={'use_sim_time': 'true'}.items()
        )
    ])

    nav2 = GroupAction([
        PushRosNamespace(robot_name),
        SetRemap(src='/tf', dst='/tf'),
        SetRemap(src='/tf_static', dst='/tf_static'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'navigation_launch.py')),
            launch_arguments={
                'use_sim_time': 'true',
                'params_file': nav2_params_file,
                # Important: If your nav file takes a namespace arg, pass it here too
                'namespace': robot_name,
                'use_namespace': 'True'
            }.items()
        )
    ])

    # D. CPP Explorer (Starts Last)
    cpp_explorer = GroupAction([
        PushRosNamespace(robot_name),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(cpp_explorer_share, 'launch', 'cpp_explore.launch.py'))
        )
    ])

    # 3. Create the Delays (The Fix)
    
    # Wait 5 seconds for Gazebo to load before starting SLAM
    delayed_slam = TimerAction(period=5.0, actions=[slam])

    # Wait 8 seconds (total) for SLAM to initialize before starting Nav2
    delayed_nav2 = TimerAction(period=8.0, actions=[nav2])

    # Wait 12 seconds (total) for Nav2 to be active before starting the explorer
    delayed_explorer = TimerAction(period=13.0, actions=[cpp_explorer])

    return LaunchDescription([
        robot_name_arg,
        gazebo_sim,
        delayed_slam,
        delayed_nav2,
        delayed_explorer
    ])