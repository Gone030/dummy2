from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    slam_launch_path = PathJoinSubstitution(
        [FindPackageShare('slam_toolbox'), 'launch', 'online_async_launch.py']
    )

    slam_config_path = PathJoinSubstitution(
        [FindPackageShare('test_navi'), 'config', 'slam.yaml']
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            name = 'sim',
            default_value= 'true',
            description= 'use sim time'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch_path),
            launch_arguments= {
                'use_sim_time': LaunchConfiguration("sim"),
                'slam_param_name' : slam_config_path
            }.items()
        ),
    ])
