from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

MAP_NAME='testmap1'

def generate_launch_description():
    nav2_launch_path = PathJoinSubstitution(
        [FindPackageShare('nav2_bringup'), 'launch', 'bringup_launch.py']
    )
    default_map_path = PathJoinSubstitution(
        [FindPackageShare('test_navi'), 'maps', f'{MAP_NAME}.yaml']
    )
    nav_config_path = PathJoinSubstitution(
        [FindPackageShare('test_navi'), 'config', 'navigation.yaml']
    )

    return  LaunchDescription([
        DeclareLaunchArgument(
            name= 'sim',
            default_value='false',
            description='use sim time'
        ),
        DeclareLaunchArgument(
            name= 'map',
            default_value= default_map_path,
            description= 'map path'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch_path),
            launch_arguments={
                'map': LaunchConfiguration("map"),
                'use_sim_time': LaunchConfiguration("sim"),
                'params_file': nav_config_path
            }.items()
        ),
    ])
