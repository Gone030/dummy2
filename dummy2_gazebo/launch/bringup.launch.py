import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_dummy2 = get_package_share_directory('dummy2_gazebo')
    pkg_nav2 = get_package_share_directory('nav2_bringup')

    world = LaunchConfiguration('world')
    rviz_config_path = os.path.join(pkg_dummy2, 'config', 'rviz2_config.rviz')
    nav2_params_path = os.path.join(
        get_package_share_directory('dummy2_gazebo'),
        'config', 'navigation.yaml'
    )

    nav_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2,'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'slam' : "1",
            'map' : os.path.join(pkg_dummy2, 'worlds/simple_map', 'my_map.yaml'),
            'params_file': nav2_params_path,
        }.items()
    )

    rviz2 = ExecuteProcess(
        cmd=['rviz2', '--display-config', rviz_config_path],
        output='screen'
    )

    declare_world_config = DeclareLaunchArgument(
        'world',
        default_value=[os.path.join(pkg_dummy2, 'worlds', 'dummy2_world.world'), ''],
        description='world file'
    )
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')]),
        launch_arguments={'world': world,
                          }.items(),
    )

    spawn_car = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dummy2, 'launch', 'description.launch.py')
        )
    )
    ld = LaunchDescription()
    ld.add_action(declare_world_config)
    ld.add_action(gazebo)
    ld.add_action(rviz2)
    ld.add_action(spawn_car)
    ld.add_action(nav_bringup)

    return ld
