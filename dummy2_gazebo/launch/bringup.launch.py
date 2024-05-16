import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_dummy2 = get_package_share_directory('dummy2_gazebo')

    world = LaunchConfiguration('world')

    declere_world_config = DeclareLaunchArgument(
        'world',
        default_value=[os.path.join(pkg_dummy2, 'worlds', 'empty.world'), ''],
        description='world file'
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')]),
        launch_arguments={'world': world}.items(),
    )
    ld = LaunchDescription()
    ld.add_action(declere_world_config)
    ld.add_action(gazebo)

    return ld
