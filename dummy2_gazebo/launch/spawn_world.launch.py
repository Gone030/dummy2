import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_dummy2 = get_package_share_directory('dummy2_gazebo')
    pkg_gazebo = get_package_share_directory('gazebo_ros')


    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(pkg_gazebo, 'launch', 'gazebo.launch.py')]),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=[os.path.join(pkg_dummy2, 'worlds', 'dummy2_world_edit.world'), ''],
            description='world file'
        ),
        gazebo
    ])



