import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription


def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_dummy2 = get_package_share_directory('dummy2_gazebo')
