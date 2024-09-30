import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.actions import TimerAction
import xacro


def generate_launch_description():
    xacro_file = os.path.join(get_package_share_directory('dummy2_gazebo'), 'urdf/', 'base.urdf.xacro')

    robot_description_config = xacro.process_file(xacro_file)
    robot_desc = robot_description_config.toxml()

    spawn_car = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "/robot_description",
                   "-entity", "dummy2",
                   "-x", "0",
                   "-y", "0"
                   ]
    )

    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{"robot_description": robot_desc}],
        output="screen"
    )
    ld =  LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='using sim time'
        ),
        TimerAction(
            period=0.0 ,
            actions=[spawn_car]
        ),
        TimerAction(
            period=0.0,
            actions=[robot_state_pub]
        )
    ])

    return ld
