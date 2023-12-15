from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    urdf_path = PathJoinSubstitution(
        [FindPackageShare('test_gazebo'), 'urdf', 'base.urdf.xacro']
    )

    world_path = PathJoinSubstitution(
        [FindPackageShare('test_gazebo'), 'worlds', 'turtlebot3_house.world']
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            name='urdf',
            default_value=urdf_path,
            description='urdf path'
        ),
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='true',
            description='use sim time'
        ),
        DeclareLaunchArgument(
            name='world',
            default_value= world_path,
            description= 'gazebo world'
        ),

        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so', '-s', 'libgazebo_ros_init.so', LaunchConfiguration('world')],
            output='screen'
        ),

        Node(
          package='gazebo_ros',
          executable='spawn_entity.py',
          name='urdf_spawner',
          output='screen',
          arguments=["-topic", "robot_description", "-entity", "test1", "-x", "-2.5", "-y", "1.0", "-z", "0.2"]
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'robot_description': Command(['xacro ', LaunchConfiguration('urdf')])
                }
            ]
        ),
        Node(
            package= 'tf2_ros',
            executable='static_transform_publisher',
            name= 'static_tf_pub_laser',
            arguments=['0','0', '0', '0', '0', '0', '0', 'base_footprint', 'lidar_link'],
        ),
        Node(
            package= 'tf2_ros',
            executable='static_transform_publisher',
            name= 'static_tf_pub_map',
            arguments=['0','0', '0', '0', '0', '0', '0', 'base_footprint', 'map'],
        ),
        Node(
            package= 'tf2_ros',
            executable='static_transform_publisher',
            name= 'static_tf_pub_odom',
            arguments=['0','0', '0', '0', '0', '0', '0', 'base_footprint', 'odom'],
        )
    ])
