import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_dummy2 = get_package_share_directory('dummy2_gazebo')
    pkg_nav2 = get_package_share_directory('nav2_bringup')

    world_config = DeclareLaunchArgument(
        'world',
        default_value=[os.path.join(pkg_dummy2, 'worlds', 'dummy2_world_edit.world'), ''],
        description='SDF world file'
    )

    rviz_config_path = os.path.join(pkg_dummy2, 'config', 'rviz2_config.rviz')
    start_rviz2_cmd = ExecuteProcess(
        cmd=['rviz2', '--display-config', rviz_config_path],
        output='screen'
    )

    nav2_params_path = os.path.join(
        get_package_share_directory('dummy2_gazebo'),
        'config', 'navigation.yaml'
    )

    bt_path = os.path.join(get_package_share_directory('dummy2_gazebo'),
                            'bt',
                            'custom_bt.xml')

    configured_nav2_params = RewrittenYaml(
        source_file=nav2_params_path,
        param_rewrites={'default_nav_to_pose_bt_xml': bt_path},
        convert_types=True
    )

    nav_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2,'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'slam' : "1",
            'map' : "",
            'params_file': configured_nav2_params,
        }.items()
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
        )
    )

    spawn_car = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dummy2, 'launch', 'description.launch.py')
        )
    )
    initial_pose_node = Node(
        package= 'dummy2_gazebo',
        executable= 'initial_pose_publisher',
        output= 'screen',
        parameters= [{
            'initial_x' : 0.0,
            'initial_y' : 0.0,
            'initial_yaw': 0.0,
        }]
    )

    ld = LaunchDescription()
    ld.add_action(world_config)
    ld.add_action(gazebo)
    ld.add_action(TimerAction(
            period=5.0,
            actions=[start_rviz2_cmd]
    ))
    ld.add_action(spawn_car)
    ld.add_action(TimerAction(
            period=0.0,
            actions=[nav_bringup]
            ))
    ld.add_action(initial_pose_node)
    return ld
