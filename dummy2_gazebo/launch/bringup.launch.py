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
    pkg_sim_car = get_package_share_directory('sim_car')

    world_config = DeclareLaunchArgument(
        'world',
        default_value=[os.path.join(pkg_dummy2, 'worlds', 'dummy2_world_edit.world'), ''],
        description='SDF world file'
    )

    bringup_dir = get_package_share_directory('sim_car')
    launch_dir = os.path.join(bringup_dir, 'launch')
    rviz_config_path = os.path.join(pkg_dummy2, 'config', 'rviz2_config.rviz')
    start_rviz2_cmd = ExecuteProcess(
        cmd=['rviz2', '--display-config', rviz_config_path],
        cwd=[launch_dir],
        output='screen'
    )

    nav2_params_path = os.path.join(
        get_package_share_directory('dummy2_gazebo'),
        'config', 'navigation.yaml'
    )

    # nav2_params_path = os.path.join(
    #     get_package_share_directory('sim_car'),
    #     'config', 'nav2_params.yaml'
    # )

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
            'map' : os.path.join(pkg_dummy2, 'worlds/simple_map', 'empty.yaml'),
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
            # os.path.join(pkg_sim_car, 'launch', 'spawn_car.launch.py'),
        )
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
    return ld
