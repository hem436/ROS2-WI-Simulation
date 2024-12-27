#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # launch_file_dir = os.path.join(get_package_share_directory('robot_controller'), 'launch')
    robot_controller = get_package_share_directory('robot_controller')
    os.environ['TURTLEBOT3_MODEL'] = "waffle_pi"

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='-2.0')
    y_pose = LaunchConfiguration('y_pose', default='0.5')

    # launch description for the amazon warehouse world
    warehouse_pkg_dir = get_package_share_directory('aws_robomaker_small_warehouse_world')
    warehouse_launch_path = os.path.join(warehouse_pkg_dir, 'launch')

    warehouse_world_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([warehouse_launch_path, '/no_roof_small_warehouse.launch.py'])
    )

    # launch description for the robot state publisher
    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_controller,'launch', 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time }.items()
    )
    print("robot_state_publisher_cmd: ", robot_state_publisher_cmd )
    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_controller,'launch', 'spawn_turtlebot3.launch.py')
        )
    )

    slam_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("slam_toolbox"),'launch', 'online_sync_launch.py')
        )
    )
    print("ksdjfkas",get_package_share_directory("turtlebot3_navigation2"),os.path.join(get_package_share_directory("turtlebot3_navigation2"),'launch', 'navigation2.launch.py'))

    # navigation_cmd = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(get_package_share_directory("turtlebot3_navigation2"),'launch', 'navigation2.launch.py')
    #     )
    # )

    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(warehouse_world_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_turtlebot_cmd)
    ld.add_action(slam_cmd)
    # ld.add_action(navigation_cmd)

    return ld