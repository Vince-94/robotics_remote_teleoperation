#!/usr/bin/env python3
from pathlib import Path
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    use_fake_camera = LaunchConfiguration("use_fake_camera", default="false")

    #! Launch ROS2 node
    status_publisher = Node(
        package="robot_launch",
        executable="status_publisher.py",
        name="status_publisher",
    )

    cmd_vel_subscriber = Node(
        package="robot_launch",
        executable="cmd_vel_subscriber.py",
        name="cmd_vel_subscriber",
    )

    camera_publisher = Node(
        package="robot_launch",
        executable="camera_publisher.py",
        name="camera_publisher",
        condition=IfCondition(PythonExpression(["'", use_fake_camera, "' == 'false'"]))
    )

    watchdog = Node(
        package="robot_launch",
        executable="watchdog.py",
        name="watchdog",
    )

    rosbridge_server = Node(
        package="rosbridge_server",
        executable="rosbridge_websocket",
        name="rosbridge_websocket",
        output='screen',
        arguments=['--port', '9090', '--address', '0.0.0.0']
    )


    #! LaunchDescription
    ld = LaunchDescription()

    ld.add_action(status_publisher)
    ld.add_action(cmd_vel_subscriber)
    ld.add_action(camera_publisher)
    ld.add_action(watchdog)
    ld.add_action(rosbridge_server)

    return ld
