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
    )

    watchdog = Node(
        package="robot_launch",
        executable="watchdog.py",
        name="watchdog",
    )


    #! LaunchDescription
    ld = LaunchDescription()

    ld.add_action(status_publisher)
    ld.add_action(cmd_vel_subscriber)
    ld.add_action(camera_publisher)
    ld.add_action(watchdog)

    return ld
