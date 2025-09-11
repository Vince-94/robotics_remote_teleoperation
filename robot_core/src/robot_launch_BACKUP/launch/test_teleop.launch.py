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
    pkg_name = "robot_launch"
    pkg = FindPackageShare(package=pkg_name).find(pkg_name)

    launch_path = PathJoinSubstitution([pkg, "launch", "teleop.launch.py"])
    launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_path]),
        launch_arguments={
            "use_fake_camera": "True",
        }.items(),
    )

    fake_cmd_vel_publisher = Node(
        package="robot_launch",
        executable="fake_cmd_vel_publisher.py",
        name="fake_cmd_vel_publisher",
    )

    fake_emergency_stop_publisher = Node(
        package="robot_launch",
        executable="fake_emergency_stop_publisher.py",
        name="fake_emergency_stop_publisher",
    )

    fake_camera_publisher = Node(
        package="robot_launch",
        executable="fake_camera_publisher.py",
        name="fake_camera_publisher",
    )

    monitor_node = Node(
        package="robot_launch",
        executable="monitor_node.py",
        name="monitor_node",
    )


    #! LaunchDescription
    ld = LaunchDescription()

    ld.add_action(launch_file)
    ld.add_action(fake_cmd_vel_publisher)
    ld.add_action(fake_emergency_stop_publisher)
    ld.add_action(fake_camera_publisher)

    ld.add_action(monitor_node)

    return ld