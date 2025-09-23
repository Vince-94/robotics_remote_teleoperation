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
    fake_odom_node = Node(
        package="robot_launch",
        executable="fake_odom_node.py",
        name="fake_odom_node",
    )

    fake_status_node = Node(
        package="robot_launch",
        executable="fake_status_node.py",
        name="fake_status_node",
    )

    diag_aggregator_node = Node(
        package="robot_launch",
        executable="diag_aggregator_node.py",
        name="diag_aggregator_node",
    )

    #! LaunchDescription
    ld = LaunchDescription()

    ld.add_action(fake_odom_node)
    ld.add_action(fake_status_node)
    ld.add_action(diag_aggregator_node)

    return ld
