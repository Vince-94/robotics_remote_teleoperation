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
    status_node = Node(
        package="robot_launch",
        executable="status_node.py",
        name="status_node",
    )


    #! LaunchDescription
    ld = LaunchDescription()

    ld.add_action(status_node)

    return ld
