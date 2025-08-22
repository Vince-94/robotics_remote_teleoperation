from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    ld.add_action(Node(package='robot_launch', executable='status_publisher.py', name='status_pub'))
    return ld
