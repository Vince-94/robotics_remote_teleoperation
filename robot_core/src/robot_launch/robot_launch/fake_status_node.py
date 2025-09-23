#!/usr/bin/env python3
"""
Publishers:
- /robot_status [std_msgs.msg.String]
"""
import rclpy
from rclpy.node import Node
from robot_launch.msg import Status
from std_msgs.msg import String, Bool

import psutil
import socket


class StatusPublisher(Node):
    def __init__(self):
        super().__init__("fake_status_node")
        # self.publisher_ = self.create_publisher(Status, "/robot_status", 10)
        self.publisher_ = self.create_publisher(String, "/robot_status", 10)
        self.timer = self.create_timer(2.0, self.publish_status)  # every 2s
        self.get_logger().info("fake_status_node node started.")

    def publish_status(self):
        status = String()
        status.data = "STATUS: OK"

        self.publisher_.publish(status)
        # self.get_logger().info(f"Status: {status}")


def main(args=None):
    rclpy.init(args=args)
    node = StatusPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(f"Stutting down node.")
    except Exception as err:
        node.get_logger().info(f"Unexpected error node: {err}.")
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()
