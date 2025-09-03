#!/usr/bin/env python3
"""
Publishers:
- /robot_status [std_msgs.msg.String]
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import psutil
import socket

class StatusPublisher(Node):
    def __init__(self):
        super().__init__('status_publisher')
        self.publisher_ = self.create_publisher(String, '/robot_status', 10)
        self.timer = self.create_timer(2.0, self.publish_status)  # every 2s
        self.get_logger().info("status_publisher node started.")

    def publish_status(self):
        cpu = psutil.cpu_percent()
        mem = psutil.virtual_memory().percent
        ip = socket.gethostbyname(socket.gethostname())
        msg = f"CPU:{cpu}%, MEM:{mem}%, IP:{ip}"
        self.publisher_.publish(String(data=msg))
        # self.get_logger().info(f"Status: {msg}")

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

if __name__ == '__main__':
    main()
