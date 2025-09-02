#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class Watchdog(Node):
    def __init__(self):
        super().__init__('watchdog')
        self.last_cmd_time = self.get_clock().now()
        self.timeout = 1.0  # seconds
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_callback,
            10)
        self.publisher_ = self.create_publisher(String, '/wheel_commands', 10)
        self.timer = self.create_timer(0.5, self.check_timeout)
        self.get_logger().info("watchdog node started.")

    def cmd_callback(self, msg: Twist):
        self.last_cmd_time = self.get_clock().now()

    def check_timeout(self):
        elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
        if elapsed > self.timeout:
            self.publisher_.publish(String(data="STOP"))
            self.get_logger().warn("No cmd_vel received -> STOP published")

def main(args=None):
    rclpy.init(args=args)
    node = Watchdog()
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
