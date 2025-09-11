#!/usr/bin/env python3
"""
Subscribers:
- /cmd_vel [geometry_msgs.msg.Twist]

Publishers:
- /wheel_commands [std_msgs.msg.String]
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class CmdVelSubscriber(Node):
    def __init__(self):
        super().__init__('cmd_vel_subscriber')
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.listener_callback,
            10)
        self.publisher_ = self.create_publisher(String, '/wheel_commands', 10)
        self.get_logger().info("cmd_vel_subscriber node started.")

    def listener_callback(self, msg: Twist):
        # Convert Twist to wheel command (simplified)
        command = f"LIN:{msg.linear.x:.2f}, ANG:{msg.angular.z:.2f}"
        self.publisher_.publish(String(data=command))
        self.get_logger().debug(f"Received cmd_vel -> Published wheel command: {command}")

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelSubscriber()
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
