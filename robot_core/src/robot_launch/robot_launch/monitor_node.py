#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from sensor_msgs.msg import Image

class MonitorNode(Node):
    def __init__(self):
        super().__init__('monitor_node')

        # Subscriptions
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(Bool, 'emergency_stop', self.emergency_callback, 10)
        self.create_subscription(Image, 'camera_feed', self.camera_callback, 10)
        self.create_subscription(Bool, 'status', self.status_callback, 10)

        self.get_logger().info("Monitor Node started. Listening to cmd_vel, emergency_stop, camera_feed, and status.")

    def cmd_vel_callback(self, msg: Twist):
        self.get_logger().info(f"[cmd_vel] linear.x={msg.linear.x:.2f}, angular.z={msg.angular.z:.2f}")

    def emergency_callback(self, msg: Bool):
        self.get_logger().warn(f"[emergency_stop] {msg.data}")

    def camera_callback(self, msg: Image):
        self.get_logger().info(f"[camera_feed] frame received: {msg.width}x{msg.height}")

    def status_callback(self, msg: Bool):
        self.get_logger().info(f"[status] robot is {'ACTIVE' if msg.data else 'STOPPED'}")


def main(args=None):
    rclpy.init(args=args)
    node = MonitorNode()
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
