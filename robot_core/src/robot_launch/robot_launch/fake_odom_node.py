#!/usr/bin/env python3
"""
Publishers:
- /robot_status [std_msgs.msg.String]
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class StatusPublisher(Node):
    def __init__(self):
        super().__init__("fake_odom_node")
        self.odom_pub_ = self.create_publisher(Odometry, "/odom", 10)
        self.timer = self.create_timer(0.1, self.timer_cb)
        self.get_logger().info("fake_odom_node node started.")

    def timer_cb(self):

        # Odometry
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.child_frame_id = "child_frame"
        odom.pose.pose.position.x = 0.0
        odom.pose.pose.position.y = 0.0
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.w = 1.0
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = 0.0
        odom.twist.twist.linear.x = 0.0
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = 0.0

        self.odom_pub_.publish(odom)


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
