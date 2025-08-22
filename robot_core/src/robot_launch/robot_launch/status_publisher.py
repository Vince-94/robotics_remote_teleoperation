#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import json
import time

class StatusPub(Node):
    def __init__(self):
        super().__init__('status_publisher')
        self.get_logger().info("Starting status_publisher...")
        self.status_pub = self.create_publisher(String, '/robot/status', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.timer = self.create_timer(0.5, self.timer_cb)
        self.counter = 0.0

    def timer_cb(self):
        # publish a small JSON status
        st = {'battery': 100 - (self.counter % 100), 'cpu': 10 + (self.counter % 5)}
        msg = String()
        msg.data = json.dumps(st)
        self.status_pub.publish(msg)

        od = Odometry()
        od.pose.pose.position.x = (self.counter * 0.05)
        od.pose.pose.position.y = 0.0
        od.pose.pose.position.z = 0.0
        od.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        self.odom_pub.publish(od)

        self.counter += 1.0

def main():
    rclpy.init()
    node = StatusPub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
