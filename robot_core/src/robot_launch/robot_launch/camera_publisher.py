#!/usr/bin/env python3
"""
Publishers:
- /camera/image_raw [sensor_msgs.msg.Image]
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        self.publisher_ = self.create_publisher(Image, '/camera/image_raw', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(0)  # USB camera
        if not self.cap.isOpened():
            self.get_logger().warn("Camera not found. Killing node...")
            self.destroy_node()
        else:
            self.get_logger().info("camera_publisher node started.")

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
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
