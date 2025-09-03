#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import datetime

class FakeCameraPublisher(Node):
    def __init__(self):
        super().__init__("fake_camera_publisher")
        self.pub = self.create_publisher(Image, "/camera/image_raw", 5)
        self.br = CvBridge()
        self.timer = self.create_timer(0.2, self.timer_callback)  # 5 Hz
        self.width = 640
        self.height = 480
        self.font = cv2.FONT_HERSHEY_SIMPLEX
        self.get_logger().info("Fake camera publisher started (synthetic frames).")

    def timer_callback(self):
        # Create a clean background and overlay timestamp + random content
        frame = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        # background gradient
        for i in range(self.height):
            color = int(255 * (i / self.height))
            frame[i, :, :] = (color // 2, color, 255 - color // 2)

        # draw a bouncing circle based on current second
        t = datetime.datetime.now(datetime.UTC).timestamp()
        cx = int((self.width / 2) + (self.width / 3) * np.sin(t))
        cy = int(self.height / 2)
        cv2.circle(frame, (cx, cy), 40, (0, 0, 255), -1)

        # overlay timestamp text
        ts = datetime.datetime.now(datetime.UTC).strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        cv2.putText(frame, f"FakeCam {ts}", (12, 30), self.font, 0.7, (255,255,255), 2, cv2.LINE_AA)

        # convert and publish
        img_msg = self.br.cv2_to_imgmsg(frame, encoding="bgr8")
        img_msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(img_msg)


def main(args=None):
    rclpy.init(args=args)
    node = FakeCameraPublisher()
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
