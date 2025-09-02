#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class FakeEmergencyStopPublisher(Node):
    def __init__(self):
        super().__init__('fake_emergency_stop_publisher')
        self.publisher_ = self.create_publisher(Bool, 'emergency_stop', 10)
        self.timer = self.create_timer(3.0, self.publish_stop)  # every 3s
        self.state = False

    def publish_stop(self):
        msg = Bool()
        msg.data = self.state
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing fake emergency_stop: {msg.data}')
        self.state = not self.state  # toggle each cycle


def main(args=None):
    rclpy.init(args=args)
    node = FakeEmergencyStopPublisher()
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
