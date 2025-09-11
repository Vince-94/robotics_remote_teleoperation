#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class FakeCmdVelPublisher(Node):
    def __init__(self):
        super().__init__('fake_cmd_vel_publisher')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(1.0, self.publish_cmd_vel)  # every 1s
        self.direction = 1

    def publish_cmd_vel(self):
        msg = Twist()
        msg.linear.x = 0.2 * self.direction
        msg.angular.z = 0.1 * self.direction
        self.publisher_.publish(msg)
        self.get_logger().debug(f'Publishing fake cmd_vel: {msg.linear.x:.2f}, {msg.angular.z:.2f}')
        self.direction *= -1  # alternate forward/backward


def main(args=None):
    rclpy.init(args=args)
    node = FakeCmdVelPublisher()
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
