#!/usr/bin/env python3
"""
Subscribers:
- /cmd_vel [geometry_msgs.msg.Twist]
- /emergency_stop [std_msgs.msg.Bool]
- /camera/image_raw [sensor_msgs.msg.Image]
- /robot_status [std_msgs.msg.String]

Publishers:
- /diagnostics [diagnostic_msgs.msg.DiagnosticArray]
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String
from sensor_msgs.msg import Image
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import time
import json

class MonitorNode(Node):
    def __init__(self):
        super().__init__('monitor_node')
        # Subscriptions we care about
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(Bool, '/emergency_stop', self.emergency_callback, 10)
        self.create_subscription(Image, '/camera/image_raw', self.camera_callback, 1)
        self.create_subscription(String, '/robot_status', self.status_callback, 10)

        # Diagnostics publisher
        self.diagnostics_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)

        # Simple state
        self.last_cmd_time = self.get_clock().now()
        self.last_status_time = self.get_clock().now()
        self.last_camera_time = self.get_clock().now()
        self.last_estop = False

        # Publish diagnostics periodically
        self.diag_timer = self.create_timer(1.0, self.publish_diagnostics)

        self.get_logger().info("Monitor Node started. Listening to cmd_vel, emergency_stop, camera, and status.")

    def cmd_vel_callback(self, msg: Twist):
        self.last_cmd_time = self.get_clock().now()
        self.get_logger().debug(f"[cmd_vel] linear.x={msg.linear.x:.2f}, angular.z={msg.angular.z:.2f}")

    def emergency_callback(self, msg: Bool):
        self.last_estop = msg.data
        self.get_logger().warn(f"[emergency_stop] {msg.data}")

    def camera_callback(self, msg: Image):
        self.last_camera_time = self.get_clock().now()
        # Only log coarse info for performance
        self.get_logger().debug(f"[camera_feed] frame received: {msg.width}x{msg.height}")

    def status_callback(self, msg: String):
        self.last_status_time = self.get_clock().now()
        # status message expected to be JSON-like from status_publisher
        try:
            parsed = json.loads(msg.data)
            self.get_logger().debug(f"[status] {parsed}")
        except Exception:
            self.get_logger().debug(f"[status] {msg.data}")

    def publish_diagnostics(self):
        now = self.get_clock().now()
        # compute age of last messages in seconds
        age_cmd = (now - self.last_cmd_time).nanoseconds * 1e-9
        age_status = (now - self.last_status_time).nanoseconds * 1e-9
        age_cam = (now - self.last_camera_time).nanoseconds * 1e-9

        # health heuristics
        cmd_ok = age_cmd < 2.0
        status_ok = age_status < 5.0
        camera_ok = age_cam < 5.0

        # prepare DiagnosticStatus entries
        diag_status_list = []

        # comms / teleop status
        s1 = DiagnosticStatus()
        s1.name = "teleop/communication"
        s1.hardware_id = "teleop_link"
        s1.level = DiagnosticStatus.OK if cmd_ok else DiagnosticStatus.WARN
        s1.message = "cmd_vel recent" if cmd_ok else f"cmd_vel stale ({age_cmd:.1f}s)"
        s1.values = [KeyValue(key="last_cmd_age_s", value=f"{age_cmd:.2f}")]
        diag_status_list.append(s1)

        # robot status publisher health
        s2 = DiagnosticStatus()
        s2.name = "robot/status_publisher"
        s2.hardware_id = "robot_core"
        s2.level = DiagnosticStatus.OK if status_ok else DiagnosticStatus.WARN
        s2.message = "status ok" if status_ok else f"status stale ({age_status:.1f}s)"
        s2.values = [KeyValue(key="last_status_age_s", value=f"{age_status:.2f}")]
        diag_status_list.append(s2)

        # camera health
        s3 = DiagnosticStatus()
        s3.name = "camera"
        s3.hardware_id = "camera0"
        s3.level = DiagnosticStatus.OK if camera_ok else DiagnosticStatus.WARN
        s3.message = "camera streaming" if camera_ok else f"camera stale ({age_cam:.1f}s)"
        s3.values = [KeyValue(key="last_camera_age_s", value=f"{age_cam:.2f}")]
        diag_status_list.append(s3)

        # emergency stop state
        s4 = DiagnosticStatus()
        s4.name = "emergency_stop"
        s4.hardware_id = "estop"
        s4.level = DiagnosticStatus.ERROR if self.last_estop else DiagnosticStatus.OK
        s4.message = "ESTOP engaged" if self.last_estop else "ESTOP released"
        s4.values = [KeyValue(key="estop_state", value=str(self.last_estop))]
        diag_status_list.append(s4)

        # compose array
        diag_array = DiagnosticArray()
        diag_array.header.stamp = self.get_clock().now().to_msg()
        diag_array.status = diag_status_list

        self.diagnostics_pub.publish(diag_array)
        self.get_logger().debug("Published diagnostics")


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
