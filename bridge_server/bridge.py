import asyncio
import json
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import websockets
from websockets import WebSocketServerProtocol


TELEMETRY_PORT = 9000

class BridgeNode(Node):
    def __init__(self, loop, telemetry_queue):
        super().__init__('teleop_bridge_node')
        self.loop = loop
        self.telemetry_queue = telemetry_queue
        self.get_logger().info("Bridge node starting...")
        # publishers & subscribers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_in', 10)
        self.create_subscription(String, '/robot/status', self.status_cb, 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)

    def status_cb(self, msg: String):
        payload = {'topic':'/robot/status', 'data': msg.data}
        # push into asyncio queue safely
        self.loop.call_soon_threadsafe(self.telemetry_queue.put_nowait, json.dumps(payload))

    def odom_cb(self, msg: Odometry):
        # basic serialization of position
        p = msg.pose.pose.position
        payload = {'topic':'/odom', 'position': {'x': p.x, 'y': p.y, 'z': p.z}}
        self.loop.call_soon_threadsafe(self.telemetry_queue.put_nowait, json.dumps(payload))

    def publish_cmd(self, cmd_dict):
        t = Twist()
        lin = cmd_dict.get('linear', {})
        ang = cmd_dict.get('angular', {})
        t.linear.x = float(lin.get('x', lin.get('x', 0.0))) if lin else 0.0
        t.linear.y = float(lin.get('y', 0.0)) if lin else 0.0
        t.linear.z = float(lin.get('z', 0.0)) if lin else 0.0
        t.angular.x = 0.0
        t.angular.y = 0.0
        t.angular.z = float(ang.get('z', 0.0)) if ang else 0.0
        self.cmd_pub.publish(t)

async def websocket_handler(websocket: WebSocketServerProtocol, path, node, telemetry_queue):
    # Very simple handler: spawn two tasks to send telemetry and receive commands
    async def sender():
        try:
            while True:
                msg = await telemetry_queue.get()
                await websocket.send(msg)
        except asyncio.CancelledError:
            return
        except Exception:
            return

    async def receiver():
        try:
            async for text in websocket:
                try:
                    obj = json.loads(text)
                except Exception:
                    continue
                # handle cmd_vel messages
                if 'cmd_vel' in obj:
                    node.publish_cmd(obj['cmd_vel'])
        except asyncio.CancelledError:
            return
        except websockets.ConnectionClosed:
            return

    send_task = asyncio.create_task(sender())
    recv_task = asyncio.create_task(receiver())
    done, pending = await asyncio.wait([send_task, recv_task], return_when=asyncio.FIRST_COMPLETED)
    for t in pending:
        t.cancel()

def start_rclpy_spin(node):
    rclpy.spin(node)

async def main():
    loop = asyncio.get_event_loop()
    telemetry_queue = asyncio.Queue()

    # initialize rclpy and node in this thread
    rclpy.init()
    node = BridgeNode(loop, telemetry_queue)

    # spin rclpy in a background thread
    t = threading.Thread(target=start_rclpy_spin, args=(node,), daemon=True)
    t.start()

    # start websocket server
    async with websockets.serve(lambda ws, p: websocket_handler(ws, p, node, telemetry_queue),
                                host='0.0.0.0', port=TELEMETRY_PORT):
        node.get_logger().info(f"Bridge websocket server running on port {TELEMETRY_PORT}")
        # keep running
        await asyncio.Future()

if __name__ == '__main__':
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
