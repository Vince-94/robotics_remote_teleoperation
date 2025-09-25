#!/usr/bin/env python3
"""
BridgeNode:
- subscribers:
  - /status [robot_launch.msg.Status]
  - /odom [nav_msgs.msg.Odometry]
  - /diagnostics [diagnostic_msgs.msg.DiagnosticArray]
- publishers:
  - /cmd_vel [geometry_msgs.msg.Twist]
  - /estop [std_msg.msg.Bool]
"""
import asyncio
import json
import threading
import websockets
from functools import partial
from typing import Any
import os
import math
from aiohttp import web

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool
from diagnostic_msgs.msg import DiagnosticArray


TELEMETRY_PORT = 9000
HEALTH_PORT = int(os.environ.get("HEALTH_PORT", 9001))

# Track active connections for /health
_active_webs = set()


class BridgeNode(Node):
    def __init__(self, loop, telemetry_queue):
        super().__init__("teleop_bridge_node")
        self.loop = loop
        self.telemetry_queue = telemetry_queue
        self.get_logger().info("Bridge node starting...")

        # subscribers
        self.create_subscription(String, "/status", self.status_cb, 10)
        self.create_subscription(Odometry, "/odom", self.odom_cb, 10)
        self.create_subscription(DiagnosticArray, "/diagnostics", self.diag_cb, 10)

        # publishers
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel_in", 10)
        self.estop_pub = self.create_publisher(Bool, "/estop", 10)

    def status_cb(self, msg: String):
        payload = {"topic":"/status", "data": msg.data}
        # push into asyncio queue safely
        self.loop.call_soon_threadsafe(self.telemetry_queue.put_nowait, json.dumps(payload))

    def odom_cb(self, msg: Odometry):
        # basic serialization of position
        p = msg.pose.pose.position
        payload = {"topic":"/odom", "position": {"x": p.x, "y": p.y, "z": p.z}}
        self.loop.call_soon_threadsafe(self.telemetry_queue.put_nowait, json.dumps(payload))

    def _normalize_level(self, level):
        """Return an int for DiagnosticStatus.level even if it's bytes."""
        if isinstance(level, int):
            return level
        if isinstance(level, (bytes, bytearray)):
            # common case: single-byte status like b'\x00' or b'\x01'
            if len(level) == 1:
                return level[0]
            # fallback: try interpreting as little-endian integer
            try:
                return int.from_bytes(level, byteorder='little', signed=False)
            except Exception:
                # last resort: try decode then int
                try:
                    return int(level.decode('utf-8', errors='ignore') or 0)
                except Exception:
                    return 0
        try:
            return int(level)
        except Exception:
            return 0

    def _make_json_safe(self, obj):
        """Recursively convert bytes/bytearray to strings or ints for JSON safety."""
        if isinstance(obj, (bytes, bytearray)):
            # try text first
            try:
                return obj.decode('utf-8')
            except Exception:
                # fallback to an integer if single-byte
                if len(obj) == 1:
                    return obj[0]
                return str(obj)
        if isinstance(obj, (str, int, float, bool)) or obj is None:
            return obj
        if isinstance(obj, dict):
            return {k: self._make_json_safe(v) for k, v in obj.items()}
        if isinstance(obj, (list, tuple)):
            return [self._make_json_safe(v) for v in obj]
        # final fallback
        return str(obj)

    def diag_cb(self, msg: DiagnosticArray):
        # Serialize status of each diagnostic
        diagnostics = []
        for status in msg.status:
            level_val = self._normalize_level(status.level)

            diagnostics.append({
                "name": status.name,
                "level": level_val,
                "message": status.message,
                "hardware_id": status.hardware_id,
                "values": [
                    {"key": v.key, "value": str(v.value)} for v in status.values
                ]
            })

        payload = {"topic": "/diagnostics", "diagnostics": diagnostics}
        # make sure any remaining bytes are converted to json-safe types
        safe_payload = self._make_json_safe(payload)

        try:
            serialized = json.dumps(safe_payload)
        except Exception as e:
            self.get_logger().error(f"JSON serialization error: {e}")
            self.get_logger().error(f"Payload: {repr(safe_payload)}")
            for status in msg.status:
                for v in status.values:
                    self.get_logger().error(f"Key: {v.key}, Value: {repr(v.value)}, Type: {type(v.value)}")
            return

        self.loop.call_soon_threadsafe(self.telemetry_queue.put_nowait, serialized)

    def publish_cmd(self, cmd: dict):
        """Convert a cmd_vel-like dict into geometry_msgs/Twist and publish it.

        `cmd` expected shape:
        { "linear": {"x":..., "y":..., "z":...}, "angular": {"x":..., "y":..., "z":...} }
        """
        msg = Twist()

        t = Twist()
        lin = cmd.get("linear", {}) or {}
        ang = cmd.get("angular", {}) or {}

        # defensive conversion
        lx = float(lin.get("x", 0.0))
        ly = float(lin.get("y", 0.0))
        lz = float(lin.get("z", 0.0))

        ax = float(ang.get("x", 0.0))
        ay = float(ang.get("y", 0.0))
        az = float(ang.get("z", 0.0))

        # Safety clamping (tunable)
        MAX_LINEAR = float(os.environ.get("MAX_LINEAR", math.inf))   # m/s
        MAX_ANGULAR = float(os.environ.get("MAX_ANGULAR", math.inf)) # rad/s

        def clamp(v, m): return max(min(v, m), -m)

        t.linear.x = clamp(lx, MAX_LINEAR)
        t.linear.y = clamp(ly, MAX_LINEAR)
        t.linear.z = clamp(lz, MAX_LINEAR)
        t.angular.x = clamp(ax, MAX_ANGULAR)
        t.angular.y = clamp(ay, MAX_ANGULAR)
        t.angular.z = clamp(az, MAX_ANGULAR)

        self.cmd_pub.publish(t)

    def publish_estop(self, value: bool):
        msg = Bool()
        msg.data = value
        self.estop_pub.publish(msg)
        self.get_logger().warning(f"eStop triggered")

# async def websocket_handler(websocket: Any, node, telemetry_queue):
async def websocket_handler(websocket: Any, *args, node, telemetry_queue):
    _active_webs.add(websocket)
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
                if "cmd_vel" in obj:
                    node.publish_cmd(obj["cmd_vel"])
                # handle estop messages
                if "estop" in obj:
                    node.publish_estop(bool(obj["estop"]))
        except asyncio.CancelledError:
            return
        except websockets.ConnectionClosed:
            return

    send_task = asyncio.create_task(sender())
    recv_task = asyncio.create_task(receiver())
    done, pending = await asyncio.wait([send_task, recv_task], return_when=asyncio.FIRST_COMPLETED)
    for t in pending:
        t.cancel()

    _active_webs.discard(websocket)


def start_rclpy_spin(node):
    rclpy.spin(node)


# Health endpoint
async def health_handler(request):
    return web.json_response({
        "status": "ok",
        "service": "bridge_server",
        "telemetry_port": TELEMETRY_PORT,
        "active_connections": len(_active_webs),
    })


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
    async with websockets.serve(
            partial(websocket_handler, node=node, telemetry_queue=telemetry_queue),
            host="0.0.0.0",
            port=TELEMETRY_PORT,
    ):
        node.get_logger().info(f"Bridge websocket server running on port {TELEMETRY_PORT}")

        # Health
        app = web.Application()
        app.router.add_get("/health", health_handler)
        runner = web.AppRunner(app)
        await runner.setup()
        site = web.TCPSite(runner, "0.0.0.0", HEALTH_PORT)
        await site.start()
        node.get_logger().info("Health endpoint running on port 9001")

        await asyncio.Future()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
