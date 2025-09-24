#!/usr/bin/env python3
"""
BridgeNode:
- subscribers:
  - /status [std_msgs.msg.String]
  - /odom [nav_msgs.msg.Odometry]
  - /diagnostics [diagnostic_msgs.msg.DiagnosticArray]
- publishers:
  - /cmd_vel_in [geometry_msgs.msg.Twist]
  - /estop [std_msgs.msg.Bool]

This bridge also exposes:
- WebSocket telemetry server on TELEMETRY_PORT (default 9000)
- HTTP /health endpoint on TELEMETRY_HTTP_PORT (default 9001)
"""
import asyncio
import json
import threading
import os
import time
from functools import partial
from typing import Any
from urllib.parse import urlparse, parse_qs

import websockets
from aiohttp import web

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String as StdString, Bool
from diagnostic_msgs.msg import DiagnosticArray


TELEMETRY_PORT = int(os.environ.get("TELEMETRY_PORT", 9000))
TELEMETRY_HTTP_PORT = int(os.environ.get("TELEMETRY_HTTP_PORT", 9001))


# Global set of active websocket connections (for /health)
_active_webs = set()


def validate_token(token: str) -> bool:
    """Simple token validator using AUTH_TOKENS env var (comma-separated)."""
    if not token:
        return False
    allowed = os.environ.get("AUTH_TOKENS", "demo_token_123")
    return token in [t.strip() for t in allowed.split(",")]


class BridgeNode(Node):
    def __init__(self, loop, telemetry_queue):
        super().__init__("teleop_bridge_node")
        self.loop = loop
        self.telemetry_queue = telemetry_queue
        self.get_logger().info("Bridge node starting...")

        # subscribers
        self.create_subscription(StdString, "/status", self.status_cb, 10)
        # self.create_subscription(Odometry, "/odom", self.odom_cb, 10)
        # self.create_subscription(DiagnosticArray, "/diagnostics", self.diag_cb, 10)

        # publishers
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel_in", 10)
        self.estop_pub = self.create_publisher(Bool, "/estop", 10)

    # --- ROS callbacks -> telemetry queue ---
    def status_cb(self, msg: StdString):
        payload = {"topic": "/status", "data": msg.data}
        self.loop.call_soon_threadsafe(self.telemetry_queue.put_nowait, json.dumps(payload))

    # def odom_cb(self, msg: Odometry):
    #     p = msg.pose.pose.position
    #     o = msg.pose.pose.orientation

    #     # twist
    #     t_lin = msg.twist.twist.linear
    #     t_ang = msg.twist.twist.angular

    #     # header (stamp may be builtin_time type with sec/nanosec)
    #     header = {}
    #     try:
    #         header = {
    #             "stamp": {
    #                 "sec": getattr(msg.header.stamp, "sec", None),
    #                 "nanosec": getattr(msg.header.stamp, "nanosec", None),
    #             },
    #             "frame_id": getattr(msg.header, "frame_id", ""),
    #         }
    #     except Exception:
    #         # be defensive if header fields aren't present
    #         header = {
    #             "stamp": {
    #                 "sec": None,
    #                 "nanosec": None,
    #             },
    #             "frame_id": "",
    #         }

    #     payload = {
    #         "topic": "/odom",
    #         "header": header,
    #         "child_frame_id": getattr(msg, "child_frame_id", ""),
    #         "pose": {
    #             "pose": {
    #                 "position": {
    #                     "x": float(p.x), "y": float(p.y), "z": float(p.z)
    #                 },
    #                 "orientation": {
    #                     "x": float(o.x), "y": float(o.y), "z": float(o.z), "w": float(o.w)
    #                 },
    #             },
    #             "covariance": []
    #         },
    #         "twist": {
    #             "twist": {
    #                 "linear": {
    #                     "x": float(t_lin.x), "y": float(t_lin.y), "z": float(t_lin.z)
    #                 },
    #                 "angular": {
    #                     "x": float(t_ang.x), "y": float(t_ang.y), "z": float(t_ang.z)
    #                 },
    #             },
    #             "covariance": []
    #         },
    #     }

    #     # safe_payload = self._make_json_safe(payload)
    #     self.loop.call_soon_threadsafe(self.telemetry_queue.put_nowait, json.dumps(payload))

    def _normalize_level(self, level):
        if isinstance(level, int):
            return level
        if isinstance(level, (bytes, bytearray)):
            if len(level) == 1:
                return level[0]
            try:
                return int.from_bytes(level, byteorder='little', signed=False)
            except Exception:
                try:
                    return int(level.decode('utf-8', errors='ignore') or 0)
                except Exception:
                    return 0
        try:
            return int(level)
        except Exception:
            return 0

    def _make_json_safe(self, obj):
        if isinstance(obj, (bytes, bytearray)):
            try:
                return obj.decode('utf-8')
            except Exception:
                if len(obj) == 1:
                    return obj[0]
                return str(obj)
        if isinstance(obj, (str, int, float, bool)) or obj is None:
            return obj
        if isinstance(obj, dict):
            return {k: self._make_json_safe(v) for k, v in obj.items()}
        if isinstance(obj, (list, tuple)):
            return [self._make_json_safe(v) for v in obj]
        return str(obj)

    # def diag_cb(self, msg: DiagnosticArray):
    #     diagnostics = []
    #     for status in msg.status:
    #         level_val = self._normalize_level(status.level)
    #         diagnostics.append({
    #             "name": status.name,
    #             "level": level_val,
    #             "message": status.message,
    #             "hardware_id": status.hardware_id,
    #             "values": [
    #                 {"key": v.key, "value": str(v.value)} for v in status.values
    #             ]
    #         })

    #     payload = {"topic": "/diagnostics", "diagnostics": diagnostics}
    #     safe_payload = self._make_json_safe(payload)

    #     try:
    #         serialized = json.dumps(safe_payload)
    #     except Exception as e:
    #         self.get_logger().error(f"JSON serialization error: {e}")
    #         self.get_logger().error(f"Payload: {repr(safe_payload)}")
    #         for status in msg.status:
    #             for v in status.values:
    #                 self.get_logger().error(f"Key: {v.key}, Value: {repr(v.value)}, Type: {type(v.value)}")
    #         return

    #     self.loop.call_soon_threadsafe(self.telemetry_queue.put_nowait, serialized)

    # --- publish helpers for incoming messages ---
    def publish_estop(self, value: bool):
        msg = Bool()
        msg.data = value
        self.estop_pub.publish(msg)

    def publish_cmd(self, cmd: dict):
        """Convert a cmd_vel dict into Twist and publish. No heavy validation here."""
        t = Twist()
        lin = cmd.get("linear", {}) or {}
        ang = cmd.get("angular", {}) or {}

        # defensive get with defaults
        t.linear.x = float(lin.get("x", 0.0))
        t.linear.y = float(lin.get("y", 0.0))
        t.linear.z = float(lin.get("z", 0.0))

        t.angular.x = float(ang.get("x", 0.0))
        t.angular.y = float(ang.get("y", 0.0))
        t.angular.z = float(ang.get("z", 0.0))

        # TODO: clamp values to safe ranges before publishing
        self.cmd_pub.publish(t)


# --- WebSocket handler ---
# --- WebSocket handler (robust for multiple websockets versions) ---
async def websocket_handler(websocket: Any, *args, node, telemetry_queue):
    """
    Compatible handler:
      - tries to extract ?token=... from common protocol attrs / args
      - if not found, expects the client to send an initial auth JSON message
        within a short timeout, e.g. {"token": "demo_token_123"} or
        {"type":"auth","token":"..."}.
    """
    # Try several candidate sources for a token (some websockets versions set different attrs)
    candidates = []
    if len(args) >= 1:
        candidates.append(("args[0]", args[0]))
    candidates.append(("websocket.path", getattr(websocket, "path", None)))
    candidates.append(("websocket.request_uri", getattr(websocket, "request_uri", None)))
    candidates.append(("websocket.raw_request_path", getattr(websocket, "raw_request_path", None)))
    # request_headers may be mapping-like; include for debug
    try:
        hdrs = getattr(websocket, "request_headers", None)
        candidates.append(("websocket.request_headers", dict(hdrs) if hdrs else None))
    except Exception:
        candidates.append(("websocket.request_headers", None))

    # Try parse token from any candidate using urlparse
    token = None
    for name, cand in candidates:
        if not cand:
            continue
        try:
            parsed = urlparse(str(cand))
            qs = parse_qs(parsed.query)
            candidate_token = qs.get("token", [None])[0]
            if candidate_token:
                token = candidate_token
                break
        except Exception:
            continue

    # If token was not present in path/url, accept an initial auth message from the client
    if not token:
        try:
            # short timeout for initial auth message
            init = await asyncio.wait_for(websocket.recv(), timeout=2.0)
            try:
                payload = json.loads(init)
            except Exception:
                payload = None

            # support several shapes: {"token":"..."}, {"type":"auth","token":"..."},
            # or {"auth": {"token":"..."}}
            if isinstance(payload, dict):
                if payload.get("token"):
                    token = payload.get("token")
                elif payload.get("type") == "auth" and payload.get("token"):
                    token = payload.get("token")
                elif isinstance(payload.get("auth"), dict) and payload["auth"].get("token"):
                    token = payload["auth"].get("token")
                else:
                    # Not an auth message — treat it as auth failure
                    await websocket.close(code=4401, reason="invalid token parse")
                    return
            else:
                await websocket.close(code=4401, reason="invalid token parse")
                return
        except asyncio.TimeoutError:
            # client did not send initial auth message
            await websocket.close(code=4401, reason="invalid token parse")
            return
        except websockets.ConnectionClosed:
            return
        except Exception:
            await websocket.close(code=4401, reason="invalid token parse")
            return

    # validate token
    if not validate_token(token):
        try:
            node.get_logger().warning(f"WS invalid token received: {token!r}")
        except Exception:
            print("WS invalid token received:", token)
        await websocket.close(code=4401, reason="invalid token")
        return

    # register active connection for health checks
    _active_webs.add(websocket)

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
                if "cmd_vel" in obj:
                    node.publish_cmd(obj["cmd_vel"])
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

    # unregister
    if websocket in _active_webs:
        _active_webs.remove(websocket)


def start_rclpy_spin(node):
    rclpy.spin(node)


# --- HTTP health server (aiohttp) ---
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

    # start aiohttp health server
    app = web.Application()
    app.add_routes([web.get("/health", health_handler)])
    runner = web.AppRunner(app)
    await runner.setup()
    site = web.TCPSite(runner, "0.0.0.0", TELEMETRY_HTTP_PORT)
    await site.start()
    node.get_logger().info(f"Bridge health HTTP server running on port {TELEMETRY_HTTP_PORT}")

    # start websocket server
    async with websockets.serve(
            partial(websocket_handler, node=node, telemetry_queue=telemetry_queue),
            host="0.0.0.0",
            port=TELEMETRY_PORT,
    ):
        node.get_logger().info(f"Bridge websocket server running on port {TELEMETRY_PORT}")
        await asyncio.Future()  # run forever


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
