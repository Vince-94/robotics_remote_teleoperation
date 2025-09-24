# ROS-WebSocket Bridge Server

- [ROS-WebSocket Bridge Server](#ros-websocket-bridge-server)
  - [Overview](#overview)
  - [ROS topics (current mapping)](#ros-topics-current-mapping)
  - [Message formats](#message-formats)
    - [Telemetry (sent by bridge)](#telemetry-sent-by-bridge)
    - [Commands (expected from clients)](#commands-expected-from-clients)
  - [How to use](#how-to-use)
    - [Local test](#local-test)
  - [Implementation notes](#implementation-notes)
  - [Limitations \& recommended improvements](#limitations--recommended-improvements)
  - [Testing \& smoke tests](#testing--smoke-tests)
  - [Roadmap](#roadmap)


## Overview

Lightweight ROS 2 ↔ Web telemetry bridge used by the Robotic Remote Teleoperation stack.

The bridge exposes robot telemetry over a single WebSocket server and accepts lightweight commands (cmd\_vel, estop) from web clients. It's designed to be simple and robust for demo and integration with the `signaling_server` + `web_dashboard`.

* Subscribes to ROS topics and forwards JSON-serialized telemetry to connected WebSocket clients.
* Receives JSON commands from WebSocket clients and publishes them to ROS topics.
* Converts non-JSON-safe values (bytes, diagnostic fields) to JSON-friendly types before sending.

**Default port:** `9000` (`TELEMETRY_PORT`)

---

## ROS topics (current mapping)

**Subscribers (from robot → bridge):**

* `/status` — `std_msgs/String` (serialized into `{topic: "/status", data: ...}`)
* `/odom` — `nav_msgs/Odometry` (bridge publishes simplified position: `{topic: "/odom", position: {x,y,z}}`)
* `/diagnostics` — `diagnostic_msgs/DiagnosticArray` (converted to a JSON-friendly diagnostics array)

**Publishers (from bridge → robot):**

* `/cmd_vel_in` — `geometry_msgs/Twist` (receives `{"cmd_vel": {...}}` JSON and republishes)
* `/estop` — `std_msgs/Bool` (receives `{"estop": true}` and republishes)

> [!Important] Topics and names are configurable by editing `bridge.py` if you want different remapping.

---

## Message formats

### Telemetry (sent by bridge)

Bridge sends JSON strings over the WebSocket. Example payloads:

* Status
    ```json
    {"topic":"/status","data":"motor_ready"}
    ```

* Odometry (position only)
    ```json
    {"topic":"/odom","position":{"x":1.23,"y":0.0,"z":0.0}}
    ```

* Diagnostics
    ```json
    {"topic":"/diagnostics","diagnostics":[{"name":"motor","level":0,"message":"OK","hardware_id":"hw0","values":[{"key":"temp","value":"42"}]}]}
    ```

Bridge tries to make values JSON-safe (`_make_json_safe`) by decoding bytes or converting single-byte values to integers.

### Commands (expected from clients)

Send JSON text frames with one of the keys below. The bridge will ignore unknown payloads.

* `cmd_vel`: expected structure matching `geometry_msgs/Twist` (you can send only linear/angular fields you need):
    ```json
    {"cmd_vel": {"linear": {"x":0.2,"y":0.0,"z":0.0}, "angular": {"x":0.0,"y":0.0,"z":0.1}}}
    ```

* `estop`: boolean
    ```json
    {"estop": true}
    ```

---

## How to use

### Local test
1. Build the container: `docker-compose build bridge`
2. Start the container: `docker-compose up bridge`
3. Interactive mode: `docker run --rm -p 9000:9000 --name bridge bridge_server`
4. Start the application: `python3 bridge.py`
5. Close the container: `docker-compose down bridge`


---

## Implementation notes

* `BridgeNode` spins `rclpy` in a background thread while using an asyncio event loop for WebSocket serving.
* Telemetry is queued into an `asyncio.Queue` by ROS callbacks using `loop.call_soon_threadsafe(...)` for safe cross-thread passing.
* `websockets.serve()` dispatches `websocket_handler` which runs `sender` and `receiver` tasks: sender drains telemetry queue and sends messages; receiver parses JSON and calls node publishing helpers.
* Robust JSON handling: `_normalize_level` and `_make_json_safe` attempt multiple strategies to convert weird types into JSON-serializable values.

---

## Limitations & recommended improvements

* **No authentication** on the telemetry WebSocket. Add token-based auth or integrate with `signaling_server` for authenticated sessions.
* **No per-session scoping**: the current server broadcasts telemetry to all connected clients. Consider adding session rooms or client registration to scope telemetry to specific operators.
* **Limited message validation**: the bridge naively accepts `cmd_vel` payloads and converts them. Add strict validation and clamping (safety limits) before publishing to `/cmd_vel_in`.
* **No rate-limiting**: add rate limiting for incoming commands to protect the robot.
* **No persistence/HA**: if you scale bridge instances, use Redis or another pub/sub to distribute telemetry and commands.

---

## Testing & smoke tests

* Reuse `tests/smoke_test_ws.py` (from the signaling server) but point the WS URL to `ws://<bridge-host>:9000` to verify telemetry flows and command round-trip.
* Add unit tests for `_make_json_safe` and `_normalize_level` to ensure diagnostic payloads are handled correctly.

---

## Roadmap

* [ ] Add token-based auth and session registration (integrate with `signaling_server`).
* [ ] Implement per-session rooms and optional replay/recording of telemetry.
* [ ] Validate / clamp `cmd_vel` inputs and add watchdog safety that zeros velocities if commands stop arriving.
* [ ] Add Prometheus metrics and health endpoint (e.g., `/health`) for Docker/compose probes.
* [ ] Provide a simple demo client (JS) that connects, shows telemetry and sends `cmd_vel`/`estop`.
* [ ] Optional: integrate aiortc to forward live video/audio (if you want the bridge to also proxy media).

