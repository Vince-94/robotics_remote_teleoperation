# Robotic Remote Teleoperation

- [Robotic Remote Teleoperation](#robotic-remote-teleoperation)
  - [Overview](#overview)
    - [Features list](#features-list)
  - [Requirements](#requirements)
  - [Workflow](#workflow)
    - [Robot Core](#robot-core)
    - [FastAPI Signaling / Backend](#fastapi-signaling--backend)
    - [ROS2-Web bridge](#ros2-web-bridge)
    - [Web Dashboard](#web-dashboard)
    - [Networking \& Docker Compose](#networking--docker-compose)
  - [How to use](#how-to-use)
    - [Setup](#setup)
  - [Roaddmap](#roaddmap)
    - [Features](#features)
    - [Safety](#safety)
    - [Telemetry](#telemetry)
    - [Logging](#logging)


## Overview

A minimal, production-minded teleoperation stack for a single mobile robot. Provides a secure web dashboard that streams low-latency video, shows telemetry, and lets an authenticated operator send safe velocity commands (joystick/keyboard) with built-in safety fallbacks.

### Features list
- Secure WebRTC video streaming (signaling via WebSocket + optional TURN).
- Telemetry streaming (pose, battery, CPU/memory) to dashboard.
- Joystick + keyboard control mapped to /cmd_vel with velocity limits.
- Watchdog safety node that stops the robot if operator disconnects or no commands arrive within a timeout.
- Operator-triggered E-STOP publishing to /estop.
- Dockerized stack and demo mode using a simulator (TurtleBot3) out of the box


## Requirements
- Docker & Docker Compose (v1.29+ recommended).
- Optional: GPU support on robot host for hardware encoding (NVIDIA drivers + nvidia-container-runtime) for Jetson/desktop.
- Modern browser (Chrome/Edge/Firefox) for the dashboard.
- ROS2 distro: Humble or later on robot_core (sim uses compatible image).


## Workflow

### Robot Core
Builds and installs a ROS2 workspace (assumes your ROS packages live under robot_core/src/) and launches a provided entrypoint (adjust launch file name).

The `robot_launch`:
- Provides Python nodes (like status_publisher.py) which talk to robot sensors, publish state, and eventually listen to teleop commands.
- Contains a launch/ file (teleop_launch.py) that wires these nodes together into a running system.

### FastAPI Signaling / Backend
A separate container (backend) runs FastAPI app for signalling (WebSocket-based). Keep signalling logic in `signaling_server/app.py`.

Its jobs is to provide API endpoints to authenticate users, manage sessions, and act as a “signaling server” for WebRTC.

### ROS2-Web bridge
A small Python bridge that uses rclpy to subscribe/publish and forwards JSON via WebSocket to the web dashboard. Place bridge logic in `bridge_server/bridge.py`.
- Subscribes/publishes to ROS topics via rclpy or rosbridge (depending on design).
- Exposes robot status (sensor data, logs, etc.) over HTTP/WebSockets to clients.
- Receives teleop commands (e.g. velocity, button presses) from the dashboard and publishes them to ROS.

### Web Dashboard
Multi-stage build for a React app. Serves the static build using serve on a soecific port.

When you run it in Docker:
- It connects to the FastAPI backend.
- Displays live robot state (status messages, telemetry) using WebSocket streams.
- Sends teleoperation commands (joystick, keyboard, buttons) → backend → ROS topics.

### Networking & Docker Compose
The glue is your docker-compose.yml, which defines:
- robot_core → ROS 2 runtime.
- backend → FastAPI signaling/bridge.
- bridge → a ROS 2 ↔ web bridge
- web_dashboard → React UI served via Node.js/NGINX.

All services are on the same Docker network so they can talk to each other by container name.

On your host, you expose ports:
- Signaling Backend API (8443:8443). Exposed publicly so browsers can negotiate connections.
- Bridge WebSocket or REST API endpoint (9000:9000). Exposed publicly so the web dashboard can send teleop commands and receive robot telemetry.
- Dashboard web UI (3000:3000). React development server. Public entry point for the web dashboard.


## How to use

### Setup
1. Build dockerfiles
   ```sh
   docker-compose build
   ```
2. Start the containers
   ```sh
   docker-compose up
   ```
3. Open the dashboard in your browser at https://<host>:8443 (or http://localhost:3000 in local dev) and log in with the demo credentials in docs/INSTALL.md.


## Roaddmap

### Features
- Web Dashboard
  - [ ] Video streaming. If you extend it with WebRTC, it can also stream video feed from the robot’s camera.

### Safety
- [ ] Watchdog node: subscribes to /cmd_vel_in (raw operator commands) and republishes to /cmd_vel after clamping and sanity checks. If no /cmd_vel_in is received for WATCHDOG_TIMEOUT_MS, publish zero velocities.
- [ ] E-STOP: an /estop topic that immediately triggers hardware-level stop (if available) and also sets a software-level block in the watchdog.
- [ ] Rate limiting: the bridge or signaling server should limit command frequency per operator.
- [ ] Single-operator lock: default behavior is first-come control; add queueing or explicit lock handing for multi-user scenarios.

### Telemetry
- [ ] /robot/status — battery %, health flags
- [ ] /robot/telemetry — CPU %, memory, temperature
- [ ] /tf, /odom — pose/transform

### Logging
- [ ] Logging is available in each container via docker-compose logs <service> and aggregated logs (optionally) via a fluentd/ELK sidecar in production.

