# Development Plan

- [Development Plan](#development-plan)
  - [Features](#features)
  - [Structure](#structure)
  - [Components](#components)
    - [Tech stack](#tech-stack)
    - [System components \& roles](#system-components--roles)
    - [Topics \& messages](#topics--messages)
    - [Safety \& reliability basics (non-negotiable)](#safety--reliability-basics-non-negotiable)
    - [Quick security checklist (minimum)](#quick-security-checklist-minimum)
  - [Plan](#plan)
    - [Streetmarks](#streetmarks)
    - [Demo script](#demo-script)
    - [Testing checklist](#testing-checklist)
    - [Nice-to-have upgrades](#nice-to-have-upgrades)
  - [Tiers](#tiers)


## Features

- Secure web dashboard (operator UI) with: live camera, telemetry (battery, CPU, pose), and joystick/keyboard control.
- Low-latency video streaming (WebRTC or GStreamer).
- ROS2 bridge to expose topics (camera, odom, tf, status) to web.
- Safe command channel: /cmd_vel with watchdog & velocity limits + E-STOP topic.
- Dockerized services + single docker-compose up to run demo (simulator or real robot).
- Demo script & short video showing install → run → teleop.


## Structure

```sh
teleop-mvp/
├── docker-compose.yml
├── robot_core/
│   ├── Dockerfile
│   ├── src/
│   │   ├── teleop_node.py
│   │   ├── safety_watchdog.cpp
│   └── launch/
├── signaling_server/
│   ├── Dockerfile
│   └── app.py  # FastAPI + WebSocket
├── bridge_server/
│   └── Dockerfile
├── web_dashboard/
│   ├── Dockerfile
│   └── src/
├── docs/
│   ├── INSTALL.md
│   └── DEMO.md
└── scripts/
    └── demo_run.sh
```

## Components

### Tech stack

- Robot runtime: ROS2 (Humble or newer).
- Web backend: FastAPI (Rest API + WebSocket) for signaling and telemetry API.
- Web frontend: React (or plain HTML/JS) using WebRTC (get camera stream) + ros2-web-bridge or custom WebSocket for topics.
- Video transport: WebRTC (best for low-latency browser streams). GStreamer pipeline on robot to feed WebRTC.
- Signaling server: simple WebSocket server in FastAPI (aiortc or simple-peer in frontend).
- Containerization: Docker + docker-compose.
- Optional: nginx + TLS (Let's Encrypt) if exposing to internet.
- Simulation for dev: Gazebo or a small TurtleBot3 world for offline testing.


### System components & roles

- robot_core (container): ROS2 runtime, teleop driver, safety nodes, video capture (GStreamer). Publishes /camera, /odom, /status; subscribes /cmd_vel, /estop.
- signaling_server (container): FastAPI WebSocket server for WebRTC signaling and operator auth.
- bridge_server (container): ros2-web-bridge or custom node that forwards ROS2 topics to the web via WebSocket.
- web_dashboard (container): frontend app (React) that shows video, telemetry, joystick and E-STOP.
- optional stun/turn: TURN server for WebRTC reliability across NAT.


### Topics & messages

- /cmd_vel — geometry_msgs/Twist (control input)
- /estop — std_msgs/Bool or custom emergency message
- /camera/image_raw — sensor_msgs/Image (video frames) or feed via WebRTC stream
- /odom — nav_msgs/Odometry
- /tf — TF2 transforms
- /robot/status — custom status (battery, CPU, errors)


### Safety & reliability basics (non-negotiable)

- Watchdog: robot stops if no /cmd_vel update for X ms (e.g., 500–1000 ms).
- Velocity limits: cap linear & angular velocities from web inputs.
- E-STOP: an operator-triggered E-STOP topic with hardware override possibility.
- Fallback behaviour: on disconnect → stop → optional return-to-home.
- Authentication: token-based auth for dashboard; TLS for signaling & websockets.
- Rate limiting: prevent brute command floods from the operator.


### Quick security checklist (minimum)

- Use HTTPS/WSS for the dashboard and signaling.
- Require an auth token for WebSocket signaling and topic access.
- If exposing to internet, use a TURN server and firewall rules; consider VPN for production.
- Log operator IDs + actions for auditing.


## Plan

### Streetmarks
- Day 1–2: Repo + Docker-compose skeleton; pick ROS2 distro; start simulated robot (TurtleBot3) container.
- Day 3–5: Implement ROS2 safety node (watchdog + velocity clamp) and simple teleop node (joystick/keyboard).
- Day 6–8: Integrate video pipeline with GStreamer + local RTSP/WebRTC test.
- Day 9–11: Build signaling server + basic frontend that displays video and sends /cmd_vel.
- Day 12–14: Add auth, E-STOP, telemetry, write demo docs and record a 2-3 minute video.

### Demo script
- docker-compose up --build (runs sim or real robot stack).
- Open https://<host>:8000 → login.
- Start video feed → show low-latency camera.
- Use joystick/keyboard → robot moves; show watchdog stopping on disconnect.
- Trigger E-STOP and recover. Record all steps for a 2-3 minute video.

### Testing checklist
- Latency test: measure RTT for control loop (goal <200ms for teleop feeling).
- Packet loss scenario: simulate packet drops and verify fallback.
- Multi-operator test: ensure only one operator can control at a time or implement lock.
- CPU/memory test for video encoding on robot hardware (Jetson/RPi).
- Security test: try connecting without token.

### Nice-to-have upgrades
- Shared-autonomy mode (operator issues high-level goals).
- Haptic feedback or predictive display to hide latency.
- Multi-robot dashboard & operator handoff.
- Recording & replay of operator session for training/QA.


## Tiers

- Basic: Dockerized demo (simulation), README, 1 support hour.
- Standard: Real-robot integration, auth, E-STOP, 2-hour onboarding.
- Premium: Fleet support, TURN server + VPN setup, 8 hours of integration and testing.
