# Web Dashboard

- [Web Dashboard](#web-dashboard)
  - [Overview](#overview)
  - [Quickstart (development)](#quickstart-development)
    - [Local (fast iteration)](#local-fast-iteration)
    - [Production / Docker](#production--docker)
  - [Configuration](#configuration)
  - [Files of interest](#files-of-interest)
  - [Integration notes / sample snippets](#integration-notes--sample-snippets)
    - [Example: connect to bridge WebSocket and handle telemetry](#example-connect-to-bridge-websocket-and-handle-telemetry)
    - [Example: WebRTC (high level)](#example-webrtc-high-level)
  - [Next steps / checklist to make it fully operational](#next-steps--checklist-to-make-it-fully-operational)


## Overview

Simple React dashboard for the **Robotic Remote Teleoperation** demo. This component is intentionally minimal and focused on the teleop demo UX: video panel, telemetry widgets, basic teleop buttons and an emergency stop.

* The React app renders a static single-page UI (`App.jsx`) with three areas: Video feed (WebRTC), Telemetry, and Controls (teleop + E‑STOP).
* Currently `App.jsx` keeps telemetry in local state (`battery`, `cpu`, `status`) and provides **stub** functions `sendCommand()` and `estop()` that print to the console. No live network integration is enabled by default.

Runtime flow (how the final integration should work):

1. On load the dashboard authenticates (e.g. via `POST /api/login` on the signaling backend) and obtains a token.
2. The dashboard opens a WebSocket to the **bridge** or **signaling** server for telemetry and command messages:

   * Example telemetry WS (bridge): `ws://<bridge-host>:9000/?token=<token>` or `ws://<bridge-host>:9000/` then send initial `{"token":"..."}` message if required.
   * Example signaling WS: `wss://<backend-host>:8443/ws?token=<token>` (used for WebRTC signaling).
3. Telemetry messages arrive as JSON text frames. The app should parse these and call `setTelemetry(...)` accordingly.
4. When the operator presses teleop buttons, the app sends a JSON command frame such as `{"cmd_vel": {"linear": {"x":0.3}, "angular": {"z":0.0}}}` to the bridge (or via signaling server which forwards to the bridge).
5. E-STOP sends `{"estop": true}` which the bridge republishes to `/estop` in ROS.
6. Video: the dashboard obtains an SDP offer/answer through the signaling server (WebRTC). After the WebRTC connection is established the remote video track is attached to a `<video>` element in the UI.

---

## Quickstart (development)

### Local (fast iteration)

```bash
# from web_dashboard/
npm ci
# if your project uses Vite or a dev server
npm run dev
# open http://localhost:3000 (or port printed by the dev server)
```

> If your package.json does not include `dev`/`start` scripts, run `npm run build` and serve the `build/` directory as below.

### Production / Docker

1. Build the container: `docker-compose build web_dashboard`
2. Start the container: `docker-compose up web_dashboard`
3. Interactive mode: `docker run --rm -p 3000:3000 --name web_dashboard web_dashboard`
4. Open http://localhost:3000
5. Close the container: `docker-compose down web_dashboard`

---

## Configuration

* The app currently uses hardcoded/demo behavior. For integration you should provide these config points (via environment variables, a small `config.js`, or by embedding into the build):

  * `REACT_APP_SIGNAL_URL` — signaling backend URL (wss\://...)
  * `REACT_APP_BRIDGE_WS` — bridge websocket URL (ws\://...)
  * `REACT_APP_API_BASE` — REST API base for authentication `/api/login` etc.

When building the Docker image, pass env values at runtime or bake them into a small `config.json` served by the static files.

---

## Files of interest

* `src/App.jsx` — main UI component (video placeholder, telemetry, controls). Add networking hooks here.
* `src/main.jsx` — React entry point.
* `public/` or `build/` — static assets after build.
* `web_dashboard/Dockerfile` — multi-stage build and `serve` command for static hosting.

---

## Integration notes / sample snippets

### Example: connect to bridge WebSocket and handle telemetry

```js
// inside a useEffect hook
const ws = new WebSocket("ws://localhost:9000/?token=demo_token_123");
ws.onopen = () => {
  // optional: if server expects initial auth JSON, send it
  ws.send(JSON.stringify({ token: 'demo_token_123' }));
};
ws.onmessage = (e) => {
  try {
    const obj = JSON.parse(e.data);
    if (obj.topic === '/status' && obj.data) {
      setTelemetry(t => ({ ...t, status: obj.data }));
    }
    if (obj.topic === '/telemetry' && obj.payload) {
      setTelemetry({ battery: obj.payload.battery, cpu: obj.payload.cpu, status: 'connected' });
    }
    // handle other topics: /odom, /diagnostics
  } catch (err) {
    console.warn('invalid telemetry frame', err);
  }
};

// sending a cmd_vel
ws.send(JSON.stringify({ cmd_vel: { linear: { x: 0.3 }, angular: { z: 0.0 } } }));
// sending estop
ws.send(JSON.stringify({ estop: true }));
```

### Example: WebRTC (high level)

1. Fetch ICE servers / token from signaling backend.
2. Create RTCPeerConnection and set `ontrack` to attach received remote stream to a `<video>` element.
3. Exchange SDP via signaling WebSocket (`offer` / `answer` / `ice` messages).

---

## Next steps / checklist to make it fully operational

* [ ] Replace video placeholder with a `<video>` element and wire WebRTC on top of the `signaling_server`.
* [ ] Implement WebSocket client to bridge for telemetry and commands; move `sendCommand` / `estop` to use that socket.
* [ ] Add authentication flow: login UI, token storage (in-memory), token refresh.
* [ ] Add per-control safety UI (dead-man switch, velocity sliders, watchdog indicators).
* [ ] Improve styling and responsive layout (mobile friendly).
