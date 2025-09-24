// web_dashboard/src/App.jsx
import React, { useEffect, useRef, useState } from "react";

/**
 * Bridge WS behavior expected:
 * - Accepts either a token in the URL (?token=...) OR an initial auth JSON: {"token":"..."}
 * - Sends telemetry frames as JSON text messages. Example:
 *   { "topic": "/status", "data": "motor_ready" }
 *   { "topic": "/telemetry", "payload": { "battery": 87.4, "cpu": 42 } }
 * - Accepts command frames:
 *   { "cmd_vel": { "linear": { "x":0.3 }, "angular": { "z": 0.0 } } }
 *   { "estop": true }
 */

// Use env var injected at build time or default to localhost demo
const DEFAULT_WS = "ws://localhost:9000/?token=demo_token_123";
const WS_URL = process.env.REACT_APP_BRIDGE_WS || DEFAULT_WS;

export default function App() {
  const [telemetry, setTelemetry] = useState({
    battery: "N/A",
    cpu: "N/A",
    status: "disconnected",
  });
  const [connStatus, setConnStatus] = useState("disconnected");
  const wsRef = useRef(null);
  const reconnectRef = useRef({ attempts: 0, timer: null });

  // open or re-open websocket
  const openWebSocket = () => {
    if (wsRef.current) {
      try { wsRef.current.close(); } catch (e) {}
      wsRef.current = null;
    }

    setConnStatus("connecting");
    const ws = new WebSocket(WS_URL);
    wsRef.current = ws;

    ws.onopen = () => {
      reconnectRef.current.attempts = 0;
      setConnStatus("connected");
      // send initial auth JSON in case server expects it
      try {
        const token = extractTokenFromUrl(WS_URL);
        if (!token) {
          ws.send(JSON.stringify({ token: "demo_token_123" }));
        }
      } catch (e) {
        // ignore
      }
    };

    ws.onmessage = (e) => {
      handleIncoming(e.data);
    };

    ws.onclose = (ev) => {
      setConnStatus("disconnected");
      // schedule reconnect with exponential backoff (capped)
      const attempts = ++reconnectRef.current.attempts;
      const delay = Math.min(1000 * 2 ** (attempts - 1), 5000);
      if (reconnectRef.current.timer) clearTimeout(reconnectRef.current.timer);
      reconnectRef.current.timer = setTimeout(() => {
        openWebSocket();
      }, delay);
    };

    ws.onerror = (err) => {
      // ensure status is updated; real errors logged in console for debugging
      console.warn("WebSocket error", err);
      try { ws.close(); } catch (e) {}
    };
  };

  // helper: pull token from URL if present
  function extractTokenFromUrl(url) {
    try {
      const u = new URL(url);
      return u.searchParams.get("token");
    } catch {
      // fallback: parse as raw string e.g. "/?token=..."
      const m = url.match(/token=([^&]+)/);
      return m ? decodeURIComponent(m[1]) : null;
    }
  }

  // message handling: update telemetry state
  function handleIncoming(raw) {
    let obj;
    try {
      obj = JSON.parse(raw);
    } catch {
      // ignore non-json frames
      return;
    }

    // support two styles: topic-based and generic telemetry
    if (obj.topic === "/status" && obj.data !== undefined) {
      setTelemetry((t) => ({ ...t, status: String(obj.data) }));
      return;
    }

    if (obj.topic === "/telemetry" && obj.payload) {
      const p = obj.payload;
      setTelemetry((t) => ({
        ...t,
        battery: p.battery !== undefined ? String(p.battery) : t.battery,
        cpu: p.cpu !== undefined ? String(p.cpu) : t.cpu,
        status: p.status || t.status,
      }));
      return;
    }

    // sometimes telemetry is sent at root with battery/cpu fields
    if (obj.battery !== undefined || obj.cpu !== undefined) {
      setTelemetry((t) => ({
        ...t,
        battery: obj.battery !== undefined ? String(obj.battery) : t.battery,
        cpu: obj.cpu !== undefined ? String(obj.cpu) : t.cpu,
      }));
      return;
    }

    // other messages: ignore or extend as needed
  }

  // send a command object (cmd: { linear: {...}, angular: {...} })
  const sendCommand = (cmd) => {
    console.log("sendCommand:", cmd);
    const ws = wsRef.current;
    if (!ws || ws.readyState !== WebSocket.OPEN) {
      console.warn("WS not connected — cannot send cmd");
      return;
    }
    const payload = { cmd_vel: cmd };
    ws.send(JSON.stringify(payload));
  };

  // emergency stop
  const estop = () => {
    console.log("E-STOP pressed");
    const ws = wsRef.current;
    if (!ws || ws.readyState !== WebSocket.OPEN) {
      console.warn("WS not connected — cannot send estop");
      return;
    }
    ws.send(JSON.stringify({ estop: true }));
    // update UI immediately
    setTelemetry((t) => ({ ...t, status: "E-STOPPED" }));
  };

  // lifecycle: open ws on mount, cleanup on unmount
  useEffect(() => {
    openWebSocket();
    return () => {
      if (reconnectRef.current.timer) clearTimeout(reconnectRef.current.timer);
      if (wsRef.current) {
        try { wsRef.current.close(); } catch (e) {}
      }
    };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []); // run once

  return (
    <div style={{ fontFamily: "sans-serif", padding: 16 }}>
      <h2>Teleop MVP — Demo Dashboard</h2>

      <div style={{ marginBottom: 8 }}>
        <strong>Bridge WS:</strong> <code>{WS_URL}</code>
        <div style={{ marginTop: 4 }}>
          <strong>Connection:</strong>{" "}
          <span style={{ color: connStatus === "connected" ? "green" : connStatus === "connecting" ? "orange" : "red" }}>
            {connStatus}
          </span>
        </div>
      </div>

      <div style={{ display: "flex", gap: 20 }}>
        <div style={{ flex: 1 }}>
          <div style={{ border: "1px solid #ddd", padding: 8, borderRadius: 6 }}>
            <div style={{ background: "#000", color: "#fff", height: 360, display: "flex", alignItems: "center", justifyContent: "center" }}>
              <span>Video feed (WebRTC) will appear here</span>
            </div>
          </div>
        </div>

        <div style={{ width: 320 }}>
          <section style={{ marginBottom: 12}}>
            <strong>Telemetry</strong>
            <div>Battery: {telemetry.battery}</div>
            <div>CPU: {telemetry.cpu}</div>
            <div>Status: {telemetry.status}</div>
          </section>

          <section style={{ marginBottom: 12 }}>
            <strong>Controls</strong>
            <div style={{ display: "grid", gridTemplateColumns: "1fr 1fr 1fr", gap: 8, marginTop: 8 }}>
              <button onClick={() => sendCommand({ linear: { x: 0.3 }, angular: { z: 0 } })}>↑</button>
              <button onClick={() => sendCommand({ linear: { x: 0 }, angular: { z: 0 } })}>⏸</button>
              <button onClick={() => sendCommand({ linear: { x: -0.3 }, angular: { z: 0 } })}>↓</button>

              <button onClick={() => sendCommand({ linear: { x: 0 }, angular: { z: -0.8 } })}>←</button>
              <button onClick={() => sendCommand({ linear: { x: 0 }, angular: { z: 0 } })}>⏺</button>
              <button onClick={() => sendCommand({ linear: { x: 0 }, angular: { z: 0.8 } })}>→</button>
            </div>
          </section>

          <section>
            <strong>Safety</strong>
            <div style={{ marginTop: 8 }}>
              <button onClick={estop} style={{ background: "red", color: "#fff", padding: "8px 12px", borderRadius: 6, border: "none" }}>E-STOP</button>
            </div>
          </section>
        </div>
      </div>
    </div>
  );
}
