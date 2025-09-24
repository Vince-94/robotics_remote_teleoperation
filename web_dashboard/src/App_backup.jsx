import React, { useState } from "react";


export default function App() {
  const [telemetry, setTelemetry] = useState({ battery: "N/A", cpu: "N/A", status: "disconnected" });

  // stub functions: replace with websocket/ws/rosbridge calls later
  const sendCommand = (cmd) => {
    console.log("sendCommand:", cmd);
    // TODO: send to /cmd_vel via ws
  };
  const estop = () => {
    console.log("E-STOP pressed");
    // TODO: publish /estop true
  };

  return (
    <div style={{ fontFamily: "sans-serif", padding: 16 }}>
      <h2>Teleop MVP — Demo Dashboard</h2>

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
              <button onClick={() => sendCommand({ linear: 0.3, angular: 0 })}>↑</button>
              <button onClick={() => sendCommand({ linear: 0, angular: 0 })}>⏸</button>
              <button onClick={() => sendCommand({ linear: -0.3, angular: 0 })}>↓</button>

              <button onClick={() => sendCommand({ linear: 0, angular: -0.8 })}>←</button>
              <button onClick={() => sendCommand({ linear: 0, angular: 0 })}>⏺</button>
              <button onClick={() => sendCommand({ linear: 0, angular: 0.8 })}>→</button>
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
