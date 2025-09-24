from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse, JSONResponse
from typing import List
from auth import validate_token

app = FastAPI(title="Teleop Signaling Backend")

# basic html page for quick check
@app.get("/")
async def index():
    return HTMLResponse("<h3>Signaling server is up. Connect via WebSocket at /ws?token=...</h3>")

# health endpoint
@app.get("/health")
async def health():
    return JSONResponse(
        {
            "status": "ok",
            "service": "signaling_server",
            "active_connections": len(manager.active) if "manager" in globals() else 0,
        }
    )

# Very small WebSocket manager
class ConnectionManager:
    def __init__(self):
        self.active: List[WebSocket] = []

    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        self.active.append(websocket)

    def disconnect(self, websocket: WebSocket):
        if websocket in self.active:
            self.active.remove(websocket)

    async def broadcast(self, message: str):
        for ws in list(self.active):
            try:
                await ws.send_text(message)
            except Exception:
                self.disconnect(ws)

manager = ConnectionManager()

@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket, token: str = ""):
    # simple token check
    if not validate_token(token):
        await websocket.close(code=4401)
        return

    await manager.connect(websocket)
    try:
        while True:
            data = await websocket.receive_text()
            # Echo for now; in real use signal SDP/ICE messages between peers
            await manager.broadcast(data)
    except WebSocketDisconnect:
        manager.disconnect(websocket)
