# tests/smoke_test_ws.py
import asyncio
import json
import sys
import time
import urllib.request
from websockets import connect, exceptions

HOST = "localhost"
PORT = 8000
TOKEN = "demo_token_123"
HEALTH_URL = f"http://{HOST}:{PORT}/health"
WS_URL = f"ws://{HOST}:{PORT}/ws?token={TOKEN}"

HEALTH_RETRIES = 6
HEALTH_RETRY_DELAY = 0.5  # seconds


def check_health():
    """Poll /health with a short retry loop. Returns (ok: bool, payload_or_error)."""
    for attempt in range(1, HEALTH_RETRIES + 1):
        try:
            with urllib.request.urlopen(HEALTH_URL, timeout=2) as resp:
                body = resp.read().decode("utf-8")
                data = json.loads(body)
                # minimal validation
                if data.get("status") == "ok":
                    return True, data
                else:
                    return False, f"unhealthy payload: {data}"
        except Exception as e:
            if attempt < HEALTH_RETRIES:
                time.sleep(HEALTH_RETRY_DELAY)
                continue
            return False, f"health check failed after {HEALTH_RETRIES} attempts: {e}"
    return False, "unknown health check failure"


async def client_send_and_expect():
    """
    Connect two clients. Client A sends a message. Client B must receive it.
    """
    try:
        async with connect(WS_URL) as ws_a, connect(WS_URL) as ws_b:
            # small delay to ensure both connections ready
            await asyncio.sleep(0.1)

            msg = {"type": "smoke", "payload": {"from": "A", "text": "hello"}}
            await ws_a.send(json.dumps(msg))

            # wait for B to receive a message
            try:
                received = await asyncio.wait_for(ws_b.recv(), timeout=2.0)
            except asyncio.TimeoutError:
                print("ERROR: client B did not receive message in time", file=sys.stderr)
                return 2

            # basic validation
            try:
                obj = json.loads(received)
            except Exception:
                print("ERROR: Received non-JSON payload:", received, file=sys.stderr)
                return 3

            if obj.get("type") == "smoke" and obj.get("payload", {}).get("text") == "hello":
                print("SUCCESS: broadcast received by client B:", obj)
                return 0
            else:
                print("ERROR: payload mismatch:", obj, file=sys.stderr)
                return 4

    except exceptions.InvalidStatusCode as e:
        print("ERROR: WebSocket handshake failed — check token or server status:", e, file=sys.stderr)
        return 5
    except Exception as e:
        print("ERROR: Unexpected error:", e, file=sys.stderr)
        return 6


def main():
    ok, info = check_health()
    if not ok:
        print("ERROR: Health check failed:", info, file=sys.stderr)
        sys.exit(1)

    print("Health check OK:", info)
    code = asyncio.run(client_send_and_expect())
    sys.exit(code)


if __name__ == "__main__":
    # run: python tests/smoke_test_ws.py
    main()
