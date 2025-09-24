# FastAPI Signaling / Backend

- [FastAPI Signaling / Backend](#fastapi-signaling--backend)
  - [Overview](#overview)
  - [How to use](#how-to-use)
  - [Configuration](#configuration)
  - [HTTP endpoints (summary)](#http-endpoints-summary)
  - [WebSocket signaling (current behavior)](#websocket-signaling-current-behavior)
  - [Implementation notes (files of interest)](#implementation-notes-files-of-interest)
  - [Limitations \& next steps (recommended)](#limitations--next-steps-recommended)
  - [Roadmap — signaling\_server (demo → production)](#roadmap--signaling_server-demo--production)
    - [Demo (minimum viable demo)](#demo-minimum-viable-demo)
    - [Short term (improve UX \& safety)](#short-term-improve-ux--safety)
    - [Mid term (reliability \& integrations)](#mid-term-reliability--integrations)
    - [Production hardening](#production-hardening)
    - [Observability, debugging \& developer UX](#observability-debugging--developer-ux)
    - [Optional / Nice-to-have features](#optional--nice-to-have-features)
    - [Prioritization notes](#prioritization-notes)


## Overview

Minimal FastAPI-based signaling + auth service for **Robotic Remote Teleoperation** — compact, production-minded, and easy to run locally.

* Serves a health HTML page at `/`.
* Provides a structured JSON health check at `/health` (status, service name, active connections).
* Accepts WebSocket clients at `/ws?token=<token>` and brokers text messages between all connected clients (simple broadcast/echo).
* Uses a tiny token validator (`signaling_server/auth.py`) that checks `AUTH_TOKENS` env var (comma-separated list). Not JWT in this minimal implementation.

> Note: this is intentionally small to keep the demo simple. See "Next steps" for production improvements.


## How to use

1. Build the container: `docker-compose build signaling`
2. Start the container: `docker-compose up signaling`
3. Interactive mode: `docker-compose run --rm signaling bash`
4. Open:
   - production: `http://<HOST>:8000/`
   - local testing: `http://localhost:8000/`
5. Health check: `curl http://localhost:8000/health`
6. Connect with token: `wscat -c "ws://localhost:8000/ws?token=demo_token_123"`
7. Close the container: `docker-compose down signaling`


## Configuration

* `AUTH_TOKENS` — comma-separated tokens allowed to connect (default: `demo_token_123`)
* `DEMO_MODE` — not used by the current `app.py`, but may be useful if you expand auth
* `TURN_URL`, `TURN_USER`, `TURN_PASS` — not used by current app but reserved for TURN
* `SIGNAL_HOST` / `SIGNAL_PORT` — bind settings controlled by your ASGI server (uvicorn/docker)


## HTTP endpoints (summary)

* `GET /` — simple HTML sanity page
* `GET /health` — JSON health response

> The minimal `app.py` does not expose the higher-level REST APIs (`/api/login`, `/api/session`, `/api/estop`) described previously — those are planned features.


## WebSocket signaling (current behavior)

Connect to: `ws://<host>:<port>/ws?token=<token>`

Behavior:

* On connect, `auth.validate_token(token)` is called. If validation fails, the socket is closed with code `4401`.
* All received text frames are broadcast to every currently connected client (simple pub/sub with global scope).
* Clients should send/receive JSON text messages (the server does not parse message types; it treats them as opaque strings).

Example client message (any text/JSON):

```json
{ "type": "offer", "sdp": "v=0..." }
```


## Implementation notes (files of interest)

* `signaling_server/app.py` — FastAPI app, `/` route and `/ws` WebSocket handler with a small `ConnectionManager` type that tracks active `WebSocket` objects and broadcasts messages.
* `signaling_server/auth.py` — `validate_token()` uses `AUTH_TOKENS` env var to allow simple token-based auth.
* `signaling_server/Dockerfile`, `requirements.txt` — containerization and deps.


## Limitations & next steps (recommended)

* Replace `AUTH_TOKENS` with proper JWT-based auth and short-lived tokens for production.
* Implement per-session routing (currently broadcast is global). Add a `session` query parameter or `join` message that scopes messages to a session room.
* Add explicit message parsing with typed payloads: `join`, `offer`, `answer`, `ice`, `telemetry`, `leave`, and authorization checks for sensitive messages (e.g., estop).
* Persist session state in Redis to support multiple replicas behind a load balancer.
* Add TLS (WSS) via reverse proxy and hardened rate limits.


## Roadmap — signaling_server (demo → production)

### Demo (minimum viable demo)
- [x] Keep current simple `/` health page and `/ws?token=...` WebSocket broadcast
- [x] Add `docs/DEMO.md` with demo tokens and simple JS client snippet
- [x] Dockerized dev image + `docker-compose` entry (already present) and demo env file
- [x] Add basic logging and a `--dev`/`DEMO_MODE` flag for verbose logs
- [x] Smoke tests: automated script that opens 2 WS clients, sends a message, verifies broadcast
- [x] `/health` JSON endpoint for smoke tests

### Short term (improve UX & safety)
- [ ] Per-session rooms: scope messages to `session_id` (query param or `join` message)
- [ ] Simple REST `POST /api/session` to create sessions and return session IDs
- [ ] `POST /api/login` returning short-lived tokens (temp in-memory creds)
- [ ] Enforce message JSON schema for known types (`join`, `offer`, `answer`, `ice`, `telemetry`, `leave`)
- [ ] Add server-side validation for `estop` messages (authorization & audit log)
- [ ] Add unit tests: auth, WS connect/close, session routing, message validation

### Mid term (reliability & integrations)
- [ ] Replace `AUTH_TOKENS` with JWT issuance + refresh / token revocation endpoint
- [ ] Move session store to Redis (pub/sub + persistent TTL) for horizontal scaling
- [ ] TURN integration: accept TURN config and support short-lived TURN credentials
- [ ] Add rate limiting (per-IP and per-token) and basic DoS protections
- [ ] Add Prometheus metrics and health/readiness endpoints
- [ ] Add integration tests with `bridge_server` and `web_dashboard` (CI job)

### Production hardening
- [ ] Run behind TLS-terminating reverse proxy (NGINX/Caddy); ensure WSS/HTTPS only
- [ ] RBAC: roles (viewer, operator, admin) and ACL checks for sensitive endpoints (estop)
- [ ] Logging/audit: persist important events (joins, estops, failed auth) to a secure store
- [ ] High-availability deployment: multiple replicas, sticky sessions or Redis-backed routing
- [ ] Secrets management: use vault/secret store for JWT secrets and TURN creds
- [ ] Penetration test / security review, and compliance checks if required

### Observability, debugging & developer UX
- [ ] Web-based admin UI to list active sessions, participants, and allow controlled admin estop
- [ ] Live replay / session recording (store SDP/ICE + signaling logs for debugging)
- [ ] Add structured logs and request tracing (OpenTelemetry)
- [ ] Developer utilities: curl/JS snippets in README, Postman collection, and a small CLI tool to create sessions/issue tokens

### Optional / Nice-to-have features
- [ ] Multi-tenant support (orgs, project IDs, per-tenant quotas)
- [ ] Session recording + replay for training and incident review
- [ ] Telemetry forwarding: optional REST/HTTP webhook to forward telemetry to external systems
- [ ] Auto-scaling TURN using a managed provider or self-hosted cluster
- [ ] WebRTC SFU integration for multi-viewer scenarios (e.g., multiple observers watching robot video)

### Prioritization notes
- **Demo → Short term:** focus on per-session routing, minimal REST APIs and message validation so the dashboard + bridge integration is realistic.
- **Mid term:** make it horizontally scalable (Redis + TURN).
- **Production:** focus on TLS, RBAC, logging/audit, and secrets management.

