# signaling_server/Dockerfile
FROM python:3.10-slim

ENV PYTHONUNBUFFERED=1 \
    APP_HOME=/app

WORKDIR ${APP_HOME}

# system deps for websockets/aiortc if you extend to WebRTC (light install here)
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    libssl-dev \
    && rm -rf /var/lib/apt/lists/*

# copy requirements
COPY ./requirements.txt ./
RUN pip install --no-cache-dir -r requirements.txt

# copy app
COPY ./app.py ./app.py
COPY ./auth.py ./auth.py
# COPY ./static ./static

EXPOSE 8443

# Run with uvicorn; you can add --ssl-keyfile/--ssl-certfile when in production
CMD ["uvicorn", "app:app", "--host", "0.0.0.0", "--port", "8443", "--ws", "websockets"]
