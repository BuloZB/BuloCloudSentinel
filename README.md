# Bulo.Cloud Sentinel

Bulo.Cloud Sentinel is a modular, containerized surveillance and situational awareness platform. This repository contains the core services and integrations required to run a scalable, secure, and extendable security system.

## Overview of Core Modules

1. **Incident Timeline & Smart Playback**
   - FastAPI backend serving incident events and metadata
   - React/Tailwind frontend component with thumbnails, filters, and search
   - Video segment API for synchronized playback

2. **Drone Command & Telemetry Hub**
   - FastAPI service with MAVLink telemetry and command endpoints
   - Frontend React component to display live telemetry and send commands
   - JWT-based security placeholder for authorized clients

3. **AI Model Management & Training Panel**
   - FastAPI endpoints for uploading, listing, activating, and rolling back YOLO/TensorFlow models
   - In-memory demo store (replaceable by MinIO)
   - Docker container for AI training microservice

4. **Device Inventory & Health Status**
   - Central registry API for cameras, sensors, and drones
   - Metadata storage, health checks, and PATCH configuration
   - Alerts via Novu (email/SMS) when critical devices go offline

5. **Access Audit Log & Session Inspector**
   - API for recording and retrieving user actions (login, settings, commands)
   - OpenSearch-ready JSON entries for analytics
   - Export endpoints for CSV and JSON

6. **AI Integrations Microservice**
   - Unified FastAPI service under `bulo-sentinel-ai/`
   - Modular adapters for:
     - ChatGPT (OpenAI)
     - Claude (Anthropic)
     - Gemini (Google)
     - DALL·E (OpenAI image)
     - Whisper (OpenAI speech-to-text)
   - Endpoints:
     - `POST /ai/chat`
     - `POST /ai/vision/analyze`
     - `POST /ai/audio/transcribe`
     - `GET /ai/status`
   - Persistent Audit Logging: `ai_audit_log.json`
   - Prometheus metrics at `/monitoring/metrics`
   - OpenAPI documentation under `bulo-sentinel-ai/openapi.yaml`
   - Frontend AI Tools Panel React component

## Deployment

All services are containerized and orchestrated via `docker-compose.yml`.  
- `backend` at port 8000  
- `ai_detection` at port 8001  
- `ai_integrations` at port 8002  
- `frontend` at port 3000  
- `rtmp_server` at ports 1935 and 8080  
- `db` (Postgres), `redis`, and `minio` for persistence

### Environment Variables

Place secrets in a `.env` file or CI/CD secrets:

```
# Backend / AI Integrations
CHATGPT_API_KEY=…
CLAUDE_API_KEY=…
GEMINI_API_KEY=…
DALLE_API_KEY=…
WHISPER_API_KEY=…
AI_AUDIT_LOG_FILE=/data/ai_audit_log.json

# Database
POSTGRES_USER=…
POSTGRES_PASSWORD=…
POSTGRES_DB=…
```

## Documentation

- OpenAPI / Swagger for each FastAPI service under `/docs`
- AI Integrations schema in `bulo-sentinel-ai/openapi.yaml`
- High-level architecture and module details in `DOCUMENTATION.md`

## Changelog

See the [CHANGELOG.md](CHANGELOG.md) (or GitHub Releases) for detailed version-by-version updates.

---

All naming, logs, UI labels, and code structures use “Bulo.Cloud Sentinel” branding.
