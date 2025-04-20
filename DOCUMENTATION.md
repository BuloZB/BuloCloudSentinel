# Bulo.Cloud Sentinel Documentation

## Introduction
Bulo.Cloud Sentinel is an enterprise-grade video surveillance and situational awareness platform. It combines real-time video streaming, AI-powered analytics, and secure control of drones and IoT devices into a modular, containerized solution.

## Core Modules

### 1. Incident Timeline & Smart Playback
- **Backend**: FastAPI service (`/incident-timeline` endpoints) serving incident logs with timestamps, AI labels, thumbnails, and metadata filters.
- **Frontend**: React + Tailwind component (`IncidentTimeline.js`) with event list, label filtering, metadata search, and synchronized video clip highlights.
- **Storage**: Events stored in a database (PostgreSQL) with full-text search through OpenSearch integration.

### 2. Drone Command & Telemetry Hub
- **Backend**: FastAPI service (`/drone-hub` endpoints) for real-time MAVLink telemetry ingestion and command dispatch (takeoff, waypoint, return home) via dynamic adapters (MAVSDK/DroneBridge).
- **Frontend**: React component (`DroneCommandHub.js`) displaying map view (Mapbox/Leaflet), telemetry data, and command UI with JWT security.
- **Video Relay**: GStreamer WebRTC/RTSP for live video streaming.

### 3. AI Model Management & Training Panel
- **Backend**: FastAPI service (`/ai-model-management`) with endpoints for upload, list, activate, and rollback of custom YOLOv8/TensorFlow models.
- **Storage**: MinIO for model blobs and JSON metadata.
- **Training**: Celery worker or cron-based batch jobs training on labeled incident footage; hot-swap support.

### 4. Device Inventory & Health Status
- **Backend**: FastAPI service (`/device-inventory`) managing registry of cameras, sensors, and drones with metadata (zone, IP, status, last activity, critical flag).
- **Frontend**: React UI for device dashboard, health indicators, manual restart/config actions.
- **Alerts**: Novu notification integration for critical device offline events.

### 5. Access Audit Log & Session Inspector
- **Backend**: FastAPI service (`/audit-log`) capturing all user sessions, logins, settings changes, and command usage.
- **Search & Export**: Full filtering, sorting, and export to CSV/JSON; OpenSearch indexing for analytics.
- **Security**: Keycloak-based RBAC for admin access.

### 6. AI Integrations Microservice
- **Location**: `bulo-sentinel-ai/`
- **Architecture**: Modular adapter pattern implementing a base `AIAdapter` interface.
- **Adapters**:
  - ChatGPT (OpenAI)
  - Claude (Anthropic)
  - Gemini (Google)
  - DALL·E (OpenAI Image)
  - Whisper (OpenAI Speech-to-Text)
- **Endpoints**:
  - `POST /ai/chat`  
  - `POST /ai/vision/analyze`  
  - `POST /ai/audio/transcribe`  
  - `GET /ai/status`  
- **Logging**: Persistent JSON audit log (`ai_audit_log.json`) via `bulo-sentinel-ai/audit_log.py`.
- **Monitoring**: Prometheus metrics (`/monitoring/metrics`) using middleware and a dedicated router (`bulo-sentinel-ai/monitoring.py`).
- **Documentation**: OpenAPI schema in `bulo-sentinel-ai/openapi.yaml`.

## Deployment & Orchestration

- **Docker Compose** (`docker-compose.yml`) orchestrates all services:
  - `backend` (8000)
  - `ai_detection` (8001)
  - `ai_integrations` (8002)
  - `frontend` (3000)
  - `rtmp_server` (1935/8080)
  - `db`, `redis`, `minio` for persistence
- **Dockerfiles** are available per service for container builds.
- **Health Checks**: Each container exposes `/health` or metrics endpoints for CI/CD readiness checks.

## Configuration

Configure all services via environment variables and `.env` files.  
- Backend & AI Integrations: API keys, audit log path  
- Database: Postgres credentials  
- MinIO: Access keys  
- Monitoring: Prometheus scraping configuration

## Changelog Highlights

- Added AI integrations microservice with multiple adapters and audit logging.  
- Implemented modular FastAPI services for drones, incidents, devices, and audits.  
- Built React/Tailwind frontend components for timeline, drone control, and AI tools.  
- Integrated Prometheus metrics and OpenAPI documentation.  
- Updated `docker-compose.yml` to include new AI microservice and monitoring volumes.

---
This documentation reflects the latest changes and enhancements across the Bulo.Cloud Sentinel platform.
