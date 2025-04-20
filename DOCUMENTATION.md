[![CI](https://github.com/BuloZB/BuloCloudSentinel/actions/workflows/ci.yml/badge.svg)](https://github.com/BuloZB/BuloCloudSentinel/actions/workflows/ci.yml) [![Docker](https://img.shields.io/docker/cloud/build/bulozbd/bulo-cloud-sentinel)](https://hub.docker.com/r/bulozbd/bulo-cloud-sentinel) [![License](https://img.shields.io/github/license/BuloZB/BuloCloudSentinel)]()

# 📖 Bulo.Cloud Sentinel Documentation

## 💡 Introduction
Bulo.Cloud Sentinel is an enterprise-grade video surveillance and situational awareness platform combining real-time video streaming, AI analytics, and secure drone/IoT control in a modular, containerized architecture.

## 📚 Core Modules

### 🎥 Incident Timeline & Smart Playback
- **Backend**: FastAPI events API (`/incident-timeline`)
- **Frontend**: React + Tailwind UI (`IncidentTimeline.js`)
- **Storage**: PostgreSQL + OpenSearch for full-text search

### 🚁 Drone Command & Telemetry Hub
- **Backend**: FastAPI MAVLink telemetry & command service (`/drone-hub`)
- **Frontend**: React map & telemetry UI (`DroneCommandHub.js`)
- **Streaming**: GStreamer WebRTC/RTSP relay

### 🤖 AI Model Management & Training Panel
- **Backend**: FastAPI model CRUD (`/ai-model-management`)
- **Storage**: MinIO object storage
- **Training**: Scheduled Celery jobs for YOLO/TensorFlow models

### 📋 Device Inventory & Health Status
- **Backend**: FastAPI catalog & health checks (`/device-inventory`)
- **Alerts**: Novu email/SMS notifications for offline devices

### 🔒 Access Audit Log & Session Inspector
- **Backend**: FastAPI audit endpoints (`/audit-log`)
- **Export**: CSV/JSON, OpenSearch indexing
- **Security**: Keycloak RBAC for admin access

### 🧠 AI Integrations Microservice
- **Location**: `bulo-sentinel-ai/`
- **Pattern**: Adapter interface (`AIAdapter`)
- **Adapters**: ChatGPT, Claude, Gemini, DALL·E, Whisper
- **Endpoints**:
  - `POST /ai/chat` 💬
  - `POST /ai/vision/analyze` 🖼️
  - `POST /ai/audio/transcribe` 🎙️
  - `GET /ai/status` 📊
- **Audit Log**: `ai_audit_log.json`
- **Monitoring**: `/monitoring/metrics` via Prometheus
- **Docs**: OpenAPI spec at `bulo-sentinel-ai/openapi.yaml`

## ☸️ Deployment & Orchestration
- **Docker Compose** (`docker-compose.yml`):
  - `backend` (8000)  
  - `ai_detection` (8001)  
  - `ai_integrations` (8002)  
  - `frontend` (3000)  
  - `rtmp_server` (1935/8080)  
  - `db`, `redis`, `minio`  
- **Commands**:
  ```bash
  docker-compose up -d
  ```

## ⚙️ Configuration
- Environment variables via `.env`:
  ```bash
  # AI Integrations
  CHATGPT_API_KEY=…
  CLAUDE_API_KEY=…
  GEMINI_API_KEY=…
  DALLE_API_KEY=…
  WHISPER_API_KEY=…
  AI_AUDIT_LOG_FILE=/data/ai_audit_log.json
  ```

## 🗂️ Documentation & Wiki
- **API Docs**: `/docs` on each FastAPI service  
- **Wiki**: https://github.com/BuloZB/BuloCloudSentinel/wiki

## 📅 Changelog Highlights
- 🆕 Added AI integrations microservice with modular adapters  
- 🔄 Implemented audit logging, Prometheus metrics, and OpenAPI docs  
- ⚙️ Integrated new services into `docker-compose.yml`  
- 📚 Updated README and Documentation with badges and icons

---
_All naming, logs, UI labels, and code structures use “Bulo.Cloud Sentinel” branding._

## 📝 User Guides & Tutorials

### Mission Planner
- How to create, edit, and manage missions
- Importing and exporting KML/KMZ mission files
- Using the 2D map and 3D mission preview
- Real-time mission control and manual overrides

### Drone Command Hub
- Sending commands and monitoring telemetry
- Switching between manual and automated flight modes

### AI Integrations
- Using AI tools for alert summarization and analysis
- Configuring API keys and audit logging

## 📦 Deployment Manifests

- Dockerfiles for all services are in their respective directories
- `docker-compose.yml` includes all core and AI microservices
- Kubernetes manifests (to be added) will support scalable deployments

---

## 📝 User Guides & Tutorials

### Mission Planner
- How to create, edit, and manage missions
- Importing and exporting KML/KMZ mission files
- Using the 2D map and 3D mission preview
- Real-time mission control and manual overrides

### Drone Command Hub
- Sending commands and monitoring telemetry
- Switching between manual and automated flight modes

### AI Integrations
- Using AI tools for alert summarization and analysis
- Configuring API keys and audit logging

## 📦 Deployment Manifests

- Dockerfiles for all services are in their respective directories
- `docker-compose.yml` includes all core and AI microservices
- Kubernetes manifests (to be added) will support scalable deployments
