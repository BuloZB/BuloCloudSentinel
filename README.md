# Bulo.Cloud Sentinel 👁️‍🗨️

[![CI](https://github.com/BuloZB/BuloCloudSentinel/actions/workflows/ci.yml/badge.svg)](https://github.com/BuloZB/BuloCloudSentinel/actions/workflows/ci.yml)
[![Docker](https://img.shields.io/docker/cloud/build/bulozbd/bulo-cloud-sentinel)](https://hub.docker.com/r/bulozbd/bulo-cloud-sentinel)
[![License](https://img.shields.io/github/license/BuloZB/BuloCloudSentinel)]()

**Modular | Containerized | Secure | Scalable**

## 🚀 Quick Start

1. Create `.env` from `.env.example` and fill secrets.
2. Run `docker-compose up -d`.
3. Access services:
   - Backend API: `http://localhost:8000`
   - AI Integrations: `http://localhost:8002`
   - Frontend: `http://localhost:3000`
   - RTMP server stats: `http://localhost:8080/stat`
   - Prometheus metrics: `http://localhost:8002/monitoring/metrics`

## 📖 Modules & Features

### 🎥 Incident Timeline & Smart Playback
- FastAPI events API
- React/Tailwind UI with thumbnails & filters
- Synchronized video clip highlights

### 🚁 Drone Command & Telemetry Hub
- MAVLink telemetry & command endpoints
- Map view, live telemetry, JWT security

### 🤖 AI Model Management & Training Panel
- Upload, activate, rollback custom YOLO/TensorFlow models
- MinIO storage & training scheduler

### 📋 Device Inventory & Health Status
- Registry for cameras, sensors, drones
- Health checks & alerts via Novu

### 🔒 Access Audit Log & Session Inspector
- Full audit of user actions
- Export to CSV/JSON, OpenSearch analytics

### 🧠 AI Integrations
- FastAPI microservice with adapters:
  - ChatGPT, Claude, Gemini, DALL·E, Whisper
- Endpoints: `/ai/chat`, `/ai/vision/analyze`, `/ai/audio/transcribe`, `/ai/status`
- Persistent audit logging & Prometheus metrics
- React AI Tools panel

## 📦 Deployment

```bash
git clone https://github.com/BuloZB/BuloCloudSentinel.git
cd BuloCloudSentinel
docker-compose up -d
```

## 📄 Documentation & Wiki

- Full docs: see [`docs/wiki/Home.md`](docs/wiki/Home.md)
- Architecture diagram and module details in the wiki.

---

*All naming, logs, UI labels, and code structures use “Bulo.Cloud Sentinel” branding.*
