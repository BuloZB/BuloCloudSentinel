# 🚀 Bulo.Cloud Sentinel

Bulo.Cloud Sentinel is an open-source modular surveillance and drone management system designed for scalability, security, and advanced AI-powered analytics.

## ✨ Features

- 🛰️ **Advanced Drone Mission Planning** with waypoint support, mission simulation, and import/export (KML/GPX).
- 🤖 **AI-Powered Anomaly Detection** for real-time video and telemetry analysis.
- 🔐 **Role-Based Access Control (RBAC)** integrated with JWT and Keycloak.
- 📊 **Comprehensive Admin Dashboard** displaying system health, drone status, and audit logs.
- 🧭 **Microservices Architecture** for telemetry, video, and mission handling.
- 🐳 **Docker and Kubernetes Deployment Support**.

## 📈 Contribution Heatmap

![Contribution Heatmap](https://github.com/BuloZB/BuloCloudSentinel/graphs/contributors-data.svg)

## 🛠️ Getting Started

### Prerequisites

- Docker and Docker Compose
- Python 3.9+
- Node.js and npm/yarn for frontend

### Setup

1. Clone the repository:
   ```
   git clone https://github.com/BuloZB/BuloCloudSentinel.git
   cd BuloCloudSentinel
   ```

2. Copy and configure environment variables:
   ```
   cp .env.example .env
   # Edit .env with your settings
   ```

3. Build and start services:
   ```
   docker-compose up --build
   ```

4. Access the frontend at `http://localhost:3000`.

## Architecture

The system is composed of multiple microservices:

- **Backend API**: FastAPI-based REST API with modular routers.
- **Dronecore**: Drone control and telemetry services with adapters for DJI, Ardupilot, PX4.
- **AI Detection**: AI modules for anomaly detection with pluggable ML models.
- **Frontend**: React-based UI with components for mission control, analytics, and admin dashboard.
- **Supporting Services**: PostgreSQL, Redis, MinIO, RTMP server.

## Security

- Full JWT-based authentication and authorization.
- Role definitions: `admin`, `operator`, `observer`.
- Backend audit logging of all access and control events.
- Planned Keycloak integration for centralized identity management.

## Development

- Backend code is in `backend/` and `dronecore/`.
- Frontend code is in `frontend/`.
- AI detection modules are in `ai_detection/` and `bulo-sentinel-ai/`.
- Use `docker-compose.yml` for local development and testing.

## Future Work

- Real-time telemetry simulation and SDK integration.
- Enhanced mission planner UI with map and import/export.
- Full Keycloak integration.
- Kubernetes Helm charts for production deployment.
- Expanded AI anomaly detection with real ML models.

## Documentation

See the `docs/wiki/` directory for detailed architecture and usage guides.

---

# License

This project is licensed under the MIT License.
