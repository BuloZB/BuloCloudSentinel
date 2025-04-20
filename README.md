# Bulo.Cloud Sentinel

Bulo.Cloud Sentinel is a comprehensive, modular, and scalable surveillance and situational awareness platform designed for modern security operations. It integrates real-time video streaming, AI-powered analytics, drone command and telemetry, and advanced mission planning into a unified system.

## Key Features

- 🎥 **Incident Timeline & Smart Playback**  
  FastAPI backend and React frontend providing event timelines with AI-generated labels, thumbnails, and synchronized video playback.

- 🚁 **Drone Command & Telemetry Hub**  
  Real-time MAVLink telemetry ingestion and command dispatch with map-based UI and JWT-secured APIs.

- 🤖 **AI Model Management & Training Panel**  
  Upload, activate, and manage custom AI models (YOLOv8, TensorFlow) with MinIO storage and scheduled training.

- 📋 **Device Inventory & Health Status**  
  Centralized registry for cameras, sensors, and drones with health monitoring and alerting via Novu.

- 🔒 **Access Audit Log & Session Inspector**  
  Comprehensive audit logging of user actions with export capabilities and RBAC security.

- 🧠 **AI Integrations Microservice**  
  Modular FastAPI service integrating multiple AI providers (ChatGPT, Claude, Gemini, DALL·E, Whisper) with audit logging and Prometheus metrics.

- 🚀 **Advanced Drone Mission Planning**  
  Web-based mission planner with 2D/3D visualization, real-time telemetry integration, hybrid flight modes, and WebSocket-based control.

## Technology Stack

- ⚛️ **Frontend:** React, Tailwind CSS, React-Leaflet, React-Three-Fiber  
- 🐍 **Backend:** FastAPI (Python), SQLAlchemy, PostgreSQL  
- 🔐 **Authentication:** Keycloak (OAuth2, RBAC)  
- 🐳 **Containerization:** Docker, Kubernetes  
- 📡 **Messaging:** MQTT, WebSockets  
- 📊 **Monitoring:** Prometheus, Grafana  
- 💾 **Storage:** MinIO, PostgreSQL

## Getting Started

1. Clone the repository:  
   ```bash
   git clone https://github.com/BuloZB/BuloCloudSentinel.git
   cd BuloCloudSentinel
   ```

2. Create a `.env` file based on `.env.example` and fill in all required secrets and API keys.

3. Build and start all services using Docker Compose:  
   ```bash
   docker-compose up -d
   ```

4. Access the services:  
   - Backend API: `http://localhost:8000`  
   - AI Integrations: `http://localhost:8002`  
   - Frontend UI: `http://localhost:3000`  
   - RTMP Server Stats: `http://localhost:8080/stat`  
   - Prometheus Metrics: `http://localhost:8002/monitoring/metrics`

## Documentation & Support

- 📚 API documentation available at `/docs` for each FastAPI service.  
- 🏗️ Detailed architecture and usage guides in the [GitHub Wiki](https://github.com/BuloZB/BuloCloudSentinel/wiki).  
- 📖 User guides and tutorials for mission planning, drone control, and AI tools included in the documentation.

## Contributing

Contributions are welcome! Please follow the established code style and submit pull requests for review.

## License

This project is licensed under the MIT License.

---

*All naming, logs, UI labels, and code structures use “Bulo.Cloud Sentinel” branding.*
