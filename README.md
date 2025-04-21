# 🚀 Bulo.Cloud Sentinel

Bulo.Cloud Sentinel is an enterprise-grade, open-source modular surveillance and drone management platform designed for scalability, security, and advanced AI-powered analytics. The system provides a comprehensive solution for managing drone fleets, planning missions, analyzing data, and ensuring regulatory compliance in complex operational environments.

## ✨ Core Features

- 🛰️ **Advanced Drone Mission Planning** with waypoint support, mission simulation, and import/export (KML/GPX)
- 🤖 **AI-Powered Anomaly Detection** for real-time video and telemetry analysis
- 🔐 **Role-Based Access Control (RBAC)** integrated with JWT and Keycloak
- 📊 **Comprehensive Admin Dashboard** displaying system health, drone status, and audit logs
- 🧭 **Microservices Architecture** for telemetry, video, and mission handling
- 🐳 **Docker and Kubernetes Deployment Support**
- 🌐 **Tactical Use Module** with sensor fusion, mesh networking, and autonomous mission planning

## 🔥 Advanced Capabilities

### 🚁 Multi-Drone Coordination
- **Fleet Management**: Create and manage fleets of drones with designated roles (leader, follower)
- **Formation Flying**: Support for various formations (line, grid, circle, V-shape, custom)
- **Swarm Behaviors**: Implement complex behaviors like follow-leader, distributed search, and perimeter surveillance
- **Real-time Telemetry**: Monitor all drones in a fleet with synchronized telemetry data

### 🚫 Geofencing & Restricted Zone Detection
- **Airspace Awareness**: Automatic detection of restricted airspace (airports, military zones, etc.)
- **Custom Geofences**: Create and manage custom no-fly zones and restricted areas
- **Mission Validation**: Validate flight plans against geofence zones before execution
- **Real-time Monitoring**: Receive alerts when approaching restricted areas during flight
- **Multiple Data Sources**: Integration with authoritative airspace databases and custom zones

### ☀️ Weather Integration
- **Real-time Weather Data**: Access current weather conditions at mission locations
- **Weather Forecasting**: Incorporate weather forecasts into mission planning
- **Weather Alerts**: Receive notifications about severe weather conditions
- **Mission Weather Validation**: Automatically check if weather conditions are suitable for flight
- **Multiple Weather Providers**: Support for various weather data sources (OpenWeatherMap, WeatherAPI, etc.)

### 🔋 Power Management
- **Battery Monitoring**: Real-time tracking of drone battery levels and health
- **Energy-Aware Mission Planning**: Calculate energy requirements for missions before execution
- **Dynamic Range Estimation**: Estimate flight range based on battery status, payload, and weather conditions
- **Low Battery Alerts**: Proactive notifications for critical battery levels
- **Return-to-Home Optimization**: Intelligent RTH planning based on remaining battery capacity

### 📜 Regulatory Compliance Tools
- **Regulation Database**: Comprehensive database of drone regulations by region
- **Permit Management**: Tools for obtaining and tracking flight permits
- **Compliance Checking**: Automatic validation of missions against local regulations
- **Flight Logs**: Detailed logging of all flights for regulatory reporting
- **Documentation Generation**: Automated creation of required documentation for authorities

## 📈 Contribution Heatmap

![Contribution Heatmap](https://github.com/BuloZB/BuloCloudSentinel/graphs/contributors-data.svg)

## 🛠️ Development & Deployment

### Prerequisites

- Docker and Docker Compose for local development
- Kubernetes cluster for production deployment
- Python 3.9+ for backend development
- Node.js and npm/yarn for frontend development

### Development Setup

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

3. Build and start services for development:
   ```
   docker-compose -f docker-compose.dev.yml up --build
   ```

4. Access the development frontend at `http://localhost:3000`

### Production Deployment

For production environments, Kubernetes deployment is recommended:

1. Configure Kubernetes secrets for sensitive information
2. Apply Kubernetes manifests:
   ```
   kubectl apply -f k8s/
   ```

3. Monitor deployment status:
   ```
   kubectl get pods -n bulo-cloud-sentinel
   ```

## 🏗️ System Architecture

Bulo.Cloud Sentinel is built on a modern, scalable microservices architecture designed for flexibility, resilience, and performance. The system can be deployed on-premises, in the cloud, or in hybrid environments.

### Core Components

- **Backend API**: FastAPI-based REST API with modular routers for extensibility and performance
- **Dronecore**: Drone control and telemetry services with adapters for multiple drone platforms (DJI, Ardupilot, PX4)
- **AI Detection**: AI modules for anomaly detection with pluggable ML models and video analytics
- **Frontend**: React-based UI with responsive components for mission control, analytics, and administration
- **Supporting Services**: PostgreSQL (data persistence), Redis (caching), MinIO (object storage), RTMP server (video streaming)

### Advanced Modules

- **Fleet Management**: Services for coordinating multiple drones with formation and behavior management
- **Geofencing**: Spatial database and validation services for airspace restrictions and custom no-fly zones
- **Weather Services**: Integration with weather APIs and forecasting for mission planning and safety
- **Power Management**: Battery monitoring and energy-aware mission planning services
- **Regulatory Compliance**: Database and validation services for drone regulations and documentation

### Tactical Use Module

- � **Sensor Fusion Engine**: Real-time data fusion from diverse sensors using Kalman filters and Bayesian algorithms
- 🕸️ **Mesh Networking**: Decentralized mesh network with secure peer-to-peer communication for extended range
- 🧠 **Autonomous Mission Planning**: AI-driven mission planning and execution with obstacle avoidance
- 💾 **Edge Computing**: On-device processing capabilities for reduced latency and offline operation

### Deployment Architecture

The system is designed to be deployed as containerized microservices using Docker and orchestrated with Kubernetes for production environments. Each component can be scaled independently based on workload requirements.

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
