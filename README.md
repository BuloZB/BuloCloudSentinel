# 🛡️ Bulo.Cloud Sentinel

> ⚠️ **IMPORTANT**: This is a development framework and not intended for production use without proper security review and customization.

Bulo.Cloud Sentinel is an open-source surveillance platform with React/Tailwind frontend, FastAPI backend, and PostgreSQL database.

## 🚀 Features

- **🔐 Authentication & Authorization**: Secure user management with role-based access control
- **📊 Dashboard**: Real-time monitoring and analytics
- **🗺️ Geospatial Tracking**: GPS integration and mapping capabilities
- **🤖 AI Integration**: Computer vision and anomaly detection
- **🔔 Alerting System**: Customizable notifications and alerts
- **📱 Mobile Support**: Responsive design for field operations
- **🔄 API Integration**: Extensible API for third-party services
- **🚁 Drone Swarm System**: Advanced multi-drone coordination and mission planning
- **👁️ Vision System**: Crowd density and vehicle analysis from aerial footage
- **🔒 Security Module**: Comprehensive security features and vulnerability protection

## 🧩 Addons

### 🛡️ Anti-Jamming Service

The Anti-Jamming Service provides GNSS anti-jamming, RF jamming detection, and resilient communication capabilities for the Bulo.Cloud Sentinel platform:

- **📡 GNSS Anti-Jamming**: Uses a multi-antenna GNSS array connected to a KrakenSDR module to mitigate jamming signals
- **🔍 RF Jamming Detection**: Uses KrakenSDR's GNU Radio blocks for Direction of Arrival (DoA) estimation
- **🔄 Frequency-Hopping**: Implements FHSS communication for resilient links between drones and ground stations
- **🧪 Jamming Simulation**: Uses HackRF One for testing jamming scenarios in various frequency bands
- **🔒 Secure Architecture**: Follows SOLID principles with proper abstractions and dependency injection
- **🔐 Security Features**: Implements HTTPS/TLS, HashiCorp Vault for secrets, and input sanitization

### 🚁 SentinelWeb

SentinelWeb is a modular, extensible web interface addon for BuloCloudSentinel, providing a user-friendly dashboard for drone management and surveillance operations. It is built on top of OpenWebUI's architecture but specifically tailored for drone operations.

#### ✨ Key Features

- **🔐 User Authentication and Management**:
  - Role-based access control (RBAC) with admin, operator, and observer roles
  - Integration with BuloCloudSentinel's JWT authentication system
  - User profile management and preferences

- **📊 Dashboard**:
  - Real-time overview of drone status, mission progress, and system health
  - Customizable widgets for telemetry data (battery levels, GPS coordinates, altitude)
  - System-wide notifications and alerts

- **📍 Mission Management**:
  - Interface for planning, simulating, and executing drone missions
  - Waypoint navigation and mission file import/export (KML/GPX formats)
  - Mission templates and saved configurations

- **📹 Live Video and Telemetry**:
  - Streaming and display of live video feeds from drones using RTMP
  - Real-time visualization of telemetry data (speed, altitude, battery status)
  - Historical telemetry data analysis and playback

- **🤖 AI Integration**:
  - Interface to configure and monitor AI-powered anomaly detection
  - Support for uploading and managing AI models for drone tasks
  - Visual analytics and insights from AI processing

- **🔗 Extensibility**:
  - Plugin system for adding custom tools and features
  - Well-documented APIs for third-party integrations
  - Custom widget development

- **📲 Offline Operation**:
  - Ability to operate without internet connection
  - Local caching of essential data
  - Synchronization when connection is restored

#### 🔄 OpenWebUI Integration

The latest version of SentinelWeb directly integrates with OpenWebUI, providing all the features of OpenWebUI plus drone-specific functionality. This integration allows for:

- 📱 Using OpenWebUI's powerful interface components
- 🔄 Maintaining compatibility with OpenWebUI updates
- 🔗 Adding drone-specific adapters and endpoints
- 👍 Seamless user experience across both platforms

#### 🏗️ Architecture

SentinelWeb is built as a standalone addon that integrates with BuloCloudSentinel's existing microservices:

- **Backend**: FastAPI-based service that connects to BuloCloudSentinel's APIs
- **Frontend**: React-based SPA with responsive design and PWA capabilities
- **Database**: PostgreSQL for persistent storage, Redis for caching
- **Integration**: WebSockets for real-time data, REST APIs for CRUD operations

### 🧠 Sentinel AI

Sentinel AI provides advanced artificial intelligence capabilities for the Bulo.Cloud Sentinel platform, including:

- **👁️ Computer Vision**: Object detection, tracking, and classification
- **🔍 Anomaly Detection**: Identifying unusual patterns and behaviors
- **📊 Predictive Analytics**: Forecasting trends and potential issues
- **🗣️ Natural Language Processing**: Command interpretation and reporting

### 🏢 Indoor Drone System

The Indoor Drone System provides advanced capabilities for operating drones in GPS-denied environments, inspired by the SU17 Smart Indoor Drone:

- **📡 LiDAR Positioning**: Mid-360 3D LiDAR sensor with FAST-LIO algorithm for precise indoor positioning
- **📷 Visual SLAM**: Quad-camera visual SLAM module for accurate positioning and mapping
- **🧭 Autonomous Navigation**: EGO-Swarm path planning algorithm for complex indoor environments
- **🎯 Target Recognition**: Advanced computer vision for object detection and tracking
- **🗺️ 3D Mapping**: Real-time 3D environment reconstruction and mapping
- **🔄 Sensor Fusion**: Tight integration of LiDAR, visual, and inertial data for robust positioning

### 🚁 Drone Swarm System

The Drone Swarm System extends Bulo.Cloud Sentinel's capabilities with advanced multi-drone coordination and autonomous mission planning:

- **🤖 Multi-Drone Coordination**: Coordinate multiple drones simultaneously for complex missions
- **🗺️ Autonomous Mission Planning**: Plan and execute autonomous missions with multiple waypoints
- **🔄 Swarm Intelligence**: Implement swarm behaviors for efficient area coverage and task distribution
- **⚠️ Collision Avoidance**: Advanced algorithms to prevent collisions between drones and obstacles
- **📡 Mesh Networking**: Resilient communication between drones even when some are out of direct range
- **🔋 Power Management**: Optimize missions based on battery levels and power consumption
- **🌦️ Weather Integration**: Adapt missions based on current and forecasted weather conditions
- **🚧 Geofencing**: Define no-fly zones and operational boundaries for safe operations

### 👁️ Vision System for Crowd and Vehicle Analysis

The Vision System microservice provides advanced computer vision capabilities for analyzing aerial footage from drones:

- **👥 Crowd Density Estimation**: Accurate estimation of crowd density from aerial footage
- **🚗 Vehicle Detection and Counting**: Detect and count vehicles by type (car, truck, bus, etc.)
- **🔄 Flow Analysis**: Track movement patterns and flow of crowds and vehicles
- **🌡️ Density Heat Maps**: Generate heat maps for visualizing crowd and vehicle density
- **📊 Occupancy Analytics**: Calculate space utilization and occupancy rates
- **📈 Trend Analysis**: Identify patterns and trends in historical data
- **🗺️ Spatial Mapping**: Map detections to geographic coordinates
- **⚡ Real-time Processing**: Process video streams in real-time with minimal latency

### 🔒 Security Module

The Security Module provides comprehensive security features for the Bulo.Cloud Sentinel platform:

- **🔐 Advanced Authentication**: JWT, OAuth2, and multi-factor authentication
- **🔑 Role-Based Access Control**: Fine-grained permission management
- **🔒 Data Encryption**: End-to-end encryption for sensitive data
- **🔍 Security Monitoring**: Real-time monitoring and alerting for security events
- **🔎 Vulnerability Scanning**: Automated scanning for known vulnerabilities
- **📝 Audit Logging**: Comprehensive logging of security-relevant events
- **🚫 Rate Limiting**: Protection against brute force and DoS attacks
- **🔗 Secure Communications**: TLS/SSL implementation for all communications
- **🔓 Input Validation**: Protection against injection attacks
- **🛡️ XSS Protection**: Frontend security utilities to prevent cross-site scripting
- **🔒 CSRF Protection**: Double Submit Cookie pattern to prevent cross-site request forgery
- **📊 Security Headers**: Comprehensive security headers including Content Security Policy
- **🔍 File Validation**: Secure file upload validation to prevent malicious file uploads
- **🔐 SQL Injection Protection**: Database security utilities to prevent SQL injection

## 🛠️ Development

### Project Structure

```
bulo-cloud-sentinel/
├── backend/              # FastAPI backend service
│   ├── api/              # API endpoints
│   ├── core/             # Core functionality
│   ├── db/               # Database models and connections
│   ├── services/         # Business logic services
│   └── main.py           # Application entry point
├── frontend/             # React frontend application
│   ├── public/           # Static assets
│   ├── src/              # Source code
│   │   ├── components/   # Reusable UI components
│   │   ├── pages/        # Page components
│   │   ├── services/     # API service clients
│   │   ├── store/        # State management
│   │   └── App.js        # Main application component
├── addons/               # Platform addons
│   ├── sentinelweb/      # SentinelWeb addon
│   └── sentinel-ai/      # Sentinel AI addon
├── drone_swarm_system/   # Drone Swarm System microservice
├── vision_system/        # Vision System for crowd and vehicle analysis
├── ai_analytics/         # Advanced AI and Analytics module
├── security/             # Security Module for comprehensive protection
└── docs/                 # Documentation
```

### CI/CD Workflows

Bulo.Cloud Sentinel uses GitHub Actions for continuous integration and deployment:

- **🛡️ Security Scanning**: Daily security scans using various tools (Safety, Bandit, npm audit, OWASP Dependency-Check, Trivy)
- **💪 Testing**: Comprehensive test suite for all components (backend, frontend, anti-jamming service, vision system)
- **🔍 Linting**: Code quality checks using Flake8, Black, isort, mypy, ESLint, and Prettier
- **📦 Docker Builds**: Automated Docker image builds for all components
- **🔄 Dependency Updates**: Regular checks for dependency updates
- **💾 Database Migrations**: Automated database migration checks

All workflows run daily at midnight to ensure the codebase remains secure and up-to-date.

### Contributing

1. Fork the repository
2. Create a feature branch: `git checkout -b feature/my-feature`
3. Commit your changes: `git commit -am 'Add new feature'`
4. Push to the branch: `git push origin feature/my-feature`
5. Submit a pull request

## 📄 License

This project is licensed under the MIT License, a permissive free software license that allows anyone to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the software.

In simple terms:
- ✅ You can use this code for commercial purposes
- ✅ You can modify the code and create derivative works
- ✅ You can distribute the original or modified code
- ✅ You can include this code in projects using different licenses
- ✅ No warranty is provided, and the authors have no liability for damages

