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

## 🧩 Addons

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
└── docs/                 # Documentation
```

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

## 👏 Acknowledgments

- OpenWebUI team for their excellent platform and integration support
- All contributors to the project
