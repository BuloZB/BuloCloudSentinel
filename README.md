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
- **✨ Drone Show**: Choreographed light shows with synchronized LED control

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
- **🔀 Multimodal Detection**: Fusion of visual, thermal, and depth data for superior detection accuracy
- **🌡️ Thermal Analysis**: Detection in low-light and adverse weather conditions
- **📏 Depth Perception**: Accurate distance estimation and 3D object localization
- **🔄 Sensor Fusion**: Advanced algorithms for integrating data from multiple sensors

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

### ✨ Drone Show Microservice

The Drone Show Microservice enables planning, simulation, and execution of choreographed drone light shows:

- **🎭 Choreography Definition**: JSON-based format for defining LED patterns and waypoints
- **🎮 Simulation & Preview**: 3D visualization of drone positions and LED states
- **🔄 Execution Engine**: Synchronized execution of choreographies across a fleet of drones
- **📊 Monitoring & Logging**: Real-time telemetry and logging of show execution
- **🔌 Integration**: Seamless integration with Bulo.Cloud Sentinel platform
- **🎨 Blender Integration**: Custom Blender add-on for creating and exporting drone choreographies
- **🎵 Music Synchronization**: Synchronize LED patterns with music
- **⏱️ Time Synchronization**: Precise timing across all drones for coordinated shows
- **🔄 Formation Transitions**: Smooth transitions between different formations (grid, circle, custom)
- **🔋 Battery Management**: Optimize shows based on battery levels
- **🔒 Security Hardened**: Comprehensive security measures and dependency management

### 🔒 Security Module

The Security Module provides comprehensive security features for the Bulo.Cloud Sentinel platform:

- **🔐 Advanced Authentication**: JWT with enhanced validation, OAuth2, and multi-factor authentication
- **🔑 Role-Based Access Control**: Fine-grained permission management with secure token validation
- **🔒 Data Encryption**: End-to-end encryption with modern algorithms and secure key management
- **🔍 Security Monitoring**: Real-time monitoring and alerting for security events
- **🔎 Vulnerability Scanning**: Automated scanning for known vulnerabilities
- **📝 Secure Audit Logging**: Comprehensive logging with sensitive data masking
- **🚫 Rate Limiting**: Configurable rate limiting for all API endpoints
- **🔗 Secure Communications**: TLS/SSL implementation with proper certificate validation
- **🔓 Input Validation**: Comprehensive validation library to prevent injection attacks
- **🛡️ XSS Protection**: Advanced HTML sanitization and Content Security Policy
- **🔒 CSRF Protection**: Double Submit Cookie pattern and SameSite cookie attributes
- **📊 Security Headers**: Comprehensive security headers including CSP and Permissions Policy
- **🔍 File Validation**: Secure file upload validation with content type verification
- **🔐 SQL Injection Protection**: Parameterized queries and ORM-based database access
- **🔄 Key Rotation**: Automatic key rotation for cryptographic keys
- **🔒 Secure Error Handling**: Error handling that prevents information leakage
- **🛡️ CORS Protection**: Strict Cross-Origin Resource Sharing configuration
- **🔐 Secure Password Handling**: Argon2id password hashing with proper salting
- **📝 Secure Logging**: Logging utilities that mask sensitive data automatically

## 🛠️ Development

### Project Structure

```
bulo-cloud-sentinel/
├── .github/              # GitHub workflows and templates
├── addons/               # Platform addons
├── ai_analytics/         # Advanced AI and Analytics module
│   ├── models/           # AI model definitions
│   │   ├── detection/    # Object detection models
│   │   ├── recognition/  # Face and license plate recognition
│   │   ├── multimodal/   # Multimodal detection models
│   │   └── predictive/   # Predictive analytics models
│   ├── api/              # API endpoints
│   ├── services/         # Business logic services
│   └── config/           # Configuration files
├── ai_detection/         # AI-based detection capabilities
├── anti_jamming_service/  # GNSS anti-jamming and RF protection
├── backend/              # FastAPI backend service
│   ├── api/              # API endpoints
│   ├── core/             # Core functionality
│   ├── db/               # Database models and connections
│   ├── services/         # Business logic services
│   └── main.py           # Application entry point
├── docs/                 # Documentation
├── dronecore/            # Core drone control libraries
├── drone_show_service/   # Drone Show microservice for light shows
│   ├── addons/           # Blender integration and other add-ons
│   ├── api/              # API endpoints
│   ├── core/             # Core functionality
│   ├── kubernetes/        # Kubernetes deployment manifests
│   ├── models/           # Data models
│   ├── services/         # Business logic services
│   └── utils/            # Utility functions
├── drone_swarm_system/   # Drone Swarm System microservice
├── frontend/             # React frontend application
│   ├── public/           # Static assets
│   └── src/              # Source code
│       ├── components/   # Reusable UI components
│       ├── pages/        # Page components
│       ├── services/     # API service clients
│       ├── store/        # State management
│       └── App.js        # Main application component
├── indoor_drone_system/  # Indoor drone navigation system
├── rtmp_server/          # RTMP server for video streaming
├── security/             # Security Module for comprehensive protection
├── sentinelweb/          # Web interface based on OpenWebUI
├── tactical_capabilities/ # Tactical capabilities modules
│   ├── ew_service/       # Electronic Warfare service
│   ├── isr_service/      # Intelligence, Surveillance, and Reconnaissance service
│   ├── sentinel_beacon/  # Meshtastic-based mesh communication for drones
│   ├── sigint_service/   # Signal Intelligence service
│   └── tacs/             # Target Acquisition and Coordination System
└── vision_system/        # Vision System for crowd and vehicle analysis
```

### Security

Bulo.Cloud Sentinel takes security seriously. We have implemented comprehensive security measures throughout the platform:

- **🔐 Security Documentation**: Detailed security documentation is available in the [SECURITY.md](SECURITY.md) file and [security guidelines](docs/security_guidelines.md).
- **🔒 Security Features**: The platform includes advanced security features such as multi-factor authentication, role-based access control, and data encryption.
- **🔍 Security Scanning**: Automated security scanning is integrated into the CI/CD pipeline to detect vulnerabilities.
- **🛡️ Vulnerability Reporting**: If you discover a security vulnerability, please follow the reporting process outlined in [SECURITY.md](SECURITY.md).
- **📝 Security Updates**: Security updates are documented in [security_vulnerability_fixes.md](docs/security_vulnerability_fixes.md).

For more information about security features, see the [Security Module](#-security-module) section.

### Contributing

1. Fork the repository
2. Create a feature branch: `git checkout -b feature/my-feature`
3. Commit your changes: `git commit -am 'Add new feature'`
4. Push to the branch: `git push origin feature/my-feature`
5. Submit a pull request
6. Follow the [security guidelines](docs/security_guidelines.md) when contributing code

## 📄 License

This project is licensed under the MIT License, a permissive free software license that allows anyone to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the software.

In simple terms:
- ✅ You can use this code for commercial purposes
- ✅ You can modify the code and create derivative works
- ✅ You can distribute the original or modified code
- ✅ You can include this code in projects using different licenses
- ✅ No warranty is provided, and the authors have no liability for damages

