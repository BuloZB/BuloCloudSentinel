# Bulo.Cloud Sentinel

<div align="center">
  <img src="docs/images/logo.png" alt="Bulo.Cloud Sentinel Logo" width="200"/>
  <h3>Multi-Role Drone Framework</h3>
  <p>Advanced surveillance, monitoring, and tactical capabilities for drone fleets</p>
</div>

## 🌟 Overview

Bulo.Cloud Sentinel is an open-source framework for building advanced drone-based surveillance and monitoring systems. It provides a comprehensive set of tools and services for managing drone fleets, processing sensor data, and extracting actionable insights.

The platform is designed to be modular, scalable, and extensible, allowing you to build custom solutions for a wide range of use cases, from security and surveillance to environmental monitoring and infrastructure inspection.

## ✨ Features

### 🚁 Core Platform

The core platform provides the foundation for building drone-based surveillance and monitoring systems:

- **🔄 Drone Fleet Management**: Centralized control and monitoring of multiple drones
- **📡 Telemetry Processing**: Real-time processing of drone telemetry data
- **🎥 Video Streaming**: Low-latency video streaming from drone cameras
- **🔌 Multi-Platform Support**: Support for ArduPilot, PX4, Betaflight, and DJI drones
- **🔌 Multi-Platform Support**: Support for ArduPilot, PX4, Betaflight, and DJI drones
- **� Multi-Platform Support**: Support for ArduPilot, PX4, Betaflight, and DJI drones
- **�🗺️ Mission Planning**: Visual mission planning and execution
- **📊 Data Visualization**: Real-time visualization of drone data
- **🔌 API Integration**: RESTful API for integration with external systems
- **🔒 Security**: End-to-end encryption and secure authentication
- **📱 Mobile Support**: Responsive web interface and mobile app
- **🌐 Mesh Networking**: Drone-to-drone communication for extended range
- **🔋 Battery Management**: Intelligent battery monitoring and management
- **🛰️ GPS Tracking**: Real-time GPS tracking and geofencing
- **⚡ Power Management**: Adaptive power management for extended flight time
- **🌦️ Weather Integration**: Real-time weather data integration
- **🔍 Regulatory Compliance**: Tools for maintaining compliance with drone regulations
- **🤖 Autonomous Operation**: Autonomous flight capabilities with obstacle avoidance
- **🔄 Multi-Drone Coordination**: Coordinated operations across multiple drones

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
- **🚀 Lightweight ML Runtime**: TinyGrad integration for efficient on-board inference
- **🔌 Multiple ML Backends**: Support for TinyGrad, PyTorch, and TensorFlow Lite
- **💻 Hardware Acceleration**: Automatic detection and utilization of CUDA, OpenCL, and CPU
- **📦 Model Hub**: Versioned model management with blue/green deployments and automatic rollback

### 🌐 SentinelWeb

SentinelWeb is a web-based interface for the Bulo.Cloud Sentinel platform, providing:

- **🔐 User Authentication**: Secure user authentication and authorization
- **📊 Dashboard**: Customizable dashboard for monitoring drone fleets
- **🎮 Mission Control**: Real-time mission control and monitoring
- **📹 Live Video**: Low-latency video streaming from drone cameras
- **🤖 AI Integration**: Integration with Sentinel AI for advanced analytics
- **🔌 Extensibility**: Plugin system for adding custom functionality
- **📱 Responsive Design**: Mobile-friendly interface for on-the-go access
- **🌐 Offline Operation**: Offline capabilities for remote operations
- **🔄 Real-time Updates**: Real-time updates of drone telemetry and status
- **🔍 Search**: Advanced search capabilities for finding specific data
- **📊 Analytics**: Comprehensive analytics and reporting
- **🔔 Notifications**: Real-time notifications for important events
- **📝 Documentation**: Integrated documentation and help system
- **🔒 Security**: End-to-end encryption and secure authentication
- **🌙 Dark Mode**: Dark mode support for reduced eye strain

### 🛡️ Tactical Capabilities

The Tactical Capabilities module provides advanced features for tactical operations:

- **🔍 ISR**: Intelligence, Surveillance, and Reconnaissance capabilities
- **📡 SIGINT**: Signal Intelligence for monitoring and analyzing electronic signals
- **⚡ Electronic Warfare**: Electronic countermeasures and protection
- **🔄 Mesh Networking**: Drone-to-drone communication for extended range
- **🤖 Autonomous Operation**: Autonomous mission execution with minimal human intervention
- **🎯 Target Acquisition**: Automated target acquisition and tracking
- **🔄 Sensor Fusion**: Integration of data from multiple sensors for enhanced situational awareness
- **🔒 Secure Communications**: Encrypted communications for sensitive operations
- **🌐 Geofencing**: Virtual boundaries for restricting drone operations
- **🔋 Power Management**: Intelligent power management for extended mission duration
- **🌦️ Weather Integration**: Real-time weather data for mission planning
- **📊 Mission Analytics**: Comprehensive analytics for mission assessment
- **🔄 Multi-Drone Coordination**: Coordinated operations across multiple drones
- **🔍 OSINT Integration**: Open-source intelligence integration for enhanced situational awareness
- **🛡️ Anti-Jamming**: Protection against GPS and communication jamming
- **🚫 Counter-UAS**: Detection, tracking, and reporting of unauthorized drone activity

### 🚫 Counter-UAS / Intrusion Detection

The Counter-UAS / Intrusion Detection module provides comprehensive capabilities for detecting and tracking unauthorized drones:

- **📡 RF Direction Finding**: KerberosSDR integration for direction of arrival estimation
- **📊 Radar Processing**: Acconeer radar integration for range and velocity measurement
- **🔄 Sensor Fusion**: Extended Kalman Filter for combining RF and radar data
- **🔔 Alert System**: Multi-level alerts with configurable thresholds
- **🌡️ Heat Map Visualization**: Real-time visualization of detection probability zones
- **📋 Track Management**: Real-time track list with threat classification
- **🔌 Event Distribution**: RabbitMQ integration for real-time event distribution
- **⚡ Low Latency**: Detection time < 2 seconds from target entry
- **📏 High Accuracy**: Bearing accuracy < 5° and range accuracy within 10%
- **🔒 Secure Operation**: Isolated components with encrypted communications

### 🗺️ Persistent Mapping

The Persistent Mapping Module generates orthomosaic images and 3D terrain meshes from drone imagery:

- **📸 Image Collection**: Upload and manage geotagged drone images
- **🌍 Orthomosaic Generation**: Create high-resolution aerial images
- **🏔️ 3D Mesh Reconstruction**: Generate 3D terrain models
- **🧩 Map Tiling**: Create map tiles for efficient streaming
- **🗄️ Geospatial Database**: Store and query mapping data with PostGIS
- **🌐 Cesium Integration**: Visualize mapping data in 3D with Cesium
- **⚙️ OpenDroneMap Integration**: Photogrammetry processing with OpenDroneMap
- **📊 Timeline Functionality**: View historical mapping data
- **📏 Measurement Tools**: Measure distance, area, and elevation
- **🔄 Asynchronous Processing**: Process large datasets in the background

### 🔍 Vision System

The Vision System provides advanced computer vision capabilities for the Bulo.Cloud Sentinel platform:

- **👁️ Object Detection**: Detect and classify objects in video streams
- **🔍 Facial Recognition**: Identify and track individuals in video streams
- **🚗 License Plate Recognition**: Identify and track vehicles by license plate
- **🔄 Multi-Camera Tracking**: Track objects across multiple camera views
- **🌡️ Thermal Imaging**: Process thermal imagery for detection in low-light conditions
- **📏 Depth Sensing**: Process depth data for 3D object localization
- **🔄 Sensor Fusion**: Combine data from multiple sensors for enhanced detection
- **🔍 Anomaly Detection**: Identify unusual patterns and behaviors
- **🔄 Motion Detection**: Detect and track motion in video streams
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

### 🌐 Digital Twin & Simulation

The Digital Twin & Simulation environment provides a deterministic, software-in-the-loop (SITL) testing platform:

- **🎮 Ignition Gazebo**: Physics and rendering engine for realistic drone simulation
- **🤖 ROS 2 Humble**: Robotics middleware for communication
- **🏙️ Urban Environments**: Detailed urban worlds with buildings, streets, traffic, and GPS multipath effects
- **🔄 Swarm Simulation**: Support for multi-drone scenarios and swarm logic testing
- **🌦️ Weather Simulation**: Realistic wind, rain, fog, and time-of-day effects with turbulence modeling
- **🚗 Traffic Simulation**: Dynamic vehicles and pedestrians with realistic movement patterns
- **📡 Advanced Sensors**: LiDAR, thermal cameras, depth cameras, and radar sensor simulation
- **🔋 Battery Simulation**: Realistic battery discharge curves and power management
- **📊 Telemetry Bridge**: Seamless integration with existing Bulo.CloudSentinel platform
- **🧪 Comprehensive Testing**: Scenario-based testing with failure injection capabilities
- **📈 Visualization Tools**: Real-time 3D visualization of trajectories, sensor data, and metrics
- **☁️ Kubernetes Deployment**: Helm charts for easy deployment to Kubernetes clusters
- **🔒 Security Hardened**: Non-root containers with read-only filesystem and parameter validation

### 🔖 Remote ID & Regulatory Compliance

The Remote ID & Regulatory Compliance module ensures compliance with international aviation standards:

- **🪪 Remote ID Broadcasting**: ASTM F3411-22a compliant broadcasting over Wi-Fi NAN and Bluetooth LE
- **📝 Flight Plan Management**: Automated submission to EASA SORA and FAA LAANC systems
- **📢 NOTAM Integration**: Import and visualization of NOTAMs with spatial conflict detection
- **🔐 Secure Communication**: Mutual TLS (mTLS) with SPIFFE IDs for secure drone-to-service communication
- **📊 Broadcast Logging**: 24-hour retention of broadcast logs for regulatory compliance
- **🌍 Multi-Region Support**: Compliance with both EU and FAA regulatory environments
- **🔄 MAVLink Integration**: Conversion of MAVLink telemetry to standardized Remote ID format
- **🚀 Kubernetes Deployment**: Helm charts for easy deployment to Kubernetes clusters

### 🔌 Dock Stations

The Dock Stations integration enables automated charging and protection for drones:

- **🏠 Multi-Vendor Support**: Integration with DJI Dock 2, Heisha Charging Pad, and DIY ESP32-powered docks
- **🔄 Automated Charging**: Automatic charging when battery levels are low
- **🌡️ Environmental Control**: Temperature and humidity monitoring and control
- **📊 Telemetry**: Real-time telemetry data from docking stations
- **🔌 Power Management Integration**: Seamless integration with the Power Management module
- **🔒 Secure Communication**: Encrypted communication with docking stations
- **🧩 Modular Design**: Adapter pattern for easy addition of new dock types

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

### 📦 Model Hub

The Model Hub provides versioned model management for the Bulo.Cloud Sentinel platform:

- **🗄️ Model Registry**: Store and version AI models with MLflow + MinIO backend
- **📊 Metadata Tracking**: Track model accuracy, hash, size, and hardware compatibility
- **🔄 Blue/Green Deployments**: Deploy models with zero downtime using Argo Rollouts
- **🔙 Automatic Rollback**: Automatically roll back to previous version if performance degrades
- **🔒 Security**: Signed models with hash verification and seccomp confinement
- **🔌 Edge Integration**: Seamless integration with Edge Kit for over-the-air updates
- **🛠️ CLI & UI**: Command-line and web interfaces for model management
- **🔄 CI/CD**: GitHub Actions for automated model uploads and deployments
- **🧠 Federated Learning**: Train models across edge devices while preserving data privacy

### 🔄 Federated Learning

The Federated Learning system enables privacy-preserving distributed training across edge devices:

- **🔒 Privacy Preservation**: Train models locally, share only gradients, raw data never leaves devices
- **🔐 Differential Privacy**: Implemented with Opacus, configurable privacy budget (ε ≤ 8)
- **🔄 Efficient Training**: Lightweight model head training on locally extracted embeddings
- **🌐 Secure Communication**: TLS over MQTT with client certificates, device identifiers removed
- **⚡ Bandwidth Efficient**: < 50 MB upload per round, optimized for constrained networks
- **🔄 Fail-Safe**: Handles client disconnections and stragglers automatically
- **🧪 Integration Testing**: CI pipeline with synthetic COCO subset for validation

### 🌦️ Weather Guard

The Weather Guard module provides comprehensive weather awareness capabilities for the platform:

- **🌤️ Weather Data Integration**: Pull forecasts from Open-Meteo API with hourly wind and rain data
- **📡 Local Weather Station**: Optional integration with MeteoShield (ESP32 + BME280) via MQTT
- **🚫 Mission Blocking**: Automatically blocks mission launch if wind > 9 m/s or rain > 0.5 mm/h
- **🏠 Indoor Fallback**: Triggers indoor mission alternatives when outdoor conditions are unfavorable
- **📊 Dashboard Integration**: 24-hour forecast graph and GO/NO-GO indicators
- **🔔 Alert System**: Notifies when weather windows open for mission execution
- **💾 Efficient Caching**: Redis-based caching with 30-minute TTL
- **🔄 Fault Tolerance**: Fallback to last good data when API is unavailable
- **🔌 Microservice Architecture**: Standalone service with FastAPI and Python 3.12

### 🌐 Edge Kit

The Edge Kit is a plug-and-play Docker bundle for deploying Bulo.CloudSentinel capabilities at the edge:

- **🖥️ Hardware Support**: Optimized for Jetson Orin Nano and Raspberry Pi 5
- **🚀 Low-Latency Inference**: Run quantized AI models (YOLOv10-nano INT8) at the edge
- **📹 Optimized Streaming**: Ingest local RTSP streams and publish low-bitrate HLS/WebRTC
- **🔄 Self-Updates**: Over-the-air updates with rollback capability
- **🔒 Secure by Design**: Read-only root FS, seccomp & AppArmor profiles, Vault integration
- **🔌 Containerized**: Three main containers (edge_inference, rtsp_relay, edge_agent)
- **📊 Performance**: End-to-end latency < 250 ms for 1080p stream
- **🛠️ Easy Deployment**: Single `docker-compose.yml` + script `edge_install.sh`

## 🚀 Getting Started

### Prerequisites

- Python 3.10+
- Docker and Docker Compose
- NVIDIA GPU with CUDA support (optional, for accelerated processing)
- Raspberry Pi 4 or newer (for edge deployment)
- NVIDIA Jetson (for edge AI processing)

### ML Backends

Bulo.Cloud Sentinel supports multiple ML backends for inference:

- **TinyGrad**: A lightweight ML runtime for on-board inference
- **PyTorch**: A full-featured ML framework for high-performance inference
- **TensorFlow Lite**: A lightweight ML runtime for mobile and edge devices

The platform also includes an enhanced inference engine with advanced capabilities:

- **Batch Inference**: Process multiple inputs in batches for improved throughput
- **Streaming Inference**: Process a continuous stream of inputs in real-time
- **Distributed Inference**: Distribute inference workloads across multiple nodes
- **Model Management**: Convert, visualize, and analyze models
- **Model Optimization**: Optimize models for improved performance
- **Model Quantization**: Quantize models for reduced size and faster inference
- **Model Hub**: Versioned model management with blue/green deployments and automatic rollback

See [Enhanced Inference Engine](docs/enhanced_inference.md) and [Model Hub](model_hub_service/docs/model_hub.md) for more details.

You can select the backend to use via the `ML_BACKEND` environment variable or the `--ml-backend` command-line option:

```bash
# Using environment variable
export ML_BACKEND=tinygrad
python -m ai.cli run --model models/mobilenet_v2.npz --image examples/data/sample_image.jpg

# Using command-line option
python -m ai.cli run --model models/mobilenet_v2.npz --image examples/data/sample_image.jpg --backend tinygrad

# Using Docker
docker run -e ML_BACKEND=tinygrad bulosentinel
```

For more information about the available backends and their capabilities, see [AI Backends Documentation](docs/ai_backends.md).

### Installation

1. Clone the repository:

```bash
git clone https://github.com/BuloZB/BuloCloudSentinel.git
cd BuloCloudSentinel
```

2. Install dependencies:

```bash
pip install -r requirements.txt
```

3. Build and run the Docker containers:

```bash
docker-compose up -d
```

### Running the TinyGrad Demo

To run the TinyGrad demo:

```bash
# Using the default backend (PyTorch)
python examples/tinygrad_demo.py

# Using TinyGrad backend
python examples/tinygrad_demo.py --backend tinygrad

# Using TinyGrad backend with CUDA
python examples/tinygrad_demo.py --backend tinygrad --device CUDA
```

### Benchmarking ML Backends

To benchmark the performance of different ML backends:

```bash
# Benchmark all backends
python examples/benchmark_backends.py

# Benchmark a specific backend
python examples/benchmark_backends.py --backends tinygrad

# Benchmark with CUDA
python examples/benchmark_backends.py --device CUDA
```

### Installing the Edge Kit

To install the Edge Kit on a Jetson Orin Nano or Raspberry Pi 5:

```bash
# Download and run the installation script
curl -fsSL https://raw.githubusercontent.com/BuloZB/BuloCloudSentinel/main/edge_kit/edge_install.sh | sudo bash

# Or clone the repository and run the script manually
git clone https://github.com/BuloZB/BuloCloudSentinel.git
cd BuloCloudSentinel/edge_kit
sudo ./edge_install.sh
```

The Edge Kit will be installed in `/opt/bulocloud/edge_kit` and configured to start automatically on boot. You can access the management interface at `http://<device-ip>:9090`.

## 🛠️ Development

### Project Structure

```
bulo-cloud-sentinel/
├── .github/              # GitHub workflows and templates
├── addons/               # Platform addons
├── ai/                   # AI core components
│   └── inference/        # ML inference backends
│       ├── base.py       # Base class for inference backends
│       ├── engine.py     # Central engine for loading backends
│       ├── tinygrad_backend.py # TinyGrad inference backend
│       ├── torch_backend.py    # PyTorch inference backend
│       ├── tflite_backend.py   # TensorFlow Lite inference backend
│       ├── batch_engine.py     # Batch inference engine
│       ├── distributed_engine.py # Distributed inference engine
│       ├── convert.py    # Model conversion utilities
│       ├── visualize.py  # Model visualization utilities
│       ├── web_converter.py # Web interface for model conversion
│       └── model_manager.py # Web interface for model management
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
├── docker/               # Docker configuration files
│   └── bulosentinel.Dockerfile # Docker configuration for tinygrad integration
├── dronecore/            # Core drone control libraries
│   ├── ardupilot_adapter.py  # ArduPilot flight controller adapter
│   ├── betaflight_adapter.py # Betaflight flight controller adapter
│   ├── dji_adapter.py        # DJI SDK adapter for DJI drones
│   ├── px4_adapter.py        # PX4 flight controller adapter
│   └── flight_controller_adapter.py # Abstract adapter interface
├── examples/             # Example scripts and demos
│   ├── tinygrad_demo.py  # Demo of TinyGrad inference backend
│   ├── benchmark_backends.py # Benchmark different ML backends
│   └── enhanced_inference_demo.py # Demo of enhanced inference capabilities
├── drone_show_service/   # Drone Show microservice for light shows
│   ├── addons/           # Blender integration and other add-ons
│   ├── api/              # API endpoints
│   ├── core/             # Core functionality
│   ├── kubernetes/        # Kubernetes deployment manifests
│   ├── models/           # Data models
│   ├── services/         # Business logic services
│   └── utils/            # Utility functions
├── drone_swarm_system/   # Drone Swarm System microservice
├── dock_driver/          # Dock Stations microservice
│   ├── adapters/         # Adapters for different dock types
│   │   ├── dji/          # DJI Dock 2 adapter
│   │   ├── heisha/       # Heisha Charging Pad adapter
│   │   ├── esp32/        # DIY ESP32-powered dock adapter
│   │   └── interface.py  # Common adapter interface
│   ├── api/              # API endpoints
│   ├── models/           # Data models
│   ├── services/         # Business logic services
│   └── utils/            # Utility functions
├── edge_kit/             # Edge Kit for low-latency inference at the perimeter
│   ├── inference/        # Edge Inference container with Triton Server
│   ├── rtsp_relay/       # RTSP Relay container for video streaming
│   ├── edge_agent/       # Edge Agent container for OTA updates
│   ├── models/           # Pre-trained models for edge inference
│   └── config/           # Configuration files for edge deployment
├── federated_learning/   # Federated Learning system for privacy-preserving training
│   ├── edge_client/      # Edge Trainer Client with Flower and PyTorch
│   ├── server/           # Federated Learning Server with model aggregation
│   ├── integration_test/ # Integration tests with synthetic data
│   ├── mqtt/             # MQTT configuration for secure communication
│   ├── config/           # Configuration files
│   └── certs/            # Certificates for secure communication
├── remoteid_service/     # Remote ID & Regulatory Compliance Service
│   ├── api/              # API endpoints for Remote ID, flight plans, and NOTAMs
│   ├── broadcast/        # Wi-Fi NAN and Bluetooth LE broadcasting
│   ├── adapters/         # Adapters for EASA SORA and FAA LAANC APIs
│   ├── cli/              # Command-line tools for Remote ID and flight plans
│   ├── config/           # Configuration files
│   ├── core/             # Core functionality
│   ├── db/               # Database models and connections
│   ├── kubernetes/       # Kubernetes deployment manifests and Helm charts
│   └── services/         # Business logic services
├── sim/                  # Digital Twin & Simulation environment
│   ├── bazel/            # Bazel build configuration
│   ├── docker/           # Docker configuration files
│   ├── helm/             # Helm charts for Kubernetes deployment
│   ├── models/           # Gazebo models
│   ├── ros2_ws/          # ROS 2 workspace
│   ├── scripts/          # Utility scripts
│   └── tests/            # Test files
├── frontend/             # React frontend application
│   ├── public/           # Static assets
│   └── src/              # Source code
│       ├── components/   # Reusable UI components
│       ├── pages/        # Page components
│       ├── services/     # API service clients
│       ├── store/        # State management
│       └── App.js        # Main application component
├── indoor_drone_system/  # Indoor drone navigation system
├── model_hub_service/    # Model Hub for versioned model management
│   ├── app/              # FastAPI application
│   │   ├── api/          # API endpoints
│   │   ├── core/         # Core functionality
│   │   ├── db/           # Database models and connections
│   │   ├── models/       # Data models
│   │   ├── services/     # Business logic services
│   │   └── utils/        # Utility functions
│   ├── docs/             # Documentation
│   ├── kubernetes/       # Kubernetes deployment manifests
│   └── tests/            # Test files
├── rtmp_server/          # RTMP server for video streaming
├── security/             # Security Module for comprehensive protection
├── sentinelweb/          # Web interface based on OpenWebUI
├── tactical_capabilities/ # Tactical capabilities modules
│   ├── ew_service/       # Electronic Warfare service
│   ├── isr_service/      # Intelligence, Surveillance, and Reconnaissance service
│   ├── sentinel_beacon/  # Meshtastic-based mesh communication for drones
│   ├── sigint_service/   # Signal Intelligence service
│   └── tacs/             # Target Acquisition and Coordination System
├── counter_uas/          # Counter-UAS / Intrusion Detection module
│   ├── hardware/         # Hardware interfaces for KerberosSDR and Acconeer radar
│   ├── processing/       # Signal processing for direction finding and radar
│   ├── services/         # Business logic services
│   ├── api/              # API endpoints
│   ├── models/           # Data models
│   ├── utils/            # Utility functions
│   ├── config/           # Configuration files
│   ├── docs/             # Documentation
│   └── tests/            # Test files
├── mapping_service/      # Persistent Mapping Module
│   ├── api/              # API endpoints
│   ├── core/             # Core functionality
│   ├── db/               # Database models and connections
│   ├── processing/       # Processing pipeline
│   ├── services/         # Business logic services
│   ├── utils/            # Utility functions
│   ├── docker/           # Docker configuration
│   ├── docs/             # Documentation
│   ├── tests/            # Test files
│   ├── main.py           # Application entry point
│   └── worker_main.py    # Worker entry point
├── tests/                # Test files
│   └── test_tinygrad_backend.py # Tests for TinyGrad inference backend
├── vision_system/        # Vision System for crowd and vehicle analysis
└── weather_guard/        # Smart Weather Awareness service
```

### Security

Bulo.Cloud Sentinel takes security seriously. We have implemented comprehensive security measures throughout the platform:

- **🔐 Security Documentation**: Detailed security documentation is available in the [SECURITY.md](SECURITY.md) file, [security guidelines](docs/security_guidelines.md), and [security incident response plan](docs/security_incident_response_plan.md).
- **🔒 Security Features**: The platform includes advanced security features such as multi-factor authentication, role-based access control, and data encryption.
- **🔍 Security Scanning**: Automated security scanning is integrated into the CI/CD pipeline to detect vulnerabilities.
- **🛡️ Vulnerability Reporting**: If you discover a security vulnerability, please follow the reporting process outlined in [SECURITY.md](SECURITY.md).
- **📝 Security Updates**: Security updates are documented in [security_vulnerability_fixes.md](docs/security_vulnerability_fixes.md).
- **🔒 Recent Security Improvements**:
  - Replaced python-jose with PyJWT to address critical vulnerabilities (CVE-2024-33664, CVE-2024-33663)
  - Implemented unified authentication module with Argon2id for secure password hashing
  - Created comprehensive input validation and sanitization framework
  - Implemented unified security middleware with security headers, CSRF protection, and rate limiting
  - Added secure error handling to prevent information leakage
  - Created comprehensive security testing framework with automated vulnerability scanning
  - Implemented detailed security incident response plan with clear procedures
  - Enhanced GitHub workflow for security scanning with multiple tools
  - Created secure coding guidelines for developers
  - Enhanced JWT token validation with full signature verification and expiration checks
  - Secure session management with strict cookie settings (SameSite=strict, HTTPS-only)
  - Restricted CORS settings to prevent cross-origin attacks
  - Removed hardcoded credentials and API keys
  - Updated vulnerable dependencies (python-jose, python-multipart, cryptography, pillow)
  - Added Content Security Policy (CSP) headers
  - Implemented token revocation and blacklisting
  - Added advanced rate limiting for authentication endpoints
  - Enhanced password validation with checks for common passwords
  - Implemented secure file upload validation
  - Added comprehensive security headers (HSTS, X-Content-Type-Options, etc.)
  - Centralized security configuration
  - Implemented automated security testing with OWASP ZAP
  - Added security monitoring and alerting system
  - Enhanced CI/CD pipeline with additional security scanning tools

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

