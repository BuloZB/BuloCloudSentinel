# SentinelWeb

SentinelWeb is a comprehensive web interface for BuloCloudSentinel, providing a user-friendly dashboard for drone management and surveillance operations. It is based on [OpenWebUI](https://github.com/open-webui/open-webui) but adapted specifically for drone operations.

## Overview

SentinelWeb provides a unified dashboard for managing drones, missions, telemetry data, and AI-powered surveillance features, with the same intuitive user experience as OpenWebUI but tailored for drone operations.

## Key Features

- **User Authentication and Management**:
  - Role-based access control (RBAC) with admin, operator, and observer roles
  - Integration with BuloCloudSentinel's JWT authentication system
  - User profile management and preferences

- **Dashboard**:
  - Real-time overview of drone status, mission progress, and system health
  - Customizable widgets for telemetry data (battery levels, GPS coordinates, altitude)
  - System-wide notifications and alerts

- **Mission Management**:
  - Interface for planning, simulating, and executing drone missions
  - Waypoint navigation and mission file import/export (KML/GPX formats)
  - Mission templates and saved configurations

- **Live Video and Telemetry**:
  - Streaming and display of live video feeds from drones using RTMP
  - Real-time visualization of telemetry data (speed, altitude, battery status)
  - Historical telemetry data analysis and playback

- **AI Integration**:
  - Interface to configure and monitor AI-powered anomaly detection
  - Support for uploading and managing AI models for drone tasks
  - Visual analytics and insights from AI processing

- **Extensibility**:
  - Plugin system for adding custom tools and features
  - Well-documented APIs for third-party integrations
  - Custom widget development

- **Offline Operation**:
  - Ability to operate without internet connection
  - Local caching of essential data
  - Synchronization when connection is restored

## Installation

### Prerequisites

- BuloCloudSentinel platform (running and configured)
- Docker and Docker Compose
- Node.js 16+ (for development)
- Python 3.9+ (for development)

### Docker Installation

`ash
# Clone the repository
git clone https://github.com/BuloZB/BuloCloudSentinel.git
cd BuloCloudSentinel/sentinelweb

# Configure environment variables
cp .env.example .env
# Edit .env with your configuration

# Start with Docker Compose
docker-compose up -d
`

## Attribution

SentinelWeb is based on OpenWebUI, which is licensed under the MIT License. We have adapted the codebase for drone operations while maintaining compliance with the original license. See the LICENSE file for details.
