# SentinelWeb

SentinelWeb is a modular, extensible web interface addon for BuloCloudSentinel, providing a user-friendly dashboard for drone management and surveillance operations.

## Overview

SentinelWeb serves as a comprehensive web interface for BuloCloudSentinel, built on top of OpenWebUI's architecture but specifically tailored for drone operations. It provides a unified dashboard for managing drones, missions, telemetry data, and AI-powered surveillance features.

## New Implementation: OpenWebUI Integration

The latest version of SentinelWeb now directly integrates with OpenWebUI, providing all the features of OpenWebUI plus drone-specific functionality. This integration allows for:

- Using OpenWebUI's powerful interface components
- Maintaining compatibility with OpenWebUI updates
- Adding drone-specific adapters and endpoints
- Seamless user experience across both platforms

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

## Architecture

SentinelWeb is built as a standalone addon that integrates with BuloCloudSentinel's existing microservices:

- **Backend**: FastAPI-based service that connects to BuloCloudSentinel's APIs
- **Frontend**: React-based SPA with responsive design and PWA capabilities
- **Database**: PostgreSQL for persistent storage, Redis for caching
- **Integration**: WebSockets for real-time data, REST APIs for CRUD operations

## Installation

### Prerequisites

- BuloCloudSentinel platform (running and configured)
- Docker and Docker Compose
- Node.js 16+ (for development)
- Python 3.9+ (for development)

### Docker Installation

#### Standard Installation

```bash
# Clone the repository
git clone https://github.com/yourusername/sentinelweb.git
cd sentinelweb

# Configure environment variables
cp .env.example .env
# Edit .env with your configuration

# Start with Docker Compose
docker-compose up -d
```

#### OpenWebUI Integration

To use the new OpenWebUI integration:

```bash
# Clone the repository
git clone https://github.com/yourusername/sentinelweb.git
cd sentinelweb

# Configure environment variables
export SENTINEL_API_URL=http://bulocloud-sentinel-api:8000
export SENTINEL_API_TOKEN=your-token
export RTMP_SERVER=rtmp://rtmp-server:1935

# Start with Docker Compose
docker-compose -f docker-compose.new.yml up -d
```

### Development Setup

#### Standard Development

```bash
# Backend setup
cd backend
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
pip install -r requirements.txt
uvicorn main:app --reload

# Frontend setup
cd frontend
npm install
npm start
```

#### OpenWebUI Integration Development

```bash
# Clone OpenWebUI repository if not already done
git clone https://github.com/open-webui/open-webui.git

# Install OpenWebUI dependencies
cd open-webui/backend
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
pip install -e .
cd ../..

# Install SentinelWeb dependencies
cd sentinelweb/backend
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
pip install -e .
cd ../..

# Run SentinelWeb with OpenWebUI integration
# On Windows
run_fixed.bat

# On Linux/macOS
chmod +x run_fixed.sh
./run_fixed.sh
```

## Configuration

### Standard Configuration

SentinelWeb can be configured through environment variables or a configuration file:

- `SENTINEL_API_URL`: URL of the BuloCloudSentinel API
- `DATABASE_URL`: PostgreSQL connection string
- `REDIS_URL`: Redis connection string
- `JWT_SECRET`: Secret key for JWT token verification (must match BuloCloudSentinel)
- `RTMP_SERVER`: RTMP server URL for video streaming

### OpenWebUI Integration Configuration

For the OpenWebUI integration, the following environment variables are used:

- `SENTINEL_API_URL`: URL of the BuloCloudSentinel API (default: http://bulocloud-sentinel-api:8000)
- `SENTINEL_API_TOKEN`: API token for authentication with BuloCloudSentinel
- `RTMP_SERVER`: RTMP server URL for video streaming (default: rtmp://rtmp-server:1935)
- `SECRET_KEY`: Secret key for session encryption

Additionally, all OpenWebUI configuration options are available. See the [OpenWebUI documentation](https://github.com/open-webui/open-webui) for details.

## Usage

### Standard Installation

After installation, access SentinelWeb at `http://localhost:3000` (or your configured URL).

1. Log in using your BuloCloudSentinel credentials
2. Navigate through the dashboard to access different features
3. Configure your preferences and workspace layout
4. Start managing your drone operations through the intuitive interface

### OpenWebUI Integration

After installation, access SentinelWeb with OpenWebUI integration at `http://localhost:8080`.

1. Log in using your OpenWebUI credentials
2. Access all OpenWebUI features as normal
3. Use the additional drone management features through the API endpoints:
   - `/api/drones`: Drone management
   - `/api/missions`: Mission planning
   - `/api/telemetry`: Telemetry monitoring
   - `/api/video`: Video streaming

## Development

### Project Structure

#### Standard Structure

```
sentinelweb/
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
├── docker/               # Docker configuration
└── docs/                 # Documentation
```

#### OpenWebUI Integration Structure

```
sentinelweb/
├── backend/
│   ├── sentinel_web/
│   │   ├── drone_adapter/    # Drone API adapter
│   │   ├── mission_adapter/  # Mission API adapter
│   │   ├── telemetry_adapter/# Telemetry API adapter
│   │   ├── video_adapter/    # Video streaming adapter
│   │   ├── routers/          # API routers
│   │   └── main_fixed.py     # Integration with OpenWebUI
│   └── setup.py              # Package setup
├── Dockerfile.new            # Docker configuration for integration
└── docker-compose.new.yml    # Docker Compose for integration
```

### Contributing

1. Fork the repository
2. Create a feature branch: `git checkout -b feature/my-feature`
3. Commit your changes: `git commit -am 'Add new feature'`
4. Push to the branch: `git push origin feature/my-feature`
5. Submit a pull request

## License

This project is licensed under the BSD-3-Clause License - see the LICENSE file for details.

## Acknowledgments

- OpenWebUI team for their excellent platform and integration support
- BuloCloudSentinel team for the core platform
- All contributors to the project
