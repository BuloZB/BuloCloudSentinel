# SentinelWeb

SentinelWeb is a modular, extensible web interface addon for BuloCloudSentinel, providing a user-friendly dashboard for drone management and surveillance operations. It is a direct adaptation of [OpenWebUI](https://github.com/open-webui/open-webui) for drone operations.

## Overview

SentinelWeb is a comprehensive web interface for BuloCloudSentinel, built by adapting OpenWebUI's architecture and codebase specifically for drone operations. It provides a unified dashboard for managing drones, missions, telemetry data, and AI-powered surveillance features, with the same intuitive user experience as OpenWebUI.

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

```bash
# Clone the repository
git clone https://github.com/BuloZB/BuloCloudSentinel.git
cd BuloCloudSentinel/addons/sentinelweb

# Configure environment variables
cp .env.example .env
# Edit .env with your configuration

# Start with Docker Compose
docker-compose up -d
```

### Development Setup

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

## Configuration

SentinelWeb can be configured through environment variables or a configuration file:

- `SENTINEL_API_URL`: URL of the BuloCloudSentinel API
- `DATABASE_URL`: PostgreSQL connection string
- `REDIS_URL`: Redis connection string
- `JWT_SECRET`: Secret key for JWT token verification (must match BuloCloudSentinel)
- `RTMP_SERVER`: RTMP server URL for video streaming

## Usage

After installation, access SentinelWeb at `http://localhost:3000` (or your configured URL).

1. Log in using your BuloCloudSentinel credentials
2. Navigate through the dashboard to access different features
3. Configure your preferences and workspace layout
4. Start managing your drone operations through the intuitive interface

## Development

### Project Structure

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

### Contributing

1. Fork the repository
2. Create a feature branch: `git checkout -b feature/my-feature`
3. Commit your changes: `git commit -am 'Add new feature'`
4. Push to the branch: `git push origin feature/my-feature`
5. Submit a pull request

## License

This project is licensed under the BSD-3-Clause License - see the LICENSE file for details.

## Acknowledgments

- [OpenWebUI](https://github.com/open-webui/open-webui) for the codebase and interface design
- BuloCloudSentinel team for the core platform
- All contributors to the project

## Attribution

SentinelWeb is built on top of OpenWebUI, which is licensed under the MIT License. We have adapted the codebase for drone operations while maintaining compliance with the original license. See the LICENSE file for details.
