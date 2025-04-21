# SentinelWeb

SentinelWeb is a web interface for BuloCloudSentinel, based on OpenWebUI. It provides a user-friendly interface for managing drones, missions, telemetry, and video streams.

## Features

- **Full OpenWebUI Functionality**: All features of OpenWebUI are available
- **Drone Management**: View and control drones
- **Mission Planning**: Create, update, and delete missions
- **Telemetry Monitoring**: View real-time telemetry data
- **Video Streaming**: Stream video from drones
- **User Authentication**: Secure access to the platform
- **Offline Operation**: Work without internet connection
- **Extensibility**: Add custom plugins and extensions

## Architecture

SentinelWeb is built on top of OpenWebUI, with additional adapters for BuloCloudSentinel:

- **DroneAdapter**: Interface with drone operations
- **MissionAdapter**: Interface with mission planning
- **TelemetryAdapter**: Interface with telemetry data
- **VideoAdapter**: Interface with video streaming

## Installation

### Prerequisites

- Python 3.9 or higher
- Git
- Docker and Docker Compose (optional, for containerized deployment)

### Option 1: Local Installation

1. Clone the repository:
   ```bash
   git clone https://github.com/your-org/sentinelweb.git
   cd sentinelweb
   ```

2. Run the setup script:
   - On Windows:
     ```bash
     run_fixed.bat
     ```
   - On Linux/macOS:
     ```bash
     chmod +x run_fixed.sh
     ./run_fixed.sh
     ```

3. Access the web interface at http://localhost:8080

### Option 2: Docker Installation

1. Clone the repository:
   ```bash
   git clone https://github.com/your-org/sentinelweb.git
   cd sentinelweb
   ```

2. Configure environment variables in `docker-compose.new.yml`:
   ```yaml
   environment:
     - SENTINEL_API_URL=http://bulocloud-sentinel-api:8000
     - SENTINEL_API_TOKEN=your-api-token
     - RTMP_SERVER=rtmp://rtmp-server:1935
     - SECRET_KEY=your-secret-key
   ```

3. Start the services:
   ```bash
   docker-compose -f docker-compose.new.yml up -d
   ```

4. Access the web interface at http://localhost:8080

## API Endpoints

SentinelWeb adds the following API endpoints to OpenWebUI:

### Drones

- `GET /api/drones`: Get all drones
- `GET /api/drones/{drone_id}`: Get a specific drone
- `POST /api/drones/{drone_id}/command`: Send a command to a drone

### Missions

- `GET /api/missions`: Get all missions
- `GET /api/missions/{mission_id}`: Get a specific mission
- `POST /api/missions`: Create a new mission

### Telemetry

- `GET /api/telemetry/{drone_id}`: Get telemetry for a specific drone
- `GET /api/telemetry/{drone_id}/battery`: Get battery status for a specific drone

### Video

- `GET /api/video/streams`: Get all video streams
- `GET /api/video/drones/{drone_id}`: Get video stream for a specific drone

## Configuration

SentinelWeb can be configured using environment variables:

- `SENTINEL_API_URL`: URL of the BuloCloudSentinel API (default: http://localhost:8000)
- `SENTINEL_API_TOKEN`: API token for authentication (default: empty)
- `RTMP_SERVER`: URL of the RTMP server for video streaming (default: rtmp://localhost:1935)
- `SECRET_KEY`: Secret key for session encryption (default: sentinelweb-secret-key)

## Development

### Project Structure

```
sentinelweb/
├── backend/
│   ├── sentinel_web/
│   │   ├── drone_adapter/
│   │   ├── mission_adapter/
│   │   ├── telemetry_adapter/
│   │   ├── video_adapter/
│   │   ├── routers/
│   │   └── main_fixed.py
│   └── setup.py
├── Dockerfile.new
└── docker-compose.new.yml
```

### Adding New Features

To add new features to SentinelWeb:

1. Create a new adapter in the appropriate directory
2. Add new endpoints to the main_fixed.py file
3. Update the documentation

## License

This project is licensed under the MIT License - see the LICENSE file for details.
