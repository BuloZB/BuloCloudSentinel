# Drone Show Microservice for Bulo.Cloud Sentinel

This microservice provides drone light show capabilities for the Bulo.Cloud Sentinel platform, enabling planning, simulation, and execution of LED choreographies across a fleet of drones.

## Features

- **Choreography Definition**: JSON-based format for defining LED patterns and waypoints
- **Simulation & Preview**: 3D visualization of drone positions and LED states
- **Execution Engine**: Synchronized execution of choreographies across a fleet of drones
- **Monitoring & Logging**: Real-time telemetry and logging of show execution
- **Integration**: Seamless integration with Bulo.Cloud Sentinel platform

## Architecture

The Drone Show microservice is built as a standalone microservice that integrates with the Bulo.Cloud Sentinel platform. It leverages concepts and code from:

- UGCS/ddc's synchronized flight-path planning
- Skybrush's server/frontend architecture
- MAVSDK Drone Show's offboard control patterns
- Clever-Show's Blender-to-drone path pipeline

## API Endpoints

- `POST /shows`: Upload a new choreography
- `GET /shows`: List all choreographies
- `GET /shows/{id}`: Get details of a specific choreography
- `GET /shows/{id}/simulate`: Run a simulation of a choreography
- `POST /shows/{id}/execute`: Execute a choreography
- `GET /shows/{id}/status`: Get the status of a choreography execution

## Installation

### Prerequisites

- Docker and Docker Compose
- Kubernetes (optional, for production deployment)
- Bulo.Cloud Sentinel platform

### Docker Deployment

```bash
# Clone the repository
git clone https://github.com/your-org/drone-show-service.git
cd drone-show-service

# Start the service
docker-compose up -d
```

### Kubernetes Deployment

```bash
# Apply Kubernetes manifests
kubectl apply -f kubernetes/
```

## Configuration

The service can be configured using environment variables:

- `SENTINEL_API_URL`: URL of the Bulo.Cloud Sentinel API
- `SENTINEL_API_TOKEN`: API token for authentication
- `DATABASE_URL`: PostgreSQL connection string
- `REDIS_URL`: Redis connection string
- `MINIO_URL`: MinIO connection string
- `RTMP_SERVER`: RTMP server URL for video streaming

## Development

### Project Structure

```
drone_show_service/
├── api/                # FastAPI application
│   ├── endpoints/      # API endpoints
│   └── main.py         # Application entry point
├── core/               # Core business logic
├── models/             # Data models
├── services/           # Service implementations
├── utils/              # Utility functions
├── Dockerfile          # Docker configuration
├── docker-compose.yml  # Docker Compose configuration
└── kubernetes/         # Kubernetes manifests
```

### Running Tests

```bash
# Run tests
pytest
```

## License

This project is licensed under the MIT License - see the LICENSE file for details.
