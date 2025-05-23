# Persistent Mapping Module for Bulo.Cloud Sentinel

## Overview

The Persistent Mapping Module is a microservice that generates orthomosaic images and 3D terrain meshes from drone imagery. It integrates with the Bulo.Cloud Sentinel platform and provides visualization capabilities through SentinelWeb.

## Features

- **Image Collection**: Upload and manage geotagged drone images
- **Orthomosaic Generation**: Create high-resolution aerial images
- **3D Mesh Reconstruction**: Generate 3D terrain models
- **Map Tiling**: Create map tiles for efficient streaming
- **Geospatial Database**: Store and query mapping data with PostGIS
- **Cesium Integration**: Visualize mapping data in 3D with Cesium
- **API**: RESTful API for integration with other systems
- **Worker**: Asynchronous processing of mapping jobs
- **Storage**: Efficient storage and retrieval of mapping assets

## Architecture

The Persistent Mapping Module consists of the following components:

- **API Service**: FastAPI application for managing mapping projects
- **Processing Worker**: Asynchronous worker for processing mapping jobs
- **Database**: PostgreSQL with PostGIS for storing metadata and geospatial data
- **Storage**: MinIO for storing mapping assets
- **OpenDroneMap**: Photogrammetry engine for processing drone imagery

For more details, see the [Architecture Documentation](docs/architecture.md).

## Installation

### Prerequisites

- Docker and Docker Compose
- 8GB+ RAM
- 50GB+ disk space

### Installation Steps

1. Clone the repository:

```bash
git clone https://github.com/BuloZB/BuloCloudSentinel.git
cd BuloCloudSentinel/mapping_service
```

2. Configure environment variables:

```bash
cp .env.example .env
# Edit .env file with your configuration
```

3. Start the services:

```bash
docker-compose up -d
```

4. Initialize the database:

```bash
docker-compose exec api alembic upgrade head
```

5. Access the API at http://localhost:8070/docs

## Usage

### Creating a Project

```bash
curl -X POST "http://localhost:8070/api/v1/projects" \
  -H "Content-Type: application/json" \
  -d '{"name": "My Project", "description": "My first mapping project"}'
```

### Uploading Images

```bash
curl -X POST "http://localhost:8070/api/v1/projects/{project_id}/images" \
  -H "Content-Type: multipart/form-data" \
  -F "files=@image1.jpg" \
  -F "files=@image2.jpg"
```

### Starting Processing

```bash
curl -X POST "http://localhost:8070/api/v1/projects/{project_id}/process" \
  -H "Content-Type: application/json" \
  -d '{"options": {"orthophoto_resolution": 5.0, "mesh_size": 200000}}'
```

### Getting Results

```bash
curl -X GET "http://localhost:8070/api/v1/projects/{project_id}/assets"
```

For more details, see the [API Documentation](docs/api.md) and [User Guide](docs/user_guide.md).

## Development

### Project Structure

```
mapping_service/
├── api/                  # API endpoints
├── core/                 # Core functionality
├── db/                   # Database models and connections
├── processing/           # Processing pipeline
├── services/             # Business logic services
├── utils/                # Utility functions
├── tests/                # Tests
├── docker/               # Docker configuration
├── docs/                 # Documentation
├── main.py               # Application entry point
├── worker_main.py        # Worker entry point
└── requirements.txt      # Python dependencies
```

### Running Tests

```bash
# Run unit tests
pytest mapping_service/tests/unit

# Run integration tests
pytest mapping_service/tests/integration

# Run all tests with coverage
pytest --cov=mapping_service
```

### Development Environment

1. Create a virtual environment:

```bash
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
```

2. Install dependencies:

```bash
pip install -r requirements.txt
```

3. Run the API service:

```bash
uvicorn mapping_service.main:app --reload
```

4. Run the worker:

```bash
python -m mapping_service.worker_main
```

## Contributing

1. Fork the repository
2. Create a feature branch: `git checkout -b feature/my-feature`
3. Commit your changes: `git commit -am 'Add my feature'`
4. Push to the branch: `git push origin feature/my-feature`
5. Submit a pull request

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Acknowledgments

- [OpenDroneMap](https://github.com/OpenDroneMap/ODM) for photogrammetry processing
- [Cesium](https://cesium.com/) for 3D visualization
- [FastAPI](https://fastapi.tiangolo.com/) for API development
- [PostgreSQL](https://www.postgresql.org/) and [PostGIS](https://postgis.net/) for geospatial database
- [MinIO](https://min.io/) for object storage
