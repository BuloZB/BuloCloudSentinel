# Persistent Mapping Module Architecture

## Overview

The Persistent Mapping Module is a microservice that generates orthomosaic images and 3D terrain meshes from drone imagery. It integrates with the Bulo.Cloud Sentinel platform and provides visualization capabilities through SentinelWeb.

## System Architecture

The Persistent Mapping Module consists of the following components:

### API Service

The API service provides RESTful endpoints for managing mapping projects, uploading images, and initiating processing jobs. It is built with FastAPI and provides the following functionality:

- Project management (create, read, update)
- Image upload and management
- Processing job management
- Asset retrieval

### Processing Worker

The processing worker is responsible for executing mapping jobs. It polls the database for queued jobs, processes them using OpenDroneMap, and stores the results in MinIO. The worker is designed to run as a separate service and can be scaled horizontally.

### OpenDroneMap Integration

The module integrates with OpenDroneMap for photogrammetry processing. It uses the NodeODM API to submit processing jobs and retrieve results. The integration supports the following features:

- Orthomosaic generation
- 3D mesh reconstruction
- Digital Surface Model (DSM) generation
- Digital Terrain Model (DTM) generation
- Point cloud generation

### Storage

The module uses MinIO for storing mapping assets. The following types of assets are stored:

- Source images
- Orthomosaics
- 3D meshes
- Point clouds
- Map tiles

### Database

The module uses PostgreSQL with PostGIS for storing metadata and geospatial information. The database schema includes the following tables:

- `mapping_projects`: Stores project metadata
- `source_images`: Stores information about uploaded images
- `mapping_assets`: Stores information about generated assets
- `processing_jobs`: Stores information about processing jobs

## Data Flow

1. **Image Upload**:
   - User uploads geotagged images through the API
   - Images are stored in MinIO
   - Image metadata is extracted and stored in the database
   - Project boundary is updated based on image locations

2. **Processing**:
   - User initiates processing through the API
   - Processing job is created and queued
   - Worker picks up the job and processes it using OpenDroneMap
   - Results are stored in MinIO
   - Asset metadata is stored in the database

3. **Visualization**:
   - User requests mapping data through SentinelWeb
   - SentinelWeb retrieves asset metadata from the API
   - SentinelWeb loads assets from MinIO
   - Assets are visualized using Cesium

## Component Diagram

```
┌─────────────────┐     ┌─────────────────┐     ┌─────────────────┐
│                 │     │                 │     │                 │
│  SentinelWeb    │────▶│  Mapping API    │────▶│  PostgreSQL     │
│                 │     │                 │     │                 │
└─────────────────┘     └─────────────────┘     └─────────────────┘
                               │  ▲                     ▲
                               │  │                     │
                               ▼  │                     │
┌─────────────────┐     ┌─────────────────┐     ┌─────────────────┐
│                 │     │                 │     │                 │
│  MinIO          │◀───▶│  Worker         │────▶│  OpenDroneMap   │
│                 │     │                 │     │                 │
└─────────────────┘     └─────────────────┘     └─────────────────┘
```

## API Endpoints

The API provides the following endpoints:

- `POST /api/v1/projects`: Create a new project
- `GET /api/v1/projects`: List projects
- `GET /api/v1/projects/{project_id}`: Get project details
- `PUT /api/v1/projects/{project_id}`: Update project
- `POST /api/v1/projects/{project_id}/images`: Upload images
- `GET /api/v1/projects/{project_id}/images`: List images
- `POST /api/v1/projects/{project_id}/process`: Start processing
- `GET /api/v1/projects/{project_id}/assets`: List assets
- `GET /api/v1/projects/{project_id}/jobs`: List processing jobs

## Database Schema

### mapping_projects

- `id`: UUID (primary key)
- `name`: String
- `description`: String
- `created_at`: Timestamp
- `updated_at`: Timestamp
- `user_id`: UUID
- `status`: String
- `image_count`: Integer
- `processing_time`: Integer
- `area_coverage`: Float
- `resolution`: Float
- `boundary`: Geography (polygon)

### source_images

- `id`: UUID (primary key)
- `project_id`: UUID (foreign key)
- `filename`: String
- `storage_path`: String
- `captured_at`: Timestamp
- `location`: Geography (point)
- `altitude`: Float
- `heading`: Float
- `metadata`: JSON

### mapping_assets

- `id`: UUID (primary key)
- `project_id`: UUID (foreign key)
- `type`: String
- `created_at`: Timestamp
- `storage_path`: String
- `file_size`: Integer
- `metadata`: JSON
- `version`: Integer
- `is_current`: Boolean

### processing_jobs

- `id`: UUID (primary key)
- `project_id`: UUID (foreign key)
- `status`: String
- `created_at`: Timestamp
- `started_at`: Timestamp
- `completed_at`: Timestamp
- `error_message`: String
- `parameters`: JSON
- `worker_id`: String

## Deployment

The module is deployed as a set of Docker containers:

- `api`: FastAPI application
- `worker`: Processing worker
- `db`: PostgreSQL with PostGIS
- `minio`: MinIO object storage
- `odm`: OpenDroneMap NodeODM

The containers are orchestrated using Docker Compose or Kubernetes.
