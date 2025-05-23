# Persistent Mapping Module API Documentation

## Overview

The Persistent Mapping Module API provides endpoints for managing mapping projects, uploading images, and initiating processing jobs. The API is built with FastAPI and follows RESTful principles.

## Base URL

The API is available at:

```
http://localhost:8070/api/v1
```

## Authentication

The API uses JWT authentication. To access protected endpoints, include an `Authorization` header with a valid JWT token:

```
Authorization: Bearer <token>
```

## Endpoints

### Projects

#### Create Project

```
POST /projects
```

Create a new mapping project.

**Request Body:**

```json
{
  "name": "Project Name",
  "description": "Project Description"
}
```

**Response:**

```json
{
  "id": "123e4567-e89b-12d3-a456-426614174000",
  "name": "Project Name",
  "description": "Project Description",
  "created_at": "2023-06-01T12:00:00Z",
  "updated_at": "2023-06-01T12:00:00Z",
  "user_id": "123e4567-e89b-12d3-a456-426614174001",
  "status": "created",
  "image_count": 0,
  "processing_time": null,
  "area_coverage": null,
  "resolution": null
}
```

#### List Projects

```
GET /projects
```

List all projects for the authenticated user.

**Response:**

```json
[
  {
    "id": "123e4567-e89b-12d3-a456-426614174000",
    "name": "Project Name",
    "description": "Project Description",
    "created_at": "2023-06-01T12:00:00Z",
    "updated_at": "2023-06-01T12:00:00Z",
    "user_id": "123e4567-e89b-12d3-a456-426614174001",
    "status": "created",
    "image_count": 0,
    "processing_time": null,
    "area_coverage": null,
    "resolution": null
  }
]
```

#### Get Project

```
GET /projects/{project_id}
```

Get a project by ID.

**Response:**

```json
{
  "id": "123e4567-e89b-12d3-a456-426614174000",
  "name": "Project Name",
  "description": "Project Description",
  "created_at": "2023-06-01T12:00:00Z",
  "updated_at": "2023-06-01T12:00:00Z",
  "user_id": "123e4567-e89b-12d3-a456-426614174001",
  "status": "created",
  "image_count": 0,
  "processing_time": null,
  "area_coverage": null,
  "resolution": null,
  "source_images": [],
  "assets": [],
  "processing_jobs": []
}
```

#### Update Project

```
PUT /projects/{project_id}
```

Update a project.

**Request Body:**

```json
{
  "name": "New Project Name",
  "description": "New Project Description"
}
```

**Response:**

```json
{
  "id": "123e4567-e89b-12d3-a456-426614174000",
  "name": "New Project Name",
  "description": "New Project Description",
  "created_at": "2023-06-01T12:00:00Z",
  "updated_at": "2023-06-01T12:01:00Z",
  "user_id": "123e4567-e89b-12d3-a456-426614174001",
  "status": "created",
  "image_count": 0,
  "processing_time": null,
  "area_coverage": null,
  "resolution": null
}
```

### Images

#### Upload Images

```
POST /projects/{project_id}/images
```

Upload images to a project. This endpoint accepts multipart/form-data with one or more image files.

**Request:**

```
Content-Type: multipart/form-data

files: [file1, file2, ...]
```

**Response:**

```json
{
  "message": "Successfully uploaded 2 images"
}
```

#### List Images

```
GET /projects/{project_id}/images
```

List all images for a project.

**Response:**

```json
[
  {
    "id": "123e4567-e89b-12d3-a456-426614174002",
    "project_id": "123e4567-e89b-12d3-a456-426614174000",
    "filename": "image1.jpg",
    "storage_path": "images/123e4567-e89b-12d3-a456-426614174000/image1.jpg",
    "captured_at": "2023-06-01T10:00:00Z",
    "location": {
      "type": "Point",
      "coordinates": [10.0, 20.0]
    },
    "altitude": 100.0,
    "heading": 90.0,
    "metadata": {
      "content_type": "image/jpeg",
      "size": 1024000
    }
  }
]
```

### Processing

#### Start Processing

```
POST /projects/{project_id}/process
```

Start processing a project.

**Request Body:**

```json
{
  "options": {
    "dsm": true,
    "dtm": true,
    "orthophoto_resolution": 5.0,
    "mesh_size": 200000,
    "use_3dmesh": true,
    "pc_quality": "medium",
    "crop": 0.0,
    "feature_quality": "high",
    "min_num_features": 8000,
    "matcher_neighbors": 8,
    "texturing_data_term": "gmi",
    "texturing_outlier_removal_type": "gauss_damping",
    "texturing_skip_local_seam_leveling": false,
    "texturing_skip_global_seam_leveling": false,
    "texturing_tone_mapping": "none",
    "dem_resolution": 5.0,
    "orthophoto": true,
    "verbose": true
  }
}
```

**Response:**

```json
{
  "id": "123e4567-e89b-12d3-a456-426614174003",
  "project_id": "123e4567-e89b-12d3-a456-426614174000",
  "status": "queued",
  "created_at": "2023-06-01T12:02:00Z",
  "started_at": null,
  "completed_at": null,
  "error_message": null,
  "parameters": {
    "dsm": true,
    "dtm": true,
    "orthophoto_resolution": 5.0,
    "mesh_size": 200000,
    "use_3dmesh": true,
    "pc_quality": "medium",
    "crop": 0.0,
    "feature_quality": "high",
    "min_num_features": 8000,
    "matcher_neighbors": 8,
    "texturing_data_term": "gmi",
    "texturing_outlier_removal_type": "gauss_damping",
    "texturing_skip_local_seam_leveling": false,
    "texturing_skip_global_seam_leveling": false,
    "texturing_tone_mapping": "none",
    "dem_resolution": 5.0,
    "orthophoto": true,
    "verbose": true
  },
  "worker_id": null
}
```

#### List Processing Jobs

```
GET /projects/{project_id}/jobs
```

List all processing jobs for a project.

**Response:**

```json
[
  {
    "id": "123e4567-e89b-12d3-a456-426614174003",
    "project_id": "123e4567-e89b-12d3-a456-426614174000",
    "status": "queued",
    "created_at": "2023-06-01T12:02:00Z",
    "started_at": null,
    "completed_at": null,
    "error_message": null,
    "parameters": {
      "dsm": true,
      "dtm": true,
      "orthophoto_resolution": 5.0,
      "mesh_size": 200000,
      "use_3dmesh": true,
      "pc_quality": "medium",
      "crop": 0.0,
      "feature_quality": "high",
      "min_num_features": 8000,
      "matcher_neighbors": 8,
      "texturing_data_term": "gmi",
      "texturing_outlier_removal_type": "gauss_damping",
      "texturing_skip_local_seam_leveling": false,
      "texturing_skip_global_seam_leveling": false,
      "texturing_tone_mapping": "none",
      "dem_resolution": 5.0,
      "orthophoto": true,
      "verbose": true
    },
    "worker_id": null
  }
]
```

### Assets

#### List Assets

```
GET /projects/{project_id}/assets
```

List all assets for a project.

**Response:**

```json
[
  {
    "id": "123e4567-e89b-12d3-a456-426614174004",
    "project_id": "123e4567-e89b-12d3-a456-426614174000",
    "type": "orthomosaic",
    "created_at": "2023-06-01T12:10:00Z",
    "storage_path": "orthomosaics/123e4567-e89b-12d3-a456-426614174000/orthomosaic.tif",
    "file_size": 102400000,
    "metadata": {
      "format": "geotiff"
    },
    "version": 1,
    "is_current": true,
    "url": "http://localhost:9005/mapping-service/orthomosaics/123e4567-e89b-12d3-a456-426614174000/orthomosaic.tif"
  },
  {
    "id": "123e4567-e89b-12d3-a456-426614174005",
    "project_id": "123e4567-e89b-12d3-a456-426614174000",
    "type": "orthomosaic_tiles",
    "created_at": "2023-06-01T12:10:00Z",
    "storage_path": "tiles/123e4567-e89b-12d3-a456-426614174000/orthomosaic",
    "file_size": null,
    "metadata": {
      "tile_format": "xyz",
      "min_zoom": 12,
      "max_zoom": 22
    },
    "version": 1,
    "is_current": true,
    "url": "http://localhost:9005/mapping-service/tiles/123e4567-e89b-12d3-a456-426614174000/orthomosaic"
  }
]
```

## Error Responses

The API returns standard HTTP status codes and error responses:

```json
{
  "detail": "Error message"
}
```

Common error codes:

- `400 Bad Request`: Invalid request parameters
- `401 Unauthorized`: Missing or invalid authentication
- `403 Forbidden`: Insufficient permissions
- `404 Not Found`: Resource not found
- `500 Internal Server Error`: Server error
