# Drone Show Microservice API Documentation

This document provides detailed information about the API endpoints available in the Drone Show microservice.

## Base URL

All API endpoints are relative to the base URL:

```
http://your-server:8000
```

## Authentication

Authentication is handled through the Bulo.Cloud Sentinel platform. All requests must include a valid JWT token in the `Authorization` header:

```
Authorization: Bearer <token>
```

## Endpoints

### Health Check

```
GET /health
```

Check if the API is running.

**Response:**

```json
{
  "status": "ok"
}
```

### Choreography Management

#### Create Choreography

```
POST /shows
```

Create a new choreography.

**Request Body:**

```json
{
  "metadata": {
    "name": "My Choreography",
    "description": "A beautiful drone show",
    "author": "John Doe",
    "tags": ["demo", "test"],
    "duration": 120.0,
    "drone_count": 10,
    "status": "draft"
  },
  "type": "waypoint",
  "trajectories": [
    {
      "drone_id": "drone_1",
      "waypoints": [
        {
          "time": 0.0,
          "position": {
            "lat": 37.7749,
            "lon": -122.4194,
            "alt": 10.0
          },
          "heading": 0.0
        },
        {
          "time": 60.0,
          "position": {
            "lat": 37.7750,
            "lon": -122.4195,
            "alt": 20.0
          },
          "heading": 90.0
        }
      ],
      "led_states": [
        {
          "time": 0.0,
          "color": {
            "r": 255,
            "g": 0,
            "b": 0
          },
          "effect": "solid"
        },
        {
          "time": 60.0,
          "color": {
            "r": 0,
            "g": 0,
            "b": 255
          },
          "effect": "solid"
        }
      ]
    }
  ]
}
```

**Response:**

```json
{
  "id": "123e4567-e89b-12d3-a456-426614174000",
  "metadata": {
    "name": "My Choreography",
    "description": "A beautiful drone show",
    "author": "John Doe",
    "created_at": "2023-06-01T12:00:00Z",
    "updated_at": "2023-06-01T12:00:00Z",
    "tags": ["demo", "test"],
    "duration": 120.0,
    "drone_count": 10,
    "status": "draft"
  },
  "type": "waypoint",
  "trajectories": [
    {
      "drone_id": "drone_1",
      "waypoints": [
        {
          "time": 0.0,
          "position": {
            "lat": 37.7749,
            "lon": -122.4194,
            "alt": 10.0
          },
          "heading": 0.0
        },
        {
          "time": 60.0,
          "position": {
            "lat": 37.7750,
            "lon": -122.4195,
            "alt": 20.0
          },
          "heading": 90.0
        }
      ],
      "led_states": [
        {
          "time": 0.0,
          "color": {
            "r": 255,
            "g": 0,
            "b": 0
          },
          "effect": "solid"
        },
        {
          "time": 60.0,
          "color": {
            "r": 0,
            "g": 0,
            "b": 255
          },
          "effect": "solid"
        }
      ]
    }
  ]
}
```

#### Get All Choreographies

```
GET /shows
```

Get a list of all choreographies.

**Query Parameters:**

- `skip` (optional): Number of records to skip (default: 0)
- `limit` (optional): Maximum number of records to return (default: 100)
- `status` (optional): Filter by status (e.g., "draft", "ready", "simulated", "executed", "archived")

**Response:**

```json
[
  {
    "id": "123e4567-e89b-12d3-a456-426614174000",
    "metadata": {
      "name": "My Choreography",
      "description": "A beautiful drone show",
      "author": "John Doe",
      "created_at": "2023-06-01T12:00:00Z",
      "updated_at": "2023-06-01T12:00:00Z",
      "tags": ["demo", "test"],
      "duration": 120.0,
      "drone_count": 10,
      "status": "draft"
    },
    "type": "waypoint",
    "trajectories": [...]
  },
  {
    "id": "223e4567-e89b-12d3-a456-426614174000",
    "metadata": {
      "name": "Another Choreography",
      "description": "Another beautiful drone show",
      "author": "Jane Doe",
      "created_at": "2023-06-02T12:00:00Z",
      "updated_at": "2023-06-02T12:00:00Z",
      "tags": ["demo", "production"],
      "duration": 180.0,
      "drone_count": 20,
      "status": "ready"
    },
    "type": "formation",
    "trajectories": [...]
  }
]
```

#### Get Choreography

```
GET /shows/{choreography_id}
```

Get a specific choreography by ID.

**Response:**

```json
{
  "id": "123e4567-e89b-12d3-a456-426614174000",
  "metadata": {
    "name": "My Choreography",
    "description": "A beautiful drone show",
    "author": "John Doe",
    "created_at": "2023-06-01T12:00:00Z",
    "updated_at": "2023-06-01T12:00:00Z",
    "tags": ["demo", "test"],
    "duration": 120.0,
    "drone_count": 10,
    "status": "draft"
  },
  "type": "waypoint",
  "trajectories": [...]
}
```

#### Update Choreography

```
PUT /shows/{choreography_id}
```

Update a specific choreography.

**Request Body:**

```json
{
  "metadata": {
    "name": "Updated Choreography",
    "description": "An updated drone show",
    "tags": ["demo", "test", "updated"]
  },
  "type": "waypoint",
  "trajectories": [...]
}
```

**Response:**

```json
{
  "id": "123e4567-e89b-12d3-a456-426614174000",
  "metadata": {
    "name": "Updated Choreography",
    "description": "An updated drone show",
    "author": "John Doe",
    "created_at": "2023-06-01T12:00:00Z",
    "updated_at": "2023-06-01T13:00:00Z",
    "tags": ["demo", "test", "updated"],
    "duration": 120.0,
    "drone_count": 10,
    "status": "draft"
  },
  "type": "waypoint",
  "trajectories": [...]
}
```

#### Delete Choreography

```
DELETE /shows/{choreography_id}
```

Delete a specific choreography.

**Response:**

```
204 No Content
```

#### Update Choreography Status

```
PATCH /shows/{choreography_id}/status
```

Update the status of a specific choreography.

**Request Body:**

```json
{
  "status": "ready"
}
```

**Response:**

```json
{
  "id": "123e4567-e89b-12d3-a456-426614174000",
  "metadata": {
    "name": "My Choreography",
    "description": "A beautiful drone show",
    "author": "John Doe",
    "created_at": "2023-06-01T12:00:00Z",
    "updated_at": "2023-06-01T13:00:00Z",
    "tags": ["demo", "test"],
    "duration": 120.0,
    "drone_count": 10,
    "status": "ready"
  },
  "type": "waypoint",
  "trajectories": [...]
}
```

### Simulation

#### Simulate Choreography

```
POST /simulation/{choreography_id}
```

Simulate a choreography.

**Request Body:**

```json
{
  "start_time": 0.0,
  "end_time": 120.0,
  "speed_factor": 1.0,
  "include_takeoff_landing": true,
  "visualize_led": true,
  "visualize_trajectories": true,
  "drone_model": "generic"
}
```

**Response:**

```json
{
  "id": "323e4567-e89b-12d3-a456-426614174000",
  "choreography_id": "123e4567-e89b-12d3-a456-426614174000",
  "settings": {
    "start_time": 0.0,
    "end_time": 120.0,
    "speed_factor": 1.0,
    "include_takeoff_landing": true,
    "visualize_led": true,
    "visualize_trajectories": true,
    "drone_model": "generic"
  },
  "frames": [...],
  "duration": 120.0,
  "created_at": "2023-06-01T14:00:00Z"
}
```

#### Get Simulation

```
GET /simulation/{simulation_id}
```

Get a specific simulation by ID.

**Response:**

```json
{
  "id": "323e4567-e89b-12d3-a456-426614174000",
  "choreography_id": "123e4567-e89b-12d3-a456-426614174000",
  "settings": {
    "start_time": 0.0,
    "end_time": 120.0,
    "speed_factor": 1.0,
    "include_takeoff_landing": true,
    "visualize_led": true,
    "visualize_trajectories": true,
    "drone_model": "generic"
  },
  "frames": [...],
  "duration": 120.0,
  "created_at": "2023-06-01T14:00:00Z"
}
```

#### Get All Simulations

```
GET /simulation
```

Get a list of all simulations.

**Query Parameters:**

- `choreography_id` (optional): Filter by choreography ID
- `skip` (optional): Number of records to skip (default: 0)
- `limit` (optional): Maximum number of records to return (default: 100)

**Response:**

```json
[
  {
    "id": "323e4567-e89b-12d3-a456-426614174000",
    "choreography_id": "123e4567-e89b-12d3-a456-426614174000",
    "settings": {
      "start_time": 0.0,
      "end_time": 120.0,
      "speed_factor": 1.0,
      "include_takeoff_landing": true,
      "visualize_led": true,
      "visualize_trajectories": true,
      "drone_model": "generic"
    },
    "frames": [...],
    "duration": 120.0,
    "created_at": "2023-06-01T14:00:00Z"
  },
  {
    "id": "423e4567-e89b-12d3-a456-426614174000",
    "choreography_id": "223e4567-e89b-12d3-a456-426614174000",
    "settings": {
      "start_time": 0.0,
      "end_time": 180.0,
      "speed_factor": 1.0,
      "include_takeoff_landing": true,
      "visualize_led": true,
      "visualize_trajectories": true,
      "drone_model": "generic"
    },
    "frames": [...],
    "duration": 180.0,
    "created_at": "2023-06-02T14:00:00Z"
  }
]
```

#### Delete Simulation

```
DELETE /simulation/{simulation_id}
```

Delete a specific simulation.

**Response:**

```
204 No Content
```

#### Simulation WebSocket

```
WebSocket /simulation/ws/{simulation_id}
```

Stream simulation data in real-time.

**Messages from Server:**

1. Initial data:

```json
{
  "type": "init",
  "data": {
    "id": "323e4567-e89b-12d3-a456-426614174000",
    "choreography_id": "123e4567-e89b-12d3-a456-426614174000",
    "settings": {
      "start_time": 0.0,
      "end_time": 120.0,
      "speed_factor": 1.0,
      "include_takeoff_landing": true,
      "visualize_led": true,
      "visualize_trajectories": true,
      "drone_model": "generic"
    },
    "duration": 120.0
  }
}
```

2. Frame data:

```json
{
  "type": "frame",
  "data": {
    "time": 0.0,
    "drone_states": {
      "drone_1": {
        "position": {
          "lat": 37.7749,
          "lon": -122.4194,
          "alt": 10.0
        },
        "heading": 0.0,
        "led_state": {
          "time": 0.0,
          "color": {
            "r": 255,
            "g": 0,
            "b": 0
          },
          "effect": "solid"
        }
      }
    }
  }
}
```

3. End message:

```json
{
  "type": "end",
  "data": {
    "id": "323e4567-e89b-12d3-a456-426614174000",
    "choreography_id": "123e4567-e89b-12d3-a456-426614174000"
  }
}
```

**Messages from Client:**

1. Acknowledge frame:

```json
{
  "action": "ack"
}
```

2. Pause simulation:

```json
{
  "action": "pause"
}
```

3. Resume simulation:

```json
{
  "action": "resume"
}
```

4. Stop simulation:

```json
{
  "action": "stop"
}
```

### Execution

#### Execute Choreography

```
POST /execution/{choreography_id}
```

Execute a choreography.

**Request Body:**

```json
{
  "include_takeoff_landing": true,
  "use_rtk": true,
  "safety_checks": true,
  "geofence_enabled": true,
  "return_home_on_low_battery": true,
  "return_home_on_connection_loss": true,
  "led_enabled": true
}
```

**Response:**

```json
{
  "id": "523e4567-e89b-12d3-a456-426614174000",
  "choreography_id": "123e4567-e89b-12d3-a456-426614174000",
  "settings": {
    "include_takeoff_landing": true,
    "use_rtk": true,
    "safety_checks": true,
    "geofence_enabled": true,
    "return_home_on_low_battery": true,
    "return_home_on_connection_loss": true,
    "led_enabled": true
  },
  "status": "pending",
  "drone_statuses": {},
  "current_time": 0.0,
  "progress": 0.0,
  "created_at": "2023-06-01T15:00:00Z",
  "updated_at": "2023-06-01T15:00:00Z"
}
```

#### Get Execution

```
GET /execution/{execution_id}
```

Get a specific execution by ID.

**Response:**

```json
{
  "id": "523e4567-e89b-12d3-a456-426614174000",
  "choreography_id": "123e4567-e89b-12d3-a456-426614174000",
  "settings": {
    "include_takeoff_landing": true,
    "use_rtk": true,
    "safety_checks": true,
    "geofence_enabled": true,
    "return_home_on_low_battery": true,
    "return_home_on_connection_loss": true,
    "led_enabled": true
  },
  "status": "in_progress",
  "drone_statuses": {
    "drone_1": {
      "drone_id": "drone_1",
      "connected": true,
      "battery_level": 95.0,
      "position": {
        "lat": 37.7749,
        "lon": -122.4194,
        "alt": 10.0
      },
      "heading": 0.0,
      "current_waypoint_index": 1,
      "status": "executing",
      "error_message": null
    }
  },
  "start_time": "2023-06-01T15:01:00Z",
  "end_time": null,
  "current_time": 30.0,
  "progress": 25.0,
  "created_at": "2023-06-01T15:00:00Z",
  "updated_at": "2023-06-01T15:01:30Z"
}
```

#### Get All Executions

```
GET /execution
```

Get a list of all executions.

**Query Parameters:**

- `choreography_id` (optional): Filter by choreography ID
- `status` (optional): Filter by status (e.g., "pending", "preparing", "in_progress", "paused", "completed", "aborted", "failed")
- `skip` (optional): Number of records to skip (default: 0)
- `limit` (optional): Maximum number of records to return (default: 100)

**Response:**

```json
[
  {
    "id": "523e4567-e89b-12d3-a456-426614174000",
    "choreography_id": "123e4567-e89b-12d3-a456-426614174000",
    "settings": {...},
    "status": "in_progress",
    "drone_statuses": {...},
    "start_time": "2023-06-01T15:01:00Z",
    "end_time": null,
    "current_time": 30.0,
    "progress": 25.0,
    "created_at": "2023-06-01T15:00:00Z",
    "updated_at": "2023-06-01T15:01:30Z"
  },
  {
    "id": "623e4567-e89b-12d3-a456-426614174000",
    "choreography_id": "223e4567-e89b-12d3-a456-426614174000",
    "settings": {...},
    "status": "completed",
    "drone_statuses": {...},
    "start_time": "2023-06-02T15:01:00Z",
    "end_time": "2023-06-02T15:04:00Z",
    "current_time": 180.0,
    "progress": 100.0,
    "created_at": "2023-06-02T15:00:00Z",
    "updated_at": "2023-06-02T15:04:00Z"
  }
]
```

#### Pause Execution

```
POST /execution/{execution_id}/pause
```

Pause a running execution.

**Response:**

```json
{
  "id": "523e4567-e89b-12d3-a456-426614174000",
  "choreography_id": "123e4567-e89b-12d3-a456-426614174000",
  "settings": {...},
  "status": "paused",
  "drone_statuses": {...},
  "start_time": "2023-06-01T15:01:00Z",
  "end_time": null,
  "current_time": 30.0,
  "progress": 25.0,
  "created_at": "2023-06-01T15:00:00Z",
  "updated_at": "2023-06-01T15:01:30Z"
}
```

#### Resume Execution

```
POST /execution/{execution_id}/resume
```

Resume a paused execution.

**Response:**

```json
{
  "id": "523e4567-e89b-12d3-a456-426614174000",
  "choreography_id": "123e4567-e89b-12d3-a456-426614174000",
  "settings": {...},
  "status": "in_progress",
  "drone_statuses": {...},
  "start_time": "2023-06-01T15:01:00Z",
  "end_time": null,
  "current_time": 30.0,
  "progress": 25.0,
  "created_at": "2023-06-01T15:00:00Z",
  "updated_at": "2023-06-01T15:01:30Z"
}
```

#### Abort Execution

```
POST /execution/{execution_id}/abort
```

Abort a running or paused execution.

**Response:**

```json
{
  "id": "523e4567-e89b-12d3-a456-426614174000",
  "choreography_id": "123e4567-e89b-12d3-a456-426614174000",
  "settings": {...},
  "status": "aborted",
  "drone_statuses": {...},
  "start_time": "2023-06-01T15:01:00Z",
  "end_time": "2023-06-01T15:02:00Z",
  "current_time": 30.0,
  "progress": 25.0,
  "created_at": "2023-06-01T15:00:00Z",
  "updated_at": "2023-06-01T15:02:00Z"
}
```

#### Get Execution Logs

```
GET /execution/{execution_id}/logs
```

Get logs for a specific execution.

**Query Parameters:**

- `level` (optional): Filter by log level (e.g., "INFO", "WARNING", "ERROR")
- `skip` (optional): Number of records to skip (default: 0)
- `limit` (optional): Maximum number of records to return (default: 100)

**Response:**

```json
[
  {
    "id": "723e4567-e89b-12d3-a456-426614174000",
    "execution_id": "523e4567-e89b-12d3-a456-426614174000",
    "timestamp": "2023-06-01T15:01:00Z",
    "level": "INFO",
    "message": "Starting execution of choreography 123e4567-e89b-12d3-a456-426614174000",
    "data": null
  },
  {
    "id": "823e4567-e89b-12d3-a456-426614174000",
    "execution_id": "523e4567-e89b-12d3-a456-426614174000",
    "timestamp": "2023-06-01T15:01:05Z",
    "level": "INFO",
    "message": "Drone drone_1 connected",
    "data": {
      "drone_id": "drone_1",
      "battery_level": 95.0
    }
  }
]
```

#### Export Execution Logs

```
GET /execution/{execution_id}/logs/export
```

Export logs for a specific execution.

**Response:**

```json
{
  "url": "logs/523e4567-e89b-12d3-a456-426614174000/logs.json"
}
```

#### Execution WebSocket

```
WebSocket /execution/ws/{execution_id}
```

Stream execution data in real-time.

**Messages from Server:**

1. Initial data:

```json
{
  "type": "init",
  "data": {
    "id": "523e4567-e89b-12d3-a456-426614174000",
    "choreography_id": "123e4567-e89b-12d3-a456-426614174000",
    "settings": {...},
    "status": "in_progress",
    "drone_statuses": {...},
    "start_time": "2023-06-01T15:01:00Z",
    "end_time": null,
    "current_time": 30.0,
    "progress": 25.0,
    "created_at": "2023-06-01T15:00:00Z",
    "updated_at": "2023-06-01T15:01:30Z"
  }
}
```

2. Update data:

```json
{
  "type": "update",
  "data": {
    "id": "523e4567-e89b-12d3-a456-426614174000",
    "choreography_id": "123e4567-e89b-12d3-a456-426614174000",
    "settings": {...},
    "status": "in_progress",
    "drone_statuses": {...},
    "start_time": "2023-06-01T15:01:00Z",
    "end_time": null,
    "current_time": 31.0,
    "progress": 25.8,
    "created_at": "2023-06-01T15:00:00Z",
    "updated_at": "2023-06-01T15:01:31Z"
  }
}
```

3. End message:

```json
{
  "type": "end",
  "data": {
    "id": "523e4567-e89b-12d3-a456-426614174000",
    "status": "completed"
  }
}
```

## Error Responses

All API endpoints return appropriate HTTP status codes and error messages in case of failure:

```json
{
  "detail": "Error message"
}
```

Common error status codes:

- `400 Bad Request`: Invalid request parameters or body
- `401 Unauthorized`: Missing or invalid authentication
- `403 Forbidden`: Insufficient permissions
- `404 Not Found`: Resource not found
- `500 Internal Server Error`: Server error
