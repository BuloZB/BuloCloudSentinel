# Dock Driver API Documentation

This document provides detailed information about the Dock Driver API endpoints.

## Authentication

The Dock Driver API uses JWT tokens for authentication. To obtain a token, send a POST request to the `/token` endpoint with your username and password.

```
POST /token
```

**Request Body:**

```json
{
  "username": "your-username",
  "password": "your-password"
}
```

**Response:**

```json
{
  "access_token": "your-jwt-token",
  "token_type": "bearer",
  "expires_in": 3600
}
```

Once you have a token, include it in the `Authorization` header of your requests:

```
Authorization: Bearer your-jwt-token
```

## Endpoints

### Health Check

```
GET /health
```

Check the health of the application.

**Response:**

```json
{
  "status": "ok"
}
```

### Get All Docks

```
GET /api/docks
```

Get a list of all configured docks.

**Response:**

```json
{
  "docks": [
    {
      "dock_id": "dji-dock-1",
      "dock_type": "dji",
      "name": "DJI Dock 1",
      "location": {
        "latitude": 37.7749,
        "longitude": -122.4194
      },
      "status": "online",
      "charging_status": "idle",
      "last_updated": "2023-01-01T00:00:00"
    },
    {
      "dock_id": "heisha-dock-1",
      "dock_type": "heisha",
      "name": "Heisha Dock 1",
      "location": {
        "latitude": 37.7749,
        "longitude": -122.4194
      },
      "status": "online",
      "charging_status": "idle",
      "last_updated": "2023-01-01T00:00:00"
    }
  ]
}
```

### Get Dock Status

```
GET /api/docks/{dock_type}/{dock_id}/status
```

Get the current status of a specific dock.

**Parameters:**

- `dock_type`: Type of the dock (dji, heisha, esp32)
- `dock_id`: ID of the dock

**Response:**

```json
{
  "dock_id": "dji-dock-1",
  "status": "online",
  "charging_status": "idle",
  "door_state": "closed",
  "drone_connected": false,
  "error_code": 0,
  "error_message": "",
  "timestamp": "2023-01-01T00:00:00"
}
```

### Get Dock Telemetry

```
GET /api/docks/{dock_type}/{dock_id}/telemetry
```

Get telemetry data from a specific dock.

**Parameters:**

- `dock_type`: Type of the dock (dji, heisha, esp32)
- `dock_id`: ID of the dock

**Response:**

```json
{
  "dock_id": "dji-dock-1",
  "temperature": 25.0,
  "humidity": 50.0,
  "charging_voltage": 12.0,
  "charging_current": 2.0,
  "battery_level": 80,
  "network_signal": 90,
  "timestamp": "2023-01-01T00:00:00"
}
```

### Open Dock

```
POST /api/docks/{dock_type}/{dock_id}/open
```

Open a specific dock.

**Parameters:**

- `dock_type`: Type of the dock (dji, heisha, esp32)
- `dock_id`: ID of the dock

**Response:**

```json
{
  "message": "Dock dji-dock-1 opened successfully"
}
```

### Close Dock

```
POST /api/docks/{dock_type}/{dock_id}/close
```

Close a specific dock.

**Parameters:**

- `dock_type`: Type of the dock (dji, heisha, esp32)
- `dock_id`: ID of the dock

**Response:**

```json
{
  "message": "Dock dji-dock-1 closed successfully"
}
```

### Start Charging

```
POST /api/docks/{dock_type}/{dock_id}/charge/start
```

Start charging a drone at a specific dock.

**Parameters:**

- `dock_type`: Type of the dock (dji, heisha, esp32)
- `dock_id`: ID of the dock

**Response:**

```json
{
  "message": "Charging started at dock dji-dock-1"
}
```

### Stop Charging

```
POST /api/docks/{dock_type}/{dock_id}/charge/stop
```

Stop charging a drone at a specific dock.

**Parameters:**

- `dock_type`: Type of the dock (dji, heisha, esp32)
- `dock_id`: ID of the dock

**Response:**

```json
{
  "message": "Charging stopped at dock dji-dock-1"
}
```

### Set Fan Speed (Heisha only)

```
POST /api/docks/heisha/{dock_id}/fan/{speed}
```

Set the fan speed for a Heisha Charging Pad.

**Parameters:**

- `dock_id`: ID of the dock
- `speed`: Fan speed (0-100%)

**Response:**

```json
{
  "message": "Fan speed set to 50% for dock heisha-dock-1"
}
```

### Set Heater State (Heisha only)

```
POST /api/docks/heisha/{dock_id}/heater/{state}
```

Set the heater state for a Heisha Charging Pad.

**Parameters:**

- `dock_id`: ID of the dock
- `state`: Heater state (on/off)

**Response:**

```json
{
  "message": "Heater on for dock heisha-dock-1"
}
```

### Set Relay State (ESP32 only)

```
POST /api/docks/esp32/{dock_id}/relay/{state}
```

Set the relay state for an ESP32-powered dock.

**Parameters:**

- `dock_id`: ID of the dock
- `state`: Relay state (on/off)

**Response:**

```json
{
  "message": "Relay on for dock esp32-dock-1"
}
```

## Error Responses

All endpoints return a standard error response format:

```json
{
  "detail": "Error message"
}
```

Common error status codes:

- `400 Bad Request`: Invalid request parameters
- `401 Unauthorized`: Missing or invalid authentication
- `404 Not Found`: Dock not found
- `500 Internal Server Error`: Server error
