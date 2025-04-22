# Integration with Bulo.Cloud Sentinel

This document provides instructions for integrating the Drone Show microservice with the Bulo.Cloud Sentinel platform.

## Overview

The Drone Show microservice is designed to integrate seamlessly with the Bulo.Cloud Sentinel platform, leveraging its existing components for drone control, telemetry, and mission planning. This integration allows for the creation, simulation, and execution of drone light shows using the Bulo.Cloud Sentinel platform.

## Architecture

The Drone Show microservice integrates with the following components of the Bulo.Cloud Sentinel platform:

1. **API Gateway**: The Drone Show microservice is exposed through the Bulo.Cloud Sentinel API Gateway, which handles authentication, authorization, and routing.

2. **Device Inventory**: The Drone Show microservice uses the Device Inventory service to access and control drones.

3. **Mission Planning**: The Drone Show microservice leverages the Mission Planning service to create and execute missions.

4. **Telemetry**: The Drone Show microservice uses the Telemetry service to receive real-time telemetry data from drones.

5. **Storage**: The Drone Show microservice uses the Storage service (MinIO) to store choreography data, simulation results, and logs.

6. **Database**: The Drone Show microservice uses its own PostgreSQL database for storing choreography and execution data.

7. **Frontend**: The Drone Show microservice provides a React component that can be integrated into the Bulo.Cloud Sentinel frontend.

## Integration Points

### API Gateway Integration

The Drone Show microservice is exposed through the Bulo.Cloud Sentinel API Gateway at the following endpoint:

```
/api/drone-show
```

All API requests to the Drone Show microservice should be made through this endpoint. For example:

```
GET /api/drone-show/shows
```

The API Gateway handles authentication and authorization, ensuring that only authorized users can access the Drone Show microservice.

### Device Inventory Integration

The Drone Show microservice integrates with the Device Inventory service to access and control drones. This integration is handled through the `SentinelIntegrationService` class, which provides methods for sending commands to drones and receiving telemetry data.

The Device Inventory service must be configured to allow the Drone Show microservice to access and control drones. This can be done by adding the Drone Show microservice to the list of authorized services in the Device Inventory service configuration.

### Mission Planning Integration

The Drone Show microservice integrates with the Mission Planning service to create and execute missions. This integration is handled through the `SentinelIntegrationService` class, which provides methods for creating and executing missions.

The Mission Planning service must be configured to allow the Drone Show microservice to create and execute missions. This can be done by adding the Drone Show microservice to the list of authorized services in the Mission Planning service configuration.

### Telemetry Integration

The Drone Show microservice integrates with the Telemetry service to receive real-time telemetry data from drones. This integration is handled through the `SentinelIntegrationService` class, which provides methods for receiving telemetry data.

The Telemetry service must be configured to allow the Drone Show microservice to receive telemetry data. This can be done by adding the Drone Show microservice to the list of authorized services in the Telemetry service configuration.

### Storage Integration

The Drone Show microservice integrates with the Storage service (MinIO) to store choreography data, simulation results, and logs. This integration is handled through the `MinioService` class, which provides methods for storing and retrieving data from MinIO.

The Storage service must be configured to allow the Drone Show microservice to access and modify data. This can be done by creating a bucket for the Drone Show microservice and configuring the appropriate access policies.

### Database Integration

The Drone Show microservice uses its own PostgreSQL database for storing choreography and execution data. This database should be deployed alongside the Bulo.Cloud Sentinel platform and configured to be accessible by the Drone Show microservice.

### Frontend Integration

The Drone Show microservice provides a React component that can be integrated into the Bulo.Cloud Sentinel frontend. This component is available in the `frontend` directory of the Drone Show microservice and can be imported into the Bulo.Cloud Sentinel frontend.

## Configuration

### Environment Variables

The Drone Show microservice can be configured using the following environment variables:

- `SENTINEL_API_URL`: URL of the Bulo.Cloud Sentinel API
- `SENTINEL_API_TOKEN`: API token for authentication
- `DATABASE_URL`: PostgreSQL connection string
- `REDIS_URL`: Redis connection string
- `MINIO_URL`: MinIO connection string
- `MINIO_ACCESS_KEY`: MinIO access key
- `MINIO_SECRET_KEY`: MinIO secret key
- `MINIO_BUCKET`: MinIO bucket name
- `RTMP_SERVER`: RTMP server URL for video streaming

### Docker Compose

The Drone Show microservice can be deployed using Docker Compose. The `docker-compose.yml` file in the Drone Show microservice directory provides a template for deploying the microservice alongside the Bulo.Cloud Sentinel platform.

### Kubernetes

The Drone Show microservice can be deployed using Kubernetes. The `kubernetes` directory in the Drone Show microservice directory provides Kubernetes manifests for deploying the microservice in a Kubernetes cluster.

## Authentication and Authorization

The Drone Show microservice uses the Bulo.Cloud Sentinel authentication and authorization system. All API requests to the Drone Show microservice must include a valid JWT token in the `Authorization` header:

```
Authorization: Bearer <token>
```

The token is validated by the API Gateway, which also enforces authorization rules based on the user's role and permissions.

## API Documentation

The Drone Show microservice provides a comprehensive API for creating, simulating, and executing drone light shows. The API documentation is available in the `docs/api.md` file in the Drone Show microservice directory.

## Frontend Integration

The Drone Show microservice provides a React component that can be integrated into the Bulo.Cloud Sentinel frontend. This component is available in the `frontend` directory of the Drone Show microservice and can be imported into the Bulo.Cloud Sentinel frontend.

### Installation

To install the Drone Show frontend component, run the following command in the Bulo.Cloud Sentinel frontend directory:

```bash
npm install --save ../drone_show_service/frontend
```

### Usage

To use the Drone Show frontend component, import it into your React component:

```jsx
import { DroneShowApp } from 'drone-show-frontend';

function App() {
  return (
    <div className="App">
      <DroneShowApp />
    </div>
  );
}
```

The `DroneShowApp` component provides a complete user interface for creating, simulating, and executing drone light shows. It can be customized using the following props:

- `apiUrl`: URL of the Drone Show API (default: `/api/drone-show`)
- `token`: JWT token for authentication (default: read from localStorage)
- `theme`: Theme configuration (default: Bulo.Cloud Sentinel theme)
- `onError`: Error handler function (default: console.error)

## Troubleshooting

### Common Issues

- **Authentication Errors**: Ensure that the JWT token is valid and has the necessary permissions.
- **Connection Errors**: Ensure that the Drone Show microservice can connect to the Bulo.Cloud Sentinel API and other services.
- **Database Errors**: Ensure that the PostgreSQL database is accessible and properly configured.
- **Storage Errors**: Ensure that the MinIO service is accessible and properly configured.
- **Drone Control Errors**: Ensure that the Device Inventory service is properly configured to allow the Drone Show microservice to access and control drones.

### Logs

The Drone Show microservice logs all activities to the console and to the PostgreSQL database. Logs can be accessed through the API or by querying the database directly.

### Support

If you encounter issues with the Drone Show microservice, please contact the Bulo.Cloud Sentinel support team or refer to the following resources:

- [Drone Show Microservice Documentation](https://bulocloud-sentinel.example.com/docs/drone-show)
- [Bulo.Cloud Sentinel Documentation](https://bulocloud-sentinel.example.com/docs)
- [Bulo.Cloud Sentinel Support](https://bulocloud-sentinel.example.com/support)
