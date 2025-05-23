# Remote ID & Regulatory Compliance Module

## Overview

The Remote ID & Regulatory Compliance module for Bulo.Cloud Sentinel provides comprehensive support for drone Remote ID broadcasting, flight plan submission, and NOTAM integration. This module ensures compliance with international aviation standards for unmanned aircraft systems (UAS), including ASTM F3411-22a for Remote ID.

## Features

### Remote ID Broadcasting

- **ASTM F3411-22a Compliance**: Fully compliant with the ASTM F3411-22a standard for Remote ID
- **Multiple Broadcast Methods**: Support for Wi-Fi Neighbor Awareness Networking (NAN) and Bluetooth Low Energy 5.0
- **Regulatory Modes**: Support for both EU and FAA regulatory environments
- **Telemetry Conversion**: Transforms MAVLink telemetry data into standardized Remote ID message format
- **Broadcast Logging**: Maintains broadcast records for a minimum of 24 hours to meet regulatory requirements

### Flight Plan Management

- **EASA SORA Integration**: Support for European SORA (Specific Operations Risk Assessment) framework
- **FAA LAANC Integration**: Support for US LAANC (Low Altitude Authorization and Notification Capability) system
- **Flight Plan Submission**: Automated submission of flight plans to regulatory authorities
- **Status Tracking**: Track the status of submitted flight plans (approved, rejected, etc.)
- **Waypoint Management**: Define and manage waypoints for flight plans

### NOTAM Integration

- **NOTAM Import**: Import NOTAMs (Notice to Air Missions) from various sources
- **Spatial Analysis**: Check flight plans against NOTAMs for conflicts
- **AIXM 5.1 Support**: Support for AIXM 5.1 format for NOTAM data
- **Visualization**: Visualize NOTAMs on maps for operational awareness

## Architecture

The Remote ID & Regulatory Compliance module is designed as a microservice that integrates with the Bulo.Cloud Sentinel platform. It consists of the following components:

### Core Components

- **API Layer**: FastAPI-based REST API for interacting with the service
- **Database Layer**: PostgreSQL with PostGIS for spatial data storage
- **Security Layer**: Authentication and authorization for secure access

### Remote ID Components

- **Broadcasting Service**: Handles the conversion and transmission of Remote ID messages
- **Message Converter**: Converts between different Remote ID message formats
- **Broadcasters**: Implements Wi-Fi NAN and Bluetooth LE broadcasting
- **Logging System**: Stores and manages broadcast logs

### Flight Plan Components

- **Flight Plan Service**: Manages flight plans and submissions
- **EASA SORA Adapter**: Interfaces with the EASA SORA API
- **FAA LAANC Adapter**: Interfaces with the FAA LAANC API
- **Waypoint Manager**: Manages waypoints for flight plans

### NOTAM Components

- **NOTAM Service**: Manages NOTAMs and checks flight plans against them
- **NOTAM AIXM Adapter**: Parses and processes AIXM 5.1 format NOTAMs
- **Spatial Analysis**: Performs spatial analysis for NOTAM conflicts

## API Reference

The Remote ID & Regulatory Compliance module provides a comprehensive REST API for interacting with the service. The API is documented using OpenAPI and is available at `/docs` when the service is running.

### Remote ID Endpoints

- `POST /api/v1/remoteid/broadcast/start`: Start Remote ID broadcasting
- `POST /api/v1/remoteid/broadcast/stop`: Stop Remote ID broadcasting
- `POST /api/v1/remoteid/broadcast/update`: Update Remote ID broadcast data
- `GET /api/v1/remoteid/broadcast/status/{drone_id}`: Get Remote ID broadcast status
- `POST /api/v1/remoteid/logs`: Get Remote ID broadcast logs

### Flight Plan Endpoints

- `POST /api/v1/flightplans`: Create a flight plan
- `GET /api/v1/flightplans/{flight_plan_id}`: Get a flight plan
- `PUT /api/v1/flightplans/{flight_plan_id}`: Update a flight plan
- `DELETE /api/v1/flightplans/{flight_plan_id}`: Delete a flight plan
- `POST /api/v1/flightplans/search`: Search for flight plans
- `POST /api/v1/flightplans/submit`: Submit a flight plan
- `POST /api/v1/flightplans/cancel`: Cancel a flight plan

### NOTAM Endpoints

- `POST /api/v1/notams/import`: Import NOTAMs
- `GET /api/v1/notams/{notam_id}`: Get a NOTAM
- `POST /api/v1/notams/search`: Search for NOTAMs
- `POST /api/v1/notams/check-flight-plan`: Check a flight plan for NOTAM conflicts
- `GET /api/v1/notams/sources`: Get available NOTAM sources
- `GET /api/v1/notams/regions/{source}`: Get available NOTAM regions for a source

## Command-Line Interface

The Remote ID & Regulatory Compliance module provides a command-line interface (CLI) for interacting with the service. The CLI is available as a Python module and can be used to perform various operations.

### Remote ID CLI

```bash
# Start Remote ID broadcasting
python -m remoteid_service.cli.remoteid start --drone-id DRN123 --mode faa

# Stop Remote ID broadcasting
python -m remoteid_service.cli.remoteid stop --drone-id DRN123

# Update Remote ID broadcast data
python -m remoteid_service.cli.remoteid update --drone-id DRN123 --latitude 37.7749 --longitude -122.4194 --altitude 100

# Get Remote ID broadcast status
python -m remoteid_service.cli.remoteid status --drone-id DRN123

# Get Remote ID broadcast logs
python -m remoteid_service.cli.remoteid logs --drone-id DRN123
```

### Flight Plan CLI

```bash
# Create a flight plan
python -m remoteid_service.cli.flightplan create --file flight_plan.yaml

# Get a flight plan
python -m remoteid_service.cli.flightplan get --id 123e4567-e89b-12d3-a456-426614174000

# Submit a flight plan
python -m remoteid_service.cli.flightplan submit --id 123e4567-e89b-12d3-a456-426614174000 --type faa_laanc
```

### NOTAM CLI

```bash
# Import NOTAMs
python -m remoteid_service.cli.notam import --source faa --region us-east

# Search for NOTAMs
python -m remoteid_service.cli.notam search --source faa --type airspace

# Check a flight plan for NOTAM conflicts
python -m remoteid_service.cli.notam check --flight-plan-id 123e4567-e89b-12d3-a456-426614174000
```

## Deployment

The Remote ID & Regulatory Compliance module can be deployed using Docker and Docker Compose. The module includes a `docker-compose.yml` file that defines the services required for the module to run.

```bash
# Build and start the services
cd remoteid_service
docker-compose up -d

# Stop the services
docker-compose down
```

For Kubernetes deployment, the module includes Kubernetes manifests and Helm charts in the `kubernetes` directory.

```bash
# Deploy using Helm
cd remoteid_service/kubernetes/helm
helm install remoteid .
```

## Security

The Remote ID & Regulatory Compliance module implements several security measures to ensure the integrity and confidentiality of the data:

- **Authentication**: JWT-based authentication for API access
- **Authorization**: Role-based access control for API endpoints
- **Encryption**: TLS encryption for all API communications
- **Mutual TLS**: Optional mutual TLS (mTLS) authentication for drone-to-service communication
- **Input Validation**: Strict validation of all API inputs
- **Audit Logging**: Comprehensive logging of all operations

## Compliance

The Remote ID & Regulatory Compliance module is designed to comply with the following standards and regulations:

- **ASTM F3411-22a**: Standard Specification for Remote ID and Tracking
- **EU Regulation 2019/947**: Rules and procedures for the operation of unmanned aircraft
- **FAA Remote ID Rule**: Remote Identification of Unmanned Aircraft
- **EASA SORA**: Specific Operations Risk Assessment
- **FAA LAANC**: Low Altitude Authorization and Notification Capability

## Future Enhancements

- **Network Remote ID**: Support for Network Remote ID in addition to Broadcast Remote ID
- **Remote ID Display**: Web-based Remote ID display for visualization
- **Additional Regulatory Frameworks**: Support for additional regulatory frameworks
- **Machine Learning**: Integration of machine learning for flight risk assessment
- **Real-time Weather**: Integration of real-time weather data for flight planning
