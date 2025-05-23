# Remote ID & Regulatory Compliance Module

## Overview

The Remote ID & Regulatory Compliance module for Bulo.Cloud Sentinel provides comprehensive support for drone Remote ID broadcasting, flight plan submission, and NOTAM integration. This module ensures compliance with international aviation standards for unmanned aircraft systems (UAS), including ASTM F3411-22a for Remote ID.

## Features

- **Remote ID Broadcasting**: Compliant with ASTM F3411-22a standards for both EU and FAA regulatory environments
- **Broadcast Methods**: Support for Wi-Fi Neighbor Awareness Networking (NAN) and Bluetooth Low Energy 5.0
- **Telemetry Conversion**: Transforms MAVLink telemetry data into standardized Remote ID message format
- **Broadcast Logging**: Maintains broadcast records for a minimum of 24 hours to meet regulatory requirements
- **Flight Plan Submission**: Automated submission capabilities supporting both EASA SORA and FAA LAANC frameworks
- **NOTAM Integration**: Import and visualization of Notice to Air Missions (NOTAM) data for operational safety
- **Secure Communication**: Mutual TLS (mTLS) authentication with SPIFFE IDs for secure communication

## Architecture

The Remote ID service is designed as a microservice that integrates with the Bulo.Cloud Sentinel platform. It consists of the following components:

- **API Layer**: FastAPI-based REST API for interacting with the service
- **Broadcasting Service**: Handles the conversion and transmission of Remote ID messages
- **Logging System**: Stores and manages broadcast logs in PostgreSQL time-series tables
- **Flight Plan Adapters**: Interfaces with regulatory APIs for flight plan submission
- **NOTAM Service**: Imports and processes NOTAM data for visualization and alerts

## Getting Started

### Prerequisites

- Python 3.10+
- PostgreSQL 14+
- Docker and Docker Compose
- Bluetooth 5.0 compatible hardware (for BLE broadcasting)
- Wi-Fi adapter supporting 802.11mc/NAN (for Wi-Fi NAN broadcasting)

### Installation

1. Clone the repository:

```bash
git clone https://github.com/BuloZB/BuloCloudSentinel.git
cd BuloCloudSentinel
```

2. Install dependencies:

```bash
cd remoteid_service
pip install -r requirements.txt
```

3. Configure the service:

Edit the `config/config.yaml` file to match your environment.

4. Run the service:

```bash
# Using Python
python main.py

# Using Docker
docker-compose up -d
```

## Usage

### Remote ID Broadcasting

To start broadcasting Remote ID:

```bash
# Using the CLI
python -m remoteid_service.cli.remoteid start --drone-id DRN123 --mode faa

# Using the API
curl -X POST "http://localhost:8080/api/v1/remoteid/broadcast" \
  -H "Content-Type: application/json" \
  -d '{"drone_id": "DRN123", "mode": "faa"}'
```

### Flight Plan Submission

To submit a flight plan:

```bash
# Using the CLI
python -m remoteid_service.cli.flightplan submit --file flight_plan.yaml

# Using the API
curl -X POST "http://localhost:8080/api/v1/flightplans" \
  -H "Content-Type: application/json" \
  -d @flight_plan.json
```

### NOTAM Integration

To import NOTAM data:

```bash
# Using the CLI
python -m remoteid_service.cli.notam import --source faa --region us-east

# Using the API
curl -X POST "http://localhost:8080/api/v1/notams/import" \
  -H "Content-Type: application/json" \
  -d '{"source": "faa", "region": "us-east"}'
```

## API Documentation

API documentation is available at `http://localhost:8080/docs` when the service is running.

## License

This project is licensed under the MIT License - see the LICENSE file for details.
