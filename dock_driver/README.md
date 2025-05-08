# Dock Driver Microservice

This microservice provides integration with various drone docking stations, allowing for automated charging, protection from weather, and extended operational capabilities.

## Features

- **Multi-Vendor Support**: Integrates with DJI Dock 2, Heisha Charging Pad, and DIY ESP32-powered docks
- **Unified API**: Provides a consistent API regardless of the underlying hardware
- **Automated Charging**: Enables automated charging when battery levels are low
- **Telemetry**: Collects and reports telemetry data from docking stations
- **Security**: Implements robust security measures for safe operation
- **Power Management Integration**: Seamlessly integrates with the Power Management module

## Architecture

The Dock Driver microservice follows the adapter pattern to provide a unified interface for different docking station types:

```
├── api/                 # FastAPI endpoints
├── adapters/            # Adapters for different dock types
│   ├── dji/             # DJI Dock 2 adapter
│   ├── heisha/          # Heisha Charging Pad adapter
│   ├── esp32/           # DIY ESP32-powered dock adapter
│   └── interface.py     # Common adapter interface
├── models/              # Data models
├── services/            # Business logic
├── utils/               # Utility functions
├── config.py            # Configuration handling
└── main.py              # Application entry point
```

## Installation

### Using Docker

```bash
# Clone the repository
git clone https://github.com/BuloZB/BuloCloudSentinel.git
cd BuloCloudSentinel/dock_driver

# Build and run the Docker container
docker-compose up -d
```

### Manual Installation

```bash
# Clone the repository
git clone https://github.com/BuloZB/BuloCloudSentinel.git
cd BuloCloudSentinel/dock_driver

# Create a virtual environment
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install dependencies
pip install -r requirements.txt

# Run the application
uvicorn main:app --host 0.0.0.0 --port 8060
```

## Configuration

The microservice is configured using a YAML file located at `config/config.yaml`. A sample configuration file is provided below:

```yaml
server:
  host: "0.0.0.0"
  port: 8060
  debug: false

security:
  jwt_secret: "your-jwt-secret"
  jwt_algorithm: "HS256"
  jwt_expiration: 3600  # Seconds

dji_dock:
  enabled: true
  api_key: "your-dji-cloud-api-key"
  api_secret: "your-dji-cloud-api-secret"
  dock_sn: "dock-serial-number"
  region: "us-east-1"  # Available regions: us-east-1, eu-central-1, ap-southeast-1
  refresh_interval: 30  # Seconds

heisha_dock:
  enabled: true
  rest_api_url: "http://heisha-dock-ip:8080/api"
  modbus_host: "heisha-dock-ip"
  modbus_port: 502
  modbus_unit_id: 1
  refresh_interval: 15  # Seconds
  username: "admin"  # Optional, for REST API authentication
  password: "password"  # Optional, for REST API authentication

esp32_dock:
  enabled: true
  mqtt_broker: "mqtt-broker-ip"
  mqtt_port: 1883
  mqtt_username: "username"  # Optional
  mqtt_password: "password"  # Optional
  mqtt_topic_prefix: "esp32_dock"
  refresh_interval: 10  # Seconds

power_management:
  integration_enabled: true
  api_url: "http://bulocloud-sentinel-api:8000/api/power-management"
  auto_charge_threshold: 30  # Percentage
  charge_complete_threshold: 90  # Percentage

logging:
  level: "INFO"  # DEBUG, INFO, WARNING, ERROR, CRITICAL
  file: "/var/log/dock_driver.log"
```

## API Reference

### Dock Status

```
GET /api/docks/{dock_type}/{dock_id}/status
```

Returns the current status of the specified docking station.

### Dock Control

```
POST /api/docks/{dock_type}/{dock_id}/open
POST /api/docks/{dock_type}/{dock_id}/close
```

Opens or closes the specified docking station.

### Charging Control

```
POST /api/docks/{dock_type}/{dock_id}/charge/start
POST /api/docks/{dock_type}/{dock_id}/charge/stop
```

Starts or stops charging for the specified docking station.

### Telemetry

```
GET /api/docks/{dock_type}/{dock_id}/telemetry
```

Returns telemetry data from the specified docking station.

## Integration with Power Management

The Dock Driver microservice integrates with the Power Management module to enable automated charging when the battery level is low. The integration works as follows:

1. The Power Management module monitors the battery level of the drone.
2. When the battery level drops below the `auto_charge_threshold`, the Power Management module triggers a return-to-home (RTH) operation.
3. Once the drone is near the docking station, the Dock Driver module takes over and guides the drone to the docking station.
4. The docking station charges the drone until the battery level reaches the `charge_complete_threshold`.
5. Once charging is complete, the drone can resume its mission or remain docked until needed.

## Security

The Dock Driver microservice implements several security measures:

- **JWT Authentication**: All API endpoints require a valid JWT token.
- **TLS/SSL Encryption**: All communication is encrypted using TLS/SSL.
- **Input Validation**: All input parameters are validated to prevent injection attacks.
- **Rate Limiting**: API endpoints are rate-limited to prevent abuse.
- **Audit Logging**: All actions are logged for audit purposes.

## License

This project is licensed under the MIT License - see the LICENSE file for details.
