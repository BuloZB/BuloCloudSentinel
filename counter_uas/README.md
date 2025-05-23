# Counter-UAS / Intrusion Detection Module

## Overview

The Counter-UAS / Intrusion Detection module provides comprehensive capabilities for detecting, tracking, and reporting unauthorized drone activity in protected airspace. It combines RF direction finding using KerberosSDR with radar-based detection using Acconeer radar sensors to provide accurate and reliable detection of drones and other aerial intruders.

## Features

- **KerberosSDR Integration**: Direction finding using 4-channel coherent RTL-SDR array
- **Acconeer Radar Integration**: Range and velocity measurement using A111 60GHz radar sensors
- **Direction Finding**: GNU Radio-based Direction of Arrival (DoA) processing using MUSIC algorithm
- **Radar Processing**: Range-Doppler processing for target velocity and distance estimation
- **Sensor Fusion**: Extended Kalman Filter (EKF) for multi-sensor data fusion
- **Event Management**: RabbitMQ integration for real-time event distribution
- **Alert System**: Multi-level alert system with email, SMS, and in-app notifications
- **User Interface**: Heat-map visualization and real-time track list

## Installation

### Prerequisites

- Python 3.8 or newer
- GNU Radio 3.8 or newer
- RTL-SDR driver
- Acconeer Python SDK
- RabbitMQ server

### Installation Steps

1. Install dependencies:

```bash
# Install system dependencies
sudo apt-get install gnuradio gnuradio-dev librtlsdr-dev rabbitmq-server

# Install Python dependencies
pip install numpy scipy matplotlib pika fastapi uvicorn acconeer-python-sdk
```

2. Install gr-kerberos:

```bash
git clone https://github.com/rfjohnso/gr-kerberos
cd gr-kerberos
mkdir build
cd build
cmake ..
make
sudo make install
sudo ldconfig
```

3. Clone the repository:

```bash
git clone https://github.com/BuloZB/BuloCloudSentinel.git
cd BuloCloudSentinel
```

4. Install the Counter-UAS module:

```bash
cd counter_uas
pip install -e .
```

## Usage

### Configuration

Edit the configuration file at `counter_uas/config/counter_uas.yaml` to match your hardware setup and requirements.

### Running the Module

```bash
# Run with default configuration
python -m counter_uas.main

# Run with custom configuration
python -m counter_uas.main --config path/to/config.yaml
```

### API

The Counter-UAS module provides a RESTful API for configuration and monitoring. The API is available at `http://localhost:8000` by default.

#### Endpoints

- `GET /api/status`: Get the status of the Counter-UAS module
- `GET /api/devices`: Get the status of all hardware devices
- `GET /api/devices/{device_id}`: Get the status of a specific hardware device
- `POST /api/devices/{device_id}/configure`: Configure a hardware device
- `GET /api/detections`: Get all current detections
- `GET /api/alerts`: Get all current alerts
- `POST /api/alerts/{alert_id}/acknowledge`: Acknowledge an alert

## Development

### Project Structure

```
counter_uas/
├── api/                  # API endpoints
├── hardware/             # Hardware interfaces and implementations
├── processing/           # Signal processing components
├── services/             # Business logic services
├── models/               # Data models
├── utils/                # Utility functions
├── config/               # Configuration files
├── tests/                # Test files
├── docs/                 # Documentation
├── main.py               # Application entry point
└── __init__.py           # Package initialization
```

### Running Tests

```bash
# Run all tests
pytest

# Run unit tests
pytest counter_uas/tests/unit

# Run integration tests
pytest counter_uas/tests/integration

# Run performance tests
pytest counter_uas/tests/performance
```

## Documentation

Comprehensive documentation is available in the `docs` directory:

- [System Architecture](docs/counter_uas.md): Overview of the system architecture and data flow
- [Hardware Setup](docs/hardware_setup.md): Instructions for setting up and configuring hardware
- [API Documentation](docs/api.md): Detailed API documentation

## License

This project is licensed under the MIT License - see the LICENSE file for details.
