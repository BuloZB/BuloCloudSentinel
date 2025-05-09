# Federated Learning for Bulo.CloudSentinel

This module implements federated learning capabilities for Bulo.CloudSentinel, allowing edge devices to train models locally while preserving data privacy.

## Overview

The federated learning system consists of the following components:

1. **Edge Trainer Client**: A container that runs on edge devices and trains models locally on device data.
2. **Federated Learning Server**: A service that aggregates model updates from clients and creates a global model.
3. **MQTT Broker**: A secure communication channel for model updates.
4. **Model Hub Integration**: Stores and manages the aggregated models.

## Architecture

![Federated Learning Architecture](../assets/images/federated_learning_architecture.png)

The system follows a client-server architecture where:

- Edge devices collect data and train models locally
- Only model gradients are shared with the server (not raw data)
- The server aggregates updates using Federated Averaging (FedAvg)
- Updated global models are distributed back to edge devices

## Features

- **Privacy Preservation**: Raw data never leaves the device
- **Differential Privacy**: Implemented using Opacus with configurable privacy budget
- **Secure Communication**: TLS over MQTT with client certificates
- **Fail-Safe Mechanisms**: Handles client disconnections and stragglers
- **Model Hub Integration**: Versioned storage of aggregated models

## Getting Started

### Prerequisites

- Docker and Docker Compose
- Python 3.10 or later
- PyTorch 1.13 or later
- NVIDIA GPU (optional, but recommended for server)

### Setup

1. Generate certificates for secure communication:

```bash
chmod +x generate_certs.sh
./generate_certs.sh
```

2. Create MQTT password file:

```bash
mkdir -p mqtt/config
echo "fluser:$(openssl passwd -apr1 your_password)" > mqtt/config/password.txt
```

3. Start the services:

```bash
docker-compose up -d
```

### Configuration

The system can be configured through environment variables or by editing the configuration files:

- `config/server.yaml`: Server configuration
- `config/client.yaml`: Client configuration
- `mqtt/config/mosquitto.conf`: MQTT broker configuration

## Usage

### Starting the Server

The server can be started using the API:

```bash
curl -X POST http://localhost:8000/start
```

### Monitoring

Monitor the federated learning process:

```bash
curl http://localhost:8000/status
```

### Client Registration

Clients automatically register with the server when they start. The server will wait for the minimum number of clients before starting the training process.

## Integration Testing

The system includes an integration test that simulates multiple clients with synthetic data:

```bash
docker-compose up integration-test
```

## Security Considerations

- All communication is encrypted using TLS
- Client authentication is required
- Differential privacy is applied to model updates
- Device identifiers are removed from training data

## Performance Expectations

- **Bandwidth Usage**: < 50 MB upload per round per device
- **Training Time**: Depends on model complexity and device capabilities
- **Privacy Guarantee**: Epsilon ≤ 8 (configurable)

## Troubleshooting

### Common Issues

1. **Connection Refused**: Check that the MQTT broker is running and accessible
2. **Certificate Errors**: Ensure certificates are properly generated and accessible
3. **Training Failures**: Check client logs for errors

### Logs

- Server logs: `docker logs fl_server`
- MQTT broker logs: `docker logs mqtt_broker`
- Client logs: Check the logs on the edge device

## License

This module is part of Bulo.CloudSentinel and is licensed under the same terms.

## Further Reading

For more detailed information, see the [Federated Learning Documentation](../docs/federated_learning.md).
