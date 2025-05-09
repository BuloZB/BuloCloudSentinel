# Federated Learning in Bulo.CloudSentinel

This document describes the Federated Learning (FL) architecture implemented in Bulo.CloudSentinel, which enables edge devices to train models locally while preserving data privacy.

## Architecture Overview

Bulo.CloudSentinel implements a federated learning system that allows edge devices to train models locally on their data and share only model updates (gradients) with a central server. This approach preserves data privacy while enabling collaborative model improvement across the fleet of devices.

![Federated Learning Architecture](../assets/images/federated_learning_architecture.png)

### Components

1. **Edge Trainer (Client)**
   - Container: `edge_flower_client`
   - Framework: Flower with PyTorch backend
   - Function: Trains lightweight model head on locally extracted embeddings
   - Data: Raw data never leaves the device

2. **Aggregator (Server)**
   - Service: `fl_server`
   - Location: Cloud deployment
   - Algorithm: Federated Averaging (FedAvg)
   - Integration: Saves aggregated models to Model Hub

3. **Model Hub**
   - Storage: Versioned model repository
   - Function: Distributes updated global models to edge devices

## Workflow

1. **Data Collection**
   - Edge devices collect data through their sensors
   - Feature extraction occurs every N frames
   - Embeddings are stored locally for training

2. **Local Training**
   - When a federated learning round begins, edge devices train on local data
   - Default configuration: 3 local epochs per round
   - Only model gradients/weights are prepared for sharing

3. **Secure Aggregation**
   - Clients send model updates to the server
   - Server aggregates updates using FedAvg algorithm
   - New global model is created and stored in Model Hub

4. **Model Distribution**
   - Updated global model is distributed back to edge devices
   - Devices update their local models with the new global weights

## Privacy and Security

### Differential Privacy

Differential Privacy is implemented using the Opacus library with the following parameters:
- Epsilon: ≤ 8 (privacy budget)
- Delta: 1e-5 (probability of privacy breach)
- Noise multiplier: Dynamically adjusted based on dataset size

This provides a mathematical guarantee that individual data points cannot be identified from the shared model updates.

### Secure Communication

- All communication uses TLS over MQTT
- Device identifiers are removed from training data and model updates
- Authentication is required for all clients connecting to the server

## Hardware Requirements

### Edge Devices

| Component | Minimum Requirement | Recommended |
|-----------|---------------------|-------------|
| CPU       | 4 cores, 1.5 GHz    | 8 cores, 2.0+ GHz |
| RAM       | 4 GB                | 8+ GB |
| GPU       | Optional            | NVIDIA Jetson or equivalent |
| Storage   | 32 GB               | 64+ GB |

### Server

| Component | Minimum Requirement | Recommended |
|-----------|---------------------|-------------|
| CPU       | 8 cores, 2.5 GHz    | 16+ cores, 3.0+ GHz |
| RAM       | 16 GB               | 32+ GB |
| GPU       | NVIDIA T4           | NVIDIA A100 or equivalent |
| Storage   | 500 GB SSD          | 1+ TB NVMe SSD |

## Network Requirements

- Bandwidth: < 50 MB upload per round per device
- Latency: Tolerant of high latency (asynchronous operation)
- Reliability: System handles intermittent connectivity

## Performance Expectations

Based on internal testing with a fleet of 10 devices:

| Metric                      | Value                |
|-----------------------------|----------------------|
| Model Convergence           | 30% faster than centralized training |
| Privacy Preservation        | No raw data leaves devices |
| Bandwidth Savings           | 95% reduction vs. raw data transfer |
| Edge Compute Utilization    | 20-30% CPU during training |
| Battery Impact              | ~5% additional consumption during training |

## Fail-Safe Mechanisms

- If a client goes offline during a round, it is skipped
- Server handles stragglers with configurable timeout
- Clients can rejoin training in subsequent rounds
- Local models are preserved if communication fails

## Integration Testing

The CI pipeline includes integration tests that:
- Spin up 3 fake clients with synthetic COCO subset
- Verify model convergence across multiple rounds
- Validate privacy guarantees
- Measure communication efficiency

## Configuration Parameters

The federated learning system can be configured through environment variables or config files:

```yaml
federated_learning:
  # Server configuration
  server:
    min_clients: 2
    max_clients: 100
    rounds: 50
    aggregation_method: "fedavg"
    timeout_seconds: 600
    
  # Client configuration
  client:
    local_epochs: 3
    batch_size: 32
    learning_rate: 0.01
    optimizer: "adam"
    
  # Privacy configuration
  privacy:
    differential_privacy: true
    epsilon: 8.0
    delta: 0.00001
    noise_multiplier: 1.0
    
  # Communication configuration
  communication:
    protocol: "mqtt"
    tls_enabled: true
    max_message_size_mb: 50
```

## Future Improvements

1. **Federated Learning Algorithms**
   - Implement FedProx for better convergence with heterogeneous clients
   - Add support for Federated Distillation to reduce communication costs

2. **Advanced Privacy**
   - Implement Secure Multi-party Computation (SMPC)
   - Add support for Homomorphic Encryption

3. **Client Selection**
   - Implement intelligent client selection based on data quality and device capabilities
   - Add support for asynchronous federated learning

4. **Model Personalization**
   - Enable personalized models for each client while maintaining global knowledge
   - Implement meta-learning approaches for faster adaptation
