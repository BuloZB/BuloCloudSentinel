# SATCOM / 5G Fallback Connectivity Module

This module provides fallback connectivity for Bulo.Cloud Sentinel drones, ensuring continuous communication even in challenging environments. It supports multiple communication methods (WiFi Mesh, 5G, and Satellite) and intelligently switches between them based on connection quality.

## Architecture Overview

The module consists of several key components:

1. **CommsAdapter Interface**: Abstract interface for different communication methods
   - `WifiMeshAdapter`: Primary mesh network communications
   - `FiveGAdapter`: 5G Standalone fallback in urban/suburban areas
   - `IridiumCertusAdapter`: Satellite fallback in remote areas

2. **ConnectionMonitor**: Continuously monitors connection quality metrics
   - Tracks RSSI, latency, packet loss, and jitter
   - Implements hysteresis algorithm to prevent rapid switching
   - Manages WireGuard tunnel transitions
   - Logs all connection events

3. **FallbackManager**: Orchestrates transitions between communication methods
   - Ensures session persistence during transitions
   - Handles buffering of critical messages during switchover

## Configuration

The module uses YAML-based configuration with support for:
- Threshold definitions for switching between adapters
- Priority ordering of communication methods
- Geographic-based rules (e.g., prefer 5G in urban areas)
- Mission-specific overrides

Example configuration:

```yaml
adapters:
  wifi_mesh:
    enabled: true
    priority: 1
    min_rssi: -75
    max_latency: 100
    max_packet_loss: 5.0
    max_jitter: 20.0
  
  five_g:
    enabled: true
    priority: 2
    min_rssi: -90
    max_latency: 150
    max_packet_loss: 10.0
    max_jitter: 30.0
    apn: "internet"
    
  iridium:
    enabled: true
    priority: 3
    min_rssi: -100
    max_latency: 1000
    max_packet_loss: 15.0
    max_jitter: 100.0

monitor:
  check_interval: 1.0  # seconds
  hysteresis_time: 5.0  # seconds
  hysteresis_count: 5   # number of consecutive checks
  
fallback:
  buffer_size: 1000     # number of messages
  critical_timeout: 30  # seconds
  session_timeout: 300  # seconds

wireguard:
  enabled: true
  interface: "wg0"
  port: 51820
  peers:
    - name: "ground_station"
      public_key: "public_key_here"
      endpoint: "endpoint:port"
      allowed_ips: "0.0.0.0/0"
```

## State Management

Connection state and transition history are stored in Redis:
- Current active adapter
- Connection quality metrics history
- Transition timestamps and reasons
- Failed connection attempts

## Integration

The module integrates with the existing Bulo.Cloud Sentinel components:
- Mesh networking system
- Telemetry service
- Mission planning
- Logging and monitoring

## Usage

```python
from comms_fallback import FallbackManager

# Initialize the fallback manager
manager = FallbackManager(config_path="config/comms_fallback.yaml")

# Start the manager
await manager.start()

# Send a message with fallback support
await manager.send_message(message, critical=True)

# Stop the manager
await manager.stop()
```

## Testing

The module includes comprehensive tests:
- Unit tests for each adapter implementation
- Integration tests for the entire fallback system
- Network condition simulation using Linux Traffic Control (`tc`)

## Performance Requirements

- Switchover time: < 3 seconds
- Zero message loss for critical messages
- Graceful degradation of non-critical services
