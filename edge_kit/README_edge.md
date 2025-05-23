# Bulo.CloudSentinel Edge Kit Technical Documentation

## Overview

The Edge Kit is a plug-and-play Docker bundle for deploying Bulo.CloudSentinel capabilities at the edge, specifically optimized for Jetson Orin Nano and Raspberry Pi 5 devices. It enables low-latency inference, optimized streaming, and secure over-the-air updates.

## Hardware Bill of Materials (BOM)

### Jetson Orin Nano Configuration

| Component | Specification | Estimated Cost |
|-----------|---------------|----------------|
| NVIDIA Jetson Orin Nano Developer Kit | 8GB RAM, 1024 CUDA cores | $499 |
| microSD Card | 64GB UHS-I Class 10 | $15 |
| Power Supply | 5V 4A (20W) | $25 |
| Cooling Fan | 40mm PWM Fan | $10 |
| Ethernet Cable | CAT6 1m | $5 |
| Optional: Wi-Fi Antenna | 2.4/5GHz dual-band | $10 |
| Optional: Camera | MIPI CSI-2 compatible | $50-150 |
| **Total** | | **$554-704** |

### Raspberry Pi 5 Configuration

| Component | Specification | Estimated Cost |
|-----------|---------------|----------------|
| Raspberry Pi 5 | 8GB RAM | $80 |
| microSD Card | 64GB UHS-I Class 10 | $15 |
| Power Supply | 5V 5A (27W) USB-C | $15 |
| Active Cooler | Official Raspberry Pi Active Cooler | $5 |
| Ethernet Cable | CAT6 1m | $5 |
| Optional: Camera | Raspberry Pi Camera Module 3 | $25-65 |
| **Total** | | **$120-185** |

## Power Specifications

### Jetson Orin Nano

- **Input Voltage**: 5V DC
- **Input Current**: 4A max
- **Power Consumption**:
  - Idle: ~5W
  - Average Load: ~10W
  - Peak Load: ~15W
- **Power Efficiency**: ~75%
- **Thermal Design Power (TDP)**: 15W
- **Operating Temperature**: 0°C to 70°C

### Raspberry Pi 5

- **Input Voltage**: 5V DC
- **Input Current**: 5A max
- **Power Consumption**:
  - Idle: ~3W
  - Average Load: ~6W
  - Peak Load: ~12W
- **Power Efficiency**: ~70%
- **Thermal Design Power (TDP)**: 12W
- **Operating Temperature**: 0°C to 70°C

## Performance Benchmarks

### Inference Performance

| Device | Model | Resolution | Precision | FPS | Latency (ms) |
|--------|-------|------------|-----------|-----|--------------|
| Jetson Orin Nano | YOLOv10n | 640x640 | FP16 | 45 | 22 |
| Jetson Orin Nano | YOLOv10n | 640x640 | INT8 | 60 | 17 |
| Jetson Orin Nano | YOLOv10n | 1280x720 | INT8 | 30 | 33 |
| Jetson Orin Nano | YOLOv10n | 1920x1080 | INT8 | 15 | 67 |
| Raspberry Pi 5 | YOLOv10n | 640x640 | FP16 | 12 | 83 |
| Raspberry Pi 5 | YOLOv10n | 640x640 | INT8 | 20 | 50 |
| Raspberry Pi 5 | YOLOv10n | 1280x720 | INT8 | 10 | 100 |

### Streaming Performance

| Device | Input Resolution | Output Format | Bitrate | Latency (ms) |
|--------|------------------|---------------|---------|--------------|
| Jetson Orin Nano | 1080p30 | HLS (720p30) | 2 Mbps | 200 |
| Jetson Orin Nano | 1080p30 | WebRTC (720p30) | 1.5 Mbps | 150 |
| Jetson Orin Nano | 720p30 | HLS (720p30) | 1.5 Mbps | 180 |
| Jetson Orin Nano | 720p30 | WebRTC (720p30) | 1 Mbps | 120 |
| Raspberry Pi 5 | 1080p30 | HLS (720p30) | 2 Mbps | 250 |
| Raspberry Pi 5 | 1080p30 | WebRTC (720p30) | 1.5 Mbps | 200 |
| Raspberry Pi 5 | 720p30 | HLS (720p30) | 1.5 Mbps | 220 |
| Raspberry Pi 5 | 720p30 | WebRTC (720p30) | 1 Mbps | 170 |

### End-to-End Latency

| Device | Pipeline | Latency (ms) |
|--------|----------|--------------|
| Jetson Orin Nano | RTSP → Inference → WebRTC | 183 |
| Jetson Orin Nano | RTSP → Inference → HLS | 233 |
| Raspberry Pi 5 | RTSP → Inference → WebRTC | 250 |
| Raspberry Pi 5 | RTSP → Inference → HLS | 300 |

## Network Requirements

- **Minimum Bandwidth**: 2 Mbps upload, 1 Mbps download
- **Recommended Bandwidth**: 5 Mbps upload, 2 Mbps download
- **Ports**:
  - 8001: Inference API
  - 8554: RTSP Server
  - 8888: HTTP Server (HLS)
  - 8889: WebRTC Signaling
  - 9090: Edge Agent API
- **Protocols**: HTTP, HTTPS, RTSP, WebRTC, MQTT, mTLS

## Security Features

- **Read-only Root Filesystem**: Prevents unauthorized modifications
- **Seccomp Profiles**: Restricts container system calls
- **AppArmor Profiles**: Limits container capabilities
- **mTLS with SPIFFE/SPIRE**: Secure device authentication
- **Vault Integration**: Secure secrets management
- **Signed OTA Updates**: Ensures update authenticity
- **Rollback Capability**: Recovers from failed updates

## Deployment Recommendations

- Use wired Ethernet connection for reliable connectivity
- Ensure adequate cooling, especially for Jetson devices
- Mount devices securely to prevent physical tampering
- Use UPS for power backup in critical deployments
- Configure firewall to restrict access to required ports only
- Regularly check for and apply security updates

## Troubleshooting

- **High Latency**: Check network conditions, reduce resolution/bitrate
- **Low FPS**: Reduce model complexity, use INT8 quantization
- **Connection Issues**: Verify network settings, check firewall rules
- **Update Failures**: Check network connectivity, verify device has sufficient storage
- **Overheating**: Ensure adequate cooling, reduce workload

## Support

For technical support, please contact:
- Email: support@bulo.cloud
- Documentation: https://docs.bulo.cloud/edge-kit
