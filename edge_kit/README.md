# Bulo.CloudSentinel Edge Kit

The Edge Kit is a plug-and-play Docker bundle for deploying Bulo.CloudSentinel capabilities at the edge, specifically optimized for Jetson Orin Nano and Raspberry Pi 5 devices.

## Features

- **Low-Latency Inference**: Run quantized AI models (YOLOv10-nano INT8) at the edge
- **Optimized Streaming**: Ingest local RTSP streams and publish low-bitrate HLS/WebRTC to the central cloud
- **Self-Updates**: Over-the-air updates with rollback capability
- **Secure by Design**: Read-only root FS, seccomp & AppArmor profiles, Vault integration

## Components

The Edge Kit consists of three main containers:

1. **edge_inference**: Triton Server with TensorRT/ONNX ORT (ARM NEON fallback)
2. **rtsp_relay**: ffmpeg or rtsp-simple-server → WebRTC (Pion) + HLS segmenter
3. **edge_agent**: Rust-based agent for OTA updates, healthchecks, and secure MQTT to core

## Hardware Requirements

### Jetson Orin Nano
- NVIDIA Jetson Orin Nano Developer Kit (8GB)
- 64GB+ microSD card (Class 10, UHS-I or better)
- 5V 4A power supply
- Ethernet connection or WiFi adapter

### Raspberry Pi 5
- Raspberry Pi 5 (8GB recommended)
- 64GB+ microSD card (Class 10, UHS-I or better)
- Official Raspberry Pi 5 Power Supply (5V 5A USB-C)
- Raspberry Pi Active Cooler
- Ethernet connection or WiFi

## Installation

1. Flash the base OS image to your device:
   - For Jetson: JetPack 6.0 or newer
   - For Raspberry Pi: Raspberry Pi OS 64-bit (Bookworm)

2. Install Docker and Docker Compose:
   ```bash
   curl -fsSL https://get.docker.com | sh
   sudo apt-get install -y docker-compose-plugin
   ```

3. Run the Edge Kit installer:
   ```bash
   curl -fsSL https://raw.githubusercontent.com/your-org/bulo-cloud-sentinel/main/edge_kit/edge_install.sh | sudo bash
   ```

## Performance Benchmarks

| Device | Model | Resolution | FPS | Latency (ms) |
|--------|-------|------------|-----|-------------|
| Jetson Orin Nano | YOLOv10-nano INT8 | 1080p | 30 | 33 |
| Raspberry Pi 5 | YOLOv10-nano INT8 | 720p | 15 | 67 |

## Security Features

- Read-only root filesystem
- Seccomp and AppArmor profiles
- mTLS with SPIFFE/SPIRE for device authentication
- Secrets loaded from Vault using short-lived tokens
- Signed OTA updates with rollback capability

## Development

See the [Development Guide](DEVELOPMENT.md) for information on building and extending the Edge Kit.
