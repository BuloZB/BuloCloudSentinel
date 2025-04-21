# Anti-Jamming Service for Bulo.Cloud Sentinel

The Anti-Jamming Service is a microservice for the Bulo.Cloud Sentinel platform that provides GNSS anti-jamming, RF jamming detection, and resilient communication capabilities.

## Features

- **GNSS Anti-Jamming**: Uses a multi-antenna GNSS array (4× patch antennas) connected to a KrakenSDR module (5 coherent SDR channels) to mitigate jamming signals.
- **RF Jamming Detection**: Uses KrakenSDR's GNU Radio blocks for Direction of Arrival (DoA) estimation to detect and locate jamming sources.
- **Jamming Simulation**: Uses HackRF One (20 MHz BW) or ADALM-PLUTO for testing in 2.4 GHz & 5 GHz bands.
- **Frequency-Hopping Spread Spectrum (FHSS)**: Uses LoRa SX127x breakout board (868 MHz) for resilient communication.

## Hardware Requirements

### GNSS Anti-Jamming
- KrakenSDR (5 coherent RTL-SDR channels)
- 4× GNSS patch antennas
- USB 3.0 hub
- Antenna array mounting hardware

### RF Jamming Detection
- KrakenSDR
- Directional antennas for 2.4 GHz and 5 GHz bands

### Jamming Simulation
- HackRF One or ADALM-PLUTO SDR
- Antennas for 2.4 GHz and 5 GHz bands

### FHSS Communication
- LoRa SX127x breakout board
- 868 MHz antenna

## Hardware Wiring Diagram

```
+----------------+     +----------------+
| GNSS Antenna 1 |     | GNSS Antenna 2 |
+-------+--------+     +--------+-------+
        |                       |
        v                       v
+-------+--------+     +--------+-------+
| KrakenSDR Ch 1 |     | KrakenSDR Ch 2 |
+----------------+     +----------------+
        ^                       ^
        |                       |
+-------+--------+     +--------+-------+
| GNSS Antenna 3 |     | GNSS Antenna 4 |
+----------------+     +----------------+
        |                       |
        v                       v
+-------+--------+     +--------+-------+
| KrakenSDR Ch 3 |     | KrakenSDR Ch 4 |
+----------------+     +----------------+
                ^
                |
                v
        +----------------+
        | KrakenSDR Ch 0 |
        | (Reference)    |
        +-------+--------+
                |
                v
        +-------+--------+
        | USB 3.0 Hub    |
        +-------+--------+
                |
                v
        +-------+--------+
        | Host Computer  |
        +-------+--------+
                ^
                |
        +-------+--------+
        | HackRF One     |
        +----------------+
                ^
                |
        +-------+--------+
        | LoRa SX127x    |
        +----------------+
```

## GNU Radio Flowgraph

The Anti-Jamming Service uses the following GNU Radio flowgraph for GNSS anti-jamming:

```
+----------------+     +----------------+     +----------------+
| RTL-SDR Source |---->| Pulse Blanking |---->| Notch Filter  |
+----------------+     +----------------+     +----------------+
                                                      |
                                                      v
+----------------+     +----------------+     +----------------+
| GNSS Processor |<----| Adaptive Filter|<----| DoA Estimation|
+----------------+     +----------------+     +----------------+
```

## Software Architecture

The Anti-Jamming Service is designed as a microservice with the following components:

- **Hardware Interfaces**: Abstractions for KrakenSDR, HackRF, and LoRa SX127x.
- **Signal Processing**: GNSS mitigation, DoA estimation, jamming detection, and FHSS.
- **API**: FastAPI endpoints for status, configuration, and control.
- **Messaging**: RabbitMQ integration for event-driven communication.

## Installation

### Using Docker

1. Clone the repository:
   ```bash
   git clone https://github.com/your-org/bulo-cloud-sentinel.git
   cd bulo-cloud-sentinel/anti_jamming_service
   ```

2. Build and run the Docker container:
   ```bash
   docker-compose up -d
   ```

### Manual Installation

1. Install system dependencies:
   ```bash
   sudo apt-get update
   sudo apt-get install -y build-essential cmake git libusb-1.0-0-dev pkg-config
   sudo apt-get install -y gnuradio gnuradio-dev python3-gnuradio
   sudo apt-get install -y rtl-sdr librtlsdr-dev
   sudo apt-get install -y hackrf libhackrf-dev
   sudo apt-get install -y gnss-sdr
   ```

2. Install Python dependencies:
   ```bash
   pip install -r requirements.txt
   ```

3. Install KrakenSDR GNU Radio blocks:
   ```bash
   git clone https://github.com/krakenrf/gr-krakensdr.git
   cd gr-krakensdr
   mkdir build
   cd build
   cmake ..
   make
   sudo make install
   sudo ldconfig
   ```

4. Run the service:
   ```bash
   python -m anti_jamming_service.main
   ```

## Configuration

The Anti-Jamming Service is configured using a YAML file located at `config/config.yaml`. You can override the configuration file location using the `ANTI_JAMMING_CONFIG` environment variable.

Example Docker Compose configuration:

```yaml
version: '3.8'

services:
  anti_jamming_service:
    build:
      context: .
      dockerfile: Dockerfile
    image: bulo-cloud-sentinel/anti-jamming-service:latest
    container_name: anti-jamming-service
    restart: unless-stopped
    privileged: true  # Required for USB device access
    volumes:
      - ./config:/app/config
      - ./logs:/app/logs
      - ./certs:/app/certs
    ports:
      - "8080:8080"
    environment:
      - ANTI_JAMMING_CONFIG=/app/config/config.yaml
      - VAULT_ADDR=http://vault:8200
      - VAULT_TOKEN=${VAULT_TOKEN}
    devices:
      - /dev/bus/usb:/dev/bus/usb  # For USB devices (KrakenSDR, HackRF)
      - /dev/ttyUSB0:/dev/ttyUSB0  # For LoRa module
    networks:
      - bulo-network
    depends_on:
      - rabbitmq
      - vault
```

## API Endpoints

The Anti-Jamming Service provides the following API endpoints:

- `GET /api/status`: Get service status.
- `GET /api/hardware`: Get hardware status.
- `POST /api/hardware/configure`: Configure hardware.
- `GET /api/processing`: Get processing status.
- `POST /api/processing/configure`: Configure processing.
- `GET /api/jamming/detect`: Detect jamming.
- `GET /api/jamming/doa`: Estimate direction of arrival.
- `POST /api/fhss/send`: Send a message using FHSS.
- `GET /api/fhss/receive`: Receive a message using FHSS.

## Performance Benchmarks

### Jamming Detection

- **Detection Latency**: < 100 ms
- **False Positive Rate**: < 1%
- **False Negative Rate**: < 5%
- **Minimum SNR**: -10 dB

### Direction of Arrival Estimation

- **Azimuth Accuracy**: ±5°
- **Elevation Accuracy**: ±10°
- **Minimum SNR**: -5 dB

### GNSS Anti-Jamming

- **Jamming Suppression**: > 30 dB
- **Maximum Number of Jammers**: 3
- **Minimum J/S Ratio**: -30 dB

### FHSS Communication

- **Hop Rate**: 10 hops/second
- **Number of Channels**: 10
- **Range**: 1 km (line of sight)
- **Data Rate**: 1.2 kbps

## Testing

To run the tests:

```bash
pytest
```

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Acknowledgements

- [KrakenSDR](https://github.com/krakenrf/krakensdr_doa)
- [GNSS-SDR](https://github.com/gnss-sdr/gnss-sdr)
- [HackRF](https://github.com/mossmann/hackrf)
- [pyLoRa](https://github.com/rpsreal/pySX127x)
