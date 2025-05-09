# Weather Guard Service for Bulo.Cloud Sentinel

This microservice provides weather awareness capabilities for the Bulo.Cloud Sentinel platform, enabling weather-based decision making for drone operations.

## Features

- **Weather Data Integration**: Pull forecasts from Open-Meteo API with hourly wind and rain data
- **Local Weather Station Support**: Optional integration with MeteoShield (ESP32 + BME280) via MQTT
- **Mission Planning Integration**: Automatically blocks mission launch if weather conditions are unsafe
- **Indoor Fallback**: Triggers indoor mission alternatives when outdoor conditions are unfavorable
- **Dashboard Integration**: Provides weather forecast visualization and GO/NO-GO indicators
- **Alert System**: Notifies when weather windows open for mission execution
- **Caching**: Efficient Redis-based caching with 30-minute TTL
- **Fault Tolerance**: Fallback to last good data when API is unavailable

## Architecture

The Weather Guard service is built as a standalone microservice that integrates with the Bulo.Cloud Sentinel platform:

- **FastAPI Backend**: High-performance, async API built with FastAPI and Python 3.12
- **HTTPX Client**: Modern async HTTP client for API requests
- **Redis Cache**: Fast in-memory cache for weather data with TTL
- **MQTT Client**: For local weather station integration
- **Docker Containerization**: Easy deployment and scaling

## API Endpoints

- `GET /api/weather/current`: Get current weather data for a location
- `GET /api/weather/forecast`: Get weather forecast for a location
- `GET /api/weather/check-mission`: Check if weather conditions are suitable for a mission
- `GET /api/weather/alerts`: Get active weather alerts for a location
- `GET /api/weather/status`: Get service status and data source information

## Installation

### Prerequisites

- Docker and Docker Compose
- Redis server
- MQTT broker (optional, for local weather station)

### Configuration

Configuration is done via environment variables or a configuration file:

```yaml
# Example configuration
service:
  host: "0.0.0.0"
  port: 8090
  log_level: "INFO"

weather:
  default_provider: "open-meteo"
  update_interval: 30  # minutes
  cache_duration: 30   # minutes
  wind_threshold: 9.0  # m/s
  rain_threshold: 0.5  # mm/h

open_meteo:
  api_url: "https://api.open-meteo.com/v1/forecast"
  timeout: 10  # seconds

redis:
  url: "redis://redis:6379/0"
  prefix: "weather_guard:"
  ttl: 1800  # seconds (30 minutes)

mqtt:
  enabled: false
  broker: "mqtt-broker"
  port: 1883
  username: ""
  password: ""
  topic_prefix: "meteo_shield"
  qos: 1
```

## Usage

### Docker Compose

```yaml
version: '3.8'

services:
  weather_guard:
    build:
      context: ./weather_guard
      dockerfile: Dockerfile
    image: bulocloud-sentinel/weather-guard:latest
    container_name: weather-guard
    restart: unless-stopped
    ports:
      - "8090:8090"
    environment:
      - REDIS_URL=redis://redis:6379/0
      - LOG_LEVEL=INFO
    networks:
      - bulocloud-network
    depends_on:
      - redis

  redis:
    image: redis:7-alpine
    container_name: weather-guard-redis
    restart: unless-stopped
    volumes:
      - redis-data:/data
    networks:
      - bulocloud-network

volumes:
  redis-data:

networks:
  bulocloud-network:
    external: true
```

## Integration with Bulo.Cloud Sentinel

The Weather Guard service integrates with the following Bulo.Cloud Sentinel components:

1. **Mission Planning**: Provides weather checks for mission planning and execution
2. **Dashboard**: Displays weather data and forecasts in the UI
3. **Alert System**: Sends notifications about weather conditions
4. **Indoor Fallback**: Triggers indoor mission alternatives when outdoor conditions are unfavorable

## Development

### Setup Development Environment

```bash
# Create virtual environment
python -m venv venv
source venv/bin/activate  # Linux/Mac
venv\Scripts\activate     # Windows

# Install dependencies
pip install -r requirements.txt

# Run development server
uvicorn weather_guard.main:app --reload
```

### Running Tests

```bash
pytest
```
