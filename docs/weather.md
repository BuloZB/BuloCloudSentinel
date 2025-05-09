# Weather Guard - Smart Weather Awareness

The Weather Guard module provides comprehensive weather awareness capabilities for the Bulo.Cloud Sentinel platform, enabling weather-based decision making for drone operations.

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

## API Reference

### Current Weather

Get current weather data for a location.

**Endpoint**: `GET /api/weather/current`

**Parameters**:
- `latitude` (float): Latitude of the location
- `longitude` (float): Longitude of the location
- `provider` (string, optional): Weather data provider (default: "open-meteo")

**Response**:
```json
{
  "success": true,
  "data": {
    "id": "3fa85f64-5717-4562-b3fc-2c963f66afa6",
    "latitude": 52.52,
    "longitude": 13.41,
    "timestamp": "2023-10-15T12:00:00Z",
    "provider": "open-meteo",
    "condition": "partly_cloudy",
    "temperature": 15.3,
    "feels_like": 14.8,
    "humidity": 65,
    "pressure": 1012.0,
    "wind_speed": 5.0,
    "wind_direction": 270,
    "wind_gust": 8.0,
    "visibility": null,
    "cloud_cover": 100,
    "precipitation": 0.1,
    "uv_index": null
  }
}
```

### Weather Forecast

Get weather forecast for a location.

**Endpoint**: `GET /api/weather/forecast`

**Parameters**:
- `latitude` (float): Latitude of the location
- `longitude` (float): Longitude of the location
- `hours` (int, optional): Number of hours to forecast (default: 24)
- `provider` (string, optional): Weather data provider (default: "open-meteo")

**Response**:
```json
{
  "success": true,
  "data": [
    {
      "id": "3fa85f64-5717-4562-b3fc-2c963f66afa6",
      "latitude": 52.52,
      "longitude": 13.41,
      "timestamp": "2023-10-15T12:00:00Z",
      "forecast_time": "2023-10-15T12:00:00Z",
      "provider": "open-meteo",
      "condition": "partly_cloudy",
      "temperature": 15.3,
      "wind_speed": 5.0,
      "wind_direction": 270,
      "precipitation": 0.1,
      "humidity": 65,
      "cloud_cover": 100
    },
    {
      "id": "4fa85f64-5717-4562-b3fc-2c963f66afa7",
      "latitude": 52.52,
      "longitude": 13.41,
      "timestamp": "2023-10-15T12:00:00Z",
      "forecast_time": "2023-10-15T13:00:00Z",
      "provider": "open-meteo",
      "condition": "partly_cloudy",
      "temperature": 16.1,
      "wind_speed": 4.8,
      "wind_direction": 275,
      "precipitation": 0.0,
      "humidity": 63,
      "cloud_cover": 75
    }
  ]
}
```

### Mission Weather Check

Check if weather conditions are suitable for a mission.

**Endpoint**: `GET /api/weather/check-mission`

**Parameters**:
- `latitude` (float): Latitude of the mission location
- `longitude` (float): Longitude of the mission location
- `start_time` (string): Mission start time (ISO 8601 format)
- `end_time` (string, optional): Mission end time (ISO 8601 format)
- `mission_id` (string, optional): Mission ID
- `provider` (string, optional): Weather data provider (default: "open-meteo")

**Response**:
```json
{
  "success": true,
  "data": {
    "id": "3fa85f64-5717-4562-b3fc-2c963f66afa6",
    "mission_id": "mission-123",
    "latitude": 52.52,
    "longitude": 13.41,
    "check_time": "2023-10-15T12:00:00Z",
    "start_time": "2023-10-15T14:00:00Z",
    "end_time": "2023-10-15T15:00:00Z",
    "is_flyable": true,
    "severity": "none",
    "current_weather": {
      "id": "3fa85f64-5717-4562-b3fc-2c963f66afa6",
      "latitude": 52.52,
      "longitude": 13.41,
      "timestamp": "2023-10-15T12:00:00Z",
      "provider": "open-meteo",
      "condition": "partly_cloudy",
      "temperature": 15.3,
      "feels_like": 14.8,
      "humidity": 65,
      "pressure": 1012.0,
      "wind_speed": 5.0,
      "wind_direction": 270,
      "wind_gust": 8.0,
      "visibility": null,
      "cloud_cover": 100,
      "precipitation": 0.1,
      "uv_index": null
    },
    "forecast": [
      {
        "id": "4fa85f64-5717-4562-b3fc-2c963f66afa7",
        "latitude": 52.52,
        "longitude": 13.41,
        "timestamp": "2023-10-15T12:00:00Z",
        "forecast_time": "2023-10-15T14:00:00Z",
        "provider": "open-meteo",
        "condition": "partly_cloudy",
        "temperature": 16.5,
        "wind_speed": 4.5,
        "wind_direction": 280,
        "precipitation": 0.0,
        "humidity": 60,
        "cloud_cover": 50
      }
    ],
    "alerts": [],
    "limitations": [],
    "recommendations": [
      "Weather conditions are suitable for the mission"
    ]
  }
}
```

## Integration with Mission Planning

The Weather Guard service integrates with the mission planning system to automatically check weather conditions before mission launch. If weather conditions are unsuitable (wind > 9 m/s or rain > 0.5 mm/h), the mission will be blocked and indoor alternatives will be suggested.

### Example Integration Code

```python
from weather_guard.client import WeatherGuardClient

async def check_mission_weather(mission_id, latitude, longitude, start_time, end_time):
    """Check if weather conditions are suitable for a mission."""
    client = WeatherGuardClient()
    result = await client.check_mission_weather(
        mission_id=mission_id,
        latitude=latitude,
        longitude=longitude,
        start_time=start_time,
        end_time=end_time,
    )
    
    if not result.is_flyable:
        # Block mission and suggest indoor alternatives
        print(f"Mission blocked due to weather: {result.limitations}")
        print(f"Recommendations: {result.recommendations}")
        return False
    
    return True
```

## Dashboard Integration

The Weather Guard service provides data for the dashboard, including a 24-hour forecast graph and a GO/NO-GO indicator.

### Example Grafana Panel

![Weather Dashboard](weather_dashboard.png)

The dashboard includes:
- Current weather conditions
- 24-hour forecast graph
- Wind speed and direction
- Precipitation forecast
- GO/NO-GO indicator
- Weather alerts

## Local Weather Station Integration

The Weather Guard service can integrate with local weather stations using the MeteoShield (ESP32 + BME280) via MQTT.

### MeteoShield Configuration

The MeteoShield should be configured to publish weather data to the MQTT broker using the following topic structure:

```
meteo_shield/<device_id>/temperature
meteo_shield/<device_id>/humidity
meteo_shield/<device_id>/pressure
meteo_shield/<device_id>/wind_speed
meteo_shield/<device_id>/wind_direction
meteo_shield/<device_id>/rain
```

### Example MeteoShield Arduino Code

```cpp
#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

// WiFi credentials
const char* ssid = "your_ssid";
const char* password = "your_password";

// MQTT broker
const char* mqtt_server = "mqtt_broker_ip";
const int mqtt_port = 1883;
const char* mqtt_user = "mqtt_user";
const char* mqtt_password = "mqtt_password";
const char* device_id = "meteo_shield_1";

// BME280 sensor
Adafruit_BME280 bme;

// WiFi and MQTT clients
WiFiClient espClient;
PubSubClient client(espClient);

void setup() {
  Serial.begin(115200);
  
  // Initialize BME280
  if (!bme.begin(0x76)) {
    Serial.println("Could not find a valid BME280 sensor!");
    while (1);
  }
  
  // Connect to WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  // Connect to MQTT broker
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
}

void loop() {
  // Reconnect to MQTT if needed
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  
  // Read sensor data
  float temperature = bme.readTemperature();
  float humidity = bme.readHumidity();
  float pressure = bme.readPressure() / 100.0F;
  
  // Publish data to MQTT
  char topic[50];
  char payload[10];
  
  sprintf(topic, "meteo_shield/%s/temperature", device_id);
  sprintf(payload, "%.2f", temperature);
  client.publish(topic, payload);
  
  sprintf(topic, "meteo_shield/%s/humidity", device_id);
  sprintf(payload, "%.2f", humidity);
  client.publish(topic, payload);
  
  sprintf(topic, "meteo_shield/%s/pressure", device_id);
  sprintf(payload, "%.2f", pressure);
  client.publish(topic, payload);
  
  // Wait 30 seconds
  delay(30000);
}
```
