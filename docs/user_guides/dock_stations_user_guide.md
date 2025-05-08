# Dock Stations User Guide

This user guide provides step-by-step instructions for configuring and using the Dock Stations feature in Bulo.CloudSentinel.

## Introduction

The Dock Stations feature enables seamless integration with various drone docking stations, allowing for automated charging, protection from weather, and extended operational capabilities. This guide will help you configure and use the Dock Stations feature effectively.

## Prerequisites

Before using the Dock Stations feature, ensure that you have the following:

- Bulo.CloudSentinel platform installed and running
- One or more compatible docking stations (DJI Dock 2, Heisha Charging Pad, or DIY ESP32-powered Dock)
- Network connectivity between the Bulo.CloudSentinel platform and the docking stations
- Required API keys and credentials for the docking stations

## Configuration

### DJI Dock 2 Configuration

To configure a DJI Dock 2, follow these steps:

1. Obtain API credentials from the DJI Developer Portal:
   - Go to [DJI Developer Portal](https://developer.dji.com/)
   - Create a new application
   - Enable the Cloud API feature
   - Note the API key and API secret

2. Configure the DJI Dock 2 in the `config.yaml` file:
   ```yaml
   dji_dock:
     enabled: true
     api_key: "your-dji-cloud-api-key"
     api_secret: "your-dji-cloud-api-secret"
     dock_sn: "dock-serial-number"
     region: "us-east-1"  # Available regions: us-east-1, eu-central-1, ap-southeast-1
     refresh_interval: 30  # Seconds
   ```

3. Restart the dock_driver microservice to apply the changes.

### Heisha Charging Pad Configuration

To configure a Heisha Charging Pad, follow these steps:

1. Ensure that the Heisha Charging Pad is connected to the network and accessible from the Bulo.CloudSentinel platform.

2. Configure the Heisha Charging Pad in the `config.yaml` file:
   ```yaml
   heisha_dock:
     enabled: true
     rest_api_url: "http://heisha-dock-ip:8080/api"
     modbus_host: "heisha-dock-ip"
     modbus_port: 502
     modbus_unit_id: 1
     refresh_interval: 15  # Seconds
     username: "admin"  # Optional, for REST API authentication
     password: "password"  # Optional, for REST API authentication
   ```

3. Restart the dock_driver microservice to apply the changes.

### DIY ESP32-powered Dock Configuration

To configure a DIY ESP32-powered Dock, follow these steps:

1. Set up the ESP32 with ESPHome:
   - Install ESPHome on your development machine
   - Create a new ESPHome configuration for the ESP32
   - Configure the MQTT integration in ESPHome
   - Flash the ESPHome firmware to the ESP32

2. Configure the ESP32-powered Dock in the `config.yaml` file:
   ```yaml
   esp32_dock:
     enabled: true
     mqtt_broker: "mqtt-broker-ip"
     mqtt_port: 1883
     mqtt_username: "username"  # Optional
     mqtt_password: "password"  # Optional
     mqtt_topic_prefix: "esp32_dock"
     refresh_interval: 10  # Seconds
   ```

3. Restart the dock_driver microservice to apply the changes.

### Power Management Integration

To enable automated charging based on battery levels, configure the Power Management integration:

```yaml
power_management:
  integration_enabled: true
  api_url: "http://bulocloud-sentinel-api:8000/api/power-management"
  auto_charge_threshold: 30  # Percentage
  charge_complete_threshold: 90  # Percentage
```

## Usage

### Accessing the Dock Stations Dashboard

To access the Dock Stations dashboard, follow these steps:

1. Open the Bulo.CloudSentinel web interface.
2. Navigate to the "Dock Stations" section in the sidebar.
3. The dashboard will display all configured docking stations and their current status.

### Monitoring Dock Status

The Dock Stations dashboard provides real-time information about the status of each docking station:

- **Status**: Online, offline, open, closed, or error
- **Charging Status**: Idle, charging, complete, or error
- **Telemetry**: Temperature, humidity, charging voltage, charging current, battery level, etc.

### Manual Control

You can manually control the docking stations from the dashboard:

1. Select a docking station from the list.
2. Use the control buttons to perform actions:
   - **Open Dock**: Open the dock door
   - **Close Dock**: Close the dock door
   - **Start Charging**: Start charging a drone
   - **Stop Charging**: Stop charging a drone
   - **Set Fan Speed** (Heisha only): Adjust the fan speed
   - **Set Heater State** (Heisha only): Turn the heater on or off
   - **Set Relay State** (ESP32 only): Control the charging relay

### Automated Charging

The Dock Stations feature can automatically charge drones when their battery level is low:

1. Ensure that the Power Management integration is enabled and configured.
2. Set the `auto_charge_threshold` to the desired battery level (e.g., 30%).
3. Set the `charge_complete_threshold` to the desired battery level (e.g., 90%).
4. When a drone's battery level drops below the `auto_charge_threshold`, it will automatically return to the nearest available docking station for charging.
5. Once the battery level reaches the `charge_complete_threshold`, the drone can resume its mission or remain docked until needed.

## Advanced Features

### Scheduled Charging

You can schedule charging operations for specific times:

1. Navigate to the "Schedules" section in the Dock Stations dashboard.
2. Click "Add Schedule" to create a new charging schedule.
3. Select the docking station, drone, and charging time.
4. Enable or disable the schedule as needed.

### Geofencing

You can define geofences around docking stations to control access:

1. Navigate to the "Geofences" section in the Dock Stations dashboard.
2. Click "Add Geofence" to create a new geofence.
3. Draw the geofence on the map and set the access rules.
4. Enable or disable the geofence as needed.

### Telemetry Alerts

You can configure alerts based on telemetry data:

1. Navigate to the "Alerts" section in the Dock Stations dashboard.
2. Click "Add Alert" to create a new alert.
3. Select the docking station, telemetry parameter, and threshold.
4. Configure the notification method (email, SMS, webhook).
5. Enable or disable the alert as needed.

## Troubleshooting

### Connection Issues

If you experience connection issues with a docking station:

1. Check that the docking station is powered on and connected to the network.
2. Verify that the IP address or hostname is correct in the configuration.
3. Ensure that there are no firewalls blocking the connection.
4. Check the network connectivity between the Bulo.CloudSentinel platform and the docking station.

### Authentication Issues

If you experience authentication issues:

1. Verify that the API key, username, and password are correct in the configuration.
2. Check that the API key has not expired or been revoked.
3. Ensure that the user account has the necessary permissions.

### Charging Issues

If you experience issues with charging:

1. Check that the drone is properly aligned with the charging pad.
2. Verify that the charging contacts are clean and undamaged.
3. Ensure that the power supply to the docking station is stable.
4. Check the charging voltage and current in the telemetry data.

## Support

If you need assistance with the Dock Stations feature, please contact our support team:

- Email: support@bulocloud.com
- Phone: +1-555-123-4567
- Web: https://bulocloud.com/support

## References

- [Dock Stations Documentation](../dock_stations.md)
- [API Documentation](../dock_driver/docs/api.md)
- [Wiki Page](../wiki/dock_stations.md)
