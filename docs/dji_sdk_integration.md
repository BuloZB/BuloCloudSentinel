# DJI SDK Integration

This document describes how to use the DJI SDK integration in Bulo.Cloud Sentinel.

## Overview

Bulo.Cloud Sentinel provides integration with DJI drones through the DJI SDK. This integration allows you to control and monitor DJI drones using the same unified interface as other drone platforms.

The integration is implemented as an adapter that conforms to the `FlightControllerAdapter` interface, making it compatible with the rest of the Bulo.Cloud Sentinel platform.

## Supported DJI Drone Models

The DJI SDK integration supports the following DJI drone models:

- Mavic Series (Mini, Air, Pro, 3)
- Phantom Series (4, 4 Pro)
- Inspire Series (2, 3)
- Matrice Series (300 RTK, 350 RTK)
- Agras Series (T10, T30)

## Prerequisites

To use the DJI SDK integration, you need:

1. A DJI Developer account
2. A registered application with DJI Developer
3. App ID and App Key from DJI Developer
4. DJI SDK installed on your system

## Installation

### Installing DJI SDK

The DJI SDK is not included with Bulo.Cloud Sentinel due to licensing restrictions. You need to download and install it separately:

1. Register for a DJI Developer account at [https://developer.dji.com/](https://developer.dji.com/)
2. Create a new application in the DJI Developer Console
3. Download the DJI SDK for your platform
4. Install the DJI SDK according to the instructions provided by DJI

### Configuring Bulo.Cloud Sentinel

Once you have installed the DJI SDK, you need to configure Bulo.Cloud Sentinel to use it:

1. Add your DJI Developer App ID and App Key to your environment variables:

```bash
export DJI_APP_ID=your_app_id_here
export DJI_APP_KEY=your_app_key_here
```

2. Alternatively, you can provide these values directly when creating the adapter.

## Usage

### Creating a DJI Adapter

You can create a DJI adapter using the `AdapterFactory`:

```python
from dronecore.adapter_factory import AdapterFactory

# Create connection parameters
connection_params = {
    "app_id": "your_app_id_here",
    "app_key": "your_app_key_here",
    "connection_type": "USB",  # USB, WIFI, or BRIDGE
    "drone_model": "Mavic 3",
    "serial_number": "your_serial_number_here"  # Optional
}

# Create DJI adapter
adapter = AdapterFactory.create_adapter("dji", connection_params)
```

### Connecting to a DJI Drone

Once you have created the adapter, you can connect to the drone:

```python
# Connect to the drone
connected = await adapter.connect()
if connected:
    print("Connected to DJI drone")
else:
    print("Failed to connect to DJI drone")
```

### Sending Commands

You can send commands to the drone using the `send_command` method:

```python
# Takeoff command
takeoff_command = {
    "command": "takeoff",
    "parameters": {}
}
result = adapter.send_command(takeoff_command)

# Land command
land_command = {
    "command": "land",
    "parameters": {}
}
result = adapter.send_command(land_command)

# Goto command
goto_command = {
    "command": "goto",
    "parameters": {
        "latitude": 37.7749,
        "longitude": -122.4194,
        "altitude": 100
    }
}
result = adapter.send_command(goto_command)

# Return to home command
rth_command = {
    "command": "return_to_home",
    "parameters": {}
}
result = adapter.send_command(rth_command)
```

### Receiving Telemetry

You can receive telemetry data from the drone using the `receive_telemetry` method:

```python
# Get telemetry data
telemetry = adapter.receive_telemetry()
print(f"Battery level: {telemetry['battery']}%")
print(f"Position: {telemetry['position']}")
print(f"Attitude: {telemetry['attitude']}")
```

### Disconnecting

When you're done, you should disconnect from the drone:

```python
# Disconnect from the drone
await adapter.disconnect()
```

## Example

See the `examples/dji_sdk_demo.py` script for a complete example of how to use the DJI SDK integration.

## Troubleshooting

### DJI SDK Not Found

If you get an error saying that the DJI SDK is not available, make sure you have installed it correctly and that it's in your Python path.

### Connection Failed

If you can't connect to your DJI drone, check the following:

1. Make sure the drone is powered on and the remote controller is connected
2. Check that you're using the correct connection type (USB, WIFI, or BRIDGE)
3. Verify that your App ID and App Key are correct
4. Check the drone's firmware version is compatible with the DJI SDK version you're using

### Command Failed

If a command fails to execute, check the following:

1. Make sure the drone is connected
2. Verify that the command is supported by your drone model
3. Check that the command parameters are valid
4. Ensure the drone is in the correct state for the command (e.g., you can't takeoff if the drone is already flying)

## Limitations

The current implementation has the following limitations:

1. Not all DJI SDK features are exposed through the adapter
2. Some advanced features may require direct use of the DJI SDK
3. The adapter is designed for basic flight control and telemetry, not for advanced features like waypoint missions or camera control

## Future Improvements

Planned improvements for the DJI SDK integration include:

1. Support for waypoint missions
2. Camera control
3. Advanced flight modes
4. Support for more DJI drone models
5. Integration with the Bulo.Cloud Sentinel mission planning system
