# Bulo.Cloud Sentinel DJI Mission Planner

This web-based UI provides a comprehensive interface for planning and executing DJI drone missions in Bulo.Cloud Sentinel.

## Features

- **Mission Planning**: Create and edit waypoint missions with a user-friendly map interface
- **Camera Actions**: Configure camera actions at each waypoint (take photo, start/stop recording)
- **Gimbal Control**: Set gimbal positions at each waypoint
- **Mission Management**: Save, load, and delete missions
- **Live Telemetry**: Monitor drone telemetry data in real-time
- **Manual Control**: Basic manual control functions (takeoff, land, return to home)
- **Dock Integration**: Support for DJI dock stations for automated missions

## Setup

1. Install the required dependencies:

```bash
pip install flask werkzeug
```

2. Run the web server:

```bash
python web/dji_mission_planner.py
```

3. Open a web browser and navigate to `http://localhost:5000`

## Usage

### Connection Settings

Before connecting to a drone, configure the connection settings:

1. Navigate to the Settings page
2. Enter your DJI Developer App ID and App Key
3. Select the appropriate connection type (USB, WiFi, or Bridge)
4. Save the settings
5. Click the Connect button in the top navigation bar

### Creating a Mission

1. Navigate to the Mission Planner page
2. Click on the map to add waypoints
3. Click on a waypoint to edit its properties (altitude, heading, actions)
4. Configure mission settings (speed, finish action, heading mode)
5. Click Save Mission to save the mission

### Executing a Mission

1. Navigate to the Missions page
2. Find the mission you want to execute
3. Click the Execute button
4. Monitor the mission progress on the Dashboard page

## Dock Integration

For automated missions using a DJI dock:

1. Configure dock settings on the Settings page
2. Schedule missions to run automatically
3. The dock will handle takeoff, mission execution, landing, and charging

## Development

The web UI is built using:

- **Backend**: Flask (Python)
- **Frontend**: HTML, CSS, JavaScript
- **Maps**: Leaflet.js
- **UI Framework**: Bootstrap 5

To extend the UI:

1. Add new API endpoints in `dji_mission_planner.py`
2. Add new UI components in the HTML templates
3. Add new functionality in the JavaScript files

## File Structure

- `dji_mission_planner.py`: Main Flask application
- `templates/`: HTML templates
- `static/css/`: CSS stylesheets
- `static/js/`: JavaScript files
- `static/img/`: Images and icons
- `missions/`: Saved mission files

## License

This project is part of Bulo.Cloud Sentinel and is licensed under the same terms as the main project.
