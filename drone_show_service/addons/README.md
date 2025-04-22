# Drone Show Blender Add-on

This add-on provides tools for creating and exporting drone show choreographies from Blender.

## Installation

1. Download the `drone_show_exporter.py` file.
2. Open Blender.
3. Go to Edit > Preferences > Add-ons.
4. Click "Install..." and select the downloaded file.
5. Enable the add-on by checking the box next to "Animation: Drone Show Exporter".

## Features

- Create and manage drone objects
- Arrange drones in formations (grid, circle, line)
- Set LED colors and effects
- Export choreographies to JSON format

## Usage

After installation, a "Drone Show" panel will appear in the 3D View sidebar (N key).

### Creating Drones

- Click "Create Drone" to create a new drone object.
- Select an existing object and click "Set as Drone" to mark it as a drone.
- Use "Batch Create Drones" to create multiple drones at once.

### Arranging Drones

- Select one or more drone objects.
- Choose a formation type (grid, circle, line).
- Set the formation parameters.
- Click the corresponding "Arrange in..." button.

### Setting LED Colors and Effects

- Select a drone object.
- Choose a color from the color picker.
- Click "Set LED Color" to apply the color.
- Choose an effect from the dropdown menu.
- Set the effect parameters.
- Click "Set LED Effect" to apply the effect.

### Animating Drones

- Enable Auto Keyframe (red circle in the timeline).
- Position drones and set LED colors/effects at different frames.
- Blender will automatically create keyframes for position, rotation, and LED properties.

### Exporting Choreographies

- Set the reference point (GPS coordinates).
- Set the animation settings (frame rate, time scale).
- Click "Export Drone Show" and choose a file location.

## File Format

The exported JSON file has the following structure:

```json
{
  "metadata": {
    "blender_version": "3.0.0",
    "export_time": "2023-06-01 12:00:00",
    "reference_lat": 37.7749,
    "reference_lon": -122.4194,
    "reference_alt": 0.0,
    "frame_rate": 30.0,
    "time_scale": 1.0,
    "frame_start": 1,
    "frame_end": 100
  },
  "objects": [
    {
      "name": "drone_1",
      "is_drone": true,
      "frames": [
        {
          "time": 0.0,
          "lat": 37.7749,
          "lon": -122.4194,
          "alt": 10.0,
          "heading": 0.0,
          "color": {
            "r": 255,
            "g": 0,
            "b": 0
          },
          "effect": "solid"
        },
        {
          "time": 1.0,
          "lat": 37.7750,
          "lon": -122.4195,
          "alt": 15.0,
          "heading": 90.0,
          "color": {
            "r": 0,
            "g": 255,
            "b": 0
          },
          "effect": "blink",
          "effect_params": {
            "frequency": 2.0
          }
        }
      ]
    }
  ]
}
```

## License

This add-on is licensed under the MIT License. See the LICENSE file for details.
