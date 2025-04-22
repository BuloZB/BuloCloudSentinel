# Blender Integration for Drone Show Microservice

This document provides instructions for integrating Blender with the Drone Show microservice to create drone show choreographies.

## Overview

Blender is a powerful 3D creation suite that can be used to create animations for drone shows. The Drone Show microservice provides a way to convert Blender animations into drone show choreographies that can be executed by a fleet of drones.

## Requirements

- Blender 2.93 or later
- Python 3.7 or later
- Drone Show Microservice API access

## Installation

1. Download and install Blender from [blender.org](https://www.blender.org/).
2. Download the Drone Show Blender Add-on from the `addons` directory of the Drone Show microservice.
3. Install the add-on in Blender:
   - Open Blender
   - Go to Edit > Preferences > Add-ons
   - Click "Install..." and select the downloaded add-on file
   - Enable the add-on by checking the box next to "Animation: Drone Show Exporter"

## Creating a Drone Show Animation

### Setting Up the Scene

1. Create a new Blender project.
2. Set the scene scale to real-world units:
   - Go to Scene Properties > Units
   - Set Unit System to "Metric"
   - Set Unit Scale to 1.0
   - Set Length to "Meters"

3. Set up the animation timeline:
   - Go to Output Properties > Frame Rate
   - Set Frame Rate to 30 fps
   - Go to Output Properties > Frame Range
   - Set Start Frame to 1
   - Set End Frame to match your desired show duration (e.g., 1800 frames for a 60-second show at 30 fps)

### Creating Drone Objects

1. For each drone in your show, create a new object:
   - Add a new Empty object (Add > Empty > Plain Axes)
   - Rename the object to "drone_X" where X is a unique identifier (e.g., "drone_1", "drone_2", etc.)
   - In Object Properties, add a custom property named "is_drone" and set it to True

2. Position the drones at their starting positions:
   - Use the Transform tools to position each drone
   - The Z-axis represents altitude
   - The X and Y axes represent longitude and latitude

### Animating Drones

1. For each drone, create keyframes for position and rotation:
   - Select the drone object
   - Move to the desired frame in the timeline
   - Position the drone
   - Press "I" and select "Location" to create a position keyframe
   - Rotate the drone to set its heading
   - Press "I" and select "Rotation" to create a rotation keyframe

2. Repeat for each keyframe in the animation.

3. Use Blender's animation tools to refine the motion:
   - Open the Graph Editor to edit animation curves
   - Use modifiers like Noise or Cycles to create complex patterns
   - Use constraints to maintain formation or follow paths

### Adding LED Effects

1. For each drone, add LED effect keyframes:
   - Select the drone object
   - Go to Object Properties > Custom Properties
   - Add a custom property named "led_color" with an RGB value (e.g., [1.0, 0.0, 0.0] for red)
   - Add a custom property named "led_effect" with a string value (e.g., "solid", "blink", "pulse")
   - Add keyframes for these properties at the desired frames

2. For effect parameters, add additional custom properties:
   - For "blink" effect, add "blink_frequency" property
   - For "pulse" effect, add "pulse_frequency" property
   - For "rainbow" effect, add "rainbow_speed" property

### Exporting the Animation

1. Once your animation is complete, export it using the Drone Show Exporter:
   - Go to File > Export > Drone Show Animation (.json)
   - Set the export options:
     - GPS Reference: Set the GPS coordinates for the origin of your Blender scene
     - Altitude Reference: Set the ground level altitude in meters
     - Frame Rate: Match your scene's frame rate
     - Time Scale: Set to 1.0 for real-time playback
   - Click "Export" and save the file

## Converting to Choreography

### Using the Python Converter

The Drone Show microservice provides a Python utility for converting Blender animations to choreographies:

```python
from drone_show_service.utils.blender_converter import convert_blender_animation, save_choreography_to_file

# Load the Blender animation
with open("my_animation.json", "r") as f:
    animation_data = json.load(f)

# Convert to choreography
choreography = convert_blender_animation(
    animation_data=animation_data,
    name="My Blender Choreography",
    description="A choreography created in Blender",
    author="John Doe"
)

# Save to file
save_choreography_to_file(choreography, "my_choreography.json")
```

### Using the API

You can also upload the Blender animation directly to the Drone Show microservice API:

```python
import requests
import json

# Load the Blender animation
with open("my_animation.json", "r") as f:
    animation_data = json.load(f)

# Prepare the request
url = "http://your-server:8000/shows/blender"
headers = {
    "Authorization": "Bearer your-token",
    "Content-Type": "application/json"
}
data = {
    "name": "My Blender Choreography",
    "description": "A choreography created in Blender",
    "author": "John Doe",
    "animation_data": animation_data
}

# Send the request
response = requests.post(url, headers=headers, json=data)

# Check the response
if response.status_code == 201:
    choreography = response.json()
    print(f"Choreography created with ID: {choreography['id']}")
else:
    print(f"Error: {response.status_code} - {response.text}")
```

## Blender Add-on Features

The Drone Show Blender Add-on provides the following features:

### Drone Show Panel

The add-on adds a "Drone Show" panel to the 3D View sidebar (N key) with the following options:

- **Create Drone**: Add a new drone object to the scene
- **Set as Drone**: Mark the selected object as a drone
- **Set LED Color**: Set the LED color for the selected drone
- **Set LED Effect**: Set the LED effect for the selected drone
- **Batch Create Drones**: Create multiple drones at once
- **Arrange in Formation**: Arrange selected drones in a formation (grid, circle, line)
- **Generate Path**: Generate a path for the selected drone
- **Synchronize Motion**: Synchronize the motion of multiple drones
- **Preview Show**: Preview the drone show in the 3D viewport
- **Export Show**: Export the drone show to a JSON file

### Formation Tools

The add-on includes tools for creating common formations:

- **Grid Formation**: Arrange drones in a grid pattern
- **Circle Formation**: Arrange drones in a circle
- **Line Formation**: Arrange drones in a line
- **Custom Formation**: Arrange drones in a custom pattern

### Path Generation

The add-on includes tools for generating paths:

- **Linear Path**: Generate a straight-line path
- **Circular Path**: Generate a circular path
- **Spiral Path**: Generate a spiral path
- **Follow Path**: Make drones follow a Blender path object
- **Follow Surface**: Make drones follow a Blender surface

### LED Effects

The add-on includes tools for creating LED effects:

- **Solid Color**: Set a solid color
- **Blink**: Make the LED blink at a specified frequency
- **Pulse**: Make the LED pulse at a specified frequency
- **Rainbow**: Cycle through rainbow colors
- **Chase**: Create a chase effect across multiple drones
- **Custom**: Create a custom effect using Blender's animation tools

## Best Practices

### Performance Optimization

- Keep the number of keyframes to a minimum
- Use Blender's F-Curve modifiers for complex animations
- Use Blender's constraints for formations and patterns
- Preview the animation at a lower frame rate before exporting

### Safety Considerations

- Ensure drones maintain a safe distance from each other
- Avoid rapid changes in direction or altitude
- Consider wind conditions when designing outdoor shows
- Include safety margins in your choreography

### Creative Tips

- Use Blender's particle systems to create complex patterns
- Use Blender's physics simulations for natural-looking movements
- Use Blender's modifiers to create symmetrical patterns
- Use Blender's animation nodes for procedural animations

## Troubleshooting

### Common Issues

- **Scale Issues**: Ensure your Blender scene is using real-world units
- **Coordinate Issues**: Check the GPS reference coordinates
- **Timing Issues**: Ensure your frame rate and time scale are correct
- **Missing Drones**: Ensure all drone objects have the "is_drone" property set to True
- **Missing LED Effects**: Ensure all drones have the "led_color" and "led_effect" properties

### Getting Help

If you encounter issues with the Blender integration, please contact the Drone Show microservice support team or refer to the following resources:

- [Drone Show Microservice Documentation](https://bulocloud-sentinel.example.com/docs/drone-show)
- [Blender Documentation](https://docs.blender.org/)
- [Drone Show Community Forum](https://bulocloud-sentinel.example.com/forum/drone-show)
