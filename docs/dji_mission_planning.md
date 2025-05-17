# DJI Mission Planning Guide

This document provides detailed information on mission planning for DJI drones in Bulo.Cloud Sentinel, including waypoint file formats, mission types, and best practices.

## Table of Contents

1. [Introduction](#introduction)
2. [Waypoint File Format](#waypoint-file-format)
3. [Mission Types](#mission-types)
4. [Camera Actions](#camera-actions)
5. [Gimbal Control](#gimbal-control)
6. [Best Practices](#best-practices)
7. [Examples](#examples)
8. [Troubleshooting](#troubleshooting)

## Introduction

Bulo.Cloud Sentinel provides comprehensive mission planning capabilities for DJI drones. Missions can be defined programmatically or using JSON waypoint files. This guide focuses on creating and executing missions using waypoint files.

## Waypoint File Format

Waypoint files use JSON format to define mission parameters and waypoints. The basic structure is as follows:

```json
{
  "mission_name": "Surveillance Mission",
  "mission_type": "waypoint",
  "speed": 5.0,
  "finish_action": "go_home",
  "heading_mode": "auto",
  "waypoints": [
    {
      "latitude": 37.7749,
      "longitude": -122.4194,
      "altitude": 50.0,
      "heading": 0,
      "stay_time": 5,
      "actions": [
        {"action": "take_photo"},
        {"action": "rotate_gimbal", "pitch": -45, "yaw": 0}
      ]
    },
    // More waypoints...
  ]
}
```

### Required Fields

- **mission_name**: Name of the mission (string)
- **mission_type**: Type of mission (string: "waypoint", "hotpoint", "follow_me", "timeline")
- **waypoints**: Array of waypoint objects

### Optional Fields

- **speed**: Flight speed in m/s (default: 5.0)
- **finish_action**: Action to take when mission completes (string: "no_action", "go_home", "auto_land", "return_to_first_waypoint")
- **heading_mode**: Heading control mode (string: "auto", "using_waypoint_heading", "toward_point_of_interest", "using_initial_direction")
- **max_flight_speed**: Maximum flight speed in m/s (default: 15.0)
- **auto_flight_speed**: Auto flight speed in m/s (default: 5.0)
- **drone_height**: Relative takeoff height in meters (default: 30.0)
- **exit_mission_on_rc_signal_lost**: Whether to exit mission on RC signal lost (boolean, default: true)
- **goto_first_waypoint_mode**: Mode to go to first waypoint (string: "safely", "point_to_point")

### Waypoint Object

Each waypoint object must contain:

- **latitude**: Latitude in degrees (float)
- **longitude**: Longitude in degrees (float)
- **altitude**: Altitude in meters (float)

Optional waypoint parameters:

- **heading**: Heading in degrees (0-359, default: 0)
- **stay_time**: Time to stay at waypoint in seconds (default: 0)
- **turn_mode**: Turn mode at waypoint (string: "clockwise", "counter_clockwise", "auto")
- **actions**: Array of action objects to execute at this waypoint

## Mission Types

### Waypoint Mission

A waypoint mission consists of a series of waypoints that the drone will visit in sequence. At each waypoint, the drone can perform actions such as taking photos or rotating the gimbal.

Example waypoint mission:

```json
{
  "mission_name": "Basic Waypoint Mission",
  "mission_type": "waypoint",
  "speed": 5.0,
  "finish_action": "go_home",
  "waypoints": [
    {
      "latitude": 37.7749,
      "longitude": -122.4194,
      "altitude": 50.0
    },
    {
      "latitude": 37.7750,
      "longitude": -122.4195,
      "altitude": 60.0
    },
    {
      "latitude": 37.7751,
      "longitude": -122.4196,
      "altitude": 70.0
    }
  ]
}
```

### Hotpoint Mission

A hotpoint mission (also known as Point of Interest or Orbit) makes the drone fly in a circle around a specified point.

Example hotpoint mission:

```json
{
  "mission_name": "Orbit Mission",
  "mission_type": "hotpoint",
  "latitude": 37.7749,
  "longitude": -122.4194,
  "altitude": 50.0,
  "radius": 10.0,
  "angular_speed": 15.0,
  "is_clockwise": true,
  "start_point": 0,
  "heading_mode": "toward_poi"
}
```

### Follow Me Mission

A follow me mission makes the drone follow a moving target (usually the operator).

Example follow me mission:

```json
{
  "mission_name": "Follow Me Mission",
  "mission_type": "follow_me",
  "initial_latitude": 37.7749,
  "initial_longitude": -122.4194,
  "altitude": 10.0,
  "follow_distance": 5.0,
  "follow_height": 10.0
}
```

## Camera Actions

Camera actions can be added to waypoints to control the camera during the mission. Available actions:

- **take_photo**: Take a single photo
- **start_recording**: Start video recording
- **stop_recording**: Stop video recording
- **set_camera_mode**: Set camera mode (PHOTO, VIDEO, PLAYBACK)
- **set_photo_mode**: Set photo mode (SINGLE, HDR, BURST, AEB, INTERVAL, PANO, EHDR)
- **set_video_resolution**: Set video resolution (RES_720P, RES_1080P, RES_2_7K, RES_4K)

Example waypoint with camera actions:

```json
{
  "latitude": 37.7749,
  "longitude": -122.4194,
  "altitude": 50.0,
  "actions": [
    {"action": "set_camera_mode", "mode": "PHOTO"},
    {"action": "take_photo"},
    {"action": "set_camera_mode", "mode": "VIDEO"},
    {"action": "start_recording"}
  ]
}
```

## Gimbal Control

Gimbal actions can be added to waypoints to control the gimbal during the mission. Available actions:

- **rotate_gimbal**: Rotate the gimbal to a specific position
- **reset_gimbal**: Reset the gimbal to its default position
- **set_gimbal_mode**: Set gimbal mode (FREE, FPV, YAW_FOLLOW, PITCH_FOLLOW, YAW_PITCH_FOLLOW)

Example waypoint with gimbal actions:

```json
{
  "latitude": 37.7749,
  "longitude": -122.4194,
  "altitude": 50.0,
  "actions": [
    {"action": "rotate_gimbal", "pitch": -90, "roll": 0, "yaw": 0},
    {"action": "take_photo"}
  ]
}
```

## Best Practices

### Mission Planning

1. **Safety First**: Always plan missions with safety in mind. Avoid obstacles and restricted areas.
2. **Battery Life**: Consider battery consumption when planning missions. Add a safety margin to ensure the drone can return home.
3. **Altitude Changes**: Gradual altitude changes are safer and more efficient than abrupt changes.
4. **Waypoint Spacing**: Keep waypoints reasonably spaced (at least 5 meters apart).
5. **Test in Simulator**: Test complex missions in a simulator before flying with a real drone.

### Camera and Gimbal

1. **Stabilization Time**: Allow the gimbal to stabilize before taking photos (1-2 seconds).
2. **Photo Interval**: When taking multiple photos, allow sufficient time between shots (at least 2 seconds).
3. **Video Recording**: Start recording before critical segments and stop after to avoid missing important footage.

### Performance Considerations

1. **Number of Waypoints**: Keep the number of waypoints reasonable (under 100 for most missions).
2. **Action Complexity**: Too many actions at a single waypoint can cause delays.
3. **Flight Speed**: Adjust flight speed based on mission objectives (slower for detailed inspection, faster for covering large areas).

## Examples

### Surveillance Mission

```json
{
  "mission_name": "Building Surveillance",
  "mission_type": "waypoint",
  "speed": 3.0,
  "finish_action": "go_home",
  "waypoints": [
    {
      "latitude": 37.7749,
      "longitude": -122.4194,
      "altitude": 50.0,
      "actions": [
        {"action": "rotate_gimbal", "pitch": -30, "yaw": 0},
        {"action": "take_photo"}
      ]
    },
    {
      "latitude": 37.7750,
      "longitude": -122.4194,
      "altitude": 50.0,
      "actions": [
        {"action": "rotate_gimbal", "pitch": -45, "yaw": 0},
        {"action": "take_photo"}
      ]
    },
    {
      "latitude": 37.7750,
      "longitude": -122.4195,
      "altitude": 50.0,
      "actions": [
        {"action": "rotate_gimbal", "pitch": -60, "yaw": 0},
        {"action": "take_photo"}
      ]
    },
    {
      "latitude": 37.7749,
      "longitude": -122.4195,
      "altitude": 50.0,
      "actions": [
        {"action": "rotate_gimbal", "pitch": -90, "yaw": 0},
        {"action": "take_photo"}
      ]
    }
  ]
}
```

## Troubleshooting

### Common Issues

1. **Mission Won't Start**: Ensure the drone has a good GPS signal and sufficient battery.
2. **Waypoint Skipping**: Check that waypoints are not too close together.
3. **Camera Actions Not Working**: Verify that the camera is properly initialized and in the correct mode.
4. **Gimbal Not Responding**: Check gimbal calibration and ensure it's not in an error state.

### Error Recovery

1. **Return to Home**: If a mission encounters an error, the drone will typically attempt to return to home.
2. **Manual Intervention**: Be prepared to take manual control if necessary.
3. **Mission Resumption**: Some missions can be paused and resumed if interrupted.

For more information, refer to the [DJI SDK Integration Guide](dji_sdk_integration.md) and the [DJI SDK Documentation](https://developer.dji.com/mobile-sdk/documentation/introduction/index.html).
