"""
DJI SDK Demo

This script demonstrates how to use the DJI SDK adapter in Bulo.Cloud Sentinel.
It provides a comprehensive interface to all DJI SDK features including:
- Basic flight control (takeoff, land, goto, return to home)
- Camera control (photo, video, settings)
- Gimbal control (rotation, modes)
- Mission control (waypoint, hotpoint, follow-me, timeline)
- Telemetry monitoring
"""

import asyncio
import logging
import argparse
import json
import time
import sys
import os
import math
from typing import Dict, Any, List, Optional
from enum import Enum

# Add project root to path to ensure imports work
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from dronecore.adapter_factory import AdapterFactory


def redact_sensitive_data(data):
    """Redact sensitive data from logs."""
    if isinstance(data, dict):
        redacted = {}
        for key, value in data.items():
            if any(pattern in key.lower() for pattern in ["password", "token", "secret", "key", "auth", "credential"]):
                redacted[key] = "***REDACTED***"
            else:
                redacted[key] = redact_sensitive_data(value)
        return redacted
    elif isinstance(data, list):
        return [redact_sensitive_data(item) for item in data]
    elif isinstance(data, str) and len(data) > 20:
        # Potentially sensitive long string
        if any(pattern in data.lower() for pattern in ["password", "token", "secret", "key", "auth", "credential"]):
            return "***REDACTED***"
    return data

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# Define command categories for better organization
class CommandCategory(Enum):
    STATUS = "status"
    FLIGHT = "flight"
    CAMERA = "camera"
    GIMBAL = "gimbal"
    MISSION = "mission"

async def main():
    """Main function to demonstrate DJI SDK adapter usage."""
    parser = argparse.ArgumentParser(description="DJI SDK Demo")

    # Connection parameters
    parser.add_argument("--app-id", type=str, help="DJI Developer App ID")
    parser.add_argument("--app-key", type=str, help="DJI Developer App Key")
    parser.add_argument("--connection-type", type=str, default="USB",
                        choices=["USB", "WIFI", "BRIDGE"],
                        help="Connection type (USB, WIFI, BRIDGE)")
    parser.add_argument("--drone-model", type=str, default="Mavic 3",
                        help="DJI drone model (e.g., Mavic 3, Phantom 4, M300 RTK)")
    parser.add_argument("--serial-number", type=str, help="Drone serial number (optional)")
    parser.add_argument("--bridge-ip", type=str, help="Bridge IP address (required for BRIDGE connection)")
    parser.add_argument("--bridge-port", type=int, help="Bridge port (required for BRIDGE connection)")

    # Command category
    parser.add_argument("--category", type=str, default="status",
                        choices=[c.value for c in CommandCategory],
                        help="Command category")

    # Status commands
    parser.add_argument("--telemetry-count", type=int, default=10,
                        help="Number of telemetry updates to display")

    # Flight commands
    parser.add_argument("--flight-command", type=str,
                        choices=["takeoff", "land", "goto", "return_to_home", "stop"],
                        help="Flight control command")
    parser.add_argument("--latitude", type=float, help="Latitude for goto command")
    parser.add_argument("--longitude", type=float, help="Longitude for goto command")
    parser.add_argument("--altitude", type=float, help="Altitude for goto command")
    parser.add_argument("--speed", type=float, default=5.0, help="Speed in m/s for goto command")

    # Camera commands
    parser.add_argument("--camera-command", type=str,
                        choices=["take_photo", "start_recording", "stop_recording",
                                "set_camera_mode", "set_photo_mode", "set_video_resolution"],
                        help="Camera control command")
    parser.add_argument("--camera-mode", type=str,
                        choices=["PHOTO", "VIDEO", "PLAYBACK"],
                        help="Camera mode for set_camera_mode command")
    parser.add_argument("--photo-mode", type=str,
                        choices=["SINGLE", "HDR", "BURST", "AEB", "INTERVAL", "PANO", "EHDR"],
                        help="Photo mode for set_photo_mode command")
    parser.add_argument("--video-resolution", type=str,
                        choices=["RES_720P", "RES_1080P", "RES_2_7K", "RES_4K"],
                        help="Video resolution for set_video_resolution command")

    # Gimbal commands
    parser.add_argument("--gimbal-command", type=str,
                        choices=["rotate_gimbal", "reset_gimbal", "set_gimbal_mode"],
                        help="Gimbal control command")
    parser.add_argument("--gimbal-pitch", type=float, help="Gimbal pitch angle (-90 to 30)")
    parser.add_argument("--gimbal-roll", type=float, help="Gimbal roll angle (-45 to 45)")
    parser.add_argument("--gimbal-yaw", type=float, help="Gimbal yaw angle (-320 to 320)")
    parser.add_argument("--gimbal-mode", type=str,
                        choices=["FREE", "FPV", "YAW_FOLLOW", "PITCH_FOLLOW", "YAW_PITCH_FOLLOW"],
                        help="Gimbal mode for set_gimbal_mode command")

    # Mission commands
    parser.add_argument("--mission-command", type=str,
                        choices=["start_waypoint_mission", "stop_waypoint_mission",
                                "pause_waypoint_mission", "resume_waypoint_mission",
                                "start_hotpoint_mission", "stop_hotpoint_mission",
                                "start_follow_me_mission", "stop_follow_me_mission"],
                        help="Mission control command")
    parser.add_argument("--waypoints-file", type=str,
                        help="JSON file containing waypoints for waypoint mission")
    parser.add_argument("--hotpoint-latitude", type=float,
                        help="Latitude for hotpoint mission")
    parser.add_argument("--hotpoint-longitude", type=float,
                        help="Longitude for hotpoint mission")
    parser.add_argument("--hotpoint-altitude", type=float,
                        help="Altitude for hotpoint mission")
    parser.add_argument("--hotpoint-radius", type=float, default=10.0,
                        help="Radius for hotpoint mission")
    parser.add_argument("--mission-speed", type=float, default=5.0,
                        help="Speed for mission in m/s")

    args = parser.parse_args()

    # Create connection parameters
    connection_params = {
        "app_id": args.app_id or "your_app_id_here",
        "app_key": args.app_key or "your_app_key_here",
        "connection_type": args.connection_type,
        "drone_model": args.drone_model,
        "enable_virtual_stick": True,
        "enable_camera": True,
        "enable_gimbal": True,
        "enable_waypoint": True,
        "enable_hotpoint": True,
        "enable_follow_me": True,
        "enable_timeline": True,
        "enable_hd_video": True,
    }

    if args.serial_number:
        connection_params["serial_number"] = args.serial_number

    if args.connection_type == "BRIDGE":
        if not args.bridge_ip or not args.bridge_port:
            logger.error("Bridge IP and port are required for BRIDGE connection")
            return
        connection_params["bridge_ip"] = args.bridge_ip
        connection_params["bridge_port"] = args.bridge_port

    # Create DJI adapter
    adapter = AdapterFactory.create_adapter("dji", connection_params)
    if not adapter:
        logger.error("Failed to create DJI adapter")
        return

    try:
        # Connect to the drone
        logger.info("Connecting to DJI drone...")
        connected = await adapter.connect()
        if not connected:
            logger.error("Failed to connect to DJI drone")
            return

        logger.info("Connected to DJI drone")

        # Execute command based on category
        category = CommandCategory(args.category)

        # STATUS category - display telemetry data
        if category == CommandCategory.STATUS:
            logger.info(f"Getting telemetry data for {args.telemetry_count} iterations...")
            for i in range(args.telemetry_count):
                telemetry = adapter.receive_telemetry()
                logger.info(f"Telemetry update {i+1}/{args.telemetry_count}:")
                logger.info(f"  Battery: {telemetry['battery']['percent']}%")
                logger.info(f"  Position: Lat={telemetry['position']['latitude']}, Lon={telemetry['position']['longitude']}, Alt={telemetry['position']['altitude']}m")
                logger.info(f"  Flight mode: {telemetry['flight_status']['mode']}")
                logger.info(f"  Status: {telemetry['flight_status']['status']}")

                # Print full telemetry data if requested
                if i == 0:  # Only print full data for first iteration to avoid flooding console
                    logger.info(f"Full telemetry data: {json.dumps(telemetry, indent=2)}")

                await asyncio.sleep(1)

            # Get additional state information
            if adapter.enable_camera:
                camera_state = await adapter.get_camera_state()
                logger.info(f"Camera state: {json.dumps(camera_state, indent=2)}")

            if adapter.enable_gimbal:
                gimbal_state = await adapter.get_gimbal_state()
                logger.info(f"Gimbal state: {json.dumps(gimbal_state, indent=2)}")

            if adapter.enable_waypoint:
                waypoint_mission_state = await adapter.get_waypoint_mission_state()
                logger.info(f"Waypoint mission state: {json.dumps(waypoint_mission_state, indent=2)}")

        # FLIGHT category - execute flight control commands
        elif category == CommandCategory.FLIGHT:
            if not args.flight_command:
                logger.error("Flight command is required for flight category")
                return

            if args.flight_command == "takeoff":
                logger.info("Executing takeoff command")
                command = {
                    "command": "takeoff",
                    "parameters": {}
                }
                result = adapter.send_command(command)
                logger.info(f"Takeoff command result: {result}")

                # Wait for takeoff to complete
                logger.info("Waiting for takeoff to complete...")
                for _ in range(5):
                    telemetry = adapter.receive_telemetry()
                    logger.info(f"  Altitude: {telemetry['position']['altitude']}m")
                    await asyncio.sleep(1)

            elif args.flight_command == "land":
                logger.info("Executing land command")
                command = {
                    "command": "land",
                    "parameters": {}
                }
                result = adapter.send_command(command)
                logger.info(f"Land command result: {result}")

                # Wait for landing to complete
                logger.info("Waiting for landing to complete...")
                for _ in range(5):
                    telemetry = adapter.receive_telemetry()
                    logger.info(f"  Altitude: {telemetry['position']['altitude']}m")
                    await asyncio.sleep(1)

            elif args.flight_command == "goto":
                if not args.latitude or not args.longitude or not args.altitude:
                    logger.error("Latitude, longitude, and altitude are required for goto command")
                    return

                logger.info(f"Executing goto command: lat={args.latitude}, lon={args.longitude}, alt={args.altitude}, speed={args.speed}")
                command = {
                    "command": "goto",
                    "parameters": {
                        "latitude": args.latitude,
                        "longitude": args.longitude,
                        "altitude": args.altitude,
                        "speed": args.speed
                    }
                }
                result = adapter.send_command(command)
                logger.info(f"Goto command result: {result}")

                # Monitor progress
                logger.info("Monitoring goto progress...")
                for _ in range(10):
                    telemetry = adapter.receive_telemetry()
                    current_lat = telemetry['position']['latitude']
                    current_lon = telemetry['position']['longitude']
                    current_alt = telemetry['position']['altitude']
                    logger.info(f"  Position: Lat={current_lat}, Lon={current_lon}, Alt={current_alt}m")
                    await asyncio.sleep(1)

            elif args.flight_command == "return_to_home":
                logger.info("Executing return to home command")
                command = {
                    "command": "return_to_home",
                    "parameters": {}
                }
                result = adapter.send_command(command)
                logger.info(f"Return to home command result: {result}")

                # Monitor progress
                logger.info("Monitoring return to home progress...")
                for _ in range(10):
                    telemetry = adapter.receive_telemetry()
                    home_distance = telemetry['flight_status']['home_distance']
                    logger.info(f"  Distance to home: {home_distance}m")
                    await asyncio.sleep(1)

            elif args.flight_command == "stop":
                logger.info("Executing stop command")
                command = {
                    "command": "stop",
                    "parameters": {}
                }
                result = adapter.send_command(command)
                logger.info(f"Stop command result: {result}")

        # CAMERA category - execute camera control commands
        elif category == CommandCategory.CAMERA:
            if not args.camera_command:
                logger.error("Camera command is required for camera category")
                return

            if not adapter.enable_camera:
                logger.error("Camera is not enabled in the adapter configuration")
                return

            if args.camera_command == "take_photo":
                logger.info("Taking photo")
                command = {
                    "command": "take_photo",
                    "parameters": {}
                }
                result = adapter.send_command(command)
                logger.info(f"Take photo command result: {result}")

            elif args.camera_command == "start_recording":
                logger.info("Starting video recording")
                command = {
                    "command": "start_recording",
                    "parameters": {}
                }
                result = adapter.send_command(command)
                logger.info(f"Start recording command result: {result}")

            elif args.camera_command == "stop_recording":
                logger.info("Stopping video recording")
                command = {
                    "command": "stop_recording",
                    "parameters": {}
                }
                result = adapter.send_command(command)
                logger.info(f"Stop recording command result: {result}")

            elif args.camera_command == "set_camera_mode":
                if not args.camera_mode:
                    logger.error("Camera mode is required for set_camera_mode command")
                    return

                logger.info(f"Setting camera mode to {args.camera_mode}")
                command = {
                    "command": "set_camera_mode",
                    "parameters": {
                        "mode": args.camera_mode
                    }
                }
                result = adapter.send_command(command)
                logger.info(f"Set camera mode command result: {result}")

            elif args.camera_command == "set_photo_mode":
                if not args.photo_mode:
                    logger.error("Photo mode is required for set_photo_mode command")
                    return

                logger.info(f"Setting photo mode to {args.photo_mode}")
                command = {
                    "command": "set_photo_mode",
                    "parameters": {
                        "mode": args.photo_mode
                    }
                }
                result = adapter.send_command(command)
                logger.info(f"Set photo mode command result: {result}")

            elif args.camera_command == "set_video_resolution":
                if not args.video_resolution:
                    logger.error("Video resolution is required for set_video_resolution command")
                    return

                logger.info(f"Setting video resolution to {args.video_resolution}")
                command = {
                    "command": "set_video_resolution",
                    "parameters": {
                        "resolution": args.video_resolution
                    }
                }
                result = adapter.send_command(command)
                logger.info(f"Set video resolution command result: {result}")

            # Get camera state after command execution
            camera_state = await adapter.get_camera_state()
            logger.info(f"Camera state after command: {json.dumps(camera_state, indent=2)}")

        # GIMBAL category - execute gimbal control commands
        elif category == CommandCategory.GIMBAL:
            if not args.gimbal_command:
                logger.error("Gimbal command is required for gimbal category")
                return

            if not adapter.enable_gimbal:
                logger.error("Gimbal is not enabled in the adapter configuration")
                return

            if args.gimbal_command == "rotate_gimbal":
                if args.gimbal_pitch is None and args.gimbal_roll is None and args.gimbal_yaw is None:
                    logger.error("At least one of pitch, roll, or yaw is required for rotate_gimbal command")
                    return

                # Use default values of 0 for any unspecified angles
                pitch = args.gimbal_pitch if args.gimbal_pitch is not None else 0.0
                roll = args.gimbal_roll if args.gimbal_roll is not None else 0.0
                yaw = args.gimbal_yaw if args.gimbal_yaw is not None else 0.0

                logger.info(f"Rotating gimbal: pitch={pitch}, roll={roll}, yaw={yaw}")
                command = {
                    "command": "rotate_gimbal",
                    "parameters": {
                        "pitch": pitch,
                        "roll": roll,
                        "yaw": yaw,
                        "duration": 1.0  # 1 second duration
                    }
                }
                result = adapter.send_command(command)
                logger.info(f"Rotate gimbal command result: {result}")

            elif args.gimbal_command == "reset_gimbal":
                logger.info("Resetting gimbal")
                command = {
                    "command": "reset_gimbal",
                    "parameters": {}
                }
                result = adapter.send_command(command)
                logger.info(f"Reset gimbal command result: {result}")

            elif args.gimbal_command == "set_gimbal_mode":
                if not args.gimbal_mode:
                    logger.error("Gimbal mode is required for set_gimbal_mode command")
                    return

                logger.info(f"Setting gimbal mode to {args.gimbal_mode}")
                command = {
                    "command": "set_gimbal_mode",
                    "parameters": {
                        "mode": args.gimbal_mode
                    }
                }
                result = adapter.send_command(command)
                logger.info(f"Set gimbal mode command result: {result}")

            # Get gimbal state after command execution
            gimbal_state = await adapter.get_gimbal_state()
            logger.info(f"Gimbal state after command: {json.dumps(gimbal_state, indent=2)}")

        # MISSION category - execute mission control commands
        elif category == CommandCategory.MISSION:
            if not args.mission_command:
                logger.error("Mission command is required for mission category")
                return

            # Waypoint mission commands
            if args.mission_command == "start_waypoint_mission":
                if not adapter.enable_waypoint:
                    logger.error("Waypoint mission is not enabled in the adapter configuration")
                    return

                # Load waypoints from file if provided, otherwise use default waypoints
                waypoints = []
                if args.waypoints_file:
                    try:
                        with open(args.waypoints_file, 'r') as f:
                            waypoints = json.load(f)
                    except Exception as e:
                        logger.error(f"Error loading waypoints file: {str(e)}")
                        return
                else:
                    # Use default waypoints around the current position
                    telemetry = adapter.receive_telemetry()
                    current_lat = telemetry['position']['latitude']
                    current_lon = telemetry['position']['longitude']
                    current_alt = telemetry['position']['altitude']

                    # Create a simple square pattern
                    waypoints = [
                        {
                            "latitude": current_lat + 0.0001,
                            "longitude": current_lon,
                            "altitude": current_alt + 10,
                            "heading": 0,
                            "stay_time": 0
                        },
                        {
                            "latitude": current_lat + 0.0001,
                            "longitude": current_lon + 0.0001,
                            "altitude": current_alt + 15,
                            "heading": 90,
                            "stay_time": 0
                        },
                        {
                            "latitude": current_lat,
                            "longitude": current_lon + 0.0001,
                            "altitude": current_alt + 20,
                            "heading": 180,
                            "stay_time": 0
                        },
                        {
                            "latitude": current_lat,
                            "longitude": current_lon,
                            "altitude": current_alt + 10,
                            "heading": 270,
                            "stay_time": 0
                        }
                    ]

                logger.info(f"Starting waypoint mission with {len(waypoints)} waypoints")
                command = {
                    "command": "start_waypoint_mission",
                    "parameters": {
                        "waypoints": waypoints,
                        "speed": args.mission_speed,
                        "finish_action": "go_home",
                        "heading_mode": "auto"
                    }
                }
                result = adapter.send_command(command)
                logger.info(f"Start waypoint mission command result: {result}")

                # Monitor mission progress
                logger.info("Monitoring waypoint mission progress...")
                for _ in range(10):
                    mission_state = await adapter.get_waypoint_mission_state()
                    logger.info(f"  Mission state: {json.dumps(mission_state, indent=2)}")
                    await asyncio.sleep(2)

            elif args.mission_command == "stop_waypoint_mission":
                if not adapter.enable_waypoint:
                    logger.error("Waypoint mission is not enabled in the adapter configuration")
                    return

                logger.info("Stopping waypoint mission")
                command = {
                    "command": "stop_waypoint_mission",
                    "parameters": {}
                }
                result = adapter.send_command(command)
                logger.info(f"Stop waypoint mission command result: {result}")

            elif args.mission_command == "pause_waypoint_mission":
                if not adapter.enable_waypoint:
                    logger.error("Waypoint mission is not enabled in the adapter configuration")
                    return

                logger.info("Pausing waypoint mission")
                command = {
                    "command": "pause_waypoint_mission",
                    "parameters": {}
                }
                result = adapter.send_command(command)
                logger.info(f"Pause waypoint mission command result: {result}")

            elif args.mission_command == "resume_waypoint_mission":
                if not adapter.enable_waypoint:
                    logger.error("Waypoint mission is not enabled in the adapter configuration")
                    return

                logger.info("Resuming waypoint mission")
                command = {
                    "command": "resume_waypoint_mission",
                    "parameters": {}
                }
                result = adapter.send_command(command)
                logger.info(f"Resume waypoint mission command result: {result}")

            # Hotpoint mission commands
            elif args.mission_command == "start_hotpoint_mission":
                if not adapter.enable_hotpoint:
                    logger.error("Hotpoint mission is not enabled in the adapter configuration")
                    return

                # Use provided coordinates or current position
                if args.hotpoint_latitude and args.hotpoint_longitude and args.hotpoint_altitude:
                    latitude = args.hotpoint_latitude
                    longitude = args.hotpoint_longitude
                    altitude = args.hotpoint_altitude
                else:
                    # Use current position
                    telemetry = adapter.receive_telemetry()
                    latitude = telemetry['position']['latitude']
                    longitude = telemetry['position']['longitude']
                    altitude = telemetry['position']['altitude']

                logger.info(f"Starting hotpoint mission: lat={latitude}, lon={longitude}, alt={altitude}, radius={args.hotpoint_radius}")
                command = {
                    "command": "start_hotpoint_mission",
                    "parameters": {
                        "latitude": latitude,
                        "longitude": longitude,
                        "altitude": altitude,
                        "radius": args.hotpoint_radius,
                        "angular_speed": 15.0,  # degrees per second
                        "is_clockwise": True
                    }
                }
                result = adapter.send_command(command)
                logger.info(f"Start hotpoint mission command result: {result}")

                # Monitor for a few seconds
                logger.info("Monitoring hotpoint mission...")
                for _ in range(10):
                    telemetry = adapter.receive_telemetry()
                    logger.info(f"  Flight mode: {telemetry['flight_status']['mode']}")
                    await asyncio.sleep(1)

            elif args.mission_command == "stop_hotpoint_mission":
                if not adapter.enable_hotpoint:
                    logger.error("Hotpoint mission is not enabled in the adapter configuration")
                    return

                logger.info("Stopping hotpoint mission")
                command = {
                    "command": "stop_hotpoint_mission",
                    "parameters": {}
                }
                result = adapter.send_command(command)
                logger.info(f"Stop hotpoint mission command result: {result}")

            # Follow me mission commands
            elif args.mission_command == "start_follow_me_mission":
                if not adapter.enable_follow_me:
                    logger.error("Follow me mission is not enabled in the adapter configuration")
                    return

                # Use current position as starting point
                telemetry = adapter.receive_telemetry()
                latitude = telemetry['position']['latitude']
                longitude = telemetry['position']['longitude']

                logger.info("Starting follow me mission at the current location.")
                command = {
                    "command": "start_follow_me_mission",
                    "parameters": {
                        "latitude": latitude,
                        "longitude": longitude,
                        "altitude": 10.0,  # meters above target
                        "follow_distance": 5.0,  # meters behind target
                        "follow_height": 10.0  # meters above target
                    }
                }
                result = adapter.send_command(command)
                logger.info(f"Start follow me mission command result: {result}")

                # Simulate target movement
                logger.info("Simulating target movement...")
                for i in range(5):
                    # Update target position (moving in a small circle)
                    new_lat = latitude + 0.00001 * math.cos(i * 0.5)
                    new_lon = longitude + 0.00001 * math.sin(i * 0.5)

                    logger.info(f"Updating target position: lat={new_lat}, lon={new_lon}")
                    command = {
                        "command": "update_follow_me_target",
                        "parameters": {
                            "latitude": new_lat,
                            "longitude": new_lon
                        }
                    }
                    adapter.send_command(command)

                    # Monitor drone position
                    telemetry = adapter.receive_telemetry()
                    drone_lat = telemetry['position']['latitude']
                    drone_lon = telemetry['position']['longitude']
                    logger.info(f"  Drone position: lat={drone_lat}, lon={drone_lon}")

                    await asyncio.sleep(2)

            elif args.mission_command == "stop_follow_me_mission":
                if not adapter.enable_follow_me:
                    logger.error("Follow me mission is not enabled in the adapter configuration")
                    return

                logger.info("Stopping follow me mission")
                command = {
                    "command": "stop_follow_me_mission",
                    "parameters": {}
                }
                result = adapter.send_command(command)
                logger.info(f"Stop follow me mission command result: {result}")

        else:
                    logger.info(redact_sensitive_data(f"  Drone position: lat={drone_lat}, lon={drone_lon}"))
            logger.error(f"Unknown command category: {args.category}")

    finally:
        # Disconnect from the drone
        logger.info("Disconnecting from DJI drone...")
        await adapter.disconnect()
        logger.info("Disconnected from DJI drone")

if __name__ == "__main__":
    asyncio.run(main())
