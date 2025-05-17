#!/usr/bin/env python3
"""
DJI Complex Mission Example

This script demonstrates a complex real-world mission scenario using the DJI SDK adapter
in Bulo.Cloud Sentinel. It combines multiple features including:
- Automated takeoff and landing
- Waypoint navigation with camera actions
- Gimbal control for camera positioning
- Telemetry monitoring and visualization
- Error handling and mission recovery
- Integration with the dock system (if available)

This example is designed to showcase how to use the DJI SDK adapter in a real-world
surveillance application.
"""

import asyncio
import logging
import argparse
import json
import time
import sys
import os
import math
import signal
import datetime
from typing import Dict, Any, List, Optional, Tuple
from enum import Enum
import traceback

# Add project root to path to ensure imports work
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from dronecore.adapter_factory import AdapterFactory

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# Mission states
class MissionState(Enum):
    IDLE = "idle"
    CONNECTING = "connecting"
    CONNECTED = "connected"
    TAKING_OFF = "taking_off"
    EXECUTING_WAYPOINTS = "executing_waypoints"
    RETURNING_HOME = "returning_home"
    LANDING = "landing"
    COMPLETED = "completed"
    ERROR = "error"

class ComplexMission:
    """Complex mission controller for DJI drones."""

    def __init__(self, connection_params: Dict[str, Any], waypoints_file: str = None):
        """
        Initialize the complex mission controller.

        Args:
            connection_params: Connection parameters for the DJI SDK adapter
            waypoints_file: Optional path to a JSON file containing waypoints
        """
        self.connection_params = connection_params
        self.waypoints_file = waypoints_file
        self.adapter = None
        self.state = MissionState.IDLE
        self.waypoints = []
        self.current_waypoint_index = 0
        self.telemetry_data = {}
        self.mission_start_time = None
        self.mission_end_time = None
        self.error_count = 0
        self.max_errors = 3
        self.running = False
        self.telemetry_task = None
        self.mission_log = []

        # Register signal handlers for graceful shutdown
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

    def _signal_handler(self, sig, frame):
        """Handle termination signals."""
        logger.info(f"Received signal {sig}, initiating safe shutdown...")
        asyncio.create_task(self.stop())

    async def initialize(self) -> bool:
        """
        Initialize the mission.

        Returns:
            bool: True if initialization was successful, False otherwise
        """
        try:
            # Create DJI adapter
            self.state = MissionState.CONNECTING
            self.adapter = AdapterFactory.create_adapter("dji", self.connection_params)
            if not self.adapter:
                logger.error("Failed to create DJI adapter")
                self.state = MissionState.ERROR
                return False

            # Connect to the drone
            logger.info("Connecting to DJI drone...")
            connected = await self.adapter.connect()
            if not connected:
                logger.error("Failed to connect to DJI drone")
                self.state = MissionState.ERROR
                return False

            logger.info("Connected to DJI drone")
            self.state = MissionState.CONNECTED

            # Load waypoints
            if not await self._load_waypoints():
                logger.error("Failed to load waypoints")
                self.state = MissionState.ERROR
                return False

            # Start telemetry monitoring
            self.running = True
            self.telemetry_task = asyncio.create_task(self._telemetry_loop())

            return True
        except Exception as e:
            logger.error(f"Error initializing mission: {str(e)}")
            logger.error(traceback.format_exc())
            self.state = MissionState.ERROR
            return False

    async def _load_waypoints(self) -> bool:
        """
        Load waypoints from file or generate default waypoints.

        Returns:
            bool: True if waypoints were loaded successfully, False otherwise
        """
        try:
            if self.waypoints_file:
                # Load waypoints from file
                logger.info(f"Loading waypoints from {self.waypoints_file}")
                with open(self.waypoints_file, 'r') as f:
                    self.waypoints = json.load(f)
            else:
                # Generate default waypoints based on current position
                logger.info("Generating default waypoints")
                telemetry = self.adapter.receive_telemetry()
                current_lat = telemetry['position']['latitude']
                current_lon = telemetry['position']['longitude']
                current_alt = telemetry['position']['altitude']

                # Create a surveillance pattern with camera actions
                self.waypoints = [
                    {
                        "latitude": current_lat + 0.0001,
                        "longitude": current_lon,
                        "altitude": current_alt + 20,
                        "heading": 0,
                        "stay_time": 5,
                        "actions": [
                            {"action": "rotate_gimbal", "pitch": -30, "yaw": 0},
                            {"action": "take_photo"}
                        ]
                    },
                    {
                        "latitude": current_lat + 0.0001,
                        "longitude": current_lon + 0.0001,
                        "altitude": current_alt + 25,
                        "heading": 90,
                        "stay_time": 5,
                        "actions": [
                            {"action": "rotate_gimbal", "pitch": -45, "yaw": 0},
                            {"action": "start_recording"}
                        ]
                    },
                    {
                        "latitude": current_lat,
                        "longitude": current_lon + 0.0001,
                        "altitude": current_alt + 30,
                        "heading": 180,
                        "stay_time": 10,
                        "actions": [
                            {"action": "rotate_gimbal", "pitch": -90, "yaw": 0}
                        ]
                    },
                    {
                        "latitude": current_lat,
                        "longitude": current_lon,
                        "altitude": current_alt + 20,
                        "heading": 270,
                        "stay_time": 5,
                        "actions": [
                            {"action": "rotate_gimbal", "pitch": -30, "yaw": 0},
                            {"action": "stop_recording"},
                            {"action": "take_photo"}
                        ]
                    }
                ]

            logger.info(f"Loaded {len(self.waypoints)} waypoints")
            return True
        except Exception as e:
            logger.error(f"Error loading waypoints: {str(e)}")
            logger.error(traceback.format_exc())
            return False

    async def _telemetry_loop(self):
        """Background task for monitoring telemetry data."""
        while self.running:
            try:
                # Get telemetry data
                self.telemetry_data = self.adapter.receive_telemetry()

                # Log critical information
                battery = self.telemetry_data['battery']['percent']
                position = self.telemetry_data['position']
                flight_status = self.telemetry_data['flight_status']

                # Check for low battery
                if battery <= self.adapter._critical_battery_warning:
                    logger.warning(f"CRITICAL BATTERY LEVEL: {battery}%")
                    if self.state not in [MissionState.RETURNING_HOME, MissionState.LANDING]:
                        logger.warning("Initiating emergency return to home due to critical battery")
                        await self._emergency_return_home()
                elif battery <= self.adapter._low_battery_warning:
                    logger.warning(f"LOW BATTERY WARNING: {battery}%")

                # Sleep briefly
                await asyncio.sleep(1)
            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error(f"Error in telemetry loop: {str(e)}")
                await asyncio.sleep(1)

    async def _execute_waypoint_actions(self, waypoint: Dict[str, Any]) -> bool:
        """
        Execute actions associated with a waypoint.

        Args:
            waypoint: Waypoint dictionary with actions

        Returns:
            bool: True if all actions were executed successfully, False otherwise
        """
        if "actions" not in waypoint:
            return True

        for action in waypoint["actions"]:
            action_type = action.get("action", "")
            logger.info(f"Executing waypoint action: {action_type}")

            try:
                if action_type == "take_photo":
                    command = {
                        "command": "take_photo",
                        "parameters": {}
                    }
                    result = self.adapter.send_command(command)
                    logger.info(f"Take photo result: {result}")

                elif action_type == "start_recording":
                    command = {
                        "command": "start_recording",
                        "parameters": {}
                    }
                    result = self.adapter.send_command(command)
                    logger.info(f"Start recording result: {result}")

                elif action_type == "stop_recording":
                    command = {
                        "command": "stop_recording",
                        "parameters": {}
                    }
                    result = self.adapter.send_command(command)
                    logger.info(f"Stop recording result: {result}")

                elif action_type == "rotate_gimbal":
                    pitch = action.get("pitch", 0)
                    roll = action.get("roll", 0)
                    yaw = action.get("yaw", 0)
                    command = {
                        "command": "rotate_gimbal",
                        "parameters": {
                            "pitch": pitch,
                            "roll": roll,
                            "yaw": yaw,
                            "duration": 1.0
                        }
                    }
                    result = self.adapter.send_command(command)
                    logger.info(f"Rotate gimbal result: {result}")

                # Wait a moment for action to complete
                await asyncio.sleep(1)

            except Exception as e:
                logger.error(f"Error executing waypoint action {action_type}: {str(e)}")
                return False

        return True

    async def _emergency_return_home(self) -> bool:
        """
        Initiate emergency return to home procedure.

        Returns:
            bool: True if return to home was initiated successfully, False otherwise
        """
        try:
            logger.warning("EMERGENCY RETURN TO HOME INITIATED")
            self.state = MissionState.RETURNING_HOME

            # Log the event
            self.mission_log.append({
                "timestamp": datetime.datetime.now().isoformat(),
                "event": "emergency_return_home",
                "reason": "critical_battery" if self.telemetry_data['battery']['percent'] <= self.adapter._critical_battery_warning else "manual"
            })

            # Send return to home command
            command = {
                "command": "return_to_home",
                "parameters": {}
            }
            result = self.adapter.send_command(command)
            logger.info(f"Return to home command result: {result}")

            return result
        except Exception as e:
            logger.error(f"Error initiating emergency return to home: {str(e)}")
            logger.error(traceback.format_exc())
            self.state = MissionState.ERROR
            return False

    async def execute_mission(self) -> bool:
        """
        Execute the complete mission.

        Returns:
            bool: True if mission was completed successfully, False otherwise
        """
        try:
            # Start mission
            self.mission_start_time = datetime.datetime.now()
            logger.info(f"Starting mission at {self.mission_start_time.isoformat()}")

            # Take off
            if not await self._take_off():
                logger.error("Failed to take off")
                return False

            # Execute waypoints
            if not await self._execute_waypoints():
                logger.error("Failed to execute waypoints")
                # Try to return home safely
                await self._emergency_return_home()
                return False

            # Return to home
            if not await self._return_to_home():
                logger.error("Failed to return to home")
                return False

            # Land
            if not await self._land():
                logger.error("Failed to land")
                return False

            # Mission completed
            self.mission_end_time = datetime.datetime.now()
            duration = (self.mission_end_time - self.mission_start_time).total_seconds()
            logger.info(f"Mission completed successfully in {duration:.1f} seconds")
            self.state = MissionState.COMPLETED

            # Log mission completion
            self.mission_log.append({
                "timestamp": self.mission_end_time.isoformat(),
                "event": "mission_completed",
                "duration": duration
            })

            return True
        except Exception as e:
            logger.error(f"Error executing mission: {str(e)}")
            logger.error(traceback.format_exc())
            self.state = MissionState.ERROR

            # Log error
            self.mission_log.append({
                "timestamp": datetime.datetime.now().isoformat(),
                "event": "mission_error",
                "error": str(e)
            })

            # Try to return home safely
            await self._emergency_return_home()
            return False

    async def _take_off(self) -> bool:
        """
        Take off the drone.

        Returns:
            bool: True if takeoff was successful, False otherwise
        """
        try:
            logger.info("Taking off...")
            self.state = MissionState.TAKING_OFF

            # Send takeoff command
            command = {
                "command": "takeoff",
                "parameters": {}
            }
            result = self.adapter.send_command(command)
            logger.info(f"Takeoff command result: {result}")

            if not result:
                logger.error("Takeoff command failed")
                self.state = MissionState.ERROR
                return False

            # Wait for takeoff to complete
            logger.info("Waiting for takeoff to complete...")
            for _ in range(10):  # Wait up to 10 seconds
                telemetry = self.adapter.receive_telemetry()
                altitude = telemetry['position']['altitude']
                logger.info(f"Current altitude: {altitude}m")

                # Check if we've reached a safe altitude
                if altitude >= 3.0:  # 3 meters is typically safe
                    logger.info("Takeoff completed successfully")
                    return True

                await asyncio.sleep(1)

            logger.warning("Takeoff may not have completed, but continuing mission")
            return True
        except Exception as e:
            logger.error(f"Error during takeoff: {str(e)}")
            logger.error(traceback.format_exc())
            self.state = MissionState.ERROR
            return False

    async def _execute_waypoints(self) -> bool:
        """
        Execute the waypoint mission.

        Returns:
            bool: True if all waypoints were executed successfully, False otherwise
        """
        try:
            logger.info(f"Executing waypoint mission with {len(self.waypoints)} waypoints")
            self.state = MissionState.EXECUTING_WAYPOINTS

            # Execute each waypoint
            for i, waypoint in enumerate(self.waypoints):
                self.current_waypoint_index = i
                logger.info(f"Navigating to waypoint {i+1}/{len(self.waypoints)}")

                # Extract waypoint parameters
                latitude = waypoint["latitude"]
                longitude = waypoint["longitude"]
                altitude = waypoint["altitude"]
                heading = waypoint.get("heading", 0)
                stay_time = waypoint.get("stay_time", 0)

                # Navigate to waypoint
                command = {
                    "command": "goto",
                    "parameters": {
                        "latitude": latitude,
                        "longitude": longitude,
                        "altitude": altitude,
                        "speed": 5.0  # 5 m/s
                    }
                }
                result = self.adapter.send_command(command)
                logger.info(f"Goto command result: {result}")

                if not result:
                    logger.error(f"Failed to navigate to waypoint {i+1}")
                    self.error_count += 1
                    if self.error_count >= self.max_errors:
                        logger.error(f"Exceeded maximum error count ({self.max_errors}), aborting mission")
                        return False
                    continue

                # Wait for arrival at waypoint
                logger.info("Waiting for arrival at waypoint...")
                arrived = await self._wait_for_arrival(latitude, longitude, altitude)
                if not arrived:
                    logger.warning(f"May not have arrived at waypoint {i+1}, but continuing")

                # Execute waypoint actions
                logger.info(f"Executing actions for waypoint {i+1}")
                await self._execute_waypoint_actions(waypoint)

                # Stay at waypoint if specified
                if stay_time > 0:
                    logger.info(f"Staying at waypoint for {stay_time} seconds")
                    await asyncio.sleep(stay_time)

            logger.info("Waypoint mission completed successfully")
            return True
        except Exception as e:
            logger.error(f"Error executing waypoints: {str(e)}")
            logger.error(traceback.format_exc())
            self.state = MissionState.ERROR
            return False

    async def _wait_for_arrival(self, target_lat: float, target_lon: float, target_alt: float, timeout: int = 30) -> bool:
        """
        Wait for arrival at a waypoint.

        Args:
            target_lat: Target latitude
            target_lon: Target longitude
            target_alt: Target altitude
            timeout: Maximum time to wait in seconds

        Returns:
            bool: True if arrival was detected, False if timeout occurred
        """
        start_time = time.time()
        arrival_threshold = 2.0  # meters

        while time.time() - start_time < timeout:
            telemetry = self.adapter.receive_telemetry()
            current_lat = telemetry['position']['latitude']
            current_lon = telemetry['position']['longitude']
            current_alt = telemetry['position']['altitude']

            # Calculate distance (simplified)
            lat_diff = (target_lat - current_lat) * 111000  # 1 degree lat = ~111km
            lon_diff = (target_lon - current_lon) * 111000 * math.cos(math.radians(current_lat))
            alt_diff = target_alt - current_alt

            distance = math.sqrt(lat_diff**2 + lon_diff**2 + alt_diff**2)

            logger.info(f"Distance to waypoint: {distance:.2f}m")

            if distance < arrival_threshold:
                logger.info("Arrived at waypoint")
                return True

            await asyncio.sleep(1)

        logger.warning(f"Timeout waiting for arrival at waypoint after {timeout} seconds")
        return False

    async def _return_to_home(self) -> bool:
        """
        Return to home location.

        Returns:
            bool: True if return to home was successful, False otherwise
        """
        try:
            logger.info("Returning to home...")
            self.state = MissionState.RETURNING_HOME

            # Send return to home command
            command = {
                "command": "return_to_home",
                "parameters": {}
            }
            result = self.adapter.send_command(command)
            logger.info(f"Return to home command result: {result}")

            if not result:
                logger.error("Return to home command failed")
                self.state = MissionState.ERROR
                return False

            # Wait for return to home to complete
            logger.info("Waiting for return to home to complete...")
            for _ in range(30):  # Wait up to 30 seconds
                telemetry = self.adapter.receive_telemetry()
                home_distance = telemetry['flight_status']['home_distance']
                logger.info(f"Distance to home: {home_distance}m")

                if home_distance < 3.0:  # 3 meters is close enough
                    logger.info("Return to home completed successfully")
                    return True

                await asyncio.sleep(1)

            logger.warning("Return to home may not have completed, but continuing")
            return True
        except Exception as e:
            logger.error(f"Error during return to home: {str(e)}")
            logger.error(traceback.format_exc())
            self.state = MissionState.ERROR
            return False

    async def _land(self) -> bool:
        """
        Land the drone.

        Returns:
            bool: True if landing was successful, False otherwise
        """
        try:
            logger.info("Landing...")
            self.state = MissionState.LANDING

            # Send land command
            command = {
                "command": "land",
                "parameters": {}
            }
            result = self.adapter.send_command(command)
            logger.info(f"Land command result: {result}")

            if not result:
                logger.error("Land command failed")
                self.state = MissionState.ERROR
                return False

            # Wait for landing to complete
            logger.info("Waiting for landing to complete...")
            for _ in range(20):  # Wait up to 20 seconds
                telemetry = self.adapter.receive_telemetry()
                altitude = telemetry['position']['altitude']
                logger.info(f"Current altitude: {altitude}m")

                if altitude < 0.5:  # Less than 0.5 meters means we've landed
                    logger.info("Landing completed successfully")
                    return True

                await asyncio.sleep(1)

            logger.warning("Landing may not have completed, but continuing")
            return True
        except Exception as e:
            logger.error(f"Error during landing: {str(e)}")
            logger.error(traceback.format_exc())
            self.state = MissionState.ERROR
            return False

    async def stop(self):
        """Stop the mission and clean up resources."""
        logger.info("Stopping mission...")

        # Cancel telemetry task
        if self.telemetry_task:
            self.running = False
            self.telemetry_task.cancel()
            try:
                await self.telemetry_task
            except asyncio.CancelledError:
                pass

        # Disconnect from drone
        if self.adapter and self.adapter._connected:
            await self.adapter.disconnect()
            logger.info("Disconnected from drone")

        logger.info("Mission stopped")

    def save_mission_log(self, filename: str) -> bool:
        """
        Save mission log to a file.

        Args:
            filename: Path to the output file

        Returns:
            bool: True if log was saved successfully, False otherwise
        """
        try:
            with open(filename, 'w') as f:
                json.dump({
                    "mission_start": self.mission_start_time.isoformat() if self.mission_start_time else None,
                    "mission_end": self.mission_end_time.isoformat() if self.mission_end_time else None,
                    "waypoints": self.waypoints,
                    "events": self.mission_log
                }, f, indent=2)
            logger.info(f"Mission log saved to {filename}")
            return True
        except Exception as e:
            logger.error(f"Error saving mission log: {str(e)}")
            return False


async def main():
    """Main function to run the complex mission example."""
    parser = argparse.ArgumentParser(description="DJI Complex Mission Example")

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

    # Mission parameters
    parser.add_argument("--waypoints-file", type=str, help="JSON file containing waypoints")
    parser.add_argument("--log-file", type=str, default="mission_log.json", help="Output file for mission log")

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

    # Create and run mission
    mission = ComplexMission(connection_params, args.waypoints_file)

    try:
        # Initialize mission
        if not await mission.initialize():
            logger.error("Failed to initialize mission")
            return

        # Execute mission
        await mission.execute_mission()

        # Save mission log
        mission.save_mission_log(args.log_file)
    finally:
        # Clean up
        await mission.stop()


if __name__ == "__main__":
    asyncio.run(main())