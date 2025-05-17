"""
DJI Dock Integration Module

This module provides integration between the DJI adapter and the dock system
for automated missions. It enables:
- Automated takeoff from dock
- Automated landing on dock
- Scheduled missions
- Battery management
- Weather-aware mission planning
"""

import asyncio
import logging
import json
import time
import os
import datetime
import math
from typing import Dict, Any, List, Optional, Tuple
from enum import Enum
import traceback

from dronecore.dji_adapter import DJIAdapter
from dock_driver.adapters.dji.adapter import DJIDockAdapter

logger = logging.getLogger(__name__)

class DockMissionState(Enum):
    """States for dock-based missions."""
    IDLE = "idle"
    INITIALIZING = "initializing"
    PREPARING_DOCK = "preparing_dock"
    OPENING_DOCK = "opening_dock"
    CONNECTING_DRONE = "connecting_drone"
    TAKING_OFF = "taking_off"
    EXECUTING_MISSION = "executing_mission"
    RETURNING_TO_DOCK = "returning_to_dock"
    LANDING_ON_DOCK = "landing_on_dock"
    CLOSING_DOCK = "closing_dock"
    CHARGING = "charging"
    COMPLETED = "completed"
    ERROR = "error"

class DJIDockIntegration:
    """Integration between DJI adapter and dock system."""

    def __init__(self, drone_adapter: DJIAdapter, dock_adapter: DJIDockAdapter):
        """
        Initialize the DJI dock integration.

        Args:
            drone_adapter: DJI drone adapter
            dock_adapter: DJI dock adapter
        """
        self.drone_adapter = drone_adapter
        self.dock_adapter = dock_adapter
        self.state = DockMissionState.IDLE
        self.mission_data = {}
        self.telemetry_data = {}
        self.dock_status = {}
        self.mission_start_time = None
        self.mission_end_time = None
        self.error_count = 0
        self.max_errors = 3
        self.running = False
        self.telemetry_task = None
        self.dock_status_task = None
        self.mission_log = []
        self.weather_conditions = {}
        self.scheduled_missions = []
        self.current_mission = None

    async def initialize(self) -> bool:
        """
        Initialize the dock integration.

        Returns:
            bool: True if initialization was successful, False otherwise
        """
        try:
            logger.info("Initializing DJI dock integration")
            self.state = DockMissionState.INITIALIZING

            # Initialize dock adapter
            logger.info("Initializing dock adapter")
            dock_initialized = await self.dock_adapter.initialize()
            if not dock_initialized:
                logger.error("Failed to initialize dock adapter")
                self.state = DockMissionState.ERROR
                return False

            # Start background tasks
            self.running = True
            self.telemetry_task = asyncio.create_task(self._telemetry_loop())
            self.dock_status_task = asyncio.create_task(self._dock_status_loop())

            logger.info("DJI dock integration initialized successfully")
            self.state = DockMissionState.IDLE
            return True
        except Exception as e:
            logger.error(f"Error initializing DJI dock integration: {str(e)}")
            logger.error(traceback.format_exc())
            self.state = DockMissionState.ERROR
            return False

    async def _telemetry_loop(self):
        """Background task for monitoring drone telemetry."""
        while self.running:
            try:
                if self.drone_adapter and self.drone_adapter._connected:
                    # Get telemetry data
                    self.telemetry_data = self.drone_adapter.receive_telemetry()

                    # Log critical information
                    battery = self.telemetry_data['battery']['percent']
                    if battery <= self.drone_adapter._critical_battery_warning:
                        logger.warning(f"CRITICAL BATTERY LEVEL: {battery}%")
                        if self.state == DockMissionState.EXECUTING_MISSION:
                            logger.warning("Initiating emergency return to dock due to critical battery")
                            await self._emergency_return_to_dock()
                    elif battery <= self.drone_adapter._low_battery_warning:
                        logger.warning(f"LOW BATTERY WARNING: {battery}%")

                # Sleep briefly
                await asyncio.sleep(1)
            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error(f"Error in telemetry loop: {str(e)}")
                await asyncio.sleep(1)

    async def _dock_status_loop(self):
        """Background task for monitoring dock status."""
        while self.running:
            try:
                # Get dock status
                self.dock_status = await self.dock_adapter.get_status()

                # Get dock telemetry
                dock_telemetry = await self.dock_adapter.get_telemetry()

                # Log important information
                logger.debug(f"Dock status: {self.dock_status}")
                logger.debug(f"Dock telemetry: {dock_telemetry}")

                # Sleep for a while
                await asyncio.sleep(5)
            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error(f"Error in dock status loop: {str(e)}")
                await asyncio.sleep(5)

    async def _emergency_return_to_dock(self) -> bool:
        """
        Initiate emergency return to dock procedure.

        Returns:
            bool: True if return to dock was initiated successfully, False otherwise
        """
        try:
            logger.warning("EMERGENCY RETURN TO DOCK INITIATED")
            self.state = DockMissionState.RETURNING_TO_DOCK

            # Log the event
            self.mission_log.append({
                "timestamp": datetime.datetime.now().isoformat(),
                "event": "emergency_return_to_dock",
                "reason": "critical_battery" if self.telemetry_data['battery']['percent'] <= self.drone_adapter._critical_battery_warning else "manual"
            })

            # Get dock position
            dock_position = await self.dock_adapter.get_position()
            if not dock_position:
                logger.error("Failed to get dock position")
                return False

            # Navigate to dock position
            command = {
                "command": "goto",
                "parameters": {
                    "latitude": dock_position["latitude"],
                    "longitude": dock_position["longitude"],
                    "altitude": dock_position["altitude"] + 10,  # 10m above dock
                    "speed": 5.0
                }
            }
            result = self.drone_adapter.send_command(command)
            logger.info(f"Goto dock command result: {result}")

            if not result:
                logger.error("Failed to navigate to dock")
                return False

            # Wait for arrival at dock
            logger.info("Waiting for arrival at dock...")
            arrived = await self._wait_for_arrival(
                dock_position["latitude"],
                dock_position["longitude"],
                dock_position["altitude"] + 10
            )

            if not arrived:
                logger.warning("May not have arrived at dock position, attempting landing anyway")

            # Prepare dock for landing
            logger.info("Preparing dock for landing")
            await self.dock_adapter.prepare_for_landing()

            # Land on dock
            logger.info("Landing on dock")
            command = {
                "command": "precise_land",
                "parameters": {
                    "latitude": dock_position["latitude"],
                    "longitude": dock_position["longitude"]
                }
            }
            result = self.drone_adapter.send_command(command)
            logger.info(f"Land on dock command result: {result}")

            return result
        except Exception as e:
            logger.error(f"Error initiating emergency return to dock: {str(e)}")
            logger.error(traceback.format_exc())
            self.state = DockMissionState.ERROR
            return False

    async def _wait_for_arrival(self, target_lat: float, target_lon: float, target_alt: float, timeout: int = 60) -> bool:
        """
        Wait for arrival at a position.

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
            if not self.drone_adapter._connected:
                logger.error("Drone disconnected while waiting for arrival")
                return False

            telemetry = self.drone_adapter.receive_telemetry()
            current_lat = telemetry['position']['latitude']
            current_lon = telemetry['position']['longitude']
            current_alt = telemetry['position']['altitude']

            # Calculate distance (simplified)
            lat_diff = (target_lat - current_lat) * 111000  # 1 degree lat = ~111km
            lon_diff = (target_lon - current_lon) * 111000 * math.cos(math.radians(current_lat))
            alt_diff = target_alt - current_alt

            distance = math.sqrt(lat_diff**2 + lon_diff**2 + alt_diff**2)

            logger.info(f"Distance to target: {distance:.2f}m")

            if distance < arrival_threshold:
                logger.info("Arrived at target position")
                return True

            await asyncio.sleep(1)

        logger.warning(f"Timeout waiting for arrival after {timeout} seconds")
        return False

    async def execute_dock_mission(self, mission_file: str) -> bool:
        """
        Execute a mission from the dock.

        Args:
            mission_file: Path to the mission file

        Returns:
            bool: True if mission was completed successfully, False otherwise
        """
        try:
            # Load mission data
            logger.info(f"Loading mission from {mission_file}")
            with open(mission_file, 'r') as f:
                self.mission_data = json.load(f)

            mission_name = self.mission_data.get("mission_name", "Unnamed Mission")
            logger.info(f"Starting dock mission: {mission_name}")

            # Start mission
            self.mission_start_time = datetime.datetime.now()
            self.current_mission = mission_file

            # Check weather conditions
            if not await self._check_weather_conditions():
                logger.error("Weather conditions not suitable for mission")
                return False

            # Prepare dock
            if not await self._prepare_dock():
                logger.error("Failed to prepare dock")
                return False

            # Connect to drone
            if not await self._connect_drone():
                logger.error("Failed to connect to drone")
                return False

            # Take off from dock
            if not await self._take_off_from_dock():
                logger.error("Failed to take off from dock")
                return False

            # Execute mission
            if not await self._execute_mission():
                logger.error("Failed to execute mission")
                # Try to return to dock safely
                await self._emergency_return_to_dock()
                return False

            # Return to dock
            if not await self._return_to_dock():
                logger.error("Failed to return to dock")
                return False

            # Close dock
            if not await self._close_dock():
                logger.error("Failed to close dock")
                return False

            # Mission completed
            self.mission_end_time = datetime.datetime.now()
            duration = (self.mission_end_time - self.mission_start_time).total_seconds()
            logger.info(f"Mission completed successfully in {duration:.1f} seconds")
            self.state = DockMissionState.COMPLETED

            # Log mission completion
            self.mission_log.append({
                "timestamp": self.mission_end_time.isoformat(),
                "event": "mission_completed",
                "duration": duration
            })

            return True
        except Exception as e:
            logger.error(f"Error executing dock mission: {str(e)}")
            logger.error(traceback.format_exc())
            self.state = DockMissionState.ERROR

            # Log error
            self.mission_log.append({
                "timestamp": datetime.datetime.now().isoformat(),
                "event": "mission_error",
                "error": str(e)
            })

            # Try to return to dock safely
            await self._emergency_return_to_dock()
            return False

    async def _check_weather_conditions(self) -> bool:
        """
        Check if weather conditions are suitable for the mission.

        Returns:
            bool: True if weather conditions are suitable, False otherwise
        """
        try:
            logger.info("Checking weather conditions")

            # Get weather data from dock telemetry
            dock_telemetry = await self.dock_adapter.get_telemetry()

            # Extract weather data
            temperature = dock_telemetry.get("temperature", 25.0)
            humidity = dock_telemetry.get("humidity", 50.0)
            wind_speed = dock_telemetry.get("wind_speed", 0.0)
            precipitation = dock_telemetry.get("precipitation", 0.0)

            # Store weather conditions
            self.weather_conditions = {
                "temperature": temperature,
                "humidity": humidity,
                "wind_speed": wind_speed,
                "precipitation": precipitation,
                "timestamp": datetime.datetime.now().isoformat()
            }

            # Check against thresholds
            max_wind_speed = self.mission_data.get("max_wind_speed", 8.0)  # m/s
            max_precipitation = self.mission_data.get("max_precipitation", 0.0)  # mm
            min_temperature = self.mission_data.get("min_temperature", -10.0)  # Celsius
            max_temperature = self.mission_data.get("max_temperature", 40.0)  # Celsius

            if wind_speed > max_wind_speed:
                logger.warning(f"Wind speed too high: {wind_speed} m/s (max: {max_wind_speed} m/s)")
                return False

            if precipitation > max_precipitation:
                logger.warning(f"Precipitation detected: {precipitation} mm (max: {max_precipitation} mm)")
                return False

            if temperature < min_temperature or temperature > max_temperature:
                logger.warning(f"Temperature out of range: {temperature}°C (range: {min_temperature}°C to {max_temperature}°C)")
                return False

            logger.info("Weather conditions suitable for mission")
            return True
        except Exception as e:
            logger.error(f"Error checking weather conditions: {str(e)}")
            logger.error(traceback.format_exc())
            return False

    async def _prepare_dock(self) -> bool:
        """
        Prepare the dock for takeoff.

        Returns:
            bool: True if dock preparation was successful, False otherwise
        """
        try:
            logger.info("Preparing dock for takeoff")
            self.state = DockMissionState.PREPARING_DOCK

            # Check dock status
            dock_status = await self.dock_adapter.get_status()
            if dock_status["status"] != "online":
                logger.error(f"Dock is not online: {dock_status['status']}")
                return False

            # Check if drone is in the dock
            if not dock_status["drone_present"]:
                logger.error("No drone detected in the dock")
                return False

            # Check battery level
            dock_telemetry = await self.dock_adapter.get_telemetry()
            battery_level = dock_telemetry.get("battery_level", 0)
            min_battery = self.mission_data.get("min_battery_level", 50)

            if battery_level < min_battery:
                logger.error(f"Battery level too low: {battery_level}% (min: {min_battery}%)")
                return False

            # Open dock
            logger.info("Opening dock")
            self.state = DockMissionState.OPENING_DOCK
            result = await self.dock_adapter.open_cover()
            if not result:
                logger.error("Failed to open dock cover")
                return False

            # Wait for dock to open
            logger.info("Waiting for dock to open...")
            for _ in range(30):  # Wait up to 30 seconds
                dock_status = await self.dock_adapter.get_status()
                if dock_status["cover_status"] == "open":
                    logger.info("Dock opened successfully")
                    return True
                await asyncio.sleep(1)

            logger.error("Timeout waiting for dock to open")
            return False
        except Exception as e:
            logger.error(f"Error preparing dock: {str(e)}")
            logger.error(traceback.format_exc())
            self.state = DockMissionState.ERROR
            return False

    async def _connect_drone(self) -> bool:
        """
        Connect to the drone.

        Returns:
            bool: True if connection was successful, False otherwise
        """
        try:
            logger.info("Connecting to drone")
            self.state = DockMissionState.CONNECTING_DRONE

            # Connect to the drone
            connected = await self.drone_adapter.connect()
            if not connected:
                logger.error("Failed to connect to drone")
                return False

            logger.info("Connected to drone successfully")
            return True
        except Exception as e:
            logger.error(f"Error connecting to drone: {str(e)}")
            logger.error(traceback.format_exc())
            self.state = DockMissionState.ERROR
            return False

    async def _take_off_from_dock(self) -> bool:
        """
        Take off from the dock.

        Returns:
            bool: True if takeoff was successful, False otherwise
        """
        try:
            logger.info("Taking off from dock")
            self.state = DockMissionState.TAKING_OFF

            # Send takeoff command
            command = {
                "command": "takeoff",
                "parameters": {}
            }
            result = self.drone_adapter.send_command(command)
            logger.info(f"Takeoff command result: {result}")

            if not result:
                logger.error("Takeoff command failed")
                self.state = DockMissionState.ERROR
                return False

            # Wait for takeoff to complete
            logger.info("Waiting for takeoff to complete...")
            for _ in range(20):  # Wait up to 20 seconds
                telemetry = self.drone_adapter.receive_telemetry()
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
            self.state = DockMissionState.ERROR
            return False

    async def _execute_mission(self) -> bool:
        """
        Execute the mission.

        Returns:
            bool: True if mission was executed successfully, False otherwise
        """
        try:
            logger.info("Executing mission")
            self.state = DockMissionState.EXECUTING_MISSION

            mission_type = self.mission_data.get("mission_type", "waypoint")

            if mission_type == "waypoint":
                return await self._execute_waypoint_mission()
            elif mission_type == "hotpoint":
                return await self._execute_hotpoint_mission()
            elif mission_type == "follow_me":
                return await self._execute_follow_me_mission()
            elif mission_type == "timeline":
                return await self._execute_timeline_mission()
            else:
                logger.error(f"Unsupported mission type: {mission_type}")
                return False
        except Exception as e:
            logger.error(f"Error executing mission: {str(e)}")
            logger.error(traceback.format_exc())
            self.state = DockMissionState.ERROR
            return False

    async def _execute_waypoint_mission(self) -> bool:
        """
        Execute a waypoint mission.

        Returns:
            bool: True if mission was executed successfully, False otherwise
        """
        try:
            waypoints = self.mission_data.get("waypoints", [])
            if not waypoints:
                logger.error("No waypoints defined in mission")
                return False

            logger.info(f"Executing waypoint mission with {len(waypoints)} waypoints")

            # Start waypoint mission
            command = {
                "command": "start_waypoint_mission",
                "parameters": {
                    "waypoints": waypoints,
                    "speed": self.mission_data.get("speed", 5.0),
                    "finish_action": self.mission_data.get("finish_action", "no_action"),
                    "heading_mode": self.mission_data.get("heading_mode", "auto")
                }
            }
            result = self.drone_adapter.send_command(command)
            logger.info(f"Start waypoint mission command result: {result}")

            if not result:
                logger.error("Failed to start waypoint mission")
                return False

            # Monitor mission progress
            logger.info("Monitoring waypoint mission progress...")
            mission_timeout = self.mission_data.get("timeout", 1800)  # 30 minutes default
            start_time = time.time()

            while time.time() - start_time < mission_timeout:
                # Check if mission is still executing
                mission_state = await self.drone_adapter.get_waypoint_mission_state()
                if not mission_state["executing"]:
                    logger.info("Waypoint mission completed")
                    return True

                # Log progress
                logger.info(f"Mission progress: Waypoint {mission_state['current_waypoint_index'] + 1}/{len(waypoints)}")

                # Check for critical conditions
                telemetry = self.drone_adapter.receive_telemetry()
                battery = telemetry['battery']['percent']

                if battery <= self.drone_adapter._critical_battery_warning:
                    logger.warning(f"CRITICAL BATTERY LEVEL: {battery}%")
                    logger.warning("Stopping mission due to critical battery")

                    # Stop mission
                    command = {
                        "command": "stop_waypoint_mission",
                        "parameters": {}
                    }
                    self.drone_adapter.send_command(command)

                    return False

                await asyncio.sleep(5)

            logger.warning(f"Mission timeout after {mission_timeout} seconds")

            # Stop mission
            command = {
                "command": "stop_waypoint_mission",
                "parameters": {}
            }
            self.drone_adapter.send_command(command)

            return False
        except Exception as e:
            logger.error(f"Error executing waypoint mission: {str(e)}")
            logger.error(traceback.format_exc())
            return False

    async def _execute_hotpoint_mission(self) -> bool:
        """
        Execute a hotpoint (orbit) mission.

        Returns:
            bool: True if mission was executed successfully, False otherwise
        """
        try:
            # Extract hotpoint parameters
            latitude = self.mission_data.get("latitude")
            longitude = self.mission_data.get("longitude")
            altitude = self.mission_data.get("altitude")
            radius = self.mission_data.get("radius", 10.0)
            angular_speed = self.mission_data.get("angular_speed", 15.0)
            is_clockwise = self.mission_data.get("is_clockwise", True)
            duration = self.mission_data.get("duration", 300)  # 5 minutes default

            if not latitude or not longitude or not altitude:
                logger.error("Latitude, longitude, and altitude are required for hotpoint mission")
                return False

            logger.info(f"Executing hotpoint mission: lat={latitude}, lon={longitude}, alt={altitude}, radius={radius}")

            # Start hotpoint mission
            command = {
                "command": "start_hotpoint_mission",
                "parameters": {
                    "latitude": latitude,
                    "longitude": longitude,
                    "altitude": altitude,
                    "radius": radius,
                    "angular_speed": angular_speed,
                    "is_clockwise": is_clockwise
                }
            }
            result = self.drone_adapter.send_command(command)
            logger.info(f"Start hotpoint mission command result: {result}")

            if not result:
                logger.error("Failed to start hotpoint mission")
                return False

            # Wait for specified duration
            logger.info(f"Hotpoint mission running for {duration} seconds")
            await asyncio.sleep(duration)

            # Stop hotpoint mission
            command = {
                "command": "stop_hotpoint_mission",
                "parameters": {}
            }
            result = self.drone_adapter.send_command(command)
            logger.info(f"Stop hotpoint mission command result: {result}")

            return result
        except Exception as e:
            logger.error(f"Error executing hotpoint mission: {str(e)}")
            logger.error(traceback.format_exc())
            return False

    async def _execute_follow_me_mission(self) -> bool:
        """
        Execute a follow me mission.

        Returns:
            bool: True if mission was executed successfully, False otherwise
        """
        # This is a placeholder as follow me missions typically require real-time target updates
        logger.warning("Follow me missions from dock are not fully supported")
        return False

    async def _execute_timeline_mission(self) -> bool:
        """
        Execute a timeline mission.

        Returns:
            bool: True if mission was executed successfully, False otherwise
        """
        # This is a placeholder as timeline missions are complex and require detailed implementation
        logger.warning("Timeline missions from dock are not fully supported")
        return False

    async def _return_to_dock(self) -> bool:
        """
        Return to the dock.

        Returns:
            bool: True if return to dock was successful, False otherwise
        """
        try:
            logger.info("Returning to dock")
            self.state = DockMissionState.RETURNING_TO_DOCK

            # Get dock position
            dock_position = await self.dock_adapter.get_position()
            if not dock_position:
                logger.error("Failed to get dock position")
                return False

            # Navigate to position above dock
            command = {
                "command": "goto",
                "parameters": {
                    "latitude": dock_position["latitude"],
                    "longitude": dock_position["longitude"],
                    "altitude": dock_position["altitude"] + 10,  # 10m above dock
                    "speed": 3.0  # Slower speed for precision
                }
            }
            result = self.drone_adapter.send_command(command)
            logger.info(f"Goto dock command result: {result}")

            if not result:
                logger.error("Failed to navigate to dock")
                return False

            # Wait for arrival at dock
            logger.info("Waiting for arrival at dock...")
            arrived = await self._wait_for_arrival(
                dock_position["latitude"],
                dock_position["longitude"],
                dock_position["altitude"] + 10
            )

            if not arrived:
                logger.warning("May not have arrived at dock position, attempting landing anyway")

            # Prepare dock for landing
            logger.info("Preparing dock for landing")
            await self.dock_adapter.prepare_for_landing()

            # Land on dock
            logger.info("Landing on dock")
            self.state = DockMissionState.LANDING_ON_DOCK
            command = {
                "command": "precise_land",
                "parameters": {
                    "latitude": dock_position["latitude"],
                    "longitude": dock_position["longitude"]
                }
            }
            result = self.drone_adapter.send_command(command)
            logger.info(f"Land on dock command result: {result}")

            if not result:
                logger.error("Failed to initiate landing on dock")
                return False

            # Wait for landing to complete
            logger.info("Waiting for landing to complete...")
            for _ in range(60):  # Wait up to 60 seconds
                telemetry = self.drone_adapter.receive_telemetry()
                altitude = telemetry['position']['altitude']
                logger.info(f"Current altitude: {altitude}m")

                if altitude < 0.5:  # Less than 0.5 meters means we've landed
                    logger.info("Landing completed successfully")

                    # Disconnect from drone
                    await self.drone_adapter.disconnect()
                    logger.info("Disconnected from drone")

                    return True

                await asyncio.sleep(1)

            logger.warning("Landing may not have completed")
            return False
        except Exception as e:
            logger.error(f"Error returning to dock: {str(e)}")
            logger.error(traceback.format_exc())
            self.state = DockMissionState.ERROR
            return False

    async def _close_dock(self) -> bool:
        """
        Close the dock.

        Returns:
            bool: True if dock was closed successfully, False otherwise
        """
        try:
            logger.info("Closing dock")
            self.state = DockMissionState.CLOSING_DOCK

            # Close dock cover
            result = await self.dock_adapter.close_cover()
            if not result:
                logger.error("Failed to close dock cover")
                return False

            # Wait for dock to close
            logger.info("Waiting for dock to close...")
            for _ in range(30):  # Wait up to 30 seconds
                dock_status = await self.dock_adapter.get_status()
                if dock_status["cover_status"] == "closed":
                    logger.info("Dock closed successfully")

                    # Start charging
                    self.state = DockMissionState.CHARGING
                    await self.dock_adapter.start_charging()

                    return True
                await asyncio.sleep(1)

            logger.error("Timeout waiting for dock to close")
            return False
        except Exception as e:
            logger.error(f"Error closing dock: {str(e)}")
            logger.error(traceback.format_exc())
            self.state = DockMissionState.ERROR
            return False

    async def schedule_mission(self, mission_file: str, schedule_time: datetime.datetime) -> bool:
        """
        Schedule a mission to run at a specific time.

        Args:
            mission_file: Path to the mission file
            schedule_time: Time to run the mission

        Returns:
            bool: True if mission was scheduled successfully, False otherwise
        """
        try:
            # Check if mission file exists
            if not os.path.exists(mission_file):
                logger.error(f"Mission file not found: {mission_file}")
                return False

            # Add mission to schedule
            mission_id = len(self.scheduled_missions) + 1
            self.scheduled_missions.append({
                "id": mission_id,
                "mission_file": mission_file,
                "schedule_time": schedule_time.isoformat(),
                "status": "scheduled"
            })

            logger.info(f"Mission scheduled: ID={mission_id}, File={mission_file}, Time={schedule_time.isoformat()}")
            return True
        except Exception as e:
            logger.error(f"Error scheduling mission: {str(e)}")
            return False

    async def run_scheduled_missions(self):
        """Run scheduled missions that are due."""
        try:
            current_time = datetime.datetime.now()

            for mission in self.scheduled_missions:
                if mission["status"] == "scheduled":
                    schedule_time = datetime.datetime.fromisoformat(mission["schedule_time"])

                    if current_time >= schedule_time:
                        logger.info(f"Running scheduled mission: ID={mission['id']}, File={mission['mission_file']}")

                        # Update mission status
                        mission["status"] = "running"

                        # Execute mission
                        success = await self.execute_dock_mission(mission["mission_file"])

                        # Update mission status
                        mission["status"] = "completed" if success else "failed"
                        mission["completion_time"] = datetime.datetime.now().isoformat()
                        mission["success"] = success
        except Exception as e:
            logger.error(f"Error running scheduled missions: {str(e)}")

    async def stop(self):
        """Stop the dock integration and clean up resources."""
        logger.info("Stopping DJI dock integration")

        # Cancel background tasks
        if self.telemetry_task:
            self.running = False
            self.telemetry_task.cancel()
            try:
                await self.telemetry_task
            except asyncio.CancelledError:
                pass

        if self.dock_status_task:
            self.dock_status_task.cancel()
            try:
                await self.dock_status_task
            except asyncio.CancelledError:
                pass

        # Disconnect from drone if connected
        if self.drone_adapter and self.drone_adapter._connected:
            await self.drone_adapter.disconnect()
            logger.info("Disconnected from drone")

        logger.info("DJI dock integration stopped")
