"""
Drone interface service for the SentinelBeacon module.

This service is responsible for interfacing with the drone, including:
- Connecting to the drone
- Getting drone telemetry
- Sending commands to the drone
- Managing autonomous beacon operations
"""

import asyncio
import logging
import math
import time
from datetime import datetime
from typing import Dict, List, Optional, Any, Callable, Tuple

from core.config import settings

logger = logging.getLogger(__name__)

class DroneInterface:
    """Drone interface service."""
    
    def __init__(self):
        """Initialize the drone interface."""
        self.connected = False
        self.running = False
        self.telemetry_callbacks = []
        self.position_callbacks = []
        self.status_callbacks = []
        self.telemetry_task = None
        self.beacon_task = None
        self.home_position = None
        self.current_position = None
        self.current_altitude = 0.0
        self.battery_level = 100.0
        self.beacon_active = False
        self.beacon_mode = settings.BEACON_MODE
    
    async def connect(self):
        """Connect to the drone."""
        logger.info(f"Connecting to drone using {settings.DRONE_INTERFACE_TYPE} interface")
        
        try:
            # Connect to the drone based on interface type
            if settings.DRONE_INTERFACE_TYPE == "mavlink":
                # Connect using MAVLink
                await self._connect_mavlink()
            elif settings.DRONE_INTERFACE_TYPE == "serial":
                # Connect using serial
                await self._connect_serial()
            elif settings.DRONE_INTERFACE_TYPE == "none":
                # No drone interface, just simulate
                await self._connect_simulation()
            else:
                logger.error(f"Unknown drone interface type: {settings.DRONE_INTERFACE_TYPE}")
                return False
            
            # Start telemetry task
            self.telemetry_task = asyncio.create_task(self._telemetry_task())
            
            # Start beacon task if in autonomous mode
            if self.beacon_mode == "autonomous":
                self.beacon_task = asyncio.create_task(self._beacon_task())
            
            self.connected = True
            self.running = True
            
            logger.info("Connected to drone")
            
            return True
        
        except Exception as e:
            logger.error(f"Failed to connect to drone: {e}")
            return False
    
    async def disconnect(self):
        """Disconnect from the drone."""
        logger.info("Disconnecting from drone")
        
        self.running = False
        
        # Cancel telemetry task
        if self.telemetry_task:
            self.telemetry_task.cancel()
            try:
                await self.telemetry_task
            except asyncio.CancelledError:
                pass
        
        # Cancel beacon task
        if self.beacon_task:
            self.beacon_task.cancel()
            try:
                await self.beacon_task
            except asyncio.CancelledError:
                pass
        
        # Return home if beacon is active
        if self.beacon_active:
            await self.return_home()
        
        self.connected = False
        
        logger.info("Disconnected from drone")
    
    async def get_position(self) -> Optional[Dict[str, float]]:
        """
        Get the drone's current position.
        
        Returns:
            Position (latitude, longitude, altitude) or None if not available
        """
        if not self.connected:
            logger.error("Not connected to drone")
            return None
        
        if not self.current_position:
            return None
        
        return {
            "latitude": self.current_position[0],
            "longitude": self.current_position[1],
            "altitude": self.current_altitude
        }
    
    async def get_telemetry(self) -> Dict[str, Any]:
        """
        Get the drone's telemetry.
        
        Returns:
            Telemetry data
        """
        if not self.connected:
            logger.error("Not connected to drone")
            return {}
        
        # Get position
        position = await self.get_position()
        
        # Create telemetry data
        telemetry = {
            "timestamp": datetime.utcnow().isoformat(),
            "position": position,
            "battery_level": self.battery_level,
            "beacon_active": self.beacon_active,
            "beacon_mode": self.beacon_mode
        }
        
        return telemetry
    
    async def start_beacon(self) -> bool:
        """
        Start the beacon operation.
        
        Returns:
            True if successful, False otherwise
        """
        if not self.connected:
            logger.error("Not connected to drone")
            return False
        
        if self.beacon_active:
            logger.warning("Beacon already active")
            return True
        
        try:
            # Get current position as home position
            if not self.home_position:
                self.home_position = self.current_position
            
            # Take off to beacon altitude
            await self._take_off(settings.BEACON_ALTITUDE)
            
            # Set beacon active
            self.beacon_active = True
            
            logger.info(f"Started beacon operation at altitude {settings.BEACON_ALTITUDE}m")
            
            return True
        
        except Exception as e:
            logger.error(f"Failed to start beacon: {e}")
            return False
    
    async def stop_beacon(self) -> bool:
        """
        Stop the beacon operation.
        
        Returns:
            True if successful, False otherwise
        """
        if not self.connected:
            logger.error("Not connected to drone")
            return False
        
        if not self.beacon_active:
            logger.warning("Beacon not active")
            return True
        
        try:
            # Return to home position
            await self.return_home()
            
            # Set beacon inactive
            self.beacon_active = False
            
            logger.info("Stopped beacon operation")
            
            return True
        
        except Exception as e:
            logger.error(f"Failed to stop beacon: {e}")
            return False
    
    async def return_home(self) -> bool:
        """
        Return the drone to home position.
        
        Returns:
            True if successful, False otherwise
        """
        if not self.connected:
            logger.error("Not connected to drone")
            return False
        
        try:
            # Check if home position is set
            if not self.home_position:
                logger.warning("Home position not set")
                return False
            
            # Send return to home command
            logger.info("Returning to home position")
            
            # In a real implementation, this would send the command to the drone
            # For now, we'll just simulate it
            await asyncio.sleep(2)
            
            # Update position to home position
            self.current_position = self.home_position
            self.current_altitude = 0.0
            
            logger.info("Returned to home position")
            
            return True
        
        except Exception as e:
            logger.error(f"Failed to return home: {e}")
            return False
    
    async def move_to_position(self, latitude: float, longitude: float, altitude: float) -> bool:
        """
        Move the drone to a specific position.
        
        Args:
            latitude: Latitude in degrees
            longitude: Longitude in degrees
            altitude: Altitude in meters
            
        Returns:
            True if successful, False otherwise
        """
        if not self.connected:
            logger.error("Not connected to drone")
            return False
        
        try:
            # Send move command
            logger.info(f"Moving to position: lat={latitude}, lon={longitude}, alt={altitude}")
            
            # In a real implementation, this would send the command to the drone
            # For now, we'll just simulate it
            await asyncio.sleep(2)
            
            # Update position
            self.current_position = (latitude, longitude)
            self.current_altitude = altitude
            
            # Call position callbacks
            position = {
                "latitude": latitude,
                "longitude": longitude,
                "altitude": altitude
            }
            for callback in self.position_callbacks:
                try:
                    callback(position)
                except Exception as e:
                    logger.error(f"Error in position callback: {e}")
            
            logger.info(f"Moved to position: lat={latitude}, lon={longitude}, alt={altitude}")
            
            return True
        
        except Exception as e:
            logger.error(f"Failed to move to position: {e}")
            return False
    
    def add_telemetry_callback(self, callback: Callable[[Dict[str, Any]], None]):
        """
        Add a callback for telemetry updates.
        
        Args:
            callback: Callback function
        """
        self.telemetry_callbacks.append(callback)
    
    def add_position_callback(self, callback: Callable[[Dict[str, float]], None]):
        """
        Add a callback for position updates.
        
        Args:
            callback: Callback function
        """
        self.position_callbacks.append(callback)
    
    def add_status_callback(self, callback: Callable[[Dict[str, Any]], None]):
        """
        Add a callback for status updates.
        
        Args:
            callback: Callback function
        """
        self.status_callbacks.append(callback)
    
    async def _connect_mavlink(self):
        """Connect to the drone using MAVLink."""
        # In a real implementation, this would connect to the drone using MAVLink
        # For now, we'll just simulate it
        await asyncio.sleep(1)
        
        # Set initial position (simulated)
        self.current_position = (37.7749, -122.4194)  # San Francisco
        self.home_position = self.current_position
        self.current_altitude = 0.0
        self.battery_level = 100.0
    
    async def _connect_serial(self):
        """Connect to the drone using serial."""
        # In a real implementation, this would connect to the drone using serial
        # For now, we'll just simulate it
        await asyncio.sleep(1)
        
        # Set initial position (simulated)
        self.current_position = (37.7749, -122.4194)  # San Francisco
        self.home_position = self.current_position
        self.current_altitude = 0.0
        self.battery_level = 100.0
    
    async def _connect_simulation(self):
        """Connect to a simulated drone."""
        await asyncio.sleep(1)
        
        # Set initial position (simulated)
        self.current_position = (37.7749, -122.4194)  # San Francisco
        self.home_position = self.current_position
        self.current_altitude = 0.0
        self.battery_level = 100.0
    
    async def _telemetry_task(self):
        """Task for updating telemetry."""
        logger.info("Starting telemetry task")
        
        try:
            while self.running:
                # Wait for a short interval
                await asyncio.sleep(1)
                
                # Skip if not connected
                if not self.connected:
                    continue
                
                # Update battery level (simulated)
                if self.beacon_active:
                    # Drain battery faster when beacon is active
                    self.battery_level = max(0.0, self.battery_level - 0.01)
                else:
                    # Drain battery slower when beacon is inactive
                    self.battery_level = max(0.0, self.battery_level - 0.001)
                
                # Check if battery is low
                if self.beacon_active and self.battery_level <= settings.BEACON_RETURN_HOME_BATTERY:
                    logger.warning(f"Battery level low ({self.battery_level}%), returning home")
                    await self.stop_beacon()
                
                # Get telemetry
                telemetry = await self.get_telemetry()
                
                # Call telemetry callbacks
                for callback in self.telemetry_callbacks:
                    try:
                        callback(telemetry)
                    except Exception as e:
                        logger.error(f"Error in telemetry callback: {e}")
        
        except asyncio.CancelledError:
            logger.info("Telemetry task cancelled")
            raise
        
        except Exception as e:
            logger.error(f"Error in telemetry task: {e}")
    
    async def _beacon_task(self):
        """Task for autonomous beacon operation."""
        logger.info("Starting beacon task")
        
        try:
            while self.running:
                # Wait for a short interval
                await asyncio.sleep(5)
                
                # Skip if not connected or beacon not active
                if not self.connected or not self.beacon_active:
                    continue
                
                # Execute beacon movement pattern
                if settings.BEACON_MOVEMENT_PATTERN == "hover":
                    # Just hover in place
                    pass
                
                elif settings.BEACON_MOVEMENT_PATTERN == "circle":
                    # Move in a circle around home position
                    await self._execute_circle_pattern()
                
                elif settings.BEACON_MOVEMENT_PATTERN == "grid":
                    # Move in a grid pattern around home position
                    await self._execute_grid_pattern()
                
                else:
                    logger.warning(f"Unknown beacon movement pattern: {settings.BEACON_MOVEMENT_PATTERN}")
        
        except asyncio.CancelledError:
            logger.info("Beacon task cancelled")
            raise
        
        except Exception as e:
            logger.error(f"Error in beacon task: {e}")
    
    async def _take_off(self, altitude: float) -> bool:
        """
        Take off to a specific altitude.
        
        Args:
            altitude: Target altitude in meters
            
        Returns:
            True if successful, False otherwise
        """
        try:
            # Send take off command
            logger.info(f"Taking off to altitude {altitude}m")
            
            # In a real implementation, this would send the command to the drone
            # For now, we'll just simulate it
            await asyncio.sleep(2)
            
            # Update altitude
            self.current_altitude = altitude
            
            # Call position callbacks
            position = {
                "latitude": self.current_position[0],
                "longitude": self.current_position[1],
                "altitude": altitude
            }
            for callback in self.position_callbacks:
                try:
                    callback(position)
                except Exception as e:
                    logger.error(f"Error in position callback: {e}")
            
            logger.info(f"Taken off to altitude {altitude}m")
            
            return True
        
        except Exception as e:
            logger.error(f"Failed to take off: {e}")
            return False
    
    async def _execute_circle_pattern(self):
        """Execute a circular movement pattern."""
        try:
            # Check if home position is set
            if not self.home_position:
                logger.warning("Home position not set")
                return
            
            # Calculate circle parameters
            radius = settings.BEACON_MAX_DISTANCE / 2  # meters
            center_lat = self.home_position[0]
            center_lon = self.home_position[1]
            
            # Calculate new position on circle
            # This is a simplified calculation that doesn't account for Earth's curvature
            # For more accuracy, use a proper geospatial library
            angle = time.time() % (2 * math.pi)  # Angle based on time
            lat_offset = radius * math.cos(angle) / 111111  # 1 degree latitude is approximately 111111 meters
            lon_offset = radius * math.sin(angle) / (111111 * math.cos(math.radians(center_lat)))  # Adjust for latitude
            
            new_lat = center_lat + lat_offset
            new_lon = center_lon + lon_offset
            
            # Move to new position
            await self.move_to_position(new_lat, new_lon, settings.BEACON_ALTITUDE)
        
        except Exception as e:
            logger.error(f"Error executing circle pattern: {e}")
    
    async def _execute_grid_pattern(self):
        """Execute a grid movement pattern."""
        try:
            # Check if home position is set
            if not self.home_position:
                logger.warning("Home position not set")
                return
            
            # Calculate grid parameters
            grid_size = settings.BEACON_MAX_DISTANCE  # meters
            grid_step = grid_size / 4  # 5x5 grid
            
            # Calculate grid position based on time
            time_index = int(time.time() / 10) % 25  # Change position every 10 seconds, 25 positions
            grid_x = (time_index % 5) - 2  # -2 to 2
            grid_y = (time_index // 5) - 2  # -2 to 2
            
            # Calculate new position on grid
            lat_offset = grid_y * grid_step / 111111  # 1 degree latitude is approximately 111111 meters
            lon_offset = grid_x * grid_step / (111111 * math.cos(math.radians(self.home_position[0])))  # Adjust for latitude
            
            new_lat = self.home_position[0] + lat_offset
            new_lon = self.home_position[1] + lon_offset
            
            # Move to new position
            await self.move_to_position(new_lat, new_lon, settings.BEACON_ALTITUDE)
        
        except Exception as e:
            logger.error(f"Error executing grid pattern: {e}")
    
    async def set_beacon_mode(self, mode: str) -> bool:
        """
        Set the beacon mode.
        
        Args:
            mode: Beacon mode (autonomous, manual, scheduled)
            
        Returns:
            True if successful, False otherwise
        """
        if mode not in ["autonomous", "manual", "scheduled"]:
            logger.error(f"Invalid beacon mode: {mode}")
            return False
        
        # Set beacon mode
        self.beacon_mode = mode
        
        # Start or stop beacon task based on mode
        if mode == "autonomous" and not self.beacon_task:
            self.beacon_task = asyncio.create_task(self._beacon_task())
        elif mode != "autonomous" and self.beacon_task:
            self.beacon_task.cancel()
            try:
                await self.beacon_task
            except asyncio.CancelledError:
                pass
            self.beacon_task = None
        
        logger.info(f"Set beacon mode to {mode}")
        
        return True
