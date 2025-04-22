"""
LED control service for the Drone Show microservice.

This module provides services for controlling drone LEDs.
"""

import logging
import asyncio
from typing import Dict, Any, List, Optional
from datetime import datetime

from drone_show_service.models.choreography import LEDState, LEDEffect, LEDColor
from drone_show_service.core.config import settings
from drone_show_service.services.sentinel_integration import SentinelIntegrationService

logger = logging.getLogger(__name__)


class LEDControlService:
    """
    Service for controlling drone LEDs.
    
    This service provides methods for sending LED commands to drones
    using the MAVLink DO_SET_LED command.
    """
    
    def __init__(self, sentinel_integration: Optional[SentinelIntegrationService] = None):
        """
        Initialize the LED control service.
        
        Args:
            sentinel_integration: Sentinel integration service
        """
        self.sentinel_integration = sentinel_integration
        self._active_shows: Dict[str, Dict[str, Any]] = {}
    
    async def set_led_state(self, drone_id: str, led_state: LEDState) -> bool:
        """
        Set the LED state for a drone.
        
        Args:
            drone_id: Drone ID
            led_state: LED state
            
        Returns:
            True if successful, False otherwise
        """
        try:
            # Convert LED state to MAVLink command
            command = self._led_state_to_command(led_state)
            
            # Send command to drone
            if self.sentinel_integration:
                return await self.sentinel_integration.send_drone_command(
                    drone_id=drone_id,
                    command="DO_SET_LED",
                    parameters=command,
                )
            
            logger.warning("Sentinel integration not available, LED command not sent")
            return False
        except Exception as e:
            logger.error(f"Error setting LED state for drone {drone_id}: {str(e)}")
            return False
    
    async def start_led_show(self, execution_id: str, drone_trajectories: Dict[str, List[LEDState]]) -> bool:
        """
        Start an LED show.
        
        Args:
            execution_id: Execution ID
            drone_trajectories: Dictionary of drone ID to list of LED states
            
        Returns:
            True if successful, False otherwise
        """
        try:
            # Store show data
            self._active_shows[execution_id] = {
                "drone_trajectories": drone_trajectories,
                "start_time": datetime.utcnow(),
                "running": True,
            }
            
            # Start background task
            asyncio.create_task(self._run_led_show(execution_id))
            
            return True
        except Exception as e:
            logger.error(f"Error starting LED show {execution_id}: {str(e)}")
            return False
    
    async def stop_led_show(self, execution_id: str) -> bool:
        """
        Stop an LED show.
        
        Args:
            execution_id: Execution ID
            
        Returns:
            True if successful, False otherwise
        """
        try:
            # Check if show exists
            if execution_id not in self._active_shows:
                logger.warning(f"LED show {execution_id} not found")
                return False
            
            # Stop show
            self._active_shows[execution_id]["running"] = False
            
            return True
        except Exception as e:
            logger.error(f"Error stopping LED show {execution_id}: {str(e)}")
            return False
    
    async def _run_led_show(self, execution_id: str):
        """
        Run an LED show.
        
        Args:
            execution_id: Execution ID
        """
        try:
            # Get show data
            show_data = self._active_shows[execution_id]
            drone_trajectories = show_data["drone_trajectories"]
            start_time = show_data["start_time"]
            
            # Run until stopped
            while show_data["running"]:
                # Calculate current time
                current_time = (datetime.utcnow() - start_time).total_seconds()
                
                # Update LED states for all drones
                for drone_id, led_states in drone_trajectories.items():
                    # Find current LED state
                    current_state = self._find_current_led_state(led_states, current_time)
                    
                    # Set LED state
                    if current_state:
                        await self.set_led_state(drone_id, current_state)
                
                # Sleep until next update
                await asyncio.sleep(1.0 / settings.LED_UPDATE_RATE)
            
            # Clean up
            del self._active_shows[execution_id]
        except Exception as e:
            logger.error(f"Error running LED show {execution_id}: {str(e)}")
    
    def _find_current_led_state(self, led_states: List[LEDState], current_time: float) -> Optional[LEDState]:
        """
        Find the current LED state based on time.
        
        Args:
            led_states: List of LED states
            current_time: Current time in seconds from start of show
            
        Returns:
            Current LED state
        """
        # Sort LED states by time
        sorted_states = sorted(led_states, key=lambda state: state.time)
        
        # Handle edge cases
        if not sorted_states:
            return None
        
        if current_time <= sorted_states[0].time:
            return sorted_states[0]
        
        if current_time >= sorted_states[-1].time:
            return sorted_states[-1]
        
        # Find surrounding LED states
        for i in range(len(sorted_states) - 1):
            if sorted_states[i].time <= current_time < sorted_states[i + 1].time:
                # For now, just return the previous state
                # In a real implementation, we would interpolate based on effect type
                return sorted_states[i]
        
        # Fallback
        return sorted_states[-1]
    
    def _led_state_to_command(self, led_state: LEDState) -> Dict[str, Any]:
        """
        Convert an LED state to a MAVLink command.
        
        Args:
            led_state: LED state
            
        Returns:
            MAVLink command parameters
        """
        # Extract color components
        r = led_state.color.r
        g = led_state.color.g
        b = led_state.color.b
        
        # Create command parameters
        command = {
            "index": 255,  # All LEDs
            "red": r,
            "green": g,
            "blue": b,
            "effect": led_state.effect,
        }
        
        # Add effect parameters if available
        if led_state.effect_params:
            command.update(led_state.effect_params)
        
        return command
