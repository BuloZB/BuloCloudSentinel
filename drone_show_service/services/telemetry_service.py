"""
Telemetry service for the Drone Show microservice.

This module provides services for receiving and processing drone telemetry.
"""

import logging
import asyncio
import json
from typing import Dict, Any, List, Optional, Callable, AsyncGenerator
from datetime import datetime

from drone_show_service.models.choreography import Position, DroneStatus
from drone_show_service.core.config import settings
from drone_show_service.services.sentinel_integration import SentinelIntegrationService

logger = logging.getLogger(__name__)


class TelemetryService:
    """
    Service for receiving and processing drone telemetry.
    
    This service provides methods for receiving telemetry from drones
    and processing it for monitoring and logging.
    """
    
    def __init__(self, sentinel_integration: Optional[SentinelIntegrationService] = None):
        """
        Initialize the telemetry service.
        
        Args:
            sentinel_integration: Sentinel integration service
        """
        self.sentinel_integration = sentinel_integration
        self._active_telemetry: Dict[str, Dict[str, Any]] = {}
        self._telemetry_callbacks: Dict[str, List[Callable[[str, Dict[str, Any]], None]]] = {}
    
    async def start_telemetry(self, execution_id: str, drone_ids: List[str]) -> bool:
        """
        Start receiving telemetry for a set of drones.
        
        Args:
            execution_id: Execution ID
            drone_ids: List of drone IDs
            
        Returns:
            True if successful, False otherwise
        """
        try:
            # Store telemetry data
            self._active_telemetry[execution_id] = {
                "drone_ids": drone_ids,
                "start_time": datetime.utcnow(),
                "running": True,
                "telemetry": {},
            }
            
            # Start background task
            asyncio.create_task(self._run_telemetry(execution_id))
            
            return True
        except Exception as e:
            logger.error(f"Error starting telemetry for execution {execution_id}: {str(e)}")
            return False
    
    async def stop_telemetry(self, execution_id: str) -> bool:
        """
        Stop receiving telemetry.
        
        Args:
            execution_id: Execution ID
            
        Returns:
            True if successful, False otherwise
        """
        try:
            # Check if telemetry exists
            if execution_id not in self._active_telemetry:
                logger.warning(f"Telemetry for execution {execution_id} not found")
                return False
            
            # Stop telemetry
            self._active_telemetry[execution_id]["running"] = False
            
            return True
        except Exception as e:
            logger.error(f"Error stopping telemetry for execution {execution_id}: {str(e)}")
            return False
    
    async def get_telemetry(self, execution_id: str) -> Dict[str, Dict[str, Any]]:
        """
        Get telemetry for all drones.
        
        Args:
            execution_id: Execution ID
            
        Returns:
            Dictionary of drone ID to telemetry data
        """
        # Check if telemetry exists
        if execution_id not in self._active_telemetry:
            logger.warning(f"Telemetry for execution {execution_id} not found")
            return {}
        
        # Return telemetry
        return self._active_telemetry[execution_id]["telemetry"]
    
    async def get_drone_status(self, execution_id: str, drone_id: str) -> Optional[DroneStatus]:
        """
        Get status for a specific drone.
        
        Args:
            execution_id: Execution ID
            drone_id: Drone ID
            
        Returns:
            Drone status
        """
        # Check if telemetry exists
        if execution_id not in self._active_telemetry:
            logger.warning(f"Telemetry for execution {execution_id} not found")
            return None
        
        # Check if drone telemetry exists
        telemetry = self._active_telemetry[execution_id]["telemetry"]
        if drone_id not in telemetry:
            logger.warning(f"Telemetry for drone {drone_id} not found")
            return None
        
        # Convert telemetry to drone status
        return self._telemetry_to_status(drone_id, telemetry[drone_id])
    
    def register_telemetry_callback(
        self, execution_id: str, callback: Callable[[str, Dict[str, Any]], None]
    ) -> bool:
        """
        Register a callback for telemetry updates.
        
        Args:
            execution_id: Execution ID
            callback: Callback function (drone_id, telemetry) -> None
            
        Returns:
            True if successful, False otherwise
        """
        try:
            # Initialize callbacks list if needed
            if execution_id not in self._telemetry_callbacks:
                self._telemetry_callbacks[execution_id] = []
            
            # Add callback
            self._telemetry_callbacks[execution_id].append(callback)
            
            return True
        except Exception as e:
            logger.error(f"Error registering telemetry callback for execution {execution_id}: {str(e)}")
            return False
    
    def unregister_telemetry_callback(
        self, execution_id: str, callback: Callable[[str, Dict[str, Any]], None]
    ) -> bool:
        """
        Unregister a telemetry callback.
        
        Args:
            execution_id: Execution ID
            callback: Callback function
            
        Returns:
            True if successful, False otherwise
        """
        try:
            # Check if callbacks exist
            if execution_id not in self._telemetry_callbacks:
                logger.warning(f"Telemetry callbacks for execution {execution_id} not found")
                return False
            
            # Remove callback
            if callback in self._telemetry_callbacks[execution_id]:
                self._telemetry_callbacks[execution_id].remove(callback)
            
            return True
        except Exception as e:
            logger.error(f"Error unregistering telemetry callback for execution {execution_id}: {str(e)}")
            return False
    
    async def get_telemetry_stream(self, execution_id: str) -> AsyncGenerator[Dict[str, Dict[str, Any]], None]:
        """
        Get a stream of telemetry updates.
        
        Args:
            execution_id: Execution ID
            
        Yields:
            Dictionary of drone ID to telemetry data
        """
        # Check if telemetry exists
        if execution_id not in self._active_telemetry:
            logger.warning(f"Telemetry for execution {execution_id} not found")
            return
        
        # Create queue for updates
        queue = asyncio.Queue()
        
        # Define callback
        def callback(drone_id: str, telemetry: Dict[str, Any]):
            # Add to queue
            asyncio.create_task(queue.put((drone_id, telemetry)))
        
        # Register callback
        self.register_telemetry_callback(execution_id, callback)
        
        try:
            # Yield initial telemetry
            yield self._active_telemetry[execution_id]["telemetry"]
            
            # Yield updates
            while execution_id in self._active_telemetry and self._active_telemetry[execution_id]["running"]:
                # Wait for update
                drone_id, telemetry = await queue.get()
                
                # Yield update
                yield {drone_id: telemetry}
        finally:
            # Unregister callback
            self.unregister_telemetry_callback(execution_id, callback)
    
    async def _run_telemetry(self, execution_id: str):
        """
        Run telemetry collection.
        
        Args:
            execution_id: Execution ID
        """
        try:
            # Get telemetry data
            telemetry_data = self._active_telemetry[execution_id]
            drone_ids = telemetry_data["drone_ids"]
            
            # Check if sentinel integration is available
            if not self.sentinel_integration:
                logger.warning("Sentinel integration not available, telemetry not collected")
                return
            
            # Start telemetry streams for all drones
            streams = {}
            for drone_id in drone_ids:
                # Start stream
                stream = self.sentinel_integration.get_telemetry_stream(drone_id)
                streams[drone_id] = stream
            
            # Process telemetry until stopped
            while telemetry_data["running"]:
                # Process telemetry for all drones
                for drone_id, stream in streams.items():
                    try:
                        # Get telemetry
                        telemetry = await anext(stream)
                        
                        # Store telemetry
                        telemetry_data["telemetry"][drone_id] = telemetry
                        
                        # Call callbacks
                        if execution_id in self._telemetry_callbacks:
                            for callback in self._telemetry_callbacks[execution_id]:
                                try:
                                    callback(drone_id, telemetry)
                                except Exception as e:
                                    logger.error(f"Error in telemetry callback: {str(e)}")
                    except StopAsyncIteration:
                        # Stream ended
                        logger.warning(f"Telemetry stream for drone {drone_id} ended")
                        streams[drone_id] = None
                    except Exception as e:
                        logger.error(f"Error processing telemetry for drone {drone_id}: {str(e)}")
                
                # Remove ended streams
                streams = {drone_id: stream for drone_id, stream in streams.items() if stream is not None}
                
                # Check if all streams ended
                if not streams:
                    logger.warning("All telemetry streams ended")
                    break
                
                # Sleep briefly
                await asyncio.sleep(0.1)
            
            # Clean up
            del self._active_telemetry[execution_id]
            if execution_id in self._telemetry_callbacks:
                del self._telemetry_callbacks[execution_id]
        except Exception as e:
            logger.error(f"Error running telemetry for execution {execution_id}: {str(e)}")
    
    def _telemetry_to_status(self, drone_id: str, telemetry: Dict[str, Any]) -> DroneStatus:
        """
        Convert telemetry to drone status.
        
        Args:
            drone_id: Drone ID
            telemetry: Telemetry data
            
        Returns:
            Drone status
        """
        # Extract position
        position = None
        if "lat" in telemetry and "lon" in telemetry and "alt" in telemetry:
            position = Position(
                lat=float(telemetry["lat"]),
                lon=float(telemetry["lon"]),
                alt=float(telemetry["alt"]),
            )
        
        # Create drone status
        return DroneStatus(
            drone_id=drone_id,
            connected=telemetry.get("connected", False),
            battery_level=float(telemetry.get("battery", 0)),
            position=position,
            heading=float(telemetry.get("heading", 0)) if "heading" in telemetry else None,
            led_state=None,  # LED state not available in telemetry
            current_waypoint_index=int(telemetry.get("waypoint_index", 0)) if "waypoint_index" in telemetry else None,
            status=telemetry.get("status", "unknown"),
            error_message=telemetry.get("error_message"),
        )
