"""
Synchronization service for the Drone Show microservice.

This module provides services for time synchronization between drones.
"""

import logging
import asyncio
import time
from typing import Dict, Any, List, Optional
from datetime import datetime

from drone_show_service.core.config import settings
from drone_show_service.services.sentinel_integration import SentinelIntegrationService

logger = logging.getLogger(__name__)


class SynchronizationService:
    """
    Service for time synchronization between drones.
    
    This service provides methods for synchronizing time between drones
    to ensure coordinated execution of choreographies.
    """
    
    def __init__(self, sentinel_integration: Optional[SentinelIntegrationService] = None):
        """
        Initialize the synchronization service.
        
        Args:
            sentinel_integration: Sentinel integration service
        """
        self.sentinel_integration = sentinel_integration
        self._active_syncs: Dict[str, Dict[str, Any]] = {}
    
    async def start_synchronization(self, execution_id: str, drone_ids: List[str]) -> bool:
        """
        Start time synchronization for a set of drones.
        
        Args:
            execution_id: Execution ID
            drone_ids: List of drone IDs
            
        Returns:
            True if successful, False otherwise
        """
        try:
            # Store sync data
            self._active_syncs[execution_id] = {
                "drone_ids": drone_ids,
                "start_time": datetime.utcnow(),
                "running": True,
                "offsets": {},
            }
            
            # Start background task
            asyncio.create_task(self._run_synchronization(execution_id))
            
            return True
        except Exception as e:
            logger.error(f"Error starting synchronization for execution {execution_id}: {str(e)}")
            return False
    
    async def stop_synchronization(self, execution_id: str) -> bool:
        """
        Stop time synchronization.
        
        Args:
            execution_id: Execution ID
            
        Returns:
            True if successful, False otherwise
        """
        try:
            # Check if sync exists
            if execution_id not in self._active_syncs:
                logger.warning(f"Synchronization for execution {execution_id} not found")
                return False
            
            # Stop sync
            self._active_syncs[execution_id]["running"] = False
            
            return True
        except Exception as e:
            logger.error(f"Error stopping synchronization for execution {execution_id}: {str(e)}")
            return False
    
    async def get_time_offsets(self, execution_id: str) -> Dict[str, float]:
        """
        Get time offsets for drones.
        
        Args:
            execution_id: Execution ID
            
        Returns:
            Dictionary of drone ID to time offset in seconds
        """
        # Check if sync exists
        if execution_id not in self._active_syncs:
            logger.warning(f"Synchronization for execution {execution_id} not found")
            return {}
        
        # Return offsets
        return self._active_syncs[execution_id]["offsets"]
    
    async def _run_synchronization(self, execution_id: str):
        """
        Run time synchronization.
        
        Args:
            execution_id: Execution ID
        """
        try:
            # Get sync data
            sync_data = self._active_syncs[execution_id]
            drone_ids = sync_data["drone_ids"]
            
            # Run until stopped
            while sync_data["running"]:
                # Synchronize time for all drones
                for drone_id in drone_ids:
                    # Measure time offset
                    offset = await self._measure_time_offset(drone_id)
                    
                    # Store offset
                    if offset is not None:
                        sync_data["offsets"][drone_id] = offset
                
                # Sleep until next sync
                await asyncio.sleep(settings.TIME_SYNC_INTERVAL)
            
            # Clean up
            del self._active_syncs[execution_id]
        except Exception as e:
            logger.error(f"Error running synchronization for execution {execution_id}: {str(e)}")
    
    async def _measure_time_offset(self, drone_id: str) -> Optional[float]:
        """
        Measure time offset for a drone.
        
        Args:
            drone_id: Drone ID
            
        Returns:
            Time offset in seconds (positive if drone time is ahead)
        """
        try:
            # Check if sentinel integration is available
            if not self.sentinel_integration:
                logger.warning("Sentinel integration not available, time offset not measured")
                return None
            
            # Get drone time
            response = await self.sentinel_integration.send_drone_command(
                drone_id=drone_id,
                command="GET_TIME",
                parameters={},
            )
            
            # Check response
            if not response or "time" not in response:
                logger.warning(f"Failed to get time from drone {drone_id}")
                return None
            
            # Calculate offset
            drone_time = float(response["time"])
            local_time = time.time()
            offset = drone_time - local_time
            
            logger.debug(f"Time offset for drone {drone_id}: {offset:.6f} seconds")
            
            return offset
        except Exception as e:
            logger.error(f"Error measuring time offset for drone {drone_id}: {str(e)}")
            return None
    
    async def synchronize_start_time(self, execution_id: str, start_time: float) -> bool:
        """
        Synchronize start time for all drones.
        
        Args:
            execution_id: Execution ID
            start_time: Start time in seconds (UNIX timestamp)
            
        Returns:
            True if successful, False otherwise
        """
        try:
            # Check if sync exists
            if execution_id not in self._active_syncs:
                logger.warning(f"Synchronization for execution {execution_id} not found")
                return False
            
            # Get sync data
            sync_data = self._active_syncs[execution_id]
            drone_ids = sync_data["drone_ids"]
            offsets = sync_data["offsets"]
            
            # Synchronize start time for all drones
            success = True
            for drone_id in drone_ids:
                # Calculate adjusted start time
                adjusted_start_time = start_time
                if drone_id in offsets:
                    adjusted_start_time -= offsets[drone_id]
                
                # Send command to drone
                if self.sentinel_integration:
                    result = await self.sentinel_integration.send_drone_command(
                        drone_id=drone_id,
                        command="SET_START_TIME",
                        parameters={"time": adjusted_start_time},
                    )
                    
                    if not result:
                        logger.warning(f"Failed to set start time for drone {drone_id}")
                        success = False
                else:
                    logger.warning("Sentinel integration not available, start time not set")
                    success = False
            
            return success
        except Exception as e:
            logger.error(f"Error synchronizing start time for execution {execution_id}: {str(e)}")
            return False
