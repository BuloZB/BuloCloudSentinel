"""
Telemetry Adapter for SentinelWeb

This module provides integration with BuloCloudSentinel's telemetry API.
"""

import logging
import httpx
from typing import Dict, List, Optional, Any

logger = logging.getLogger(__name__)

class TelemetryAdapter:
    """
    Adapter for telemetry operations.
    
    This class provides methods to interact with BuloCloudSentinel's telemetry API.
    """
    
    def __init__(self, api_url: str, api_token: str):
        """Initialize the telemetry adapter.
        
        Args:
            api_url: The URL of the BuloCloudSentinel API.
            api_token: The API token for authentication.
        """
        self.api_url = api_url
        self.api_token = api_token
        self.headers = {
            "Authorization": f"Bearer {api_token}",
            "Content-Type": "application/json"
        }
        logger.info(f"Initialized TelemetryAdapter with API URL: {api_url}")
    
    async def get_telemetry(self, drone_id: str) -> Optional[Dict[str, Any]]:
        """Get the latest telemetry for a drone.
        
        Args:
            drone_id: The ID of the drone.
            
        Returns:
            The telemetry data or None if not available.
        """
        try:
            async with httpx.AsyncClient() as client:
                response = await client.get(
                    f"{self.api_url}/api/drones/{drone_id}/telemetry",
                    headers=self.headers,
                    timeout=10.0
                )
                response.raise_for_status()
                return response.json()
        except httpx.HTTPError as e:
            logger.error(f"Error getting telemetry for drone {drone_id}: {str(e)}")
            return None
    
    async def get_telemetry_history(self, drone_id: str, start_time: str = None, end_time: str = None, limit: int = 100) -> List[Dict[str, Any]]:
        """Get telemetry history for a drone.
        
        Args:
            drone_id: The ID of the drone.
            start_time: Optional start time for filtering (ISO format).
            end_time: Optional end time for filtering (ISO format).
            limit: Maximum number of records to return.
            
        Returns:
            A list of telemetry records.
        """
        params = {"limit": limit}
        if start_time:
            params["start_time"] = start_time
        if end_time:
            params["end_time"] = end_time
        
        try:
            async with httpx.AsyncClient() as client:
                response = await client.get(
                    f"{self.api_url}/api/drones/{drone_id}/telemetry/history",
                    headers=self.headers,
                    params=params,
                    timeout=10.0
                )
                response.raise_for_status()
                return response.json()
        except httpx.HTTPError as e:
            logger.error(f"Error getting telemetry history for drone {drone_id}: {str(e)}")
            return []
    
    async def get_battery_status(self, drone_id: str) -> Optional[Dict[str, Any]]:
        """Get the battery status for a drone.
        
        Args:
            drone_id: The ID of the drone.
            
        Returns:
            The battery status data or None if not available.
        """
        try:
            async with httpx.AsyncClient() as client:
                response = await client.get(
                    f"{self.api_url}/api/drones/{drone_id}/battery",
                    headers=self.headers,
                    timeout=10.0
                )
                response.raise_for_status()
                return response.json()
        except httpx.HTTPError as e:
            logger.error(f"Error getting battery status for drone {drone_id}: {str(e)}")
            return None
    
    async def get_gps_position(self, drone_id: str) -> Optional[Dict[str, Any]]:
        """Get the GPS position for a drone.
        
        Args:
            drone_id: The ID of the drone.
            
        Returns:
            The GPS position data or None if not available.
        """
        try:
            async with httpx.AsyncClient() as client:
                response = await client.get(
                    f"{self.api_url}/api/drones/{drone_id}/gps",
                    headers=self.headers,
                    timeout=10.0
                )
                response.raise_for_status()
                return response.json()
        except httpx.HTTPError as e:
            logger.error(f"Error getting GPS position for drone {drone_id}: {str(e)}")
            return None
