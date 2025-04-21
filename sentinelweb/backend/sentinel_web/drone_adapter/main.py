"""
Drone Adapter for SentinelWeb

This module provides integration with BuloCloudSentinel's drone API.
"""

import logging
import httpx
from typing import Dict, List, Optional, Any

logger = logging.getLogger(__name__)

class DroneAdapter:
    """
    Adapter for drone operations.
    
    This class provides methods to interact with BuloCloudSentinel's drone API.
    """
    
    def __init__(self, api_url: str, api_token: str):
        """Initialize the drone adapter.
        
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
        logger.info(f"Initialized DroneAdapter with API URL: {api_url}")
    
    async def get_drones(self) -> List[Dict[str, Any]]:
        """Get all drones.
        
        Returns:
            A list of drone objects.
        """
        try:
            async with httpx.AsyncClient() as client:
                response = await client.get(
                    f"{self.api_url}/api/drones",
                    headers=self.headers,
                    timeout=10.0
                )
                response.raise_for_status()
                return response.json()
        except httpx.HTTPError as e:
            logger.error(f"Error getting drones: {str(e)}")
            return []
    
    async def get_drone(self, drone_id: str) -> Optional[Dict[str, Any]]:
        """Get a drone by ID.
        
        Args:
            drone_id: The ID of the drone.
            
        Returns:
            The drone object or None if not found.
        """
        try:
            async with httpx.AsyncClient() as client:
                response = await client.get(
                    f"{self.api_url}/api/drones/{drone_id}",
                    headers=self.headers,
                    timeout=10.0
                )
                response.raise_for_status()
                return response.json()
        except httpx.HTTPError as e:
            logger.error(f"Error getting drone {drone_id}: {str(e)}")
            return None
    
    async def send_command(self, drone_id: str, command: str, params: Dict[str, Any] = None) -> bool:
        """Send a command to a drone.
        
        Args:
            drone_id: The ID of the drone.
            command: The command to send.
            params: Optional parameters for the command.
            
        Returns:
            True if the command was sent successfully, False otherwise.
        """
        if params is None:
            params = {}
        
        try:
            async with httpx.AsyncClient() as client:
                response = await client.post(
                    f"{self.api_url}/api/drones/{drone_id}/command",
                    headers=self.headers,
                    json={"command": command, "params": params},
                    timeout=10.0
                )
                response.raise_for_status()
                return True
        except httpx.HTTPError as e:
            logger.error(f"Error sending command to drone {drone_id}: {str(e)}")
            return False
