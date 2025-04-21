"""
Drone Client for SentinelWeb

This module provides a client for interacting with BuloCloudSentinel's drone API.
"""

import httpx
import logging
from typing import Dict, List, Any, Optional
from pydantic import BaseModel

logger = logging.getLogger(__name__)

class DroneInfo(BaseModel):
    """Drone information model."""
    id: str
    name: str
    model: str
    status: str
    battery_level: Optional[float] = None
    location: Optional[Dict[str, float]] = None
    is_active: bool = True

class DroneClient:
    """
    Client for drone operations.
    
    This class provides methods for interacting with BuloCloudSentinel's drone API.
    """
    
    def __init__(self, api_url: str, api_key: Optional[str] = None):
        """
        Initialize the drone client.
        
        Args:
            api_url: URL of the BuloCloudSentinel API
            api_key: Optional API key for authentication
        """
        self.api_url = api_url.rstrip('/')
        self.api_key = api_key
        self.headers = {}
        
        if api_key:
            self.headers["Authorization"] = f"Bearer {api_key}"
    
    async def list_drones(self) -> List[DroneInfo]:
        """
        List all available drones.
        
        Returns:
            List of drone information
        """
        try:
            async with httpx.AsyncClient() as client:
                response = await client.get(
                    f"{self.api_url}/api/device-inventory/drones",
                    headers=self.headers,
                    timeout=10.0
                )
                response.raise_for_status()
                
                drones_data = response.json()
                return [DroneInfo(**drone) for drone in drones_data]
        except Exception as e:
            logger.error(f"Error listing drones: {str(e)}")
            return []
    
    async def get_drone(self, drone_id: str) -> Optional[DroneInfo]:
        """
        Get information about a specific drone.
        
        Args:
            drone_id: ID of the drone
            
        Returns:
            Drone information if found, None otherwise
        """
        try:
            async with httpx.AsyncClient() as client:
                response = await client.get(
                    f"{self.api_url}/api/device-inventory/drones/{drone_id}",
                    headers=self.headers,
                    timeout=10.0
                )
                response.raise_for_status()
                
                drone_data = response.json()
                return DroneInfo(**drone_data)
        except Exception as e:
            logger.error(f"Error getting drone {drone_id}: {str(e)}")
            return None
    
    async def send_command(self, drone_id: str, command: str, parameters: Dict[str, Any] = None) -> Dict[str, Any]:
        """
        Send a command to a drone.
        
        Args:
            drone_id: ID of the drone
            command: Command to send
            parameters: Optional command parameters
            
        Returns:
            Command response
        """
        try:
            data = {
                "command": command,
                "parameters": parameters or {}
            }
            
            async with httpx.AsyncClient() as client:
                response = await client.post(
                    f"{self.api_url}/api/device-inventory/drones/{drone_id}/command",
                    headers=self.headers,
                    json=data,
                    timeout=30.0
                )
                response.raise_for_status()
                
                return response.json()
        except Exception as e:
            logger.error(f"Error sending command to drone {drone_id}: {str(e)}")
            return {"error": str(e)}
    
    async def get_drone_status(self, drone_id: str) -> Dict[str, Any]:
        """
        Get the current status of a drone.
        
        Args:
            drone_id: ID of the drone
            
        Returns:
            Drone status information
        """
        try:
            async with httpx.AsyncClient() as client:
                response = await client.get(
                    f"{self.api_url}/api/device-inventory/drones/{drone_id}/status",
                    headers=self.headers,
                    timeout=10.0
                )
                response.raise_for_status()
                
                return response.json()
        except Exception as e:
            logger.error(f"Error getting status for drone {drone_id}: {str(e)}")
            return {"status": "unknown", "error": str(e)}
