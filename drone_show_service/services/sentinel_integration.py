"""
Sentinel integration service for the Drone Show microservice.

This module provides services for integrating with the Bulo.Cloud Sentinel platform.
"""

import logging
import json
import asyncio
from typing import Dict, Any, List, Optional, AsyncGenerator
import httpx

from drone_show_service.core.config import settings

logger = logging.getLogger(__name__)


class SentinelIntegrationService:
    """
    Service for integrating with the Bulo.Cloud Sentinel platform.
    
    This service provides methods for communicating with the Bulo.Cloud Sentinel API,
    including sending commands to drones and receiving telemetry.
    """
    
    def __init__(self, api_url: Optional[str] = None, api_token: Optional[str] = None):
        """
        Initialize the Sentinel integration service.
        
        Args:
            api_url: Bulo.Cloud Sentinel API URL
            api_token: API token for authentication
        """
        self.api_url = api_url or settings.SENTINEL_API_URL
        self.api_token = api_token or settings.SENTINEL_API_TOKEN
        self.headers = {
            "Authorization": f"Bearer {self.api_token}",
            "Content-Type": "application/json",
        }
    
    async def send_drone_command(self, drone_id: str, command: str, parameters: Dict[str, Any]) -> Dict[str, Any]:
        """
        Send a command to a drone.
        
        Args:
            drone_id: Drone ID
            command: Command name
            parameters: Command parameters
            
        Returns:
            Command response
        """
        try:
            # Prepare request data
            data = {
                "command": command,
                "parameters": parameters,
            }
            
            # Send request
            async with httpx.AsyncClient() as client:
                response = await client.post(
                    f"{self.api_url}/api/device-inventory/drones/{drone_id}/command",
                    headers=self.headers,
                    json=data,
                    timeout=30.0,
                )
                
                # Check response
                response.raise_for_status()
                
                # Return response data
                return response.json()
        except Exception as e:
            logger.error(f"Error sending command to drone {drone_id}: {str(e)}")
            return {"error": str(e)}
    
    async def get_drone_info(self, drone_id: str) -> Dict[str, Any]:
        """
        Get information about a drone.
        
        Args:
            drone_id: Drone ID
            
        Returns:
            Drone information
        """
        try:
            # Send request
            async with httpx.AsyncClient() as client:
                response = await client.get(
                    f"{self.api_url}/api/device-inventory/drones/{drone_id}",
                    headers=self.headers,
                    timeout=10.0,
                )
                
                # Check response
                response.raise_for_status()
                
                # Return response data
                return response.json()
        except Exception as e:
            logger.error(f"Error getting drone info for {drone_id}: {str(e)}")
            return {"error": str(e)}
    
    async def get_drones(self) -> List[Dict[str, Any]]:
        """
        Get all available drones.
        
        Returns:
            List of drones
        """
        try:
            # Send request
            async with httpx.AsyncClient() as client:
                response = await client.get(
                    f"{self.api_url}/api/device-inventory/drones",
                    headers=self.headers,
                    timeout=10.0,
                )
                
                # Check response
                response.raise_for_status()
                
                # Return response data
                return response.json()
        except Exception as e:
            logger.error(f"Error getting drones: {str(e)}")
            return []
    
    async def get_telemetry(self, drone_id: str) -> Dict[str, Any]:
        """
        Get telemetry for a drone.
        
        Args:
            drone_id: Drone ID
            
        Returns:
            Telemetry data
        """
        try:
            # Send request
            async with httpx.AsyncClient() as client:
                response = await client.get(
                    f"{self.api_url}/api/device-inventory/drones/{drone_id}/telemetry",
                    headers=self.headers,
                    timeout=10.0,
                )
                
                # Check response
                response.raise_for_status()
                
                # Return response data
                return response.json()
        except Exception as e:
            logger.error(f"Error getting telemetry for drone {drone_id}: {str(e)}")
            return {"error": str(e)}
    
    async def get_telemetry_stream(self, drone_id: str) -> AsyncGenerator[Dict[str, Any], None]:
        """
        Get a telemetry stream for a drone.
        
        Args:
            drone_id: Drone ID
            
        Yields:
            Telemetry data
        """
        try:
            # Convert HTTP URL to WebSocket URL
            url = f"{self.api_url}/api/ws/telemetry/{drone_id}"
            if url.startswith("http://"):
                url = url.replace("http://", "ws://")
            elif url.startswith("https://"):
                url = url.replace("https://", "wss://")
            
            # Connect to WebSocket
            async with httpx.AsyncClient() as client:
                async with client.stream("GET", url, headers=self.headers, timeout=None) as response:
                    async for line in response.aiter_lines():
                        if not line.strip():
                            continue
                        
                        try:
                            # Parse JSON
                            data = json.loads(line)
                            
                            # Yield data
                            yield data
                        except Exception as e:
                            logger.error(f"Error parsing telemetry data: {str(e)}")
        except Exception as e:
            logger.error(f"Error streaming telemetry for drone {drone_id}: {str(e)}")
    
    async def create_mission(self, mission_data: Dict[str, Any]) -> Dict[str, Any]:
        """
        Create a mission.
        
        Args:
            mission_data: Mission data
            
        Returns:
            Created mission
        """
        try:
            # Send request
            async with httpx.AsyncClient() as client:
                response = await client.post(
                    f"{self.api_url}/api/mission-planning/missions",
                    headers=self.headers,
                    json=mission_data,
                    timeout=10.0,
                )
                
                # Check response
                response.raise_for_status()
                
                # Return response data
                return response.json()
        except Exception as e:
            logger.error(f"Error creating mission: {str(e)}")
            return {"error": str(e)}
    
    async def execute_mission(self, mission_id: str, drone_id: str) -> Dict[str, Any]:
        """
        Execute a mission.
        
        Args:
            mission_id: Mission ID
            drone_id: Drone ID
            
        Returns:
            Execution response
        """
        try:
            # Prepare request data
            data = {
                "mission_id": mission_id,
                "drone_id": drone_id,
            }
            
            # Send request
            async with httpx.AsyncClient() as client:
                response = await client.post(
                    f"{self.api_url}/api/mission-execution/execute",
                    headers=self.headers,
                    json=data,
                    timeout=10.0,
                )
                
                # Check response
                response.raise_for_status()
                
                # Return response data
                return response.json()
        except Exception as e:
            logger.error(f"Error executing mission {mission_id}: {str(e)}")
            return {"error": str(e)}
