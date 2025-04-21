"""
Mission Adapter for SentinelWeb

This module provides integration with BuloCloudSentinel's mission API.
"""

import logging
import httpx
from typing import Dict, List, Optional, Any

logger = logging.getLogger(__name__)

class MissionAdapter:
    """
    Adapter for mission operations.
    
    This class provides methods to interact with BuloCloudSentinel's mission API.
    """
    
    def __init__(self, api_url: str, api_token: str):
        """Initialize the mission adapter.
        
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
        logger.info(f"Initialized MissionAdapter with API URL: {api_url}")
    
    async def get_missions(self) -> List[Dict[str, Any]]:
        """Get all missions.
        
        Returns:
            A list of mission objects.
        """
        try:
            async with httpx.AsyncClient() as client:
                response = await client.get(
                    f"{self.api_url}/api/missions",
                    headers=self.headers,
                    timeout=10.0
                )
                response.raise_for_status()
                return response.json()
        except httpx.HTTPError as e:
            logger.error(f"Error getting missions: {str(e)}")
            return []
    
    async def get_mission(self, mission_id: str) -> Optional[Dict[str, Any]]:
        """Get a mission by ID.
        
        Args:
            mission_id: The ID of the mission.
            
        Returns:
            The mission object or None if not found.
        """
        try:
            async with httpx.AsyncClient() as client:
                response = await client.get(
                    f"{self.api_url}/api/missions/{mission_id}",
                    headers=self.headers,
                    timeout=10.0
                )
                response.raise_for_status()
                return response.json()
        except httpx.HTTPError as e:
            logger.error(f"Error getting mission {mission_id}: {str(e)}")
            return None
    
    async def create_mission(self, mission_data: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        """Create a new mission.
        
        Args:
            mission_data: The mission data.
            
        Returns:
            The created mission object or None if creation failed.
        """
        try:
            async with httpx.AsyncClient() as client:
                response = await client.post(
                    f"{self.api_url}/api/missions",
                    headers=self.headers,
                    json=mission_data,
                    timeout=10.0
                )
                response.raise_for_status()
                return response.json()
        except httpx.HTTPError as e:
            logger.error(f"Error creating mission: {str(e)}")
            return None
    
    async def update_mission(self, mission_id: str, mission_data: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        """Update a mission.
        
        Args:
            mission_id: The ID of the mission.
            mission_data: The updated mission data.
            
        Returns:
            The updated mission object or None if update failed.
        """
        try:
            async with httpx.AsyncClient() as client:
                response = await client.put(
                    f"{self.api_url}/api/missions/{mission_id}",
                    headers=self.headers,
                    json=mission_data,
                    timeout=10.0
                )
                response.raise_for_status()
                return response.json()
        except httpx.HTTPError as e:
            logger.error(f"Error updating mission {mission_id}: {str(e)}")
            return None
    
    async def delete_mission(self, mission_id: str) -> bool:
        """Delete a mission.
        
        Args:
            mission_id: The ID of the mission.
            
        Returns:
            True if the mission was deleted successfully, False otherwise.
        """
        try:
            async with httpx.AsyncClient() as client:
                response = await client.delete(
                    f"{self.api_url}/api/missions/{mission_id}",
                    headers=self.headers,
                    timeout=10.0
                )
                response.raise_for_status()
                return True
        except httpx.HTTPError as e:
            logger.error(f"Error deleting mission {mission_id}: {str(e)}")
            return False
