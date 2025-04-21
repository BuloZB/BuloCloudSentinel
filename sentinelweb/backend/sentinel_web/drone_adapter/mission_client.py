"""
Mission Client for SentinelWeb

This module provides a client for interacting with BuloCloudSentinel's mission API.
"""

import httpx
import logging
from typing import Dict, List, Any, Optional
from pydantic import BaseModel
from datetime import datetime

logger = logging.getLogger(__name__)

class Waypoint(BaseModel):
    """Waypoint model."""
    latitude: float
    longitude: float
    altitude: float
    heading: Optional[float] = None
    speed: Optional[float] = None
    action: Optional[str] = None
    hold_time: Optional[int] = None
    parameters: Dict[str, Any] = {}

class Mission(BaseModel):
    """Mission model."""
    id: str
    name: str
    description: Optional[str] = None
    created_by: str
    created_at: datetime
    updated_at: datetime
    status: str
    drone_id: Optional[str] = None
    waypoints: List[Waypoint]
    parameters: Dict[str, Any] = {}

class MissionClient:
    """
    Client for mission operations.
    
    This class provides methods for interacting with BuloCloudSentinel's mission API.
    """
    
    def __init__(self, api_url: str, api_key: Optional[str] = None):
        """
        Initialize the mission client.
        
        Args:
            api_url: URL of the BuloCloudSentinel API
            api_key: Optional API key for authentication
        """
        self.api_url = api_url.rstrip('/')
        self.api_key = api_key
        self.headers = {}
        
        if api_key:
            self.headers["Authorization"] = f"Bearer {api_key}"
    
    async def list_missions(
        self, 
        status: Optional[str] = None,
        drone_id: Optional[str] = None
    ) -> List[Mission]:
        """
        List all missions with optional filtering.
        
        Args:
            status: Optional status filter
            drone_id: Optional drone ID filter
            
        Returns:
            List of missions
        """
        try:
            params = {}
            if status:
                params["status"] = status
            if drone_id:
                params["drone_id"] = drone_id
            
            async with httpx.AsyncClient() as client:
                response = await client.get(
                    f"{self.api_url}/api/missions",
                    params=params,
                    headers=self.headers,
                    timeout=10.0
                )
                response.raise_for_status()
                
                missions_data = response.json()
                return [Mission(**mission) for mission in missions_data]
        except Exception as e:
            logger.error(f"Error listing missions: {str(e)}")
            return []
    
    async def get_mission(self, mission_id: str) -> Optional[Mission]:
        """
        Get a specific mission.
        
        Args:
            mission_id: ID of the mission
            
        Returns:
            Mission if found, None otherwise
        """
        try:
            async with httpx.AsyncClient() as client:
                response = await client.get(
                    f"{self.api_url}/api/missions/{mission_id}",
                    headers=self.headers,
                    timeout=10.0
                )
                response.raise_for_status()
                
                mission_data = response.json()
                return Mission(**mission_data)
        except Exception as e:
            logger.error(f"Error getting mission {mission_id}: {str(e)}")
            return None
    
    async def create_mission(self, mission_data: Dict[str, Any]) -> Optional[Mission]:
        """
        Create a new mission.
        
        Args:
            mission_data: Mission data
            
        Returns:
            Created mission if successful, None otherwise
        """
        try:
            async with httpx.AsyncClient() as client:
                response = await client.post(
                    f"{self.api_url}/api/missions",
                    json=mission_data,
                    headers=self.headers,
                    timeout=10.0
                )
                response.raise_for_status()
                
                created_mission = response.json()
                return Mission(**created_mission)
        except Exception as e:
            logger.error(f"Error creating mission: {str(e)}")
            return None
    
    async def update_mission(self, mission_id: str, mission_data: Dict[str, Any]) -> Optional[Mission]:
        """
        Update a mission.
        
        Args:
            mission_id: ID of the mission to update
            mission_data: Updated mission data
            
        Returns:
            Updated mission if successful, None otherwise
        """
        try:
            async with httpx.AsyncClient() as client:
                response = await client.put(
                    f"{self.api_url}/api/missions/{mission_id}",
                    json=mission_data,
                    headers=self.headers,
                    timeout=10.0
                )
                response.raise_for_status()
                
                updated_mission = response.json()
                return Mission(**updated_mission)
        except Exception as e:
            logger.error(f"Error updating mission {mission_id}: {str(e)}")
            return None
    
    async def delete_mission(self, mission_id: str) -> bool:
        """
        Delete a mission.
        
        Args:
            mission_id: ID of the mission to delete
            
        Returns:
            True if successful, False otherwise
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
        except Exception as e:
            logger.error(f"Error deleting mission {mission_id}: {str(e)}")
            return False
    
    async def execute_mission(self, mission_id: str) -> Dict[str, Any]:
        """
        Execute a mission.
        
        Args:
            mission_id: ID of the mission to execute
            
        Returns:
            Execution response
        """
        try:
            async with httpx.AsyncClient() as client:
                response = await client.post(
                    f"{self.api_url}/api/missions/{mission_id}/execute",
                    headers=self.headers,
                    timeout=10.0
                )
                response.raise_for_status()
                
                return response.json()
        except Exception as e:
            logger.error(f"Error executing mission {mission_id}: {str(e)}")
            return {"error": str(e)}
    
    async def simulate_mission(self, mission_id: str) -> Dict[str, Any]:
        """
        Simulate a mission.
        
        Args:
            mission_id: ID of the mission to simulate
            
        Returns:
            Simulation response
        """
        try:
            async with httpx.AsyncClient() as client:
                response = await client.post(
                    f"{self.api_url}/api/missions/{mission_id}/simulate",
                    headers=self.headers,
                    timeout=10.0
                )
                response.raise_for_status()
                
                return response.json()
        except Exception as e:
            logger.error(f"Error simulating mission {mission_id}: {str(e)}")
            return {"error": str(e)}
    
    async def import_mission(self, file_content: bytes, file_format: str, name: str) -> Optional[Mission]:
        """
        Import a mission from a file.
        
        Args:
            file_content: Content of the file
            file_format: Format of the file (kml, gpx)
            name: Name for the imported mission
            
        Returns:
            Imported mission if successful, None otherwise
        """
        try:
            files = {"file": (f"mission.{file_format}", file_content)}
            data = {"name": name}
            
            async with httpx.AsyncClient() as client:
                response = await client.post(
                    f"{self.api_url}/api/missions/import",
                    files=files,
                    data=data,
                    headers=self.headers,
                    timeout=30.0
                )
                response.raise_for_status()
                
                imported_mission = response.json()
                return Mission(**imported_mission)
        except Exception as e:
            logger.error(f"Error importing mission: {str(e)}")
            return None
    
    async def export_mission(self, mission_id: str, export_format: str) -> Optional[bytes]:
        """
        Export a mission to a file.
        
        Args:
            mission_id: ID of the mission to export
            export_format: Format to export (kml, gpx)
            
        Returns:
            File content if successful, None otherwise
        """
        try:
            async with httpx.AsyncClient() as client:
                response = await client.get(
                    f"{self.api_url}/api/missions/{mission_id}/export/{export_format}",
                    headers=self.headers,
                    timeout=10.0
                )
                response.raise_for_status()
                
                return response.content
        except Exception as e:
            logger.error(f"Error exporting mission {mission_id}: {str(e)}")
            return None
