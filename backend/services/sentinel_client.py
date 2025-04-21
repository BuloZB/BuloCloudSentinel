"""
SentinelWeb Backend - Sentinel Client

This module provides a client for communicating with the BuloCloudSentinel API.
"""

import logging
from typing import Dict, Any, Optional, List

import httpx
from fastapi import HTTPException, status

from backend.core.config import settings

logger = logging.getLogger(__name__)

class SentinelClient:
    """Client for communicating with BuloCloudSentinel API."""
    
    def __init__(self, base_url: str, token: Optional[str] = None):
        """
        Initialize the Sentinel client.
        
        Args:
            base_url: Base URL of the BuloCloudSentinel API
            token: Optional JWT token for authentication
        """
        self.base_url = base_url
        self.token = token
        
    async def _request(
        self, 
        method: str, 
        endpoint: str, 
        data: Optional[Dict[str, Any]] = None,
        params: Optional[Dict[str, Any]] = None,
        headers: Optional[Dict[str, str]] = None,
        timeout: int = 30
    ) -> Any:
        """
        Make a request to the BuloCloudSentinel API.
        
        Args:
            method: HTTP method (GET, POST, PUT, DELETE)
            endpoint: API endpoint
            data: Optional request data
            params: Optional query parameters
            headers: Optional request headers
            timeout: Request timeout in seconds
            
        Returns:
            Response data
            
        Raises:
            HTTPException: If the request fails
        """
        url = f"{self.base_url.rstrip('/')}/{endpoint.lstrip('/')}"
        
        # Prepare headers
        request_headers = headers or {}
        if self.token:
            request_headers["Authorization"] = f"Bearer {self.token}"
            
        try:
            async with httpx.AsyncClient() as client:
                response = await client.request(
                    method=method,
                    url=url,
                    json=data,
                    params=params,
                    headers=request_headers,
                    timeout=timeout
                )
                
                # Check for errors
                response.raise_for_status()
                
                # Return response data
                return response.json()
                
        except httpx.HTTPStatusError as e:
            logger.error(f"HTTP error: {e}")
            raise HTTPException(
                status_code=e.response.status_code,
                detail=str(e)
            )
        except httpx.RequestError as e:
            logger.error(f"Request error: {e}")
            raise HTTPException(
                status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
                detail=f"Error communicating with BuloCloudSentinel: {str(e)}"
            )
        except Exception as e:
            logger.error(f"Unexpected error: {e}")
            raise HTTPException(
                status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
                detail=f"Unexpected error: {str(e)}"
            )
    
    async def authenticate(self, username: str, password: str) -> Dict[str, Any]:
        """
        Authenticate with BuloCloudSentinel.
        
        Args:
            username: Username
            password: Password
            
        Returns:
            Authentication response with access token
        """
        data = {
            "username": username,
            "password": password
        }
        
        return await self._request(
            method="POST",
            endpoint="/api/login",
            data=data
        )
    
    async def get_current_user(self) -> Dict[str, Any]:
        """
        Get current user information.
        
        Returns:
            Current user information
        """
        return await self._request(
            method="GET",
            endpoint="/api/me"
        )
    
    async def check_health(self) -> Dict[str, Any]:
        """
        Check BuloCloudSentinel health.
        
        Returns:
            Health check response
        """
        return await self._request(
            method="GET",
            endpoint="/api/health"
        )
    
    async def get_drones(self) -> List[Dict[str, Any]]:
        """
        Get all drones.
        
        Returns:
            List of drones
        """
        return await self._request(
            method="GET",
            endpoint="/api/device-inventory/drones"
        )
    
    async def get_drone(self, drone_id: str) -> Dict[str, Any]:
        """
        Get drone by ID.
        
        Args:
            drone_id: Drone ID
            
        Returns:
            Drone information
        """
        return await self._request(
            method="GET",
            endpoint=f"/api/device-inventory/drones/{drone_id}"
        )
    
    async def get_missions(self) -> List[Dict[str, Any]]:
        """
        Get all missions.
        
        Returns:
            List of missions
        """
        return await self._request(
            method="GET",
            endpoint="/api/missions"
        )
    
    async def get_mission(self, mission_id: str) -> Dict[str, Any]:
        """
        Get mission by ID.
        
        Args:
            mission_id: Mission ID
            
        Returns:
            Mission information
        """
        return await self._request(
            method="GET",
            endpoint=f"/api/missions/{mission_id}"
        )
    
    async def create_mission(self, mission_data: Dict[str, Any]) -> Dict[str, Any]:
        """
        Create a new mission.
        
        Args:
            mission_data: Mission data
            
        Returns:
            Created mission
        """
        return await self._request(
            method="POST",
            endpoint="/api/missions",
            data=mission_data
        )
    
    async def update_mission(self, mission_id: str, mission_data: Dict[str, Any]) -> Dict[str, Any]:
        """
        Update a mission.
        
        Args:
            mission_id: Mission ID
            mission_data: Mission data
            
        Returns:
            Updated mission
        """
        return await self._request(
            method="PUT",
            endpoint=f"/api/missions/{mission_id}",
            data=mission_data
        )
    
    async def delete_mission(self, mission_id: str) -> Dict[str, Any]:
        """
        Delete a mission.
        
        Args:
            mission_id: Mission ID
            
        Returns:
            Deletion response
        """
        return await self._request(
            method="DELETE",
            endpoint=f"/api/missions/{mission_id}"
        )
    
    async def execute_mission(self, mission_id: str) -> Dict[str, Any]:
        """
        Execute a mission.
        
        Args:
            mission_id: Mission ID
            
        Returns:
            Execution response
        """
        return await self._request(
            method="POST",
            endpoint=f"/api/missions/{mission_id}/execute"
        )
    
    async def get_telemetry(self, drone_id: str) -> Dict[str, Any]:
        """
        Get drone telemetry.
        
        Args:
            drone_id: Drone ID
            
        Returns:
            Telemetry data
        """
        return await self._request(
            method="GET",
            endpoint=f"/api/telemetry/{drone_id}"
        )
    
    async def get_battery_status(self, drone_id: str) -> Dict[str, Any]:
        """
        Get drone battery status.
        
        Args:
            drone_id: Drone ID
            
        Returns:
            Battery status
        """
        return await self._request(
            method="GET",
            endpoint=f"/api/power/battery/{drone_id}/current"
        )
    
    async def get_battery_history(
        self, 
        drone_id: str, 
        start_time: Optional[str] = None,
        end_time: Optional[str] = None,
        interval: str = "1m"
    ) -> Dict[str, Any]:
        """
        Get drone battery history.
        
        Args:
            drone_id: Drone ID
            start_time: Optional start time (ISO format)
            end_time: Optional end time (ISO format)
            interval: Time interval (e.g., "1m", "5m", "1h")
            
        Returns:
            Battery history
        """
        params = {"interval": interval}
        if start_time:
            params["start_time"] = start_time
        if end_time:
            params["end_time"] = end_time
            
        return await self._request(
            method="GET",
            endpoint=f"/api/power/battery/{drone_id}/history",
            params=params
        )
    
    async def predict_energy(self, prediction_data: Dict[str, Any]) -> Dict[str, Any]:
        """
        Predict energy consumption for a mission.
        
        Args:
            prediction_data: Prediction request data
            
        Returns:
            Energy prediction
        """
        return await self._request(
            method="POST",
            endpoint="/api/power/energy/predict",
            data=prediction_data
        )
    
    async def get_ai_models(self) -> List[Dict[str, Any]]:
        """
        Get all AI models.
        
        Returns:
            List of AI models
        """
        return await self._request(
            method="GET",
            endpoint="/api/ai/models"
        )
    
    async def analyze_video(self, video_url: str, options: Dict[str, Any]) -> Dict[str, Any]:
        """
        Analyze video with AI.
        
        Args:
            video_url: URL of the video to analyze
            options: Analysis options
            
        Returns:
            Analysis results
        """
        data = {
            "video_url": video_url,
            "options": options
        }
        
        return await self._request(
            method="POST",
            endpoint="/api/ai/analyze-video",
            data=data
        )
