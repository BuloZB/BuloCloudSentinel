"""
Video Adapter for SentinelWeb

This module provides integration with BuloCloudSentinel's video streaming API.
"""

import logging
import httpx
from typing import Dict, List, Optional, Any

logger = logging.getLogger(__name__)

class VideoAdapter:
    """
    Adapter for video streaming operations.
    
    This class provides methods to interact with BuloCloudSentinel's video streaming API.
    """
    
    def __init__(self, api_url: str, rtmp_server: str, api_token: str):
        """Initialize the video adapter.
        
        Args:
            api_url: The URL of the BuloCloudSentinel API.
            rtmp_server: The RTMP server URL.
            api_token: The API token for authentication.
        """
        self.api_url = api_url
        self.rtmp_server = rtmp_server
        self.api_token = api_token
        self.headers = {
            "Authorization": f"Bearer {api_token}",
            "Content-Type": "application/json"
        }
        logger.info(f"Initialized VideoAdapter with API URL: {api_url}, RTMP server: {rtmp_server}")
    
    async def get_video_streams(self) -> List[Dict[str, Any]]:
        """Get all available video streams.
        
        Returns:
            A list of video stream objects.
        """
        try:
            async with httpx.AsyncClient() as client:
                response = await client.get(
                    f"{self.api_url}/api/video/streams",
                    headers=self.headers,
                    timeout=10.0
                )
                response.raise_for_status()
                return response.json()
        except httpx.HTTPError as e:
            logger.error(f"Error getting video streams: {str(e)}")
            return []
    
    async def get_drone_stream(self, drone_id: str) -> Optional[Dict[str, Any]]:
        """Get the video stream for a specific drone.
        
        Args:
            drone_id: The ID of the drone.
            
        Returns:
            The video stream object or None if not found.
        """
        try:
            async with httpx.AsyncClient() as client:
                response = await client.get(
                    f"{self.api_url}/api/drones/{drone_id}/video",
                    headers=self.headers,
                    timeout=10.0
                )
                response.raise_for_status()
                return response.json()
        except httpx.HTTPError as e:
            logger.error(f"Error getting video stream for drone {drone_id}: {str(e)}")
            return None
    
    async def start_recording(self, drone_id: str) -> bool:
        """Start recording video for a drone.
        
        Args:
            drone_id: The ID of the drone.
            
        Returns:
            True if recording started successfully, False otherwise.
        """
        try:
            async with httpx.AsyncClient() as client:
                response = await client.post(
                    f"{self.api_url}/api/drones/{drone_id}/video/record/start",
                    headers=self.headers,
                    timeout=10.0
                )
                response.raise_for_status()
                return True
        except httpx.HTTPError as e:
            logger.error(f"Error starting recording for drone {drone_id}: {str(e)}")
            return False
    
    async def stop_recording(self, drone_id: str) -> bool:
        """Stop recording video for a drone.
        
        Args:
            drone_id: The ID of the drone.
            
        Returns:
            True if recording stopped successfully, False otherwise.
        """
        try:
            async with httpx.AsyncClient() as client:
                response = await client.post(
                    f"{self.api_url}/api/drones/{drone_id}/video/record/stop",
                    headers=self.headers,
                    timeout=10.0
                )
                response.raise_for_status()
                return True
        except httpx.HTTPError as e:
            logger.error(f"Error stopping recording for drone {drone_id}: {str(e)}")
            return False
    
    async def get_recordings(self, drone_id: str = None) -> List[Dict[str, Any]]:
        """Get all available recordings.
        
        Args:
            drone_id: Optional ID of the drone to filter recordings.
            
        Returns:
            A list of recording objects.
        """
        url = f"{self.api_url}/api/video/recordings"
        if drone_id:
            url += f"?drone_id={drone_id}"
        
        try:
            async with httpx.AsyncClient() as client:
                response = await client.get(
                    url,
                    headers=self.headers,
                    timeout=10.0
                )
                response.raise_for_status()
                return response.json()
        except httpx.HTTPError as e:
            logger.error(f"Error getting recordings: {str(e)}")
            return []
    
    def get_rtmp_url(self, stream_key: str) -> str:
        """Get the RTMP URL for a stream key.
        
        Args:
            stream_key: The stream key.
            
        Returns:
            The RTMP URL.
        """
        return f"{self.rtmp_server}/{stream_key}"
