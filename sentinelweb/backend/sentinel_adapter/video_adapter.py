"""
Video Adapter for SentinelWeb

This module adapts OpenWebUI's file handling to work with drone video streams.
It maps OpenWebUI's file concepts to video concepts.
"""

import httpx
import logging
from typing import Dict, List, Any, Optional
from pydantic import BaseModel
from datetime import datetime

logger = logging.getLogger(__name__)

class VideoStream(BaseModel):
    """Video stream model."""
    id: str
    drone_id: str
    name: str
    url: str
    type: str
    status: str
    resolution: Optional[str] = None
    created_at: datetime

class VideoRecording(BaseModel):
    """Video recording model."""
    id: str
    drone_id: str
    name: str
    url: str
    size: int
    duration: int
    created_at: datetime
    status: str

class VideoAdapter:
    """
    Adapter for video operations.
    
    This class adapts OpenWebUI's file operations to work with video streams.
    It maps file concepts to video concepts:
    - Files -> Video streams and recordings
    - File uploads -> Video recordings
    - File downloads -> Video playback
    """
    
    def __init__(self, api_url: str, rtmp_server: str, api_key: Optional[str] = None):
        """
        Initialize the video adapter.
        
        Args:
            api_url: URL of the BuloCloudSentinel API
            rtmp_server: URL of the RTMP server
            api_key: Optional API key for authentication
        """
        self.api_url = api_url.rstrip('/')
        self.rtmp_server = rtmp_server.rstrip('/')
        self.api_key = api_key
        self.headers = {}
        
        if api_key:
            self.headers["Authorization"] = f"Bearer {api_key}"
    
    async def list_streams(self, drone_id: Optional[str] = None) -> List[VideoStream]:
        """
        List all available video streams.
        
        This method adapts OpenWebUI's list_files to list video streams.
        
        Args:
            drone_id: Optional drone ID filter
            
        Returns:
            List of video streams
        """
        try:
            params = {}
            if drone_id:
                params["drone_id"] = drone_id
            
            async with httpx.AsyncClient() as client:
                response = await client.get(
                    f"{self.api_url}/api/video/streams",
                    params=params,
                    headers=self.headers,
                    timeout=10.0
                )
                response.raise_for_status()
                
                streams_data = response.json()
                return [VideoStream(**stream) for stream in streams_data]
        except Exception as e:
            logger.error(f"Error listing video streams: {str(e)}")
            return []
    
    async def get_stream(self, stream_id: str) -> Optional[VideoStream]:
        """
        Get information about a specific video stream.
        
        This method adapts OpenWebUI's get_file to get video stream information.
        
        Args:
            stream_id: ID of the video stream
            
        Returns:
            Video stream information if found, None otherwise
        """
        try:
            async with httpx.AsyncClient() as client:
                response = await client.get(
                    f"{self.api_url}/api/video/streams/{stream_id}",
                    headers=self.headers,
                    timeout=10.0
                )
                response.raise_for_status()
                
                stream_data = response.json()
                return VideoStream(**stream_data)
        except Exception as e:
            logger.error(f"Error getting video stream {stream_id}: {str(e)}")
            return None
    
    async def get_stream_url(self, drone_id: str, stream_type: str = "main") -> str:
        """
        Get the URL for a video stream.
        
        This method adapts OpenWebUI's get_file_url to get video stream URLs.
        
        Args:
            drone_id: ID of the drone
            stream_type: Type of stream (main, thermal, etc.)
            
        Returns:
            URL for the video stream
        """
        try:
            async with httpx.AsyncClient() as client:
                response = await client.get(
                    f"{self.api_url}/api/video/drones/{drone_id}/stream",
                    params={"type": stream_type},
                    headers=self.headers,
                    timeout=10.0
                )
                response.raise_for_status()
                
                data = response.json()
                return data.get("url", f"{self.rtmp_server}/live/{drone_id}_{stream_type}")
        except Exception as e:
            logger.error(f"Error getting stream URL for drone {drone_id}: {str(e)}")
            return f"{self.rtmp_server}/live/{drone_id}_{stream_type}"
    
    async def list_recordings(
        self,
        drone_id: Optional[str] = None,
        start_date: Optional[datetime] = None,
        end_date: Optional[datetime] = None
    ) -> List[VideoRecording]:
        """
        List all available video recordings.
        
        This method adapts OpenWebUI's list_files to list video recordings.
        
        Args:
            drone_id: Optional drone ID filter
            start_date: Optional start date filter
            end_date: Optional end date filter
            
        Returns:
            List of video recordings
        """
        try:
            params = {}
            if drone_id:
                params["drone_id"] = drone_id
            if start_date:
                params["start_date"] = start_date.isoformat()
            if end_date:
                params["end_date"] = end_date.isoformat()
            
            async with httpx.AsyncClient() as client:
                response = await client.get(
                    f"{self.api_url}/api/video/recordings",
                    params=params,
                    headers=self.headers,
                    timeout=10.0
                )
                response.raise_for_status()
                
                recordings_data = response.json()
                return [VideoRecording(**recording) for recording in recordings_data]
        except Exception as e:
            logger.error(f"Error listing video recordings: {str(e)}")
            return []
    
    async def get_recording(self, recording_id: str) -> Optional[VideoRecording]:
        """
        Get information about a specific video recording.
        
        This method adapts OpenWebUI's get_file to get video recording information.
        
        Args:
            recording_id: ID of the video recording
            
        Returns:
            Video recording information if found, None otherwise
        """
        try:
            async with httpx.AsyncClient() as client:
                response = await client.get(
                    f"{self.api_url}/api/video/recordings/{recording_id}",
                    headers=self.headers,
                    timeout=10.0
                )
                response.raise_for_status()
                
                recording_data = response.json()
                return VideoRecording(**recording_data)
        except Exception as e:
            logger.error(f"Error getting video recording {recording_id}: {str(e)}")
            return None
    
    async def start_recording(self, drone_id: str, name: Optional[str] = None) -> Dict[str, Any]:
        """
        Start recording video from a drone.
        
        This method adapts OpenWebUI's upload_file to start video recording.
        
        Args:
            drone_id: ID of the drone
            name: Optional name for the recording
            
        Returns:
            Recording information
        """
        try:
            data = {"drone_id": drone_id}
            if name:
                data["name"] = name
            
            async with httpx.AsyncClient() as client:
                response = await client.post(
                    f"{self.api_url}/api/video/recordings/start",
                    json=data,
                    headers=self.headers,
                    timeout=10.0
                )
                response.raise_for_status()
                
                return response.json()
        except Exception as e:
            logger.error(f"Error starting recording for drone {drone_id}: {str(e)}")
            return {"error": str(e)}
    
    async def stop_recording(self, recording_id: str) -> Dict[str, Any]:
        """
        Stop recording video.
        
        This method adapts OpenWebUI's delete_file to stop video recording.
        
        Args:
            recording_id: ID of the recording to stop
            
        Returns:
            Result of stopping the recording
        """
        try:
            async with httpx.AsyncClient() as client:
                response = await client.post(
                    f"{self.api_url}/api/video/recordings/{recording_id}/stop",
                    headers=self.headers,
                    timeout=10.0
                )
                response.raise_for_status()
                
                return response.json()
        except Exception as e:
            logger.error(f"Error stopping recording {recording_id}: {str(e)}")
            return {"error": str(e)}
    
    async def delete_recording(self, recording_id: str) -> bool:
        """
        Delete a video recording.
        
        This method adapts OpenWebUI's delete_file to delete video recordings.
        
        Args:
            recording_id: ID of the recording to delete
            
        Returns:
            True if successful, False otherwise
        """
        try:
            async with httpx.AsyncClient() as client:
                response = await client.delete(
                    f"{self.api_url}/api/video/recordings/{recording_id}",
                    headers=self.headers,
                    timeout=10.0
                )
                response.raise_for_status()
                
                return True
        except Exception as e:
            logger.error(f"Error deleting recording {recording_id}: {str(e)}")
            return False
    
    async def get_recording_url(self, recording_id: str) -> Optional[str]:
        """
        Get the URL for a video recording.
        
        This method adapts OpenWebUI's get_file_url to get video recording URLs.
        
        Args:
            recording_id: ID of the recording
            
        Returns:
            URL for the video recording if available, None otherwise
        """
        try:
            async with httpx.AsyncClient() as client:
                response = await client.get(
                    f"{self.api_url}/api/video/recordings/{recording_id}/url",
                    headers=self.headers,
                    timeout=10.0
                )
                response.raise_for_status()
                
                data = response.json()
                return data.get("url")
        except Exception as e:
            logger.error(f"Error getting recording URL for {recording_id}: {str(e)}")
            return None
