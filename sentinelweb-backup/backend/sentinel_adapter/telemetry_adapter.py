"""
Telemetry Adapter for SentinelWeb

This module adapts OpenWebUI's streaming handling to work with drone telemetry.
It maps OpenWebUI's streaming concepts to telemetry concepts.
"""

import httpx
import json
import logging
import asyncio
from typing import Dict, List, Any, Optional, Callable, AsyncGenerator
from pydantic import BaseModel
from datetime import datetime

logger = logging.getLogger(__name__)

class TelemetryData(BaseModel):
    """Telemetry data model."""
    drone_id: str
    timestamp: datetime
    position: Dict[str, float]
    attitude: Dict[str, float]
    battery: Dict[str, Any]
    speed: float
    status: str
    sensors: Dict[str, Any] = {}

class TelemetryAdapter:
    """
    Adapter for telemetry operations.
    
    This class adapts OpenWebUI's streaming operations to work with telemetry.
    It maps streaming concepts to telemetry concepts:
    - Streaming -> Telemetry streaming
    - Stream events -> Telemetry updates
    """
    
    def __init__(self, api_url: str, api_key: Optional[str] = None):
        """
        Initialize the telemetry adapter.
        
        Args:
            api_url: URL of the BuloCloudSentinel API
            api_key: Optional API key for authentication
        """
        self.api_url = api_url.rstrip('/')
        self.api_key = api_key
        self.headers = {}
        
        if api_key:
            self.headers["Authorization"] = f"Bearer {api_key}"
    
    async def get_current_telemetry(self, drone_id: str) -> Optional[TelemetryData]:
        """
        Get current telemetry for a drone.
        
        Args:
            drone_id: ID of the drone
            
        Returns:
            Current telemetry data if available, None otherwise
        """
        try:
            async with httpx.AsyncClient() as client:
                response = await client.get(
                    f"{self.api_url}/api/telemetry/{drone_id}",
                    headers=self.headers,
                    timeout=10.0
                )
                response.raise_for_status()
                
                telemetry_data = response.json()
                return TelemetryData(**telemetry_data)
        except Exception as e:
            logger.error(f"Error getting telemetry for drone {drone_id}: {str(e)}")
            return None
    
    async def get_telemetry_history(
        self,
        drone_id: str,
        start_time: Optional[datetime] = None,
        end_time: Optional[datetime] = None,
        interval: str = "1m"
    ) -> List[TelemetryData]:
        """
        Get historical telemetry for a drone.
        
        Args:
            drone_id: ID of the drone
            start_time: Optional start time
            end_time: Optional end time
            interval: Time interval (e.g., "1m", "5m", "1h")
            
        Returns:
            List of telemetry data points
        """
        try:
            params = {"interval": interval}
            if start_time:
                params["start_time"] = start_time.isoformat()
            if end_time:
                params["end_time"] = end_time.isoformat()
            
            async with httpx.AsyncClient() as client:
                response = await client.get(
                    f"{self.api_url}/api/telemetry/{drone_id}/history",
                    params=params,
                    headers=self.headers,
                    timeout=10.0
                )
                response.raise_for_status()
                
                history_data = response.json()
                return [TelemetryData(**item) for item in history_data.get("data", [])]
        except Exception as e:
            logger.error(f"Error getting telemetry history for drone {drone_id}: {str(e)}")
            return []
    
    async def stream_telemetry(
        self,
        drone_id: str,
        callback: Callable[[TelemetryData], None]
    ) -> None:
        """
        Stream telemetry data for a drone.
        
        This method adapts OpenWebUI's streaming to telemetry streaming.
        
        Args:
            drone_id: ID of the drone
            callback: Callback function to handle telemetry updates
        """
        try:
            url = f"{self.api_url}/api/ws/telemetry/{drone_id}"
            
            # Convert HTTP URL to WebSocket URL
            if url.startswith("http://"):
                url = url.replace("http://", "ws://")
            elif url.startswith("https://"):
                url = url.replace("https://", "wss://")
            
            async with httpx.AsyncClient() as client:
                async with client.stream("GET", url, headers=self.headers, timeout=None) as response:
                    async for line in response.aiter_lines():
                        if not line.strip():
                            continue
                        
                        try:
                            data = json.loads(line)
                            telemetry = TelemetryData(**data)
                            callback(telemetry)
                        except Exception as e:
                            logger.error(f"Error parsing telemetry data: {str(e)}")
        except Exception as e:
            logger.error(f"Error streaming telemetry for drone {drone_id}: {str(e)}")
    
    async def get_telemetry_stream(self, drone_id: str) -> AsyncGenerator[TelemetryData, None]:
        """
        Get a telemetry stream for a drone.
        
        This method adapts OpenWebUI's streaming to telemetry streaming.
        
        Args:
            drone_id: ID of the drone
            
        Yields:
            Telemetry data updates
        """
        try:
            url = f"{self.api_url}/api/ws/telemetry/{drone_id}"
            
            # Convert HTTP URL to WebSocket URL
            if url.startswith("http://"):
                url = url.replace("http://", "ws://")
            elif url.startswith("https://"):
                url = url.replace("https://", "wss://")
            
            async with httpx.AsyncClient() as client:
                async with client.stream("GET", url, headers=self.headers, timeout=None) as response:
                    async for line in response.aiter_lines():
                        if not line.strip():
                            continue
                        
                        try:
                            data = json.loads(line)
                            telemetry = TelemetryData(**data)
                            yield telemetry
                        except Exception as e:
                            logger.error(f"Error parsing telemetry data: {str(e)}")
        except Exception as e:
            logger.error(f"Error streaming telemetry for drone {drone_id}: {str(e)}")
    
    async def get_battery_status(self, drone_id: str) -> Dict[str, Any]:
        """
        Get battery status for a drone.
        
        Args:
            drone_id: ID of the drone
            
        Returns:
            Battery status information
        """
        try:
            async with httpx.AsyncClient() as client:
                response = await client.get(
                    f"{self.api_url}/api/power/battery/{drone_id}/current",
                    headers=self.headers,
                    timeout=10.0
                )
                response.raise_for_status()
                
                return response.json()
        except Exception as e:
            logger.error(f"Error getting battery status for drone {drone_id}: {str(e)}")
            return {"error": str(e)}
    
    async def get_battery_history(
        self,
        drone_id: str,
        start_time: Optional[datetime] = None,
        end_time: Optional[datetime] = None,
        interval: str = "1m"
    ) -> Dict[str, Any]:
        """
        Get battery history for a drone.
        
        Args:
            drone_id: ID of the drone
            start_time: Optional start time
            end_time: Optional end time
            interval: Time interval (e.g., "1m", "5m", "1h")
            
        Returns:
            Battery history data
        """
        try:
            params = {"interval": interval}
            if start_time:
                params["start_time"] = start_time.isoformat()
            if end_time:
                params["end_time"] = end_time.isoformat()
            
            async with httpx.AsyncClient() as client:
                response = await client.get(
                    f"{self.api_url}/api/power/battery/{drone_id}/history",
                    params=params,
                    headers=self.headers,
                    timeout=10.0
                )
                response.raise_for_status()
                
                return response.json()
        except Exception as e:
            logger.error(f"Error getting battery history for drone {drone_id}: {str(e)}")
            return {"error": str(e)}
