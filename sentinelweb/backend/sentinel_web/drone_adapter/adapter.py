"""
Main Adapter for SentinelWeb

This module provides the main integration point between SentinelWeb and BuloCloudSentinel.
It initializes all clients and provides a unified interface for the frontend.
"""

import logging
import os
from typing import Dict, Any, Optional

from .drone_client import DroneClient
from .mission_client import MissionClient
from .telemetry_client import TelemetryClient
from .video_client import VideoClient

logger = logging.getLogger(__name__)

class DroneAdapter:
    """
    Main adapter for SentinelWeb.
    
    This class provides a unified interface for all SentinelWeb clients.
    It initializes all clients and provides methods to access them.
    """
    
    def __init__(self):
        """Initialize the SentinelWeb adapter."""
        # Get configuration from environment variables
        self.bulocloud_api_url = os.environ.get("BULOCLOUD_API_URL", "http://bulocloud-api:8000")
        self.bulocloud_api_key = os.environ.get("BULOCLOUD_API_KEY", "")
        self.rtmp_server = os.environ.get("RTMP_SERVER", "rtmp://rtmp-server:1935")
        
        # Initialize clients
        self.drone_client = DroneClient(self.bulocloud_api_url, self.bulocloud_api_key)
        self.mission_client = MissionClient(self.bulocloud_api_url, self.bulocloud_api_key)
        self.telemetry_client = TelemetryClient(self.bulocloud_api_url, self.bulocloud_api_key)
        self.video_client = VideoClient(self.bulocloud_api_url, self.rtmp_server, self.bulocloud_api_key)
        
        logger.info(f"DroneAdapter initialized with API URL: {self.bulocloud_api_url}")
    
    def get_drone_client(self) -> DroneClient:
        """Get the drone client."""
        return self.drone_client
    
    def get_mission_client(self) -> MissionClient:
        """Get the mission client."""
        return self.mission_client
    
    def get_telemetry_client(self) -> TelemetryClient:
        """Get the telemetry client."""
        return self.telemetry_client
    
    def get_video_client(self) -> VideoClient:
        """Get the video client."""
        return self.video_client
    
    def get_config(self) -> Dict[str, Any]:
        """Get the adapter configuration."""
        return {
            "bulocloud_api_url": self.bulocloud_api_url,
            "rtmp_server": self.rtmp_server
        }

# Create a singleton instance
drone_adapter = DroneAdapter()

def get_drone_adapter() -> DroneAdapter:
    """Get the singleton DroneAdapter instance."""
    return drone_adapter
