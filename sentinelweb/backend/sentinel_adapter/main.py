"""
Main Adapter for SentinelWeb

This module provides the main integration point between OpenWebUI and BuloCloudSentinel.
It initializes all adapters and provides a unified interface for the frontend.
"""

import logging
import os
from typing import Dict, Any, Optional

from .drone_adapter import DroneAdapter
from .mission_adapter import MissionAdapter
from .telemetry_adapter import TelemetryAdapter
from .video_adapter import VideoAdapter

logger = logging.getLogger(__name__)

class SentinelAdapter:
    """
    Main adapter for SentinelWeb.
    
    This class provides a unified interface for all SentinelWeb adapters.
    It initializes all adapters and provides methods to access them.
    """
    
    def __init__(self):
        """Initialize the SentinelWeb adapter."""
        # Get configuration from environment variables
        self.bulocloud_api_url = os.environ.get("BULOCLOUD_API_URL", "http://bulocloud-api:8000")
        self.bulocloud_api_key = os.environ.get("BULOCLOUD_API_KEY", "")
        self.rtmp_server = os.environ.get("RTMP_SERVER", "rtmp://rtmp-server:1935")
        
        # Initialize adapters
        self.drone_adapter = DroneAdapter(self.bulocloud_api_url, self.bulocloud_api_key)
        self.mission_adapter = MissionAdapter(self.bulocloud_api_url, self.bulocloud_api_key)
        self.telemetry_adapter = TelemetryAdapter(self.bulocloud_api_url, self.bulocloud_api_key)
        self.video_adapter = VideoAdapter(self.bulocloud_api_url, self.rtmp_server, self.bulocloud_api_key)
        
        logger.info(f"SentinelAdapter initialized with API URL: {self.bulocloud_api_url}")
    
    def get_drone_adapter(self) -> DroneAdapter:
        """Get the drone adapter."""
        return self.drone_adapter
    
    def get_mission_adapter(self) -> MissionAdapter:
        """Get the mission adapter."""
        return self.mission_adapter
    
    def get_telemetry_adapter(self) -> TelemetryAdapter:
        """Get the telemetry adapter."""
        return self.telemetry_adapter
    
    def get_video_adapter(self) -> VideoAdapter:
        """Get the video adapter."""
        return self.video_adapter
    
    def get_config(self) -> Dict[str, Any]:
        """Get the adapter configuration."""
        return {
            "bulocloud_api_url": self.bulocloud_api_url,
            "rtmp_server": self.rtmp_server
        }

# Create a singleton instance
sentinel_adapter = SentinelAdapter()

def get_sentinel_adapter() -> SentinelAdapter:
    """Get the singleton SentinelAdapter instance."""
    return sentinel_adapter
