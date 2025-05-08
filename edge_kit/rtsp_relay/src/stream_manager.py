#!/usr/bin/env python3
"""
Bulo.CloudSentinel RTSP Stream Manager

This module provides a manager for RTSP streams, handling conversion to HLS and WebRTC.
"""

import os
import sys
import logging
import json
import time
import asyncio
import signal
import subprocess
import threading
import yaml
from typing import Dict, List, Any, Optional, Union, Tuple
from enum import Enum
from pathlib import Path
import uuid
from dataclasses import dataclass, field

from fastapi import WebSocket

# Configure logging
log_level = os.environ.get("LOG_LEVEL", "INFO").upper()
logging.basicConfig(
    level=getattr(logging, log_level),
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
)
logger = logging.getLogger("stream_manager")

# Data classes
class StreamStatus(str, Enum):
    """Stream status."""
    OFFLINE = "offline"
    CONNECTING = "connecting"
    ONLINE = "online"
    ERROR = "error"

@dataclass
class StreamInfo:
    """Stream information."""
    id: str
    name: str
    url: str
    status: StreamStatus
    hls_url: Optional[str] = None
    webrtc_url: Optional[str] = None
    error_message: Optional[str] = None
    enabled: bool = True
    created_at: float = field(default_factory=time.time)
    updated_at: float = field(default_factory=time.time)

@dataclass
class WebRTCConnection:
    """WebRTC connection."""
    id: str
    websocket: WebSocket
    peer_connection: Any = None
    created_at: float = field(default_factory=time.time)

class StreamManager:
    """
    Stream manager for Bulo.CloudSentinel RTSP Relay.
    
    This class manages RTSP streams, handling conversion to HLS and WebRTC.
    """
    
    def __init__(
        self,
        rtsp_sources_config: str = "/config/rtsp_sources.yaml",
        hls_segment_duration: int = 2,
        webrtc_ice_servers: str = "stun:stun.l.google.com:19302",
        stream_buffer_size: int = 1024,
    ):
        """
        Initialize the stream manager.
        
        Args:
            rtsp_sources_config: Path to RTSP sources configuration file
            hls_segment_duration: HLS segment duration in seconds
            webrtc_ice_servers: WebRTC ICE servers
            stream_buffer_size: Stream buffer size
        """
        self.rtsp_sources_config = rtsp_sources_config
        self.hls_segment_duration = hls_segment_duration
        self.webrtc_ice_servers = webrtc_ice_servers
        self.stream_buffer_size = stream_buffer_size
        
        # Initialize streams
        self.streams: Dict[str, StreamInfo] = {}
        self.processes: Dict[str, subprocess.Popen] = {}
        self.webrtc_connections: Dict[str, Dict[str, WebRTCConnection]] = {}
        self.active_connections = 0
        
        # Create directories
        os.makedirs("/storage/hls", exist_ok=True)
        
        # Metrics
        self.metrics = {
            "streams_total": 0,
            "streams_active": 0,
            "connections_total": 0,
            "connections_active": 0,
            "errors": 0,
        }
    
    async def start(self):
        """Start the stream manager."""
        logger.info("Starting stream manager")
        
        # Load RTSP sources
        await self.load_rtsp_sources()
        
        # Start enabled streams
        for stream_id, stream in self.streams.items():
            if stream.enabled:
                await self.start_stream(stream_id)
    
    async def shutdown(self):
        """Shutdown the stream manager."""
        logger.info("Shutting down stream manager")
        
        # Stop all streams
        for stream_id in list(self.streams.keys()):
            await self.stop_stream(stream_id)
    
    async def load_rtsp_sources(self):
        """Load RTSP sources from configuration file."""
        try:
            # Check if configuration file exists
            if not os.path.exists(self.rtsp_sources_config):
                logger.warning(f"RTSP sources configuration file not found: {self.rtsp_sources_config}")
                return
            
            # Load configuration
            with open(self.rtsp_sources_config, "r") as f:
                config = yaml.safe_load(f)
            
            # Add streams
            for source in config.get("sources", []):
                stream_id = str(uuid.uuid4())
                self.streams[stream_id] = StreamInfo(
                    id=stream_id,
                    name=source.get("name", f"stream_{stream_id}"),
                    url=source.get("url", ""),
                    status=StreamStatus.OFFLINE,
                    enabled=source.get("enabled", True),
                )
                
                # Update metrics
                self.metrics["streams_total"] += 1
            
            logger.info(f"Loaded {len(self.streams)} RTSP sources")
            
        except Exception as e:
            logger.error(f"Error loading RTSP sources: {str(e)}")
            self.metrics["errors"] += 1
    
    async def list_streams(self) -> List[StreamInfo]:
        """List all streams."""
        return list(self.streams.values())
    
    async def get_stream(self, stream_id: str) -> StreamInfo:
        """Get a stream by ID."""
        if stream_id not in self.streams:
            raise ValueError(f"Stream not found: {stream_id}")
        return self.streams[stream_id]
    
    async def add_stream(self, name: str, url: str, enabled: bool = True) -> StreamInfo:
        """Add a new stream."""
        stream_id = str(uuid.uuid4())
        self.streams[stream_id] = StreamInfo(
            id=stream_id,
            name=name,
            url=url,
            status=StreamStatus.OFFLINE,
            enabled=enabled,
        )
        
        # Update metrics
        self.metrics["streams_total"] += 1
        
        # Start stream if enabled
        if enabled:
            await self.start_stream(stream_id)
        
        return self.streams[stream_id]
    
    async def remove_stream(self, stream_id: str):
        """Remove a stream."""
        if stream_id not in self.streams:
            raise ValueError(f"Stream not found: {stream_id}")
        
        # Stop stream if running
        if stream_id in self.processes:
            await self.stop_stream(stream_id)
        
        # Remove stream
        del self.streams[stream_id]
    
    async def start_stream(self, stream_id: str):
        """Start a stream."""
        if stream_id not in self.streams:
            raise ValueError(f"Stream not found: {stream_id}")
        
        # Check if stream is already running
        if stream_id in self.processes:
            logger.warning(f"Stream already running: {stream_id}")
            return
        
        # Get stream info
        stream = self.streams[stream_id]
        
        # Update status
        stream.status = StreamStatus.CONNECTING
        stream.updated_at = time.time()
        
        try:
            # Start FFmpeg process for HLS
            hls_output_dir = f"/storage/hls/{stream_id}"
            os.makedirs(hls_output_dir, exist_ok=True)
            
            # FFmpeg command for HLS
            ffmpeg_cmd = [
                "ffmpeg",
                "-i", stream.url,
                "-c:v", "libx264",
                "-preset", "ultrafast",
                "-tune", "zerolatency",
                "-c:a", "aac",
                "-f", "hls",
                "-hls_time", str(self.hls_segment_duration),
                "-hls_list_size", "10",
                "-hls_flags", "delete_segments",
                "-hls_segment_filename", f"{hls_output_dir}/segment_%03d.ts",
                f"{hls_output_dir}/playlist.m3u8",
            ]
            
            # Start FFmpeg process
            process = subprocess.Popen(
                ffmpeg_cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                universal_newlines=True,
            )
            
            # Store process
            self.processes[stream_id] = process
            
            # Update stream info
            stream.hls_url = f"/streams/{stream_id}/hls"
            stream.webrtc_url = f"/streams/{stream_id}/webrtc"
            stream.status = StreamStatus.ONLINE
            stream.updated_at = time.time()
            
            # Update metrics
            self.metrics["streams_active"] += 1
            
            logger.info(f"Started stream: {stream_id}")
            
            # Start monitoring process
            asyncio.create_task(self._monitor_stream(stream_id))
            
        except Exception as e:
            logger.error(f"Error starting stream {stream_id}: {str(e)}")
            stream.status = StreamStatus.ERROR
            stream.error_message = str(e)
            stream.updated_at = time.time()
            self.metrics["errors"] += 1
    
    async def stop_stream(self, stream_id: str):
        """Stop a stream."""
        if stream_id not in self.streams:
            raise ValueError(f"Stream not found: {stream_id}")
        
        # Check if stream is running
        if stream_id not in self.processes:
            logger.warning(f"Stream not running: {stream_id}")
            return
        
        # Get stream info
        stream = self.streams[stream_id]
        
        try:
            # Terminate FFmpeg process
            process = self.processes[stream_id]
            process.terminate()
            
            # Wait for process to terminate
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                # Force kill if timeout
                process.kill()
            
            # Remove process
            del self.processes[stream_id]
            
            # Update stream info
            stream.status = StreamStatus.OFFLINE
            stream.updated_at = time.time()
            
            # Update metrics
            self.metrics["streams_active"] -= 1
            
            logger.info(f"Stopped stream: {stream_id}")
            
        except Exception as e:
            logger.error(f"Error stopping stream {stream_id}: {str(e)}")
            stream.status = StreamStatus.ERROR
            stream.error_message = str(e)
            stream.updated_at = time.time()
            self.metrics["errors"] += 1
    
    async def get_hls_playlist(self, stream_id: str) -> str:
        """Get HLS playlist for a stream."""
        if stream_id not in self.streams:
            raise ValueError(f"Stream not found: {stream_id}")
        
        # Check if stream is running
        if stream_id not in self.processes:
            raise ValueError(f"Stream not running: {stream_id}")
        
        # Get playlist path
        playlist_path = f"/storage/hls/{stream_id}/playlist.m3u8"
        
        # Check if playlist exists
        if not os.path.exists(playlist_path):
            raise ValueError(f"HLS playlist not found: {playlist_path}")
        
        return playlist_path
    
    async def register_webrtc_connection(self, stream_id: str, connection_id: str, websocket: WebSocket):
        """Register a WebRTC connection."""
        if stream_id not in self.streams:
            raise ValueError(f"Stream not found: {stream_id}")
        
        # Initialize connections dict for stream if not exists
        if stream_id not in self.webrtc_connections:
            self.webrtc_connections[stream_id] = {}
        
        # Register connection
        self.webrtc_connections[stream_id][connection_id] = WebRTCConnection(
            id=connection_id,
            websocket=websocket,
        )
        
        # Update metrics
        self.metrics["connections_total"] += 1
        self.metrics["connections_active"] += 1
        self.active_connections += 1
        
        logger.info(f"Registered WebRTC connection: {connection_id} for stream: {stream_id}")
    
    async def unregister_webrtc_connection(self, stream_id: str, connection_id: str):
        """Unregister a WebRTC connection."""
        if stream_id not in self.webrtc_connections:
            return
        
        if connection_id not in self.webrtc_connections[stream_id]:
            return
        
        # Close peer connection if exists
        connection = self.webrtc_connections[stream_id][connection_id]
        if connection.peer_connection:
            await connection.peer_connection.close()
        
        # Remove connection
        del self.webrtc_connections[stream_id][connection_id]
        
        # Update metrics
        self.metrics["connections_active"] -= 1
        self.active_connections -= 1
        
        logger.info(f"Unregistered WebRTC connection: {connection_id} for stream: {stream_id}")
    
    async def handle_webrtc_offer(self, stream_id: str, connection_id: str, offer: Dict[str, Any]):
        """Handle WebRTC offer."""
        if stream_id not in self.webrtc_connections:
            raise ValueError(f"Stream not found: {stream_id}")
        
        if connection_id not in self.webrtc_connections[stream_id]:
            raise ValueError(f"Connection not found: {connection_id}")
        
        # This is a placeholder - in a real implementation, you would handle WebRTC offers
        # using a library like aiortc
        
        # For now, just send a dummy answer
        connection = self.webrtc_connections[stream_id][connection_id]
        await connection.websocket.send_text(json.dumps({
            "type": "answer",
            "sdp": "dummy_sdp",
        }))
    
    async def handle_webrtc_ice_candidate(self, stream_id: str, connection_id: str, candidate: Dict[str, Any]):
        """Handle WebRTC ICE candidate."""
        if stream_id not in self.webrtc_connections:
            raise ValueError(f"Stream not found: {stream_id}")
        
        if connection_id not in self.webrtc_connections[stream_id]:
            raise ValueError(f"Connection not found: {connection_id}")
        
        # This is a placeholder - in a real implementation, you would handle ICE candidates
        # using a library like aiortc
        pass
    
    async def get_metrics(self) -> Dict[str, Any]:
        """Get metrics from the stream manager."""
        return self.metrics
    
    async def _monitor_stream(self, stream_id: str):
        """Monitor a stream process."""
        if stream_id not in self.processes:
            return
        
        process = self.processes[stream_id]
        stream = self.streams[stream_id]
        
        # Wait for process to complete
        returncode = await asyncio.to_thread(process.wait)
        
        # Check if process exited normally
        if returncode != 0:
            logger.error(f"Stream process exited with error: {stream_id}, returncode: {returncode}")
            
            # Get error output
            stderr = process.stderr.read() if process.stderr else ""
            
            # Update stream info
            stream.status = StreamStatus.ERROR
            stream.error_message = f"Process exited with code {returncode}: {stderr}"
            stream.updated_at = time.time()
            
            # Update metrics
            self.metrics["errors"] += 1
            self.metrics["streams_active"] -= 1
        else:
            # Update stream info
            stream.status = StreamStatus.OFFLINE
            stream.updated_at = time.time()
            
            # Update metrics
            self.metrics["streams_active"] -= 1
        
        # Remove process
        if stream_id in self.processes:
            del self.processes[stream_id]
        
        logger.info(f"Stream process ended: {stream_id}")
