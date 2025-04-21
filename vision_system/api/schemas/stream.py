"""
Stream schemas for the Vision System.
"""

from pydantic import BaseModel, Field
from typing import List, Dict, Any, Optional
from datetime import datetime
from enum import Enum

class StreamStatus(str, Enum):
    """Stream status enumeration."""
    OFFLINE = "offline"
    CONNECTING = "connecting"
    ONLINE = "online"
    ERROR = "error"
    RECORDING = "recording"

class StreamSourceType(str, Enum):
    """Stream source type enumeration."""
    RTSP = "rtsp"
    RTMP = "rtmp"
    HTTP = "http"
    FILE = "file"
    DRONE = "drone"
    USB = "usb"
    IP_CAMERA = "ip_camera"
    CUSTOM = "custom"

class StreamSource(BaseModel):
    """Stream source model."""
    type: StreamSourceType
    url: str
    credentials: Optional[Dict[str, str]] = None
    parameters: Optional[Dict[str, Any]] = None

class StreamStats(BaseModel):
    """Stream statistics model."""
    resolution: Dict[str, int]  # width, height
    fps: float
    bitrate: Optional[float] = None  # kbps
    codec: Optional[str] = None
    uptime: Optional[float] = None  # seconds
    frames_processed: Optional[int] = None
    dropped_frames: Optional[int] = None
    last_frame_time: Optional[datetime] = None

class StreamRecording(BaseModel):
    """Stream recording model."""
    id: str
    stream_id: str
    start_time: datetime
    end_time: Optional[datetime] = None
    duration: Optional[float] = None  # seconds
    file_path: Optional[str] = None
    file_size: Optional[int] = None  # bytes
    format: str = "mp4"
    status: str  # recording, completed, failed
    parameters: Optional[Dict[str, Any]] = None

class StreamCreate(BaseModel):
    """Model for creating a new stream."""
    name: str
    source: StreamSource
    description: Optional[str] = None
    location: Optional[Dict[str, float]] = None  # lat, lon, alt
    parameters: Optional[Dict[str, Any]] = None
    auto_start: bool = False
    tags: Optional[List[str]] = None

class StreamUpdate(BaseModel):
    """Model for updating a stream."""
    name: Optional[str] = None
    source: Optional[StreamSource] = None
    description: Optional[str] = None
    location: Optional[Dict[str, float]] = None
    parameters: Optional[Dict[str, Any]] = None
    tags: Optional[List[str]] = None

class Stream(BaseModel):
    """Stream model."""
    id: str
    name: str
    source: StreamSource
    description: Optional[str] = None
    location: Optional[Dict[str, float]] = None
    parameters: Optional[Dict[str, Any]] = None
    status: StreamStatus = StreamStatus.OFFLINE
    stats: Optional[StreamStats] = None
    last_connected: Optional[datetime] = None
    tags: Optional[List[str]] = None
    created_at: datetime = Field(default_factory=datetime.now)
    updated_at: datetime = Field(default_factory=datetime.now)
