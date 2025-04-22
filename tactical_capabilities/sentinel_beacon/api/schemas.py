"""
API schemas for the SentinelBeacon module.
"""

from datetime import datetime
from typing import Dict, List, Optional, Any, Union
from pydantic import BaseModel, Field, UUID4
from enum import Enum

# Enums
class NodeType(str, Enum):
    """Node type enumeration."""
    DRONE = "drone"
    GROUND_STATION = "ground_station"
    RELAY = "relay"
    MOBILE = "mobile"
    FIXED = "fixed"
    UNKNOWN = "unknown"

class MessageType(str, Enum):
    """Message type enumeration."""
    TEXT = "text"
    POSITION = "position"
    TELEMETRY = "telemetry"
    COMMAND = "command"
    STATUS = "status"
    EMERGENCY = "emergency"
    OTHER = "other"

class MessagePriority(str, Enum):
    """Message priority enumeration."""
    LOW = "low"
    NORMAL = "normal"
    HIGH = "high"
    EMERGENCY = "emergency"

class ChannelRole(str, Enum):
    """Channel role enumeration."""
    PRIMARY = "PRIMARY"
    SECONDARY = "SECONDARY"
    DISABLED = "DISABLED"

class BeaconMode(str, Enum):
    """Beacon mode enumeration."""
    AUTONOMOUS = "autonomous"
    MANUAL = "manual"
    SCHEDULED = "scheduled"

class BeaconMovementPattern(str, Enum):
    """Beacon movement pattern enumeration."""
    HOVER = "hover"
    CIRCLE = "circle"
    GRID = "grid"

# Base schemas
class PositionSchema(BaseModel):
    """Position schema."""
    latitude: float
    longitude: float
    altitude: Optional[float] = None
    time: Optional[datetime] = None

# Node schemas
class NodeBase(BaseModel):
    """Base node schema."""
    id: str
    name: str
    type: NodeType
    position: Optional[PositionSchema] = None
    last_seen: Optional[datetime] = None
    battery_level: Optional[float] = None
    signal_strength: Optional[int] = None
    neighbors: List[str] = []
    metadata: Optional[Dict[str, Any]] = None

class Node(NodeBase):
    """Node schema."""
    pass

# Message schemas
class MessageBase(BaseModel):
    """Base message schema."""
    text: str
    to_id: Optional[str] = None
    channel_name: Optional[str] = None
    type: MessageType = MessageType.TEXT
    priority: MessagePriority = MessagePriority.NORMAL
    want_ack: bool = True
    metadata: Optional[Dict[str, Any]] = None

class MessageCreate(MessageBase):
    """Message creation schema."""
    pass

class Message(MessageBase):
    """Message schema."""
    id: str
    from_id: str
    timestamp: datetime
    acknowledged: bool = False
    hop_count: int = 0

# Channel schemas
class ChannelBase(BaseModel):
    """Base channel schema."""
    name: str
    role: ChannelRole
    enabled: bool = True
    psk: Optional[str] = None
    metadata: Optional[Dict[str, Any]] = None

class ChannelCreate(ChannelBase):
    """Channel creation schema."""
    pass

class Channel(ChannelBase):
    """Channel schema."""
    index: int

# Network schemas
class NetworkStatus(BaseModel):
    """Network status schema."""
    local_node: Dict[str, Any]
    nodes: List[Node]
    channels: List[Channel]
    node_count: int
    channel_count: int
    timestamp: datetime

# Beacon schemas
class BeaconConfig(BaseModel):
    """Beacon configuration schema."""
    mode: BeaconMode
    altitude: float
    hover_time: int
    return_home_battery: int
    max_distance: float
    movement_pattern: BeaconMovementPattern

class BeaconStatus(BaseModel):
    """Beacon status schema."""
    active: bool
    mode: BeaconMode
    position: Optional[PositionSchema] = None
    battery_level: float
    start_time: Optional[datetime] = None
    elapsed_time: Optional[int] = None  # seconds

# Telemetry schemas
class TelemetryData(BaseModel):
    """Telemetry data schema."""
    timestamp: datetime
    position: Optional[PositionSchema] = None
    battery_level: float
    beacon_active: bool
    beacon_mode: str
    metadata: Optional[Dict[str, Any]] = None
