"""
Remote ID schemas for the Remote ID & Regulatory Compliance Service.

This module contains Pydantic models for Remote ID API request and response validation.
"""

from datetime import datetime
from enum import Enum
from typing import Dict, List, Optional, Any, Union
from uuid import UUID

from pydantic import BaseModel, Field, validator

# Enums
class RemoteIDMode(str, Enum):
    """Remote ID broadcast mode."""
    FAA = "faa"
    EU = "eu"
    CUSTOM = "custom"

class BroadcastMethod(str, Enum):
    """Remote ID broadcast method."""
    WIFI_NAN = "wifi_nan"
    BLUETOOTH_LE = "bluetooth_le"
    NETWORK = "network"
    ASTM_NETWORK = "astm_network"

class OperatorIDType(str, Enum):
    """Operator ID type."""
    FAA_REGISTRATION = "faa_registration"
    EASA_REGISTRATION = "easa_registration"
    UTM_REGISTRATION = "utm_registration"
    OTHER = "other"

# Base models
class Position(BaseModel):
    """Position model."""
    latitude: float = Field(..., ge=-90, le=90)
    longitude: float = Field(..., ge=-180, le=180)
    altitude: float  # meters above ground level
    altitude_reference: Optional[str] = "agl"  # agl or msl

class Velocity(BaseModel):
    """Velocity model."""
    speed_horizontal: float  # m/s
    speed_vertical: Optional[float] = None  # m/s
    heading: Optional[float] = None  # degrees, 0-359

class OperatorID(BaseModel):
    """Operator ID model."""
    id: str
    type: OperatorIDType

# Request models
class StartBroadcastRequest(BaseModel):
    """Start Remote ID broadcast request."""
    drone_id: str
    mode: RemoteIDMode
    methods: List[BroadcastMethod] = [BroadcastMethod.WIFI_NAN, BroadcastMethod.BLUETOOTH_LE]
    operator_id: Optional[OperatorID] = None
    serial_number: Optional[str] = None
    session_id: Optional[str] = None
    initial_position: Optional[Position] = None
    initial_velocity: Optional[Velocity] = None
    metadata: Optional[Dict[str, Any]] = None

class StopBroadcastRequest(BaseModel):
    """Stop Remote ID broadcast request."""
    drone_id: str
    session_id: Optional[str] = None

class UpdateBroadcastRequest(BaseModel):
    """Update Remote ID broadcast request."""
    drone_id: str
    position: Position
    velocity: Optional[Velocity] = None
    session_id: Optional[str] = None
    timestamp: Optional[datetime] = None

class GetBroadcastLogsRequest(BaseModel):
    """Get Remote ID broadcast logs request."""
    drone_id: Optional[str] = None
    start_time: Optional[datetime] = None
    end_time: Optional[datetime] = None
    limit: Optional[int] = 100
    offset: Optional[int] = 0

# Response models
class BroadcastStatus(BaseModel):
    """Remote ID broadcast status."""
    drone_id: str
    broadcasting: bool
    mode: Optional[RemoteIDMode] = None
    methods: Optional[List[BroadcastMethod]] = None
    session_id: Optional[str] = None
    last_update: Optional[datetime] = None
    position: Optional[Position] = None
    velocity: Optional[Velocity] = None

class BroadcastLog(BaseModel):
    """Remote ID broadcast log."""
    id: UUID
    drone_id: str
    timestamp: datetime
    mode: RemoteIDMode
    method: BroadcastMethod
    position: Position
    velocity: Optional[Velocity] = None
    operator_id: Optional[str] = None
    serial_number: Optional[str] = None
    session_id: Optional[str] = None
    message_data: Optional[Dict[str, Any]] = None

class BroadcastLogsResponse(BaseModel):
    """Remote ID broadcast logs response."""
    logs: List[BroadcastLog]
    total: int
    limit: int
    offset: int

# ASTM F3411-22a message models
class ASTMLocationMessage(BaseModel):
    """ASTM F3411-22a location message."""
    message_type: str = "location"
    uas_id: str
    operator_id: Optional[str] = None
    position: Position
    velocity: Optional[Velocity] = None
    timestamp: datetime
    operational_status: Optional[str] = None

class ASTMOperatorIDMessage(BaseModel):
    """ASTM F3411-22a operator ID message."""
    message_type: str = "operator_id"
    uas_id: str
    operator_id: str
    operator_id_type: OperatorIDType
    timestamp: datetime

class ASTMBasicIDMessage(BaseModel):
    """ASTM F3411-22a basic ID message."""
    message_type: str = "basic_id"
    uas_id: str
    uas_id_type: str  # serial_number, registration, etc.
    timestamp: datetime

class ASTMSelfIDMessage(BaseModel):
    """ASTM F3411-22a self ID message."""
    message_type: str = "self_id"
    uas_id: str
    description: str
    timestamp: datetime

class ASTMSystemMessage(BaseModel):
    """ASTM F3411-22a system message."""
    message_type: str = "system"
    uas_id: str
    operator_location: Optional[Position] = None
    area_count: Optional[int] = None
    area_radius: Optional[float] = None
    area_ceiling: Optional[float] = None
    area_floor: Optional[float] = None
    category: Optional[str] = None
    class_value: Optional[str] = None
    timestamp: datetime

class ASTMMessage(BaseModel):
    """ASTM F3411-22a message."""
    message: Union[
        ASTMLocationMessage,
        ASTMOperatorIDMessage,
        ASTMBasicIDMessage,
        ASTMSelfIDMessage,
        ASTMSystemMessage,
    ]
