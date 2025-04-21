"""
Drone schemas for the Drone Swarm System.
"""

from pydantic import BaseModel, Field
from typing import List, Dict, Any, Optional
from datetime import datetime
from enum import Enum

class DroneType(str, Enum):
    """Drone type enumeration."""
    QUADCOPTER = "quadcopter"
    HEXACOPTER = "hexacopter"
    OCTOCOPTER = "octocopter"
    FIXED_WING = "fixed_wing"
    VTOL = "vtol"
    OTHER = "other"

class DroneStatus(str, Enum):
    """Drone status enumeration."""
    OFFLINE = "offline"
    ONLINE = "online"
    ARMED = "armed"
    FLYING = "flying"
    MISSION = "mission"
    RETURNING = "returning"
    ERROR = "error"
    UNKNOWN = "unknown"

class Position(BaseModel):
    """3D position model."""
    latitude: float
    longitude: float
    altitude: float
    relative_altitude: Optional[float] = None

class Attitude(BaseModel):
    """Attitude model (roll, pitch, yaw)."""
    roll: float
    pitch: float
    yaw: float

class Battery(BaseModel):
    """Battery state model."""
    voltage: float
    current: Optional[float] = None
    level: float  # 0-100%
    remaining_time: Optional[float] = None  # seconds
    temperature: Optional[float] = None  # celsius

class GPSInfo(BaseModel):
    """GPS information model."""
    fix_type: int  # 0-5, higher is better
    satellites_visible: int
    hdop: Optional[float] = None
    vdop: Optional[float] = None

class SensorStatus(BaseModel):
    """Sensor status model."""
    gps: bool
    imu: bool
    barometer: bool
    magnetometer: bool
    lidar: Optional[bool] = None
    camera: Optional[bool] = None
    obstacle_detection: Optional[bool] = None

class DroneCapabilities(BaseModel):
    """Drone capabilities model."""
    max_speed: float  # m/s
    max_altitude: float  # meters
    max_flight_time: float  # minutes
    max_range: float  # meters
    max_payload: Optional[float] = None  # grams
    obstacle_avoidance: bool = False
    return_to_home: bool = True
    precision_landing: bool = False
    follow_me: bool = False
    waypoint_navigation: bool = True
    geofencing: bool = True
    indoor_flight: bool = False
    camera: bool = False
    camera_gimbal: bool = False
    camera_resolution: Optional[str] = None
    video_streaming: bool = False
    thermal_camera: bool = False
    lidar: bool = False
    spotlight: bool = False
    speaker: bool = False
    payload_drop: bool = False
    custom_capabilities: Optional[Dict[str, Any]] = None

class DroneState(BaseModel):
    """Drone state model."""
    drone_id: str
    status: DroneStatus
    position: Optional[Position] = None
    attitude: Optional[Attitude] = None
    velocity: Optional[Dict[str, float]] = None
    battery: Optional[Battery] = None
    gps: Optional[GPSInfo] = None
    sensor_status: Optional[SensorStatus] = None
    armed: bool = False
    mode: Optional[str] = None
    airspeed: Optional[float] = None
    groundspeed: Optional[float] = None
    heading: Optional[float] = None
    throttle: Optional[float] = None
    mission_progress: Optional[float] = None  # 0-100%
    home_position: Optional[Position] = None
    last_updated: datetime = Field(default_factory=datetime.now)

class DroneCommand(BaseModel):
    """Drone command model."""
    command_id: str
    drone_id: str
    command: str
    parameters: Optional[Dict[str, Any]] = None
    status: str  # pending, sent, executed, failed
    result: Optional[Dict[str, Any]] = None
    created_at: datetime = Field(default_factory=datetime.now)
    updated_at: datetime = Field(default_factory=datetime.now)

class DroneCreate(BaseModel):
    """Model for creating a new drone."""
    name: str
    drone_type: DroneType
    model: str
    serial_number: Optional[str] = None
    connection_type: str  # mavlink, dronekit, dji, etc.
    connection_params: Dict[str, Any]
    capabilities: Optional[DroneCapabilities] = None
    description: Optional[str] = None
    tags: Optional[List[str]] = None

class DroneUpdate(BaseModel):
    """Model for updating a drone."""
    name: Optional[str] = None
    drone_type: Optional[DroneType] = None
    model: Optional[str] = None
    serial_number: Optional[str] = None
    connection_type: Optional[str] = None
    connection_params: Optional[Dict[str, Any]] = None
    capabilities: Optional[DroneCapabilities] = None
    description: Optional[str] = None
    tags: Optional[List[str]] = None

class Drone(BaseModel):
    """Drone model."""
    id: str
    name: str
    drone_type: DroneType
    model: str
    serial_number: Optional[str] = None
    connection_type: str
    connection_params: Dict[str, Any]
    capabilities: DroneCapabilities
    description: Optional[str] = None
    tags: Optional[List[str]] = None
    status: DroneStatus = DroneStatus.OFFLINE
    last_connected: Optional[datetime] = None
    created_at: datetime = Field(default_factory=datetime.now)
    updated_at: datetime = Field(default_factory=datetime.now)
