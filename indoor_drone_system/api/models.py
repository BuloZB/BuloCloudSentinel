"""
Data models for the Indoor Drone System API.
"""

from typing import List, Dict, Any, Optional
from pydantic import BaseModel, Field
from datetime import datetime

class Position(BaseModel):
    """3D position model."""
    x: float
    y: float
    z: float

class Orientation(BaseModel):
    """Quaternion orientation model."""
    x: float
    y: float
    z: float
    w: float

class LinearVelocity(BaseModel):
    """Linear velocity model."""
    x: float
    y: float
    z: float

class AngularVelocity(BaseModel):
    """Angular velocity model."""
    x: float
    y: float
    z: float

class Velocity(BaseModel):
    """Combined velocity model."""
    linear: LinearVelocity
    angular: AngularVelocity

class DroneState(BaseModel):
    """Drone state model."""
    drone_id: str
    position: Dict[str, float]
    orientation: Dict[str, float]
    velocity: Dict[str, Dict[str, float]]
    battery_level: float
    status: str
    timestamp: str

class Waypoint(BaseModel):
    """Waypoint model for mission planning."""
    position: Position
    orientation: Optional[Orientation] = None
    wait_time: float = 0.0
    action: Optional[str] = None
    action_params: Optional[Dict[str, Any]] = None

class MissionPlan(BaseModel):
    """Mission plan model."""
    name: str
    description: Optional[str] = None
    waypoints: List[Waypoint]
    drone_id: Optional[str] = None
    created_at: str = Field(default_factory=lambda: datetime.now().isoformat())
    updated_at: str = Field(default_factory=lambda: datetime.now().isoformat())
    status: str = "created"

class SensorData(BaseModel):
    """Sensor data model."""
    sensor_id: str
    sensor_type: str
    timestamp: str
    data: Dict[str, Any]

class MapData(BaseModel):
    """Map data model."""
    map_id: str
    name: str
    created_at: str
    resolution: float
    width: int
    height: int
    data: Optional[List[int]] = None
    status: str = "created"

class LidarScan(BaseModel):
    """LiDAR scan data model."""
    timestamp: str
    ranges: List[float]
    intensities: Optional[List[float]] = None
    angle_min: float
    angle_max: float
    angle_increment: float
    time_increment: float
    scan_time: float
    range_min: float
    range_max: float

class PointCloud(BaseModel):
    """Point cloud data model."""
    timestamp: str
    points: List[Dict[str, float]]
    fields: List[str]
    is_dense: bool

class CameraImage(BaseModel):
    """Camera image metadata model."""
    timestamp: str
    width: int
    height: int
    encoding: str
    is_bigendian: bool
    step: int
    data_url: str  # URL to fetch the actual image data

class SlamStatus(BaseModel):
    """SLAM status model."""
    timestamp: str
    status: str
    position_uncertainty: float
    map_quality: float
    loop_closures: int
    processing_time: float
