"""
Enhanced mission planning models for Bulo.Cloud Sentinel.

This module provides comprehensive data models for various mission types including:
- Waypoint missions
- Mapping missions
- Orbit missions
- Facade missions
"""

from enum import Enum
from typing import List, Dict, Any, Optional, Union
from pydantic import BaseModel, Field
import uuid
from datetime import datetime


class MissionType(str, Enum):
    """Types of missions supported by the system."""
    WAYPOINT = "waypoint"
    MAPPING = "mapping"
    ORBIT = "orbit"
    FACADE = "facade"
    PANO = "pano"
    HOVER = "hover"


class CameraAction(str, Enum):
    """Camera actions that can be performed at waypoints or during flight."""
    NONE = "none"
    TAKE_PHOTO = "take_photo"
    START_RECORDING = "start_recording"
    STOP_RECORDING = "stop_recording"
    TILT_CAMERA = "tilt_camera"
    POINT_CAMERA = "point_camera"
    ZOOM = "zoom"
    PANO_CAPTURE = "pano_capture"


class ActionTrigger(str, Enum):
    """When to trigger camera actions."""
    ON_ARRIVAL = "on_arrival"
    DURING_MOVEMENT = "during_movement"
    TIMED = "timed"
    DISTANCE = "distance"


class TerrainFollowMode(str, Enum):
    """Terrain following modes."""
    NONE = "none"
    AGL = "agl"  # Above Ground Level
    TERRAIN_FOLLOW = "terrain_follow"


class CameraSettings(BaseModel):
    """Camera settings for missions."""
    mode: str = "auto"
    iso: Optional[int] = None
    shutter_speed: Optional[str] = None
    aperture: Optional[float] = None
    white_balance: Optional[str] = None
    file_format: Optional[str] = None
    focus_mode: Optional[str] = None
    metering_mode: Optional[str] = None


class CameraActionDetails(BaseModel):
    """Detailed configuration for camera actions."""
    action: CameraAction
    trigger: ActionTrigger = ActionTrigger.ON_ARRIVAL
    interval_seconds: Optional[float] = None
    interval_distance: Optional[float] = None
    gimbal_pitch: Optional[float] = None
    gimbal_yaw: Optional[float] = None
    gimbal_roll: Optional[float] = None
    zoom_level: Optional[float] = None
    settings: Optional[CameraSettings] = None


class Position(BaseModel):
    """Geographic position with optional altitude."""
    latitude: float
    longitude: float
    altitude: Optional[float] = None
    relative_altitude: Optional[float] = None  # For AGL mode


class Waypoint(BaseModel):
    """Waypoint for mission planning."""
    id: str = Field(default_factory=lambda: str(uuid.uuid4()))
    position: Position
    speed: Optional[float] = None
    heading: Optional[float] = None
    actions: List[CameraActionDetails] = []
    loiter_time: Optional[float] = None
    acceptance_radius: Optional[float] = None
    pass_through: bool = False
    order: int


class MappingSettings(BaseModel):
    """Settings specific to mapping missions."""
    area_mode: str = "polygon"  # polygon, rectangle, corridor
    capture_mode: str = "overlap"  # overlap, distance
    front_overlap: Optional[float] = 75.0  # percentage
    side_overlap: Optional[float] = 65.0  # percentage
    capture_distance: Optional[float] = None  # meters
    flight_speed: Optional[float] = 5.0  # m/s
    flight_altitude: Optional[float] = 50.0  # meters
    gimbal_pitch: Optional[float] = -90.0  # degrees
    camera_trigger: str = "auto"  # auto, manual, distance, time
    grid_angle: Optional[float] = 0.0  # degrees
    use_crosshatch: bool = False
    crosshatch_angle: Optional[float] = 90.0  # degrees
    margin: Optional[float] = 0.0  # meters
    use_terrain_following: bool = False
    terrain_following_mode: TerrainFollowMode = TerrainFollowMode.NONE
    camera_settings: Optional[CameraSettings] = None


class OrbitSettings(BaseModel):
    """Settings specific to orbit missions."""
    center: Position
    radius: float
    altitude: float
    heading_mode: str = "poi"  # poi, tangent, manual
    gimbal_mode: str = "follow"  # follow, manual
    gimbal_pitch: Optional[float] = -20.0  # degrees
    speed: Optional[float] = 2.0  # m/s
    rotation_direction: str = "clockwise"  # clockwise, counterclockwise
    start_angle: Optional[float] = 0.0  # degrees
    end_angle: Optional[float] = 360.0  # degrees
    num_rotations: Optional[float] = 1.0
    capture_mode: str = "none"  # none, continuous, interval
    photo_interval: Optional[float] = None  # seconds or degrees
    use_terrain_following: bool = False
    terrain_following_mode: TerrainFollowMode = TerrainFollowMode.NONE
    camera_settings: Optional[CameraSettings] = None


class FacadeSettings(BaseModel):
    """Settings specific to facade missions."""
    start_position: Position
    end_position: Position
    height: float
    layers: int = 1
    layer_height: Optional[float] = None
    distance_from_facade: float
    flight_speed: Optional[float] = 2.0  # m/s
    capture_mode: str = "overlap"  # overlap, distance
    overlap: Optional[float] = 70.0  # percentage
    capture_distance: Optional[float] = None  # meters
    gimbal_pitch: Optional[float] = 0.0  # degrees
    camera_settings: Optional[CameraSettings] = None


class PanoSettings(BaseModel):
    """Settings specific to panorama missions."""
    position: Position
    pano_type: str = "spherical"  # spherical, cylindrical, super_resolution
    num_images: Optional[int] = None
    overlap: Optional[float] = 30.0  # percentage
    start_angle: Optional[float] = 0.0  # degrees
    end_angle: Optional[float] = 360.0  # degrees
    rows: Optional[int] = 3
    camera_settings: Optional[CameraSettings] = None


class HoverSettings(BaseModel):
    """Settings specific to hover missions."""
    position: Position
    duration: float  # seconds
    heading: Optional[float] = None
    gimbal_pitch: Optional[float] = None
    camera_action: Optional[CameraAction] = None
    camera_settings: Optional[CameraSettings] = None


class MissionSettings(BaseModel):
    """Combined settings for all mission types."""
    waypoint: Optional[List[Waypoint]] = None
    mapping: Optional[MappingSettings] = None
    orbit: Optional[OrbitSettings] = None
    facade: Optional[FacadeSettings] = None
    pano: Optional[PanoSettings] = None
    hover: Optional[HoverSettings] = None


class Mission(BaseModel):
    """Complete mission definition."""
    id: str = Field(default_factory=lambda: str(uuid.uuid4()))
    name: str
    description: Optional[str] = None
    mission_type: MissionType
    created_at: datetime = Field(default_factory=datetime.utcnow)
    updated_at: datetime = Field(default_factory=datetime.utcnow)
    settings: MissionSettings
    boundary_points: Optional[List[Position]] = None
    home_position: Optional[Position] = None
    max_altitude: Optional[float] = None
    max_distance: Optional[float] = None
    auto_takeoff: bool = True
    auto_land: bool = True
    fail_safe_return_home: bool = True
    terrain_following: TerrainFollowMode = TerrainFollowMode.NONE
    notes: Optional[str] = None
    tags: List[str] = []
    is_template: bool = False
    template_category: Optional[str] = None


class MissionTemplate(BaseModel):
    """Template for creating new missions."""
    id: str = Field(default_factory=lambda: str(uuid.uuid4()))
    name: str
    description: Optional[str] = None
    mission_type: MissionType
    category: str
    thumbnail_url: Optional[str] = None
    settings: MissionSettings
    tags: List[str] = []


class MissionExecutionStatus(str, Enum):
    """Status of mission execution."""
    PENDING = "pending"
    PREPARING = "preparing"
    IN_PROGRESS = "in_progress"
    PAUSED = "paused"
    COMPLETED = "completed"
    ABORTED = "aborted"
    FAILED = "failed"


class MissionExecution(BaseModel):
    """Record of a mission execution."""
    id: str = Field(default_factory=lambda: str(uuid.uuid4()))
    mission_id: str
    drone_id: str
    start_time: Optional[datetime] = None
    end_time: Optional[datetime] = None
    status: MissionExecutionStatus = MissionExecutionStatus.PENDING
    telemetry_log: Optional[str] = None
    completed_waypoints: int = 0
    total_waypoints: int = 0
    current_position: Optional[Position] = None
    battery_start: Optional[float] = None
    battery_end: Optional[float] = None
    distance_traveled: Optional[float] = None
    max_altitude: Optional[float] = None
    max_speed: Optional[float] = None
    photos_taken: Optional[int] = None
    video_duration: Optional[float] = None
    notes: Optional[str] = None
    weather_conditions: Optional[Dict[str, Any]] = None
