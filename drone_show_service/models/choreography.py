"""
Choreography models for the Drone Show microservice.

This module provides data models for defining and storing drone show choreographies.
"""

from enum import Enum
from typing import List, Dict, Any, Optional, Union
from pydantic import BaseModel, Field, validator
from datetime import datetime
import uuid


class LEDColor(BaseModel):
    """RGB color for LED control."""
    r: int = Field(..., ge=0, le=255)
    g: int = Field(..., ge=0, le=255)
    b: int = Field(..., ge=0, le=255)


class LEDEffect(str, Enum):
    """Types of LED effects."""
    SOLID = "solid"
    BLINK = "blink"
    PULSE = "pulse"
    RAINBOW = "rainbow"
    CHASE = "chase"
    CUSTOM = "custom"


class LEDState(BaseModel):
    """LED state at a specific time."""
    time: float  # Time in seconds from start of show
    color: LEDColor
    effect: LEDEffect = LEDEffect.SOLID
    effect_params: Optional[Dict[str, Any]] = None


class Position(BaseModel):
    """3D position with latitude, longitude, and altitude."""
    lat: float  # Latitude in degrees
    lon: float  # Longitude in degrees
    alt: float  # Altitude in meters above ground level


class Waypoint(BaseModel):
    """Waypoint with position and time."""
    time: float  # Time in seconds from start of show
    position: Position
    heading: Optional[float] = None  # Heading in degrees (0-360)
    led_state: Optional[LEDState] = None


class DroneTrajectory(BaseModel):
    """Trajectory for a single drone."""
    drone_id: str
    waypoints: List[Waypoint]
    led_states: List[LEDState]


class FormationType(str, Enum):
    """Types of formations."""
    GRID = "grid"
    CIRCLE = "circle"
    LINE = "line"
    CUSTOM = "custom"


class Formation(BaseModel):
    """Formation definition."""
    type: FormationType
    params: Dict[str, Any]


class ChoreographyType(str, Enum):
    """Types of choreographies."""
    WAYPOINT = "waypoint"  # Direct waypoint-based choreography
    FORMATION = "formation"  # Formation-based choreography
    BLENDER = "blender"  # Blender-exported choreography
    PROCEDURAL = "procedural"  # Procedurally generated choreography


class ChoreographyStatus(str, Enum):
    """Status of a choreography."""
    DRAFT = "draft"
    READY = "ready"
    SIMULATED = "simulated"
    EXECUTED = "executed"
    ARCHIVED = "archived"


class ChoreographyMetadata(BaseModel):
    """Metadata for a choreography."""
    name: str
    description: Optional[str] = None
    author: Optional[str] = None
    created_at: datetime = Field(default_factory=datetime.utcnow)
    updated_at: datetime = Field(default_factory=datetime.utcnow)
    tags: List[str] = []
    duration: float  # Duration in seconds
    drone_count: int
    status: ChoreographyStatus = ChoreographyStatus.DRAFT


class Choreography(BaseModel):
    """Complete choreography definition."""
    id: str = Field(default_factory=lambda: str(uuid.uuid4()))
    metadata: ChoreographyMetadata
    type: ChoreographyType
    trajectories: List[DroneTrajectory]
    formations: Optional[List[Formation]] = None
    music_file: Optional[str] = None  # Path to music file
    music_bpm: Optional[float] = None  # Beats per minute
    music_offset: Optional[float] = None  # Offset in seconds
    boundary: Optional[List[Position]] = None  # Geofence boundary
    home_position: Optional[Position] = None  # Home position for all drones
    notes: Optional[str] = None  # Additional notes


class ChoreographyCreate(BaseModel):
    """Model for creating a new choreography."""
    metadata: ChoreographyMetadata
    type: ChoreographyType
    trajectories: List[DroneTrajectory]
    formations: Optional[List[Formation]] = None
    music_file: Optional[str] = None
    music_bpm: Optional[float] = None
    music_offset: Optional[float] = None
    boundary: Optional[List[Position]] = None
    home_position: Optional[Position] = None
    notes: Optional[str] = None


class ChoreographyUpdate(BaseModel):
    """Model for updating a choreography."""
    metadata: Optional[ChoreographyMetadata] = None
    type: Optional[ChoreographyType] = None
    trajectories: Optional[List[DroneTrajectory]] = None
    formations: Optional[List[Formation]] = None
    music_file: Optional[str] = None
    music_bpm: Optional[float] = None
    music_offset: Optional[float] = None
    boundary: Optional[List[Position]] = None
    home_position: Optional[Position] = None
    notes: Optional[str] = None


class ChoreographyResponse(Choreography):
    """Response model for choreography operations."""
    pass


class SimulationSettings(BaseModel):
    """Settings for simulation."""
    start_time: float = 0.0  # Start time in seconds
    end_time: Optional[float] = None  # End time in seconds (None for full duration)
    speed_factor: float = 1.0  # Playback speed factor
    include_takeoff_landing: bool = True  # Include takeoff and landing phases
    visualize_led: bool = True  # Visualize LED states
    visualize_trajectories: bool = True  # Visualize trajectories
    drone_model: str = "generic"  # Drone model for visualization


class SimulationFrame(BaseModel):
    """Single frame of simulation data."""
    time: float  # Time in seconds from start of show
    drone_states: Dict[str, Dict[str, Any]]  # Drone ID -> state (position, orientation, LED)


class SimulationResponse(BaseModel):
    """Response model for simulation operations."""
    id: str
    choreography_id: str
    settings: SimulationSettings
    frames: List[SimulationFrame]
    duration: float  # Duration in seconds
    created_at: datetime = Field(default_factory=datetime.utcnow)


class ExecutionStatus(str, Enum):
    """Status of a choreography execution."""
    PENDING = "pending"
    PREPARING = "preparing"
    IN_PROGRESS = "in_progress"
    PAUSED = "paused"
    COMPLETED = "completed"
    ABORTED = "aborted"
    FAILED = "failed"


class ExecutionSettings(BaseModel):
    """Settings for execution."""
    include_takeoff_landing: bool = True  # Include takeoff and landing phases
    use_rtk: bool = True  # Use RTK GPS for positioning
    safety_checks: bool = True  # Perform safety checks before execution
    geofence_enabled: bool = True  # Enable geofence
    return_home_on_low_battery: bool = True  # Return home on low battery
    return_home_on_connection_loss: bool = True  # Return home on connection loss
    led_enabled: bool = True  # Enable LED control


class DroneStatus(BaseModel):
    """Status of a drone during execution."""
    drone_id: str
    connected: bool
    battery_level: float  # Percentage
    position: Optional[Position] = None
    heading: Optional[float] = None  # Degrees
    led_state: Optional[LEDState] = None
    current_waypoint_index: Optional[int] = None
    status: str  # "idle", "takeoff", "executing", "landing", "returned", "error"
    error_message: Optional[str] = None


class ExecutionResponse(BaseModel):
    """Response model for execution operations."""
    id: str
    choreography_id: str
    settings: ExecutionSettings
    status: ExecutionStatus
    drone_statuses: Dict[str, DroneStatus]
    start_time: Optional[datetime] = None
    end_time: Optional[datetime] = None
    current_time: float = 0.0  # Current time in seconds from start of show
    progress: float = 0.0  # Progress percentage (0-100)
    error_message: Optional[str] = None
    created_at: datetime = Field(default_factory=datetime.utcnow)
    updated_at: datetime = Field(default_factory=datetime.utcnow)
