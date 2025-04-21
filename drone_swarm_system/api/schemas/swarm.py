"""
Swarm schemas for the Drone Swarm System.
"""

from pydantic import BaseModel, Field
from typing import List, Dict, Any, Optional
from datetime import datetime
from enum import Enum

class SwarmStatus(str, Enum):
    """Swarm status enumeration."""
    INACTIVE = "inactive"
    FORMING = "forming"
    ACTIVE = "active"
    MISSION = "mission"
    DISBANDING = "disbanding"
    ERROR = "error"

class FormationType(str, Enum):
    """Formation type enumeration."""
    NONE = "none"
    LINE = "line"
    COLUMN = "column"
    V_SHAPE = "v_shape"
    TRIANGLE = "triangle"
    SQUARE = "square"
    CIRCLE = "circle"
    GRID = "grid"
    CUSTOM = "custom"

class BehaviorType(str, Enum):
    """Behavior type enumeration."""
    NONE = "none"
    FOLLOW_LEADER = "follow_leader"
    MAINTAIN_FORMATION = "maintain_formation"
    AREA_COVERAGE = "area_coverage"
    PERIMETER_SURVEILLANCE = "perimeter_surveillance"
    OBSTACLE_AVOIDANCE = "obstacle_avoidance"
    DYNAMIC_POSITIONING = "dynamic_positioning"
    CUSTOM = "custom"

class SwarmFormation(BaseModel):
    """Swarm formation model."""
    formation_type: FormationType
    parameters: Optional[Dict[str, Any]] = None
    spacing: float = 5.0  # meters
    altitude_offset: Optional[float] = None  # meters
    heading_mode: str = "align"  # align, independent, towards_center
    reference_drone_id: Optional[str] = None

class SwarmBehavior(BaseModel):
    """Swarm behavior model."""
    behavior_type: BehaviorType
    parameters: Optional[Dict[str, Any]] = None
    priority: int = 1  # higher number = higher priority

class MeshNetworkStatus(BaseModel):
    """Mesh network status model."""
    active: bool
    connected_drones: int
    signal_quality: float  # 0-100%
    bandwidth: Optional[float] = None  # Mbps
    latency: Optional[float] = None  # ms
    topology: Optional[Dict[str, List[str]]] = None  # drone_id -> [connected_drone_ids]

class SwarmState(BaseModel):
    """Swarm state model."""
    swarm_id: str
    status: SwarmStatus
    drone_count: int
    active_drones: List[str]
    formation: Optional[SwarmFormation] = None
    behavior: Optional[SwarmBehavior] = None
    mesh_network: Optional[MeshNetworkStatus] = None
    mission_id: Optional[str] = None
    centroid: Optional[Dict[str, float]] = None  # lat, lon, alt
    dispersion: Optional[float] = None  # meters
    last_updated: datetime = Field(default_factory=datetime.now)

class SwarmCreate(BaseModel):
    """Model for creating a new swarm."""
    name: str
    description: Optional[str] = None
    drone_ids: List[str]
    formation: Optional[SwarmFormation] = None
    behavior: Optional[SwarmBehavior] = None
    parameters: Optional[Dict[str, Any]] = None
    tags: Optional[List[str]] = None

class SwarmUpdate(BaseModel):
    """Model for updating a swarm."""
    name: Optional[str] = None
    description: Optional[str] = None
    drone_ids: Optional[List[str]] = None
    formation: Optional[SwarmFormation] = None
    behavior: Optional[SwarmBehavior] = None
    parameters: Optional[Dict[str, Any]] = None
    tags: Optional[List[str]] = None

class Swarm(BaseModel):
    """Swarm model."""
    id: str
    name: str
    description: Optional[str] = None
    drone_ids: List[str]
    formation: Optional[SwarmFormation] = None
    behavior: Optional[SwarmBehavior] = None
    parameters: Optional[Dict[str, Any]] = None
    tags: Optional[List[str]] = None
    status: SwarmStatus = SwarmStatus.INACTIVE
    created_at: datetime = Field(default_factory=datetime.now)
    updated_at: datetime = Field(default_factory=datetime.now)
