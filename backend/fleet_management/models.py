"""
Fleet management models for Bulo.Cloud Sentinel.

This module provides data models for managing fleets of drones,
including formations, behaviors, and fleet missions.
"""

from enum import Enum
from typing import List, Dict, Any, Optional, Union
from pydantic import BaseModel, Field
import uuid
from datetime import datetime


class DroneRole(str, Enum):
    """Roles that drones can take in a fleet."""
    LEADER = "leader"
    FOLLOWER = "follower"
    SCOUT = "scout"
    RELAY = "relay"
    OBSERVER = "observer"


class FormationType(str, Enum):
    """Types of formations for drone fleets."""
    LINE = "line"
    GRID = "grid"
    CIRCLE = "circle"
    V_SHAPE = "v_shape"
    CUSTOM = "custom"


class BehaviorType(str, Enum):
    """Types of swarm behaviors."""
    FOLLOW_LEADER = "follow_leader"
    DISTRIBUTED_SEARCH = "distributed_search"
    PERIMETER_SURVEILLANCE = "perimeter_surveillance"
    OBSTACLE_AVOIDANCE = "obstacle_avoidance"
    CUSTOM = "custom"


class FleetDrone(BaseModel):
    """A drone in a fleet with its role and position."""
    drone_id: str
    role: DroneRole
    relative_position: Optional[Dict[str, float]] = None
    parameters: Dict[str, Any] = Field(default_factory=dict)


class Fleet(BaseModel):
    """A fleet of drones."""
    id: str = Field(default_factory=lambda: str(uuid.uuid4()))
    name: str
    description: Optional[str] = None
    drones: List[FleetDrone] = Field(default_factory=list)
    created_at: datetime = Field(default_factory=datetime.utcnow)
    updated_at: datetime = Field(default_factory=datetime.utcnow)


class Formation(BaseModel):
    """A formation for a fleet of drones."""
    id: str = Field(default_factory=lambda: str(uuid.uuid4()))
    name: str
    type: FormationType
    parameters: Dict[str, Any] = Field(default_factory=dict)
    drone_positions: Dict[str, Dict[str, float]] = Field(default_factory=dict)


class Behavior(BaseModel):
    """A swarm behavior for a fleet of drones."""
    id: str = Field(default_factory=lambda: str(uuid.uuid4()))
    name: str
    type: BehaviorType
    parameters: Dict[str, Any] = Field(default_factory=dict)


class FleetMission(BaseModel):
    """A mission for a fleet of drones."""
    id: str = Field(default_factory=lambda: str(uuid.uuid4()))
    fleet_id: str
    name: str
    description: Optional[str] = None
    formation_id: Optional[str] = None
    behavior_id: Optional[str] = None
    drone_missions: Dict[str, str] = Field(default_factory=dict)  # Mapping of drone_id to mission_id
    created_at: datetime = Field(default_factory=datetime.utcnow)
    updated_at: datetime = Field(default_factory=datetime.utcnow)
    status: str = "pending"


# Request/Response models
class FleetCreate(BaseModel):
    """Request model for creating a fleet."""
    name: str
    description: Optional[str] = None
    drones: List[FleetDrone] = Field(default_factory=list)


class FleetResponse(Fleet):
    """Response model for fleet operations."""
    pass


class FormationCreate(BaseModel):
    """Request model for creating a formation."""
    name: str
    type: FormationType
    parameters: Dict[str, Any] = Field(default_factory=dict)
    drone_positions: Optional[Dict[str, Dict[str, float]]] = None


class FormationResponse(Formation):
    """Response model for formation operations."""
    pass


class BehaviorCreate(BaseModel):
    """Request model for creating a behavior."""
    name: str
    type: BehaviorType
    parameters: Dict[str, Any] = Field(default_factory=dict)


class BehaviorResponse(Behavior):
    """Response model for behavior operations."""
    pass


class FleetMissionCreate(BaseModel):
    """Request model for creating a fleet mission."""
    name: str
    description: Optional[str] = None
    formation_id: Optional[str] = None
    behavior_id: Optional[str] = None
    mission_template_id: Optional[str] = None
    individual_missions: Optional[Dict[str, str]] = None  # Mapping of drone_id to mission_id


class FleetMissionResponse(FleetMission):
    """Response model for fleet mission operations."""
    pass


# Database models
class FleetModel(BaseModel):
    """Database model for a fleet."""
    id: str
    name: str
    description: Optional[str] = None
    drones: str  # JSON string of drones
    created_at: datetime
    updated_at: datetime


class FormationModel(BaseModel):
    """Database model for a formation."""
    id: str
    name: str
    type: str
    parameters: str  # JSON string of parameters
    drone_positions: str  # JSON string of drone positions


class BehaviorModel(BaseModel):
    """Database model for a behavior."""
    id: str
    name: str
    type: str
    parameters: str  # JSON string of parameters


class FleetMissionModel(BaseModel):
    """Database model for a fleet mission."""
    id: str
    fleet_id: str
    name: str
    description: Optional[str] = None
    formation_id: Optional[str] = None
    behavior_id: Optional[str] = None
    drone_missions: str  # JSON string of drone missions
    created_at: datetime
    updated_at: datetime
    status: str
