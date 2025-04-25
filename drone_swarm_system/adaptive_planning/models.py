"""
Models for adaptive mission planning.

This module provides data models for adaptive mission planning, including
mission plans, waypoints, adaptation rules, and environmental conditions.
"""

from enum import Enum, auto
from typing import List, Dict, Any, Optional, Union, Set, Tuple
from pydantic import BaseModel, Field, validator
from datetime import datetime
import uuid


class AdaptationPriority(str, Enum):
    """Priority levels for adaptation actions."""
    CRITICAL = "critical"  # Safety-critical adaptations (collision avoidance, emergency landing)
    HIGH = "high"          # High-priority adaptations (battery conservation, weather avoidance)
    MEDIUM = "medium"      # Medium-priority adaptations (route optimization, timing adjustments)
    LOW = "low"            # Low-priority adaptations (mission enhancements, quality improvements)


class AdaptationStatus(str, Enum):
    """Status of an adaptation action."""
    PENDING = "pending"        # Adaptation is pending execution
    EXECUTING = "executing"    # Adaptation is currently being executed
    COMPLETED = "completed"    # Adaptation has been successfully completed
    FAILED = "failed"          # Adaptation has failed
    CANCELLED = "cancelled"    # Adaptation has been cancelled


class TriggerType(str, Enum):
    """Types of adaptation triggers."""
    OBSTACLE = "obstacle"              # New obstacle detected
    WEATHER = "weather"                # Weather condition change
    BATTERY = "battery"                # Battery level change
    TIMING = "timing"                  # Mission timing change
    TELEMETRY = "telemetry"            # Drone telemetry change
    SENSOR = "sensor"                  # Sensor data change
    COMMUNICATION = "communication"    # Communication status change
    MISSION = "mission"                # Mission parameter change
    MANUAL = "manual"                  # Manual trigger from operator
    SWARM = "swarm"                    # Swarm coordination trigger
    GEOFENCE = "geofence"              # Geofence boundary interaction
    SYSTEM = "system"                  # System status change


class ActionType(str, Enum):
    """Types of adaptation actions."""
    REROUTE = "reroute"                # Change route to avoid obstacle
    ADJUST_ALTITUDE = "adjust_altitude"  # Change altitude
    ADJUST_SPEED = "adjust_speed"      # Change speed
    WAIT = "wait"                      # Wait at current position
    RETURN_HOME = "return_home"        # Return to home position
    LAND = "land"                      # Land at current or specified position
    ABORT = "abort"                    # Abort mission
    ADD_WAYPOINT = "add_waypoint"      # Add new waypoint
    REMOVE_WAYPOINT = "remove_waypoint"  # Remove waypoint
    MODIFY_WAYPOINT = "modify_waypoint"  # Modify existing waypoint
    REORDER_WAYPOINTS = "reorder_waypoints"  # Change waypoint order
    CHANGE_SENSOR = "change_sensor"    # Change sensor settings
    NOTIFY = "notify"                  # Send notification
    CUSTOM = "custom"                  # Custom action


class Position(BaseModel):
    """3D position with latitude, longitude, and altitude."""
    latitude: float
    longitude: float
    altitude: float
    reference: str = "amsl"  # amsl, agl, ellipsoid


class Velocity(BaseModel):
    """3D velocity vector."""
    north: float  # m/s
    east: float   # m/s
    down: float   # m/s


class Attitude(BaseModel):
    """Attitude (orientation) in Euler angles."""
    roll: float   # degrees
    pitch: float  # degrees
    yaw: float    # degrees


class SensorAction(BaseModel):
    """Action for a drone sensor."""
    sensor_id: str
    action: str
    parameters: Dict[str, Any] = {}


class AdaptiveWaypoint(BaseModel):
    """
    Waypoint for adaptive mission planning.
    
    This extends the basic waypoint model with additional fields for
    adaptive planning, such as timing constraints and alternative positions.
    """
    id: str = Field(default_factory=lambda: str(uuid.uuid4()))
    position: Position
    heading: Optional[float] = None  # degrees, None for auto
    speed: Optional[float] = None  # m/s, None for default
    
    # Timing constraints
    arrival_time: Optional[datetime] = None  # Target arrival time
    departure_time: Optional[datetime] = None  # Target departure time
    loiter_time: Optional[float] = None  # seconds to wait at waypoint
    time_tolerance: Optional[float] = None  # seconds of acceptable deviation
    
    # Execution parameters
    acceptance_radius: Optional[float] = None  # meters
    pass_through: bool = False
    
    # Actions at waypoint
    sensor_actions: List[SensorAction] = []
    
    # Adaptation parameters
    is_critical: bool = False  # Critical waypoints cannot be removed
    priority: int = 1  # 1-10, higher is more important
    alternatives: List[Position] = []  # Alternative positions for this waypoint
    
    # Metadata
    created_at: datetime = Field(default_factory=datetime.utcnow)
    updated_at: datetime = Field(default_factory=datetime.utcnow)
    metadata: Dict[str, Any] = {}


class EnvironmentalCondition(BaseModel):
    """Environmental condition data."""
    condition_type: str  # weather, obstacle, traffic, etc.
    position: Optional[Position] = None
    radius: Optional[float] = None  # meters
    polygon: Optional[List[Position]] = None
    value: Any
    confidence: float = 1.0  # 0.0-1.0
    source: str
    timestamp: datetime = Field(default_factory=datetime.utcnow)
    expiration: Optional[datetime] = None
    metadata: Dict[str, Any] = {}


class ObstacleData(BaseModel):
    """Data about an obstacle."""
    id: str = Field(default_factory=lambda: str(uuid.uuid4()))
    position: Position
    velocity: Optional[Velocity] = None
    dimensions: Optional[Dict[str, float]] = None  # width, height, depth in meters
    obstacle_type: str  # static, dynamic, restricted_airspace, etc.
    confidence: float = 1.0  # 0.0-1.0
    first_detected: datetime = Field(default_factory=datetime.utcnow)
    last_updated: datetime = Field(default_factory=datetime.utcnow)
    source: str
    metadata: Dict[str, Any] = {}


class MissionConstraint(BaseModel):
    """Constraint for mission planning."""
    constraint_type: str  # altitude, speed, battery, etc.
    min_value: Optional[float] = None
    max_value: Optional[float] = None
    target_value: Optional[float] = None
    tolerance: Optional[float] = None
    is_hard_constraint: bool = True  # Hard constraints cannot be violated
    priority: int = 1  # 1-10, higher is more important
    metadata: Dict[str, Any] = {}


class AdaptationTrigger(BaseModel):
    """
    Trigger for adaptation actions.
    
    This defines when an adaptation should be considered based on
    environmental conditions, mission state, or other factors.
    """
    id: str = Field(default_factory=lambda: str(uuid.uuid4()))
    name: str
    description: Optional[str] = None
    trigger_type: TriggerType
    
    # Condition parameters
    condition: str  # Python expression or rule language
    parameters: Dict[str, Any] = {}
    
    # Metadata
    created_at: datetime = Field(default_factory=datetime.utcnow)
    updated_at: datetime = Field(default_factory=datetime.utcnow)
    enabled: bool = True
    metadata: Dict[str, Any] = {}


class AdaptationAction(BaseModel):
    """
    Action to take when an adaptation is triggered.
    
    This defines what changes should be made to the mission plan
    when an adaptation trigger is activated.
    """
    id: str = Field(default_factory=lambda: str(uuid.uuid4()))
    name: str
    description: Optional[str] = None
    action_type: ActionType
    
    # Action parameters
    parameters: Dict[str, Any] = {}
    
    # Execution constraints
    priority: AdaptationPriority = AdaptationPriority.MEDIUM
    requires_approval: bool = False
    timeout: Optional[float] = None  # seconds
    
    # Metadata
    created_at: datetime = Field(default_factory=datetime.utcnow)
    updated_at: datetime = Field(default_factory=datetime.utcnow)
    metadata: Dict[str, Any] = {}


class AdaptationRule(BaseModel):
    """
    Rule linking triggers to actions.
    
    This defines what actions should be taken when specific triggers
    are activated, along with priority and other metadata.
    """
    id: str = Field(default_factory=lambda: str(uuid.uuid4()))
    name: str
    description: Optional[str] = None
    
    # Triggers and actions
    triggers: List[str]  # List of trigger IDs
    actions: List[str]   # List of action IDs
    
    # Rule parameters
    priority: AdaptationPriority = AdaptationPriority.MEDIUM
    enabled: bool = True
    
    # Metadata
    created_at: datetime = Field(default_factory=datetime.utcnow)
    updated_at: datetime = Field(default_factory=datetime.utcnow)
    metadata: Dict[str, Any] = {}


class AdaptationEvent(BaseModel):
    """
    Record of an adaptation event.
    
    This records when an adaptation was triggered, what actions were taken,
    and the result of the adaptation.
    """
    id: str = Field(default_factory=lambda: str(uuid.uuid4()))
    mission_id: str
    rule_id: str
    trigger_id: str
    action_ids: List[str]
    
    # Event details
    timestamp: datetime = Field(default_factory=datetime.utcnow)
    status: AdaptationStatus = AdaptationStatus.PENDING
    result: Optional[Dict[str, Any]] = None
    
    # Metadata
    metadata: Dict[str, Any] = {}


class AdaptiveMissionPlan(BaseModel):
    """
    Adaptive mission plan.
    
    This extends the basic mission plan with additional fields for
    adaptive planning, such as adaptation rules and environmental conditions.
    """
    id: str = Field(default_factory=lambda: str(uuid.uuid4()))
    name: str
    description: Optional[str] = None
    
    # Mission parameters
    waypoints: List[AdaptiveWaypoint] = []
    constraints: List[MissionConstraint] = []
    
    # Adaptation parameters
    triggers: List[AdaptationTrigger] = []
    actions: List[AdaptationAction] = []
    rules: List[AdaptationRule] = []
    
    # Environmental data
    environmental_conditions: List[EnvironmentalCondition] = []
    obstacles: List[ObstacleData] = []
    
    # Execution state
    is_active: bool = False
    current_waypoint_index: Optional[int] = None
    adaptation_events: List[AdaptationEvent] = []
    
    # Metadata
    created_at: datetime = Field(default_factory=datetime.utcnow)
    updated_at: datetime = Field(default_factory=datetime.utcnow)
    metadata: Dict[str, Any] = {}
    
    @validator('rules')
    def validate_rules(cls, rules, values):
        """Validate that rule triggers and actions exist."""
        if not values.get('triggers') or not values.get('actions'):
            return rules
        
        trigger_ids = {t.id for t in values.get('triggers', [])}
        action_ids = {a.id for a in values.get('actions', [])}
        
        for rule in rules:
            for trigger_id in rule.triggers:
                if trigger_id not in trigger_ids:
                    raise ValueError(f"Rule {rule.name} references non-existent trigger ID: {trigger_id}")
            
            for action_id in rule.actions:
                if action_id not in action_ids:
                    raise ValueError(f"Rule {rule.name} references non-existent action ID: {action_id}")
        
        return rules
