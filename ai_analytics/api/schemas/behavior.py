"""
Behavior analysis API schemas for the AI Analytics module.
"""

from pydantic import BaseModel, Field
from typing import List, Dict, Any, Optional
from datetime import datetime
from .detection import Point, BoundingBox

class BehaviorZone(BaseModel):
    """A zone for behavior analysis."""
    id: str
    name: str
    camera_id: str
    points: List[Point]
    enabled_patterns: List[str]
    description: Optional[str] = None
    created_at: datetime = Field(default_factory=datetime.now)
    updated_at: datetime = Field(default_factory=datetime.now)

class BehaviorPattern(BaseModel):
    """A behavior pattern for analysis."""
    id: str
    name: str
    description: Optional[str] = None
    pattern_type: str  # loitering, running, direction_change, crowd, etc.
    parameters: Dict[str, Any]
    alert_level: str = "info"  # info, warning, critical
    created_at: datetime = Field(default_factory=datetime.now)
    updated_at: datetime = Field(default_factory=datetime.now)

class BehaviorObject(BaseModel):
    """An object involved in a behavior event."""
    tracking_id: str
    class_id: str
    class_name: str
    bounding_box: BoundingBox
    trajectory: Optional[List[Point]] = None
    velocity: Optional[Dict[str, float]] = None
    attributes: Optional[Dict[str, Any]] = None

class BehaviorEvent(BaseModel):
    """A behavior event detected by analysis."""
    id: str
    camera_id: str
    zone_id: Optional[str] = None
    zone_name: Optional[str] = None
    pattern_id: str
    pattern_name: str
    pattern_type: str
    alert_level: str
    timestamp: datetime
    duration: Optional[float] = None  # in seconds
    objects: List[BehaviorObject]
    confidence: float
    details: Optional[Dict[str, Any]] = None
    snapshot_url: Optional[str] = None

class BehaviorConfig(BaseModel):
    """Configuration for behavior analysis."""
    enabled: bool = True
    loitering_threshold: int = 60  # seconds
    crowd_threshold: int = 5  # number of people
    running_speed_threshold: float = 2.0  # m/s
    direction_change_threshold: float = 90.0  # degrees
    max_analysis_fps: Optional[int] = None
