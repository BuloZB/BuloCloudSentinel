"""
Detection API schemas for the AI Analytics module.
"""

from pydantic import BaseModel, Field
from typing import List, Dict, Any, Optional
from datetime import datetime

class Point(BaseModel):
    """A 2D point."""
    x: float
    y: float

class BoundingBox(BaseModel):
    """A bounding box for an object."""
    x: float
    y: float
    width: float
    height: float

class ObjectClass(BaseModel):
    """An object class for detection."""
    id: str
    name: str
    description: Optional[str] = None
    color: Optional[str] = None

class DetectionZone(BaseModel):
    """A zone for object detection."""
    id: str
    name: str
    camera_id: str
    points: List[Point]
    enabled_classes: Optional[List[str]] = None
    min_confidence: Optional[float] = None
    created_at: datetime = Field(default_factory=datetime.now)
    updated_at: datetime = Field(default_factory=datetime.now)

class DetectedObject(BaseModel):
    """A detected object."""
    id: str
    class_id: str
    class_name: str
    confidence: float
    bounding_box: BoundingBox
    tracking_id: Optional[str] = None
    attributes: Optional[Dict[str, Any]] = None

class DetectionResult(BaseModel):
    """A detection result."""
    id: str
    camera_id: str
    timestamp: datetime
    frame_id: int
    objects: List[DetectedObject]
    zones_triggered: Optional[List[str]] = None
    processing_time: float  # in milliseconds

class DetectionConfig(BaseModel):
    """Configuration for object detection."""
    model_name: str
    confidence_threshold: float = 0.5
    enabled_classes: List[str]
    zones: Optional[List[DetectionZone]] = None
    max_detection_fps: Optional[int] = None
    tracking_enabled: bool = True
    tracking_max_age: Optional[int] = None  # frames
    tracking_min_hits: Optional[int] = None  # frames
    tracking_iou_threshold: Optional[float] = None
