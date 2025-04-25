"""
Schemas for multimodal detection API.

This module provides Pydantic models for multimodal detection API requests and responses.
"""

from typing import List, Dict, Any, Optional, Union
from datetime import datetime
from pydantic import BaseModel, Field


class BoundingBox(BaseModel):
    """Bounding box for detected objects."""
    
    x: float = Field(..., description="X coordinate of the top-left corner")
    y: float = Field(..., description="Y coordinate of the top-left corner")
    width: float = Field(..., description="Width of the bounding box")
    height: float = Field(..., description="Height of the bounding box")


class ObjectClass(BaseModel):
    """Class information for detected objects."""
    
    id: int = Field(..., description="Class ID")
    name: str = Field(..., description="Class name")
    confidence: float = Field(..., description="Detection confidence")


class MultimodalDetectionParameters(BaseModel):
    """Parameters for multimodal detection."""
    
    confidence_threshold: float = Field(0.5, description="Confidence threshold for detection")
    nms_threshold: float = Field(0.45, description="Non-maximum suppression threshold")
    max_detections: int = Field(100, description="Maximum number of detections to return")
    enabled_classes: Optional[List[str]] = Field(None, description="List of enabled class names")
    disabled_classes: Optional[List[str]] = Field(None, description="List of disabled class names")
    modality_weights: Optional[Dict[str, float]] = Field(None, description="Weights for each modality")


class RegionOfInterest(BaseModel):
    """Region of interest for detection."""
    
    type: str = Field(..., description="Type of ROI (rectangle, polygon)")
    x: Optional[float] = Field(None, description="X coordinate (for rectangle)")
    y: Optional[float] = Field(None, description="Y coordinate (for rectangle)")
    width: Optional[float] = Field(None, description="Width (for rectangle)")
    height: Optional[float] = Field(None, description="Height (for rectangle)")
    points: Optional[List[List[float]]] = Field(None, description="Points (for polygon)")


class MultimodalDetectionZone(BaseModel):
    """Detection zone for triggering events."""
    
    id: str = Field(..., description="Zone ID")
    name: str = Field(..., description="Zone name")
    type: str = Field(..., description="Type of zone (rectangle, polygon)")
    x: Optional[float] = Field(None, description="X coordinate (for rectangle)")
    y: Optional[float] = Field(None, description="Y coordinate (for rectangle)")
    width: Optional[float] = Field(None, description="Width (for rectangle)")
    height: Optional[float] = Field(None, description="Height (for rectangle)")
    points: Optional[List[List[float]]] = Field(None, description="Points (for polygon)")
    enabled_classes: Optional[List[str]] = Field(None, description="List of enabled class names for this zone")
    min_confidence: Optional[float] = Field(None, description="Minimum confidence for this zone")
    min_dwell_time: Optional[float] = Field(None, description="Minimum dwell time in seconds")
    max_objects: Optional[int] = Field(None, description="Maximum number of objects in zone")
    trigger_on_enter: bool = Field(True, description="Trigger when object enters zone")
    trigger_on_exit: bool = Field(True, description="Trigger when object exits zone")
    trigger_on_dwell: bool = Field(False, description="Trigger when object dwells in zone")


class MultimodalDetectionConfig(BaseModel):
    """Configuration for multimodal detection."""
    
    enabled: bool = Field(True, description="Whether detection is enabled")
    interval: Optional[float] = Field(None, description="Detection interval in seconds")
    max_duration: Optional[float] = Field(None, description="Maximum duration in seconds (0 = no limit)")
    region_of_interest: Optional[RegionOfInterest] = Field(None, description="Region of interest")
    parameters: Optional[MultimodalDetectionParameters] = Field(None, description="Detection parameters")
    detection_zones: Optional[List[MultimodalDetectionZone]] = Field(None, description="Detection zones")
    save_frames: bool = Field(False, description="Whether to save frames with detections")
    save_path: Optional[str] = Field(None, description="Path to save frames")
    publish_events: bool = Field(True, description="Whether to publish detection events")
    event_throttle: Optional[float] = Field(None, description="Event throttle in seconds")


class ModalityInfo(BaseModel):
    """Information about a modality."""
    
    width: int = Field(..., description="Frame width")
    height: int = Field(..., description="Frame height")
    channels: int = Field(..., description="Number of channels")


class MultimodalDetectedObject(BaseModel):
    """Detected object from multimodal detection."""
    
    id: str = Field(..., description="Object ID")
    bbox: BoundingBox = Field(..., description="Bounding box")
    class_info: ObjectClass = Field(..., description="Class information")
    modalities: List[str] = Field(..., description="Modalities that detected this object")
    zones: List[str] = Field(default_factory=list, description="Zones containing this object")
    attributes: Dict[str, Any] = Field(default_factory=dict, description="Additional attributes")
    tracking_id: Optional[str] = Field(None, description="Tracking ID (if tracking is enabled)")


class MultimodalDetectionResult(BaseModel):
    """Result of multimodal detection."""
    
    timestamp: datetime = Field(..., description="Timestamp of the detection")
    processing_time: float = Field(..., description="Processing time in milliseconds")
    objects: List[MultimodalDetectedObject] = Field(..., description="Detected objects")
    modality_info: Dict[str, ModalityInfo] = Field(..., description="Information about each modality")
    frame_urls: Optional[Dict[str, str]] = Field(None, description="URLs of saved frames")


class StartDetectionRequest(BaseModel):
    """Request to start multimodal detection."""
    
    stream_ids: Dict[str, str] = Field(..., description="Dictionary mapping modality names to stream IDs")
    config: MultimodalDetectionConfig = Field(..., description="Detection configuration")


class StartDetectionResponse(BaseModel):
    """Response to start multimodal detection request."""
    
    task_id: str = Field(..., description="Task ID for the detection task")
    status: str = Field(..., description="Status of the task")


class StopDetectionRequest(BaseModel):
    """Request to stop multimodal detection."""
    
    task_id: str = Field(..., description="Task ID to stop")


class StopDetectionResponse(BaseModel):
    """Response to stop multimodal detection request."""
    
    task_id: str = Field(..., description="Task ID that was stopped")
    status: str = Field(..., description="Status of the operation")


class GetDetectionStatusResponse(BaseModel):
    """Response to get detection status request."""
    
    task_id: str = Field(..., description="Task ID")
    status: str = Field(..., description="Status of the task")
    stream_ids: Optional[Dict[str, str]] = Field(None, description="Dictionary mapping modality names to stream IDs")
    config: Optional[MultimodalDetectionConfig] = Field(None, description="Detection configuration")
    start_time: Optional[datetime] = Field(None, description="Start time of the task")
    elapsed_time: Optional[float] = Field(None, description="Elapsed time in seconds")


class GetActiveTasksResponse(BaseModel):
    """Response to get active tasks request."""
    
    tasks: List[GetDetectionStatusResponse] = Field(..., description="List of active tasks")
