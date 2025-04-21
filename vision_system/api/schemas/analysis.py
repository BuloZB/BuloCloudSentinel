"""
Analysis schemas for the Vision System.
"""

from pydantic import BaseModel, Field
from typing import List, Dict, Any, Optional
from datetime import datetime
from enum import Enum

class AnalysisType(str, Enum):
    """Analysis type enumeration."""
    CROWD = "crowd"
    VEHICLE = "vehicle"
    FLOW = "flow"
    ALL = "all"

class AnalysisJobStatus(str, Enum):
    """Analysis job status enumeration."""
    PENDING = "pending"
    RUNNING = "running"
    COMPLETED = "completed"
    FAILED = "failed"
    CANCELED = "canceled"

class RegionOfInterest(BaseModel):
    """Region of interest model."""
    type: str  # polygon, rectangle, circle
    points: Optional[List[Dict[str, float]]] = None
    center: Optional[Dict[str, float]] = None
    radius: Optional[float] = None
    width: Optional[float] = None
    height: Optional[float] = None
    name: Optional[str] = None

class DetectedObject(BaseModel):
    """Detected object model."""
    id: str
    type: str  # person, car, truck, etc.
    confidence: float
    bounding_box: Dict[str, float]  # x, y, width, height (normalized 0-1)
    position: Optional[Dict[str, float]] = None  # x, y, z in world coordinates
    tracking_id: Optional[str] = None
    attributes: Optional[Dict[str, Any]] = None

class CrowdDensityMap(BaseModel):
    """Crowd density map model."""
    width: int
    height: int
    data: List[float]  # Flattened 2D array of density values
    min_value: float
    max_value: float
    total_count: float

class VehicleCount(BaseModel):
    """Vehicle count model."""
    total: int
    by_type: Dict[str, int]  # car: 5, truck: 2, etc.
    by_region: Optional[Dict[str, int]] = None  # region1: 3, region2: 4, etc.

class TrackPoint(BaseModel):
    """Track point model."""
    x: float
    y: float
    frame: int
    timestamp: datetime

class Track(BaseModel):
    """Object track model."""
    id: str
    object_type: str
    points: List[TrackPoint]
    start_time: datetime
    end_time: Optional[datetime] = None
    length: float  # pixels or meters
    average_speed: Optional[float] = None  # pixels/s or m/s
    is_complete: bool = False

class FlowVector(BaseModel):
    """Flow vector model."""
    start_x: float
    start_y: float
    end_x: float
    end_y: float
    magnitude: float
    direction: float  # degrees
    count: int

class CrowdAnalysisResult(BaseModel):
    """Crowd analysis result model."""
    timestamp: datetime
    stream_id: str
    frame_id: Optional[int] = None
    region_of_interest: Optional[RegionOfInterest] = None
    people_count: int
    density_map: Optional[CrowdDensityMap] = None
    detected_people: Optional[List[DetectedObject]] = None
    average_density: float  # people per square meter
    max_density: float  # people per square meter
    density_threshold_exceeded: bool = False
    processing_time: float  # milliseconds

class VehicleAnalysisResult(BaseModel):
    """Vehicle analysis result model."""
    timestamp: datetime
    stream_id: str
    frame_id: Optional[int] = None
    region_of_interest: Optional[RegionOfInterest] = None
    vehicle_count: VehicleCount
    detected_vehicles: Optional[List[DetectedObject]] = None
    occupancy_rate: Optional[float] = None  # 0-1
    count_threshold_exceeded: bool = False
    processing_time: float  # milliseconds

class FlowAnalysisResult(BaseModel):
    """Flow analysis result model."""
    timestamp: datetime
    stream_id: str
    start_frame_id: Optional[int] = None
    end_frame_id: Optional[int] = None
    region_of_interest: Optional[RegionOfInterest] = None
    tracks: List[Track]
    flow_vectors: List[FlowVector]
    dominant_direction: Optional[float] = None  # degrees
    average_speed: Optional[float] = None  # pixels/s or m/s
    congestion_level: Optional[float] = None  # 0-1
    processing_time: float  # milliseconds

class AnalysisResult(BaseModel):
    """Analysis result model."""
    id: str
    timestamp: datetime
    stream_id: str
    analysis_type: AnalysisType
    job_id: Optional[str] = None
    crowd_result: Optional[CrowdAnalysisResult] = None
    vehicle_result: Optional[VehicleAnalysisResult] = None
    flow_result: Optional[FlowAnalysisResult] = None
    parameters: Optional[Dict[str, Any]] = None
    processing_time: float  # milliseconds

class AnalysisJob(BaseModel):
    """Analysis job model."""
    id: str
    stream_id: str
    analysis_types: List[AnalysisType]
    region_of_interest: Optional[RegionOfInterest] = None
    start_time: Optional[datetime] = None
    end_time: Optional[datetime] = None
    parameters: Optional[Dict[str, Any]] = None
    status: AnalysisJobStatus
    progress: float = 0.0  # 0-1
    result_ids: List[str] = []
    created_at: datetime = Field(default_factory=datetime.now)
    updated_at: datetime = Field(default_factory=datetime.now)
    completed_at: Optional[datetime] = None
    error_message: Optional[str] = None

class AnalysisConfig(BaseModel):
    """Analysis configuration model."""
    crowd_detection_enabled: bool = True
    vehicle_detection_enabled: bool = True
    flow_analysis_enabled: bool = True
    crowd_density_threshold: float = 2.0  # people per square meter
    vehicle_count_threshold: int = 50
    analysis_interval: float = 1.0  # seconds
    detection_confidence: float = 0.5
    max_detection_fps: Optional[int] = None
    tracking_enabled: bool = True
    tracking_max_age: int = 30  # frames
    tracking_min_hits: int = 3  # frames
    tracking_iou_threshold: float = 0.3
    crowd_model: str = "csrnet"
    vehicle_model: str = "yolov8m"
    tracking_model: str = "deepsort"
    enable_gpu: bool = True
    batch_size: int = 1
