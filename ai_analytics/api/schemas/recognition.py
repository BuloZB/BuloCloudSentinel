"""
Recognition API schemas for the AI Analytics module.
"""

from pydantic import BaseModel, Field
from typing import List, Dict, Any, Optional
from datetime import datetime
from .detection import BoundingBox

class FaceImage(BaseModel):
    """A face image for a profile."""
    id: str
    profile_id: str
    image_url: str
    embedding: Optional[List[float]] = None
    created_at: datetime = Field(default_factory=datetime.now)

class FaceProfile(BaseModel):
    """A face profile for recognition."""
    id: str
    name: str
    description: Optional[str] = None
    tags: Optional[List[str]] = None
    images: List[FaceImage]
    alert_on_recognition: bool = False
    created_at: datetime = Field(default_factory=datetime.now)
    updated_at: datetime = Field(default_factory=datetime.now)

class LicensePlate(BaseModel):
    """A license plate for recognition."""
    id: str
    plate_number: str
    description: Optional[str] = None
    tags: Optional[List[str]] = None
    alert_on_recognition: bool = False
    created_at: datetime = Field(default_factory=datetime.now)
    updated_at: datetime = Field(default_factory=datetime.now)

class RecognizedFace(BaseModel):
    """A recognized face."""
    bounding_box: BoundingBox
    profile_id: Optional[str] = None
    profile_name: Optional[str] = None
    confidence: float
    embedding: Optional[List[float]] = None

class RecognizedLicensePlate(BaseModel):
    """A recognized license plate."""
    bounding_box: BoundingBox
    plate_number: str
    confidence: float
    plate_id: Optional[str] = None

class RecognitionResult(BaseModel):
    """A recognition result."""
    id: str
    camera_id: str
    timestamp: datetime
    frame_id: int
    faces: Optional[List[RecognizedFace]] = None
    license_plates: Optional[List[RecognizedLicensePlate]] = None
    processing_time: float  # in milliseconds

class RecognitionConfig(BaseModel):
    """Configuration for recognition."""
    face_recognition_enabled: bool = True
    license_plate_recognition_enabled: bool = True
    face_recognition_threshold: float = 0.7
    license_plate_confidence_threshold: float = 0.8
    max_recognition_fps: Optional[int] = None
