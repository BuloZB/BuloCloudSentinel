"""
Calibration schemas for the Vision System.
"""

from pydantic import BaseModel, Field
from typing import List, Dict, Any, Optional
from datetime import datetime
import numpy as np

class CalibrationType(str, Enum):
    """Calibration type enumeration."""
    INTRINSIC = "intrinsic"
    EXTRINSIC = "extrinsic"
    PERSPECTIVE = "perspective"

class CameraMatrix(BaseModel):
    """Camera matrix model."""
    fx: float
    fy: float
    cx: float
    cy: float
    matrix: List[List[float]]  # 3x3 matrix

    class Config:
        arbitrary_types_allowed = True

class DistortionCoefficients(BaseModel):
    """Distortion coefficients model."""
    k1: float
    k2: float
    p1: float
    p2: float
    k3: Optional[float] = None
    coefficients: List[float]

    class Config:
        arbitrary_types_allowed = True

class PerspectiveMap(BaseModel):
    """Perspective map model."""
    stream_id: str
    image_points: List[Dict[str, float]]  # [{x: 0.1, y: 0.2}, ...]
    world_points: List[Dict[str, float]]  # [{x: 10.5, y: 20.3}, ...]
    transformation_matrix: List[List[float]]  # 3x3 matrix
    inverse_matrix: List[List[float]]  # 3x3 matrix
    created_at: datetime = Field(default_factory=datetime.now)
    updated_at: datetime = Field(default_factory=datetime.now)

    class Config:
        arbitrary_types_allowed = True

class ScaleFactor(BaseModel):
    """Scale factor model."""
    stream_id: str
    pixels_per_unit: float
    unit: str = "meters"
    reference_distance: float
    reference_points: List[Dict[str, float]]  # [{x: 0.1, y: 0.2}, {x: 0.3, y: 0.4}]
    created_at: datetime = Field(default_factory=datetime.now)
    updated_at: datetime = Field(default_factory=datetime.now)

class CalibrationConfig(BaseModel):
    """Calibration configuration model."""
    stream_id: str
    calibration_types: List[CalibrationType] = []
    has_intrinsic_calibration: bool = False
    has_extrinsic_calibration: bool = False
    has_perspective_map: bool = False
    has_scale_factor: bool = False
    camera_matrix: Optional[CameraMatrix] = None
    distortion_coefficients: Optional[DistortionCoefficients] = None
    perspective_map: Optional[PerspectiveMap] = None
    scale_factor: Optional[ScaleFactor] = None
    parameters: Optional[Dict[str, Any]] = None

class CalibrationResult(BaseModel):
    """Calibration result model."""
    id: str
    stream_id: str
    calibration_type: CalibrationType
    timestamp: datetime
    camera_matrix: Optional[CameraMatrix] = None
    distortion_coefficients: Optional[DistortionCoefficients] = None
    perspective_map: Optional[PerspectiveMap] = None
    scale_factor: Optional[ScaleFactor] = None
    reprojection_error: Optional[float] = None
    parameters: Optional[Dict[str, Any]] = None
    created_at: datetime = Field(default_factory=datetime.now)
