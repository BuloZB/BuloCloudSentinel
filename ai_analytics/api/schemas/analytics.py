"""
Analytics API schemas for the AI Analytics module.
"""

from pydantic import BaseModel, Field
from typing import List, Dict, Any, Optional
from datetime import datetime

class PredictionModel(BaseModel):
    """A prediction model for analytics."""
    id: str
    name: str
    description: Optional[str] = None
    model_type: str  # time_series, classification, regression
    target_entity: str  # person, vehicle, face, license_plate, etc.
    target_property: str  # count, presence, direction, etc.
    parameters: Dict[str, Any]
    cameras: List[str]
    status: str = "created"  # created, training, ready, failed
    last_trained: Optional[datetime] = None
    accuracy: Optional[float] = None
    created_at: datetime = Field(default_factory=datetime.now)
    updated_at: datetime = Field(default_factory=datetime.now)

class Prediction(BaseModel):
    """A prediction from a model."""
    id: str
    model_id: str
    camera_id: Optional[str] = None
    timestamp: datetime
    prediction_time: datetime
    value: Any
    confidence: Optional[float] = None
    features: Optional[Dict[str, Any]] = None

class Anomaly(BaseModel):
    """An anomaly detected by analytics."""
    id: str
    camera_id: str
    timestamp: datetime
    entity_type: str  # person, vehicle, face, license_plate, etc.
    property: str  # count, presence, direction, etc.
    expected_value: Any
    actual_value: Any
    severity: float  # 0.0 to 1.0
    description: str
    snapshot_url: Optional[str] = None

class AnalyticsReport(BaseModel):
    """An analytics report."""
    id: str
    report_type: str  # daily, weekly, monthly, custom
    start_date: datetime
    end_date: Optional[datetime] = None
    cameras: Optional[List[str]] = None
    entities: Optional[List[str]] = None
    include_predictions: bool = False
    include_anomalies: bool = False
    format: str = "json"
    status: str = "generating"  # generating, ready, failed
    url: Optional[str] = None
    created_at: datetime = Field(default_factory=datetime.now)
    completed_at: Optional[datetime] = None

class AnalyticsConfig(BaseModel):
    """Configuration for analytics."""
    enabled: bool = True
    anomaly_detection_enabled: bool = True
    prediction_enabled: bool = True
    data_retention_days: int = 90
    training_interval_hours: int = 24
