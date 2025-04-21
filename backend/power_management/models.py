"""
Power Management models for Bulo.Cloud Sentinel.

This module provides data models for battery monitoring, energy prediction,
and power management settings.
"""

from enum import Enum
from typing import List, Dict, Any, Optional, Union
from pydantic import BaseModel, Field, validator
import uuid
from datetime import datetime, timedelta

class BatteryStatus(str, Enum):
    """Battery status types."""
    NORMAL = "normal"
    WARNING = "warning"
    CRITICAL = "critical"
    UNKNOWN = "unknown"

class BatteryMetrics(BaseModel):
    """Current battery metrics."""
    id: str = Field(default_factory=lambda: str(uuid.uuid4()))
    drone_id: str
    timestamp: datetime = Field(default_factory=datetime.utcnow)
    voltage: float  # Volts
    current: float  # Amperes
    capacity_percent: float  # Percentage (0-100)
    temperature: float  # Celsius
    discharge_rate: Optional[float] = None  # mAh/minute
    cycle_count: Optional[int] = None  # Number of charge cycles
    health_percent: Optional[float] = None  # Battery health percentage
    estimated_time_remaining: Optional[int] = None  # Seconds
    status: BatteryStatus = BatteryStatus.NORMAL
    raw_data: Optional[Dict[str, Any]] = None

class BatteryThresholds(BaseModel):
    """Battery alert thresholds."""
    id: str = Field(default_factory=lambda: str(uuid.uuid4()))
    drone_id: str
    warning_threshold: float = 30.0  # Percentage
    critical_threshold: float = 15.0  # Percentage
    temperature_max: float = 60.0  # Celsius
    voltage_min: Optional[float] = None  # Volts
    auto_return_threshold: float = 20.0  # Percentage
    created_at: datetime = Field(default_factory=datetime.utcnow)
    updated_at: datetime = Field(default_factory=datetime.utcnow)

class EnergyPredictionModel(str, Enum):
    """Energy prediction model types."""
    LINEAR = "linear"
    POLYNOMIAL = "polynomial"
    NEURAL_NETWORK = "neural_network"
    ENSEMBLE = "ensemble"
    CUSTOM = "custom"

class EnergyPrediction(BaseModel):
    """Energy consumption prediction for a mission."""
    id: str = Field(default_factory=lambda: str(uuid.uuid4()))
    mission_id: str
    drone_id: str
    timestamp: datetime = Field(default_factory=datetime.utcnow)
    model_type: EnergyPredictionModel
    estimated_consumption: float  # Percentage
    estimated_duration: int  # Seconds
    estimated_range: float  # Meters
    confidence: float  # 0-1
    margin_of_error: float  # Percentage
    is_feasible: bool
    factors: Dict[str, float] = Field(default_factory=dict)  # Influence factors
    recommendations: Optional[List[str]] = None

class PowerOptimizationSettings(BaseModel):
    """Power optimization settings."""
    id: str = Field(default_factory=lambda: str(uuid.uuid4()))
    drone_id: str
    enable_optimization: bool = True
    prioritize_mission_completion: bool = True
    max_speed_reduction: float = 30.0  # Percentage
    min_altitude: float = 20.0  # Meters
    max_altitude: float = 120.0  # Meters
    enable_auto_rth: bool = True
    enable_dynamic_waypoints: bool = True
    created_at: datetime = Field(default_factory=datetime.utcnow)
    updated_at: datetime = Field(default_factory=datetime.utcnow)

class BatteryHealthRecord(BaseModel):
    """Battery health record."""
    id: str = Field(default_factory=lambda: str(uuid.uuid4()))
    drone_id: str
    battery_id: str
    timestamp: datetime = Field(default_factory=datetime.utcnow)
    cycle_count: int
    health_percent: float  # 0-100
    initial_capacity: float  # mAh
    current_capacity: float  # mAh
    internal_resistance: Optional[float] = None  # mOhm
    temperature_profile: Optional[Dict[str, float]] = None
    estimated_remaining_cycles: Optional[int] = None
    recommendation: Optional[str] = None

# API Request/Response Models
class BatteryCurrentResponse(BatteryMetrics):
    """Response model for current battery status."""
    pass

class BatteryHistoryItem(BaseModel):
    """Historical battery data point."""
    timestamp: datetime
    voltage: float
    current: float
    capacity_percent: float
    temperature: float
    discharge_rate: Optional[float] = None
    status: BatteryStatus

class BatteryHistoryResponse(BaseModel):
    """Response model for battery history."""
    drone_id: str
    start_time: datetime
    end_time: datetime
    interval: str
    data: List[BatteryHistoryItem]

class FleetBatteryResponse(BaseModel):
    """Response model for fleet battery status."""
    fleet_id: str
    timestamp: datetime = Field(default_factory=datetime.utcnow)
    drones: Dict[str, BatteryMetrics]

class BatteryThresholdsRequest(BaseModel):
    """Request model for setting battery thresholds."""
    warning_threshold: float
    critical_threshold: float
    temperature_max: float
    voltage_min: Optional[float] = None
    auto_return_threshold: float

class BatteryThresholdsResponse(BatteryThresholds):
    """Response model for battery thresholds."""
    pass

class EnergyPredictionRequest(BaseModel):
    """Request model for energy prediction."""
    mission_id: Optional[str] = None
    drone_id: str
    waypoints: Optional[List[Dict[str, Any]]] = None
    payload_weight: Optional[float] = None  # kg
    wind_speed: Optional[float] = None  # m/s
    wind_direction: Optional[float] = None  # degrees
    temperature: Optional[float] = None  # Celsius
    model_type: Optional[EnergyPredictionModel] = None

class EnergyPredictionResponse(EnergyPrediction):
    """Response model for energy prediction."""
    pass

class OptimizeMissionRequest(BaseModel):
    """Request model for mission optimization."""
    mission_id: str
    drone_id: str
    current_battery: float  # Percentage
    optimization_level: Optional[float] = 0.5  # 0-1

class OptimizeMissionResponse(BaseModel):
    """Response model for mission optimization."""
    mission_id: str
    original_consumption: float  # Percentage
    optimized_consumption: float  # Percentage
    savings: float  # Percentage
    original_duration: int  # Seconds
    optimized_duration: int  # Seconds
    modifications: List[str]
    optimized_waypoints: Optional[List[Dict[str, Any]]] = None

class BatteryHealthRequest(BaseModel):
    """Request model for battery health analysis."""
    drone_id: str
    battery_id: Optional[str] = None
    start_date: Optional[datetime] = None
    end_date: Optional[datetime] = None

class BatteryHealthResponse(BaseModel):
    """Response model for battery health analysis."""
    drone_id: str
    battery_id: str
    current_health: float  # Percentage
    degradation_rate: float  # Percentage per cycle
    estimated_remaining_cycles: int
    recommendation: str
    history: List[BatteryHealthRecord]

# Database models
class BatteryMetricsModel(BaseModel):
    """Database model for battery metrics."""
    id: str
    drone_id: str
    timestamp: datetime
    voltage: float
    current: float
    capacity_percent: float
    temperature: float
    discharge_rate: Optional[float] = None
    cycle_count: Optional[int] = None
    health_percent: Optional[float] = None
    estimated_time_remaining: Optional[int] = None
    status: str
    raw_data: Optional[str] = None  # JSON string

class BatteryThresholdsModel(BaseModel):
    """Database model for battery thresholds."""
    id: str
    drone_id: str
    warning_threshold: float
    critical_threshold: float
    temperature_max: float
    voltage_min: Optional[float] = None
    auto_return_threshold: float
    created_at: datetime
    updated_at: datetime

class EnergyPredictionModel(BaseModel):
    """Database model for energy prediction."""
    id: str
    mission_id: str
    drone_id: str
    timestamp: datetime
    model_type: str
    estimated_consumption: float
    estimated_duration: int
    estimated_range: float
    confidence: float
    margin_of_error: float
    is_feasible: bool
    factors: str  # JSON string
    recommendations: Optional[str] = None  # JSON string

class PowerOptimizationSettingsModel(BaseModel):
    """Database model for power optimization settings."""
    id: str
    drone_id: str
    enable_optimization: bool
    prioritize_mission_completion: bool
    max_speed_reduction: float
    min_altitude: float
    max_altitude: float
    enable_auto_rth: bool
    enable_dynamic_waypoints: bool
    created_at: datetime
    updated_at: datetime

class BatteryHealthRecordModel(BaseModel):
    """Database model for battery health record."""
    id: str
    drone_id: str
    battery_id: str
    timestamp: datetime
    cycle_count: int
    health_percent: float
    initial_capacity: float
    current_capacity: float
    internal_resistance: Optional[float] = None
    temperature_profile: Optional[str] = None  # JSON string
    estimated_remaining_cycles: Optional[int] = None
    recommendation: Optional[str] = None
