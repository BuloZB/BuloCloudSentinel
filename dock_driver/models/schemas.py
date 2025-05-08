"""
Data Models for Dock Driver Microservice

This module defines the data models for the Dock Driver microservice.
"""

from pydantic import BaseModel, Field
from typing import Dict, Any, List, Optional
from enum import Enum
from datetime import datetime


class DockStatus(str, Enum):
    """Dock status enum."""
    UNKNOWN = "unknown"
    ONLINE = "online"
    OFFLINE = "offline"
    OPEN = "open"
    CLOSED = "closed"
    ERROR = "error"


class ChargingStatus(str, Enum):
    """Charging status enum."""
    UNKNOWN = "unknown"
    IDLE = "idle"
    CHARGING = "charging"
    COMPLETE = "complete"
    ERROR = "error"


class DockType(str, Enum):
    """Dock type enum."""
    DJI = "dji"
    HEISHA = "heisha"
    ESP32 = "esp32"


class DockItem(BaseModel):
    """Dock item model."""
    dock_id: str = Field(..., description="ID of the dock")
    dock_type: DockType = Field(..., description="Type of the dock")
    name: str = Field(..., description="Name of the dock")
    location: Optional[Dict[str, float]] = Field(None, description="Location of the dock (latitude, longitude)")
    status: DockStatus = Field(DockStatus.UNKNOWN, description="Status of the dock")
    charging_status: ChargingStatus = Field(ChargingStatus.UNKNOWN, description="Charging status of the dock")
    last_updated: Optional[datetime] = Field(None, description="Last update time")


class DockListResponse(BaseModel):
    """Response model for listing docks."""
    docks: List[DockItem] = Field(..., description="List of docks")


class DockStatusResponse(BaseModel):
    """Response model for dock status."""
    dock_id: str = Field(..., description="ID of the dock")
    status: DockStatus = Field(..., description="Status of the dock")
    charging_status: ChargingStatus = Field(..., description="Charging status of the dock")
    door_state: Optional[str] = Field(None, description="State of the dock door")
    drone_connected: Optional[bool] = Field(None, description="Whether a drone is connected to the dock")
    error_code: Optional[int] = Field(None, description="Error code (if any)")
    error_message: Optional[str] = Field(None, description="Error message (if any)")
    timestamp: str = Field(..., description="Timestamp of the status")


class DockTelemetryResponse(BaseModel):
    """Response model for dock telemetry."""
    dock_id: str = Field(..., description="ID of the dock")
    temperature: Optional[float] = Field(None, description="Temperature (°C)")
    humidity: Optional[float] = Field(None, description="Humidity (%)")
    charging_voltage: Optional[float] = Field(None, description="Charging voltage (V)")
    charging_current: Optional[float] = Field(None, description="Charging current (A)")
    battery_level: Optional[int] = Field(None, description="Battery level (%)")
    network_signal: Optional[int] = Field(None, description="Network signal strength (%)")
    fan_speed: Optional[int] = Field(None, description="Fan speed (%)")
    relay_state: Optional[bool] = Field(None, description="Relay state (on/off)")
    timestamp: str = Field(..., description="Timestamp of the telemetry")


class ErrorResponse(BaseModel):
    """Error response model."""
    detail: str = Field(..., description="Error message")


class SuccessResponse(BaseModel):
    """Success response model."""
    message: str = Field(..., description="Success message")


class TokenResponse(BaseModel):
    """Token response model."""
    access_token: str = Field(..., description="JWT access token")
    token_type: str = Field("bearer", description="Token type")
    expires_in: int = Field(..., description="Token expiration time in seconds")


class UserCredentials(BaseModel):
    """User credentials model."""
    username: str = Field(..., description="Username")
    password: str = Field(..., description="Password")


class UserInfo(BaseModel):
    """User information model."""
    username: str = Field(..., description="Username")
    email: Optional[str] = Field(None, description="Email address")
    full_name: Optional[str] = Field(None, description="Full name")
    disabled: Optional[bool] = Field(None, description="Whether the user is disabled")
    roles: List[str] = Field([], description="User roles")
