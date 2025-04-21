"""
API schemas for the Anti-Jamming Service.

This module defines Pydantic models for API requests and responses.
"""

from pydantic import BaseModel, Field, validator
from typing import Dict, List, Optional, Any, Union
from enum import Enum
import time


class StatusResponse(BaseModel):
    """Status response model."""
    status: str = Field(..., description="Service status")
    version: str = Field(..., description="Service version")
    uptime: float = Field(..., description="Service uptime in seconds")
    hardware: Dict[str, Any] = Field(..., description="Hardware status")
    processing: Dict[str, Any] = Field(..., description="Processing status")


class HardwareType(str, Enum):
    """Hardware type enumeration."""
    KRAKEN_SDR = "kraken_sdr"
    HACKRF = "hackrf"
    LORA_SX127X = "lora_sx127x"


class HardwareStatusResponse(BaseModel):
    """Hardware status response model."""
    type: HardwareType = Field(..., description="Hardware type")
    initialized: bool = Field(..., description="Whether the hardware is initialized")
    status: Dict[str, Any] = Field(..., description="Hardware-specific status")


class ProcessingType(str, Enum):
    """Processing type enumeration."""
    GNSS_MITIGATION = "gnss_mitigation"
    DOA_ESTIMATION = "doa_estimation"
    JAMMING_DETECTION = "jamming_detection"
    FHSS = "fhss"


class ProcessingStatusResponse(BaseModel):
    """Processing status response model."""
    type: ProcessingType = Field(..., description="Processing type")
    enabled: bool = Field(..., description="Whether the processing is enabled")
    status: Dict[str, Any] = Field(..., description="Processing-specific status")


class JammingDetectionResponse(BaseModel):
    """Jamming detection response model."""
    is_jamming: bool = Field(..., description="Whether jamming is detected")
    jamming_type: str = Field(..., description="Type of jamming detected")
    confidence: float = Field(..., description="Confidence level (0-1)")
    snr_db: float = Field(..., description="Signal-to-noise ratio in dB")
    timestamp: float = Field(..., description="Timestamp of detection")


class DoAEstimationResponse(BaseModel):
    """Direction of Arrival estimation response model."""
    azimuth: float = Field(..., description="Azimuth angle in degrees")
    elevation: float = Field(..., description="Elevation angle in degrees")
    confidence: float = Field(..., description="Confidence level (0-1)")
    timestamp: float = Field(default_factory=time.time, description="Timestamp of estimation")


class HardwareConfigRequest(BaseModel):
    """Hardware configuration request model."""
    type: HardwareType = Field(..., description="Hardware type")
    config: Dict[str, Any] = Field(..., description="Hardware configuration")
    
    @validator('config')
    def validate_config(cls, v, values):
        """Validate hardware configuration."""
        hardware_type = values.get('type')
        
        if hardware_type == HardwareType.KRAKEN_SDR:
            # Validate KrakenSDR configuration
            if 'center_frequency' in v and (not isinstance(v['center_frequency'], int) or v['center_frequency'] < 0):
                raise ValueError("Invalid center frequency")
            if 'sample_rate' in v and (not isinstance(v['sample_rate'], int) or v['sample_rate'] < 0):
                raise ValueError("Invalid sample rate")
            if 'gain' in v and (not isinstance(v['gain'], (int, float)) or v['gain'] < 0):
                raise ValueError("Invalid gain")
        
        elif hardware_type == HardwareType.HACKRF:
            # Validate HackRF configuration
            if 'center_frequency' in v and (not isinstance(v['center_frequency'], int) or v['center_frequency'] < 0):
                raise ValueError("Invalid center frequency")
            if 'tx_frequency' in v and (not isinstance(v['tx_frequency'], int) or v['tx_frequency'] < 0):
                raise ValueError("Invalid TX frequency")
            if 'sample_rate' in v and (not isinstance(v['sample_rate'], int) or v['sample_rate'] < 0):
                raise ValueError("Invalid sample rate")
        
        elif hardware_type == HardwareType.LORA_SX127X:
            # Validate LoRa SX127x configuration
            if 'frequency' in v and (not isinstance(v['frequency'], int) or v['frequency'] < 0):
                raise ValueError("Invalid frequency")
            if 'spreading_factor' in v and (not isinstance(v['spreading_factor'], int) or v['spreading_factor'] < 7 or v['spreading_factor'] > 12):
                raise ValueError("Invalid spreading factor (must be 7-12)")
            if 'bandwidth' in v and (not isinstance(v['bandwidth'], int) or v['bandwidth'] < 0):
                raise ValueError("Invalid bandwidth")
        
        return v


class ProcessingConfigRequest(BaseModel):
    """Processing configuration request model."""
    type: ProcessingType = Field(..., description="Processing type")
    config: Dict[str, Any] = Field(..., description="Processing configuration")
    
    @validator('config')
    def validate_config(cls, v, values):
        """Validate processing configuration."""
        processing_type = values.get('type')
        
        if processing_type == ProcessingType.GNSS_MITIGATION:
            # Validate GNSS mitigation configuration
            if 'enabled' in v and not isinstance(v['enabled'], bool):
                raise ValueError("Invalid enabled flag")
        
        elif processing_type == ProcessingType.DOA_ESTIMATION:
            # Validate DoA estimation configuration
            if 'enabled' in v and not isinstance(v['enabled'], bool):
                raise ValueError("Invalid enabled flag")
            if 'num_sources' in v and (not isinstance(v['num_sources'], int) or v['num_sources'] < 1):
                raise ValueError("Invalid number of sources")
        
        elif processing_type == ProcessingType.JAMMING_DETECTION:
            # Validate jamming detection configuration
            if 'enabled' in v and not isinstance(v['enabled'], bool):
                raise ValueError("Invalid enabled flag")
            if 'sample_size' in v and (not isinstance(v['sample_size'], int) or v['sample_size'] < 1):
                raise ValueError("Invalid sample size")
        
        elif processing_type == ProcessingType.FHSS:
            # Validate FHSS configuration
            if 'enabled' in v and not isinstance(v['enabled'], bool):
                raise ValueError("Invalid enabled flag")
            if 'num_channels' in v and (not isinstance(v['num_channels'], int) or v['num_channels'] < 1):
                raise ValueError("Invalid number of channels")
            if 'hop_interval' in v and (not isinstance(v['hop_interval'], (int, float)) or v['hop_interval'] <= 0):
                raise ValueError("Invalid hop interval")
        
        return v


class MessageRequest(BaseModel):
    """Message request model."""
    message: str = Field(..., description="Message to send")
    
    @validator('message')
    def validate_message(cls, v):
        """Validate message."""
        if not v:
            raise ValueError("Message cannot be empty")
        return v


class MessageResponse(BaseModel):
    """Message response model."""
    success: bool = Field(..., description="Whether the message was sent successfully")
    timestamp: float = Field(default_factory=time.time, description="Timestamp of message")


class ErrorResponse(BaseModel):
    """Error response model."""
    error: str = Field(..., description="Error message")
    detail: Optional[str] = Field(None, description="Detailed error information")
