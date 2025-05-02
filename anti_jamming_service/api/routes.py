"""
API routes for the Anti-Jamming Service.

This module defines FastAPI routes for the Anti-Jamming Service.
"""

import logging

from security.validation.unified_validation import (
    validate_email,
    validate_username,
    validate_name,
    validate_uuid,
    validate_url,
    sanitize_string,
    sanitize_html,
    check_sql_injection,
    input_validator,
    form_validator,
    request_validator,
)
import time
import asyncio
from typing import Dict, List, Optional, Any, Union
from fastapi import APIRouter, Depends, HTTPException, Request, Response, status
from fastapi.security import APIKeyHeader
from starlette.middleware.base import BaseHTTPMiddleware
from starlette.responses import JSONResponse

from anti_jamming_service import __version__
from anti_jamming_service.api.schemas import (
    StatusResponse, HardwareStatusResponse, ProcessingStatusResponse,
    JammingDetectionResponse, DoAEstimationResponse, HardwareConfigRequest,
    ProcessingConfigRequest, MessageRequest, MessageResponse, ErrorResponse,
    HardwareType, ProcessingType
)
from anti_jamming_service.utils.config import get_config
from anti_jamming_service.utils.vault import get_secret

logger = logging.getLogger(__name__)

# Create router
router = APIRouter()

# API key header
API_KEY_HEADER = APIKeyHeader(name="X-API-Key")

# Service start time
START_TIME = time.time()


# Rate limiting middleware
class RateLimitingMiddleware(BaseHTTPMiddleware):
    """
    Rate limiting middleware.
    
    This middleware implements rate limiting for API endpoints.
    """
    
    def __init__(self, app, rate_limit: str = "60/minute"):
        """
        Initialize the middleware.
        
        Args:
            app: FastAPI application.
            rate_limit: Rate limit string (e.g., "60/minute").
        """
        super().__init__(app)
        self.rate_limit = rate_limit
        self.clients = {}
    
    async def dispatch(self, request: Request, call_next):
        """
        Dispatch the request.
        
        Args:
            request: FastAPI request.
            call_next: Next middleware.
            
        Returns:
            Response: FastAPI response.
        """
        # Get client IP
        client_ip = request.client.host
        
        # Check if rate limited
        if self._is_rate_limited(client_ip):
            return JSONResponse(
                status_code=status.HTTP_429_TOO_MANY_REQUESTS,
                content={"error": "Too many requests", "detail": "Rate limit exceeded"}
            )
        
        # Process request
        response = await call_next(request)
        return response
    
    def _is_rate_limited(self, client_ip: str) -> bool:
        """
        Check if client is rate limited.
        
        Args:
            client_ip: Client IP address.
            
        Returns:
            bool: True if rate limited, False otherwise.
        """
        # Parse rate limit
        limit, period = self._parse_rate_limit()
        
        # Get current time
        now = time.time()
        
        # Initialize client if not exists
        if client_ip not in self.clients:
            self.clients[client_ip] = {"count": 0, "reset_time": now + period}
        
        # Reset count if period expired
        if now > self.clients[client_ip]["reset_time"]:
            self.clients[client_ip] = {"count": 0, "reset_time": now + period}
        
        # Increment count
        self.clients[client_ip]["count"] += 1
        
        # Check if rate limited
        return self.clients[client_ip]["count"] > limit
    
    def _parse_rate_limit(self) -> tuple:
        """
        Parse rate limit string.
        
        Returns:
            tuple: (limit, period in seconds)
        """
        parts = self.rate_limit.split("/")
        limit = int(parts[0])
        
        if parts[1] == "second":
            period = 1
        elif parts[1] == "minute":
            period = 60
        elif parts[1] == "hour":
            period = 3600
        elif parts[1] == "day":
            period = 86400
        else:
            period = 60  # Default to minute
        
        return limit, period


# Authentication dependency
async def authenticate(api_key: str = Depends(API_KEY_HEADER)) -> bool:
    """
    Authenticate API request.
    
    Args:
        api_key: API key from header.
        
    Returns:
        bool: True if authenticated, False otherwise.
        
    Raises:
        HTTPException: If authentication fails.
    """
    # Get expected API key
    expected_api_key = get_secret("api_key")
    
    # If no API key is configured, allow all requests
    if not expected_api_key:
        return True
    
    # Check API key
    if api_key != expected_api_key:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid API key",
            headers={"WWW-Authenticate": "ApiKey"},
        )
    
    return True


# Routes

@router.get("/status", response_model=StatusResponse, tags=["status"])
async def get_status(request: Request, authenticated: bool = Depends(authenticate)):
    """
    Get service status.
    
    Returns:
        StatusResponse: Service status.
    """
    # Get hardware status
    hardware_status = {}
    for hardware in request.app.state.hardware.values():
        hardware_status[hardware.__class__.__name__] = await hardware.get_status()
    
    # Get processing status
    processing_status = {}
    for processor in request.app.state.processors.values():
        processing_status[processor.__class__.__name__] = await processor.get_status()
    
    return {
        "status": "running",
        "version": __version__,
        "uptime": time.time() - START_TIME,
        "hardware": hardware_status,
        "processing": processing_status
    }


@router.get("/hardware", response_model=List[HardwareStatusResponse], tags=["hardware"])
async def get_hardware_status(request: Request, authenticated: bool = Depends(authenticate)):
    """
    Get hardware status.
    
    Returns:
        List[HardwareStatusResponse]: Hardware status.
    """
    result = []
    
    for hardware_type, hardware in request.app.state.hardware.items():
        status = await hardware.get_status()
        result.append({
            "type": hardware_type,
            "initialized": hardware.initialized,
            "status": status
        })
    
    return result


@router.post("/hardware/configure", response_model=Dict[str, Any], tags=["hardware"])
async def configure_hardware(
    request: Request,
    config_request: HardwareConfigRequest,
    authenticated: bool = Depends(authenticate)
):
    """
    Configure hardware.
    
    Args:
        config_request: Hardware configuration request.
        
    Returns:
        Dict[str, Any]: Configuration result.
    """
    hardware_type = config_request.type.value
    
    # Check if hardware exists
    if hardware_type not in request.app.state.hardware:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Hardware {hardware_type} not found"
        )
    
    # Get hardware
    hardware = request.app.state.hardware[hardware_type]
    
    # Configure hardware
    success = await hardware.configure(config_request.config)
    
    if not success:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Failed to configure hardware {hardware_type}"
        )
    
    # Get updated status
    status_data = await hardware.get_status()
    
    return {
        "success": True,
        "type": hardware_type,
        "status": status_data
    }


@router.get("/processing", response_model=List[ProcessingStatusResponse], tags=["processing"])
async def get_processing_status(request: Request, authenticated: bool = Depends(authenticate)):
    """
    Get processing status.
    
    Returns:
        List[ProcessingStatusResponse]: Processing status.
    """
    result = []
    
    for processing_type, processor in request.app.state.processors.items():
        status = await processor.get_status()
        result.append({
            "type": processing_type,
            "enabled": status.get("enabled", False),
            "status": status
        })
    
    return result


@router.post("/processing/configure", response_model=Dict[str, Any], tags=["processing"])
async def configure_processing(
    request: Request,
    config_request: ProcessingConfigRequest,
    authenticated: bool = Depends(authenticate)
):
    """
    Configure processing.
    
    Args:
        config_request: Processing configuration request.
        
    Returns:
        Dict[str, Any]: Configuration result.
    """
    processing_type = config_request.type.value
    
    # Check if processor exists
    if processing_type not in request.app.state.processors:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Processor {processing_type} not found"
        )
    
    # Get processor
    processor = request.app.state.processors[processing_type]
    
    # Configure processor
    success = await processor.configure(config_request.config)
    
    if not success:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Failed to configure processor {processing_type}"
        )
    
    # Get updated status
    status_data = await processor.get_status()
    
    return {
        "success": True,
        "type": processing_type,
        "status": status_data
    }


@router.get("/jamming/detect", response_model=JammingDetectionResponse, tags=["jamming"])
async def detect_jamming(request: Request, authenticated: bool = Depends(authenticate)):
    """
    Detect jamming.
    
    Returns:
        JammingDetectionResponse: Jamming detection result.
    """
    # Check if jamming detector exists
    if "jamming_detection" not in request.app.state.processors:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="Jamming detector not found"
        )
    
    # Get jamming detector
    detector = request.app.state.processors["jamming_detection"]
    
    # Get detection result
    result = detector.last_detection
    
    return {
        "is_jamming": result["is_jamming"],
        "jamming_type": result["jamming_type"].name,
        "confidence": result["confidence"],
        "snr_db": result["snr_db"],
        "timestamp": result["timestamp"]
    }


@router.get("/jamming/doa", response_model=DoAEstimationResponse, tags=["jamming"])
async def estimate_doa(request: Request, authenticated: bool = Depends(authenticate)):
    """
    Estimate direction of arrival.
    
    Returns:
        DoAEstimationResponse: DoA estimation result.
    """
    # Check if DoA estimator exists
    if "doa_estimation" not in request.app.state.processors:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="DoA estimator not found"
        )
    
    # Get DoA estimator
    estimator = request.app.state.processors["doa_estimation"]
    
    # Get estimation result
    result = await estimator.estimate()
    
    return {
        "azimuth": result["azimuth"],
        "elevation": result["elevation"],
        "confidence": result["confidence"],
        "timestamp": time.time()
    }


@router.post("/fhss/send", response_model=MessageResponse, tags=["fhss"])
async def send_message(
    request: Request,
    message_request: MessageRequest,
    authenticated: bool = Depends(authenticate)
):
    """
    Send a message using FHSS.
    
    Args:
        message_request: Message request.
        
    Returns:
        MessageResponse: Message response.
    """
    # Check if FHSS processor exists
    if "fhss" not in request.app.state.processors:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="FHSS processor not found"
        )
    
    # Get FHSS processor
    fhss = request.app.state.processors["fhss"]
    
    # Send message
    success = await fhss.send_message(message_request.message.encode())
    
    if not success:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to send message"
        )
    
    return {
        "success": True,
        "timestamp": time.time()
    }


@router.get("/fhss/receive", response_model=Optional[MessageResponse], tags=["fhss"])
async def receive_message(
    request: Request,
    timeout: float = 1.0,
    authenticated: bool = Depends(authenticate)
):
    """
    Receive a message using FHSS.
    
    Args:
        timeout: Timeout in seconds.
        
    Returns:
        Optional[MessageResponse]: Message response, or None if no message received.
    """
    # Check if FHSS processor exists
    if "fhss" not in request.app.state.processors:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="FHSS processor not found"
        )
    
    # Get FHSS processor
    fhss = request.app.state.processors["fhss"]
    
    # Receive message
    message = await fhss.receive_message(timeout)
    
    if not message:
        return None
    
    return {
        "success": True,
        "message": message.decode(),
        "timestamp": time.time()
    }
