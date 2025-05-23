"""
Remote ID endpoints for the Remote ID & Regulatory Compliance Service.

This module provides API endpoints for Remote ID broadcasting and logging.
"""

from datetime import datetime
from typing import Dict, List, Optional, Any
from uuid import UUID

from fastapi import APIRouter, Depends, HTTPException, status, Request, Security
from sqlalchemy.ext.asyncio import AsyncSession

from remoteid_service.api.schemas.remoteid import (
    StartBroadcastRequest,
    StopBroadcastRequest,
    UpdateBroadcastRequest,
    GetBroadcastLogsRequest,
    BroadcastStatus,
    BroadcastLog,
    BroadcastLogsResponse,
    ASTMMessage,
)
from remoteid_service.core.security import User, get_current_user
from remoteid_service.core.settings import Settings, get_settings
from remoteid_service.db.session import get_db_session
from remoteid_service.services.remoteid_broadcast import RemoteIDBroadcastService
from remoteid_service.services.remoteid_logging import RemoteIDLoggingService

router = APIRouter()

# Helper functions
async def get_broadcast_service(request: Request) -> RemoteIDBroadcastService:
    """
    Get the Remote ID broadcast service.
    
    Args:
        request: FastAPI request
        
    Returns:
        RemoteIDBroadcastService: Broadcast service
        
    Raises:
        HTTPException: If the service is not available
    """
    if not hasattr(request.app.state, "broadcast_service"):
        raise HTTPException(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail="Remote ID broadcasting service is not available",
        )
    
    return request.app.state.broadcast_service

async def get_logging_service(request: Request) -> RemoteIDLoggingService:
    """
    Get the Remote ID logging service.
    
    Args:
        request: FastAPI request
        
    Returns:
        RemoteIDLoggingService: Logging service
        
    Raises:
        HTTPException: If the service is not available
    """
    if not hasattr(request.app.state, "logging_service"):
        raise HTTPException(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail="Remote ID logging service is not available",
        )
    
    return request.app.state.logging_service

# Endpoints
@router.post("/broadcast/start", response_model=BroadcastStatus)
async def start_broadcast(
    request: StartBroadcastRequest,
    fastapi_request: Request,
    current_user: User = Security(get_current_user, scopes=["remoteid:write"]),
    broadcast_service: RemoteIDBroadcastService = Depends(get_broadcast_service),
    db: AsyncSession = Depends(get_db_session),
) -> BroadcastStatus:
    """
    Start Remote ID broadcasting for a drone.
    
    Args:
        request: Start broadcast request
        fastapi_request: FastAPI request
        current_user: Current user
        broadcast_service: Remote ID broadcast service
        db: Database session
        
    Returns:
        BroadcastStatus: Broadcast status
    """
    try:
        # Start broadcasting
        status = await broadcast_service.start_broadcast(
            drone_id=request.drone_id,
            mode=request.mode,
            methods=request.methods,
            operator_id=request.operator_id,
            serial_number=request.serial_number,
            session_id=request.session_id,
            initial_position=request.initial_position,
            initial_velocity=request.initial_velocity,
            metadata=request.metadata,
            db=db,
        )
        
        return status
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error starting Remote ID broadcast: {str(e)}",
        )

@router.post("/broadcast/stop", response_model=BroadcastStatus)
async def stop_broadcast(
    request: StopBroadcastRequest,
    fastapi_request: Request,
    current_user: User = Security(get_current_user, scopes=["remoteid:write"]),
    broadcast_service: RemoteIDBroadcastService = Depends(get_broadcast_service),
    db: AsyncSession = Depends(get_db_session),
) -> BroadcastStatus:
    """
    Stop Remote ID broadcasting for a drone.
    
    Args:
        request: Stop broadcast request
        fastapi_request: FastAPI request
        current_user: Current user
        broadcast_service: Remote ID broadcast service
        db: Database session
        
    Returns:
        BroadcastStatus: Broadcast status
    """
    try:
        # Stop broadcasting
        status = await broadcast_service.stop_broadcast(
            drone_id=request.drone_id,
            session_id=request.session_id,
            db=db,
        )
        
        return status
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error stopping Remote ID broadcast: {str(e)}",
        )

@router.post("/broadcast/update", response_model=BroadcastStatus)
async def update_broadcast(
    request: UpdateBroadcastRequest,
    fastapi_request: Request,
    current_user: User = Security(get_current_user, scopes=["remoteid:write"]),
    broadcast_service: RemoteIDBroadcastService = Depends(get_broadcast_service),
    db: AsyncSession = Depends(get_db_session),
) -> BroadcastStatus:
    """
    Update Remote ID broadcast data for a drone.
    
    Args:
        request: Update broadcast request
        fastapi_request: FastAPI request
        current_user: Current user
        broadcast_service: Remote ID broadcast service
        db: Database session
        
    Returns:
        BroadcastStatus: Broadcast status
    """
    try:
        # Update broadcast
        status = await broadcast_service.update_broadcast(
            drone_id=request.drone_id,
            position=request.position,
            velocity=request.velocity,
            session_id=request.session_id,
            timestamp=request.timestamp or datetime.utcnow(),
            db=db,
        )
        
        return status
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error updating Remote ID broadcast: {str(e)}",
        )

@router.get("/broadcast/status/{drone_id}", response_model=BroadcastStatus)
async def get_broadcast_status(
    drone_id: str,
    fastapi_request: Request,
    current_user: User = Security(get_current_user, scopes=["remoteid:read"]),
    broadcast_service: RemoteIDBroadcastService = Depends(get_broadcast_service),
) -> BroadcastStatus:
    """
    Get Remote ID broadcast status for a drone.
    
    Args:
        drone_id: Drone ID
        fastapi_request: FastAPI request
        current_user: Current user
        broadcast_service: Remote ID broadcast service
        
    Returns:
        BroadcastStatus: Broadcast status
    """
    try:
        # Get broadcast status
        status = await broadcast_service.get_broadcast_status(drone_id)
        
        return status
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error getting Remote ID broadcast status: {str(e)}",
        )

@router.post("/logs", response_model=BroadcastLogsResponse)
async def get_broadcast_logs(
    request: GetBroadcastLogsRequest,
    fastapi_request: Request,
    current_user: User = Security(get_current_user, scopes=["remoteid:read"]),
    logging_service: RemoteIDLoggingService = Depends(get_logging_service),
    db: AsyncSession = Depends(get_db_session),
) -> BroadcastLogsResponse:
    """
    Get Remote ID broadcast logs.
    
    Args:
        request: Get broadcast logs request
        fastapi_request: FastAPI request
        current_user: Current user
        logging_service: Remote ID logging service
        db: Database session
        
    Returns:
        BroadcastLogsResponse: Broadcast logs
    """
    try:
        # Get broadcast logs
        logs, total = await logging_service.get_logs(
            drone_id=request.drone_id,
            start_time=request.start_time,
            end_time=request.end_time,
            limit=request.limit,
            offset=request.offset,
            db=db,
        )
        
        return BroadcastLogsResponse(
            logs=logs,
            total=total,
            limit=request.limit or 100,
            offset=request.offset or 0,
        )
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error getting Remote ID broadcast logs: {str(e)}",
        )

@router.get("/message/{drone_id}", response_model=ASTMMessage)
async def get_astm_message(
    drone_id: str,
    fastapi_request: Request,
    current_user: User = Security(get_current_user, scopes=["remoteid:read"]),
    broadcast_service: RemoteIDBroadcastService = Depends(get_broadcast_service),
) -> ASTMMessage:
    """
    Get ASTM F3411-22a message for a drone.
    
    Args:
        drone_id: Drone ID
        fastapi_request: FastAPI request
        current_user: Current user
        broadcast_service: Remote ID broadcast service
        
    Returns:
        ASTMMessage: ASTM message
    """
    try:
        # Get ASTM message
        message = await broadcast_service.get_astm_message(drone_id)
        
        return message
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error getting ASTM message: {str(e)}",
        )
