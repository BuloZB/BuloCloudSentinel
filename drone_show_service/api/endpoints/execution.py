"""
API endpoints for execution.

This module provides API endpoints for executing drone show choreographies.
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
from typing import List, Optional
from fastapi import APIRouter, Depends, HTTPException, Query, Path, status, WebSocket, WebSocketDisconnect
from sqlalchemy.ext.asyncio import AsyncSession
import json
import asyncio

from drone_show_service.core.database import get_db
from drone_show_service.models.choreography import ExecutionSettings, ExecutionResponse
from drone_show_service.services.execution_service import ExecutionService
from drone_show_service.services.logging_service import LoggingService
from drone_show_service.api.main import get_execution_service, get_logging_service

logger = logging.getLogger(__name__)

router = APIRouter()


@router.post("/{choreography_id}", response_model=ExecutionResponse)
async def execute_choreography(
    settings: ExecutionSettings,
    choreography_id: str = Path(...),
    db: AsyncSession = Depends(get_db),
    execution_service: ExecutionService = Depends(get_execution_service),
):
    """
    Execute a choreography.

    Args:
        settings: Execution settings
        choreography_id: Choreography ID
        db: Database session
        execution_service: Execution service

    Returns:
        Execution response
    """
    try:
        return await execution_service.execute_choreography(db, choreography_id, settings)
    except ValueError as e:
        logger.error(f"Error executing choreography {choreography_id}: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=str(e),
        )
    except Exception as e:
        logger.error(f"Error executing choreography {choreography_id}: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error executing choreography: {str(e)}",
        )


@router.get("/{execution_id}", response_model=ExecutionResponse)
async def get_execution(
    execution_id: str = Path(...),
    db: AsyncSession = Depends(get_db),
    execution_service: ExecutionService = Depends(get_execution_service),
):
    """
    Get an execution by ID.

    Args:
        execution_id: Execution ID
        db: Database session
        execution_service: Execution service

    Returns:
        Execution
    """
    try:
        execution = await execution_service.get_execution(db, execution_id)
        if not execution:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail=f"Execution {execution_id} not found",
            )
        return execution
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error getting execution {execution_id}: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error getting execution: {str(e)}",
        )


@router.get("/", response_model=List[ExecutionResponse])
async def get_executions(
    choreography_id: Optional[str] = Query(None),
    status: Optional[str] = Query(None),
    skip: int = Query(0, ge=0),
    limit: int = Query(100, ge=1, le=1000),
    db: AsyncSession = Depends(get_db),
    execution_service: ExecutionService = Depends(get_execution_service),
):
    """
    Get all executions.

    Args:
        choreography_id: Filter by choreography ID
        status: Filter by status
        skip: Number of records to skip
        limit: Maximum number of records to return
        db: Database session
        execution_service: Execution service

    Returns:
        List of executions
    """
    try:
        return await execution_service.get_executions(db, choreography_id, status, skip, limit)
    except Exception as e:
        logger.error(f"Error getting executions: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error getting executions: {str(e)}",
        )


@router.post("/{execution_id}/pause", response_model=ExecutionResponse)
async def pause_execution(
    execution_id: str = Path(...),
    db: AsyncSession = Depends(get_db),
    execution_service: ExecutionService = Depends(get_execution_service),
):
    """
    Pause an execution.

    Args:
        execution_id: Execution ID
        db: Database session
        execution_service: Execution service

    Returns:
        Updated execution
    """
    try:
        execution = await execution_service.pause_execution(db, execution_id)
        if not execution:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail=f"Execution {execution_id} not found or cannot be paused",
            )
        return execution
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error pausing execution {execution_id}: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error pausing execution: {str(e)}",
        )


@router.post("/{execution_id}/resume", response_model=ExecutionResponse)
async def resume_execution(
    execution_id: str = Path(...),
    db: AsyncSession = Depends(get_db),
    execution_service: ExecutionService = Depends(get_execution_service),
):
    """
    Resume a paused execution.

    Args:
        execution_id: Execution ID
        db: Database session
        execution_service: Execution service

    Returns:
        Updated execution
    """
    try:
        execution = await execution_service.resume_execution(db, execution_id)
        if not execution:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail=f"Execution {execution_id} not found or cannot be resumed",
            )
        return execution
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error resuming execution {execution_id}: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error resuming execution: {str(e)}",
        )


@router.post("/{execution_id}/abort", response_model=ExecutionResponse)
async def abort_execution(
    execution_id: str = Path(...),
    db: AsyncSession = Depends(get_db),
    execution_service: ExecutionService = Depends(get_execution_service),
):
    """
    Abort an execution.

    Args:
        execution_id: Execution ID
        db: Database session
        execution_service: Execution service

    Returns:
        Updated execution
    """
    try:
        execution = await execution_service.abort_execution(db, execution_id)
        if not execution:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail=f"Execution {execution_id} not found or cannot be aborted",
            )
        return execution
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error aborting execution {execution_id}: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error aborting execution: {str(e)}",
        )


@router.get("/{execution_id}/logs")
async def get_execution_logs(
    execution_id: str = Path(...),
    level: Optional[str] = Query(None),
    skip: int = Query(0, ge=0),
    limit: int = Query(100, ge=1, le=1000),
    db: AsyncSession = Depends(get_db),
    logging_service: LoggingService = Depends(get_logging_service),
):
    """
    Get logs for an execution.

    Args:
        execution_id: Execution ID
        level: Filter by log level
        skip: Number of records to skip
        limit: Maximum number of records to return
        db: Database session
        logging_service: Logging service

    Returns:
        List of log entries
    """
    try:
        return await logging_service.get_logs(db, execution_id, level, skip, limit)
    except Exception as e:
        logger.error(f"Error getting logs for execution {execution_id}: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error getting logs: {str(e)}",
        )


@router.get("/{execution_id}/logs/export")
async def export_execution_logs(
    execution_id: str = Path(...),
    db: AsyncSession = Depends(get_db),
    logging_service: LoggingService = Depends(get_logging_service),
):
    """
    Export logs for an execution.

    Args:
        execution_id: Execution ID
        db: Database session
        logging_service: Logging service

    Returns:
        URL to exported logs
    """
    try:
        url = await logging_service.export_logs(db, execution_id)
        if not url:
            raise HTTPException(
                status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
                detail=f"Error exporting logs for execution {execution_id}",
            )
        return {"url": url}
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error exporting logs for execution {execution_id}: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error exporting logs: {str(e)}",
        )


@router.websocket("/ws/{execution_id}")
async def execution_websocket(
    websocket: WebSocket,
    execution_id: str,
    db: AsyncSession = Depends(get_db),
    execution_service: ExecutionService = Depends(get_execution_service),
):
    """
    WebSocket endpoint for streaming execution data.

    Args:
        websocket: WebSocket connection
        execution_id: Execution ID
        db: Database session
        execution_service: Execution service
    """
    try:
        # Accept connection
        await websocket.accept()

        # Get execution
        execution = await execution_service.get_execution(db, execution_id)
        if not execution:
            await websocket.close(code=1000, reason=f"Execution {execution_id} not found")
            return

        # Send initial data
        await websocket.send_json({
            "type": "init",
            "data": execution.dict(),
        })

        # Stream updates
        while True:
            # Get latest execution
            execution = await execution_service.get_execution(db, execution_id)
            if not execution:
                break

            # Send update
            await websocket.send_json({
                "type": "update",
                "data": execution.dict(),
            })

            # Check if execution is completed, aborted, or failed
            if execution.status in ["completed", "aborted", "failed"]:
                break

            # Wait for next update
            await asyncio.sleep(1.0)

        # Send end message
        await websocket.send_json({
            "type": "end",
            "data": {
                "id": execution_id,
                "status": execution.status if execution else "unknown",
            }
        })
    except WebSocketDisconnect:
        logger.info(f"Client disconnected from execution {execution_id}")
    except Exception as e:
        logger.error(f"Error in execution WebSocket {execution_id}: {str(e)}")
        try:
            await websocket.close(code=1011, reason=str(e))
        except Exception as close_error:
            logger.error(f"Error closing WebSocket: {str(close_error)}")
