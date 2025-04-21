"""
API endpoints for mission execution.

This module provides FastAPI endpoints for executing and controlling missions.
"""

from fastapi import APIRouter, HTTPException, Depends, WebSocket, WebSocketDisconnect, BackgroundTasks
from pydantic import BaseModel, Field
from typing import Dict, List, Any, Optional, Union
from datetime import datetime
import json
import asyncio

from backend.mission_execution.execution_service import MissionExecutionService, ExecutionCommand
from backend.mission_planning.models import MissionExecutionStatus
from backend.api.dependencies import get_current_user

router = APIRouter(prefix="/mission-execution", tags=["Mission Execution"])

# Pydantic models
class ExecuteMissionRequest(BaseModel):
    """Request model for executing a mission."""
    mission_id: str
    drone_id: str


class ExecutionControlRequest(BaseModel):
    """Request model for controlling a mission execution."""
    command: str
    params: Optional[Dict[str, Any]] = None


class ExecutionStatusResponse(BaseModel):
    """Response model for execution status."""
    execution_id: str
    mission_id: str
    drone_id: str
    status: str
    current_waypoint: int
    total_waypoints: int
    paused: bool
    telemetry: Optional[Dict[str, Any]] = None
    start_time: Optional[str] = None
    end_time: Optional[str] = None
    elapsed_time: Optional[float] = None


# Global service instance
execution_service = None


# WebSocket connections
active_connections: Dict[str, List[WebSocket]] = {}


# Initialize service
def initialize_service(service: MissionExecutionService):
    """Initialize the execution service."""
    global execution_service
    execution_service = service


# API endpoints
@router.post("/execute", response_model=ExecutionStatusResponse)
async def execute_mission(
    request: ExecuteMissionRequest,
    current_user: str = Depends(get_current_user)
):
    """Execute a mission."""
    if not execution_service:
        raise HTTPException(status_code=500, detail="Execution service not initialized")
    
    execution = await execution_service.execute_mission(request.mission_id, request.drone_id)
    if not execution:
        raise HTTPException(status_code=400, detail="Failed to execute mission")
    
    status = await execution_service.get_execution_status(execution.id)
    return ExecutionStatusResponse(**status)


@router.post("/{execution_id}/control", response_model=ExecutionStatusResponse)
async def control_execution(
    execution_id: str,
    request: ExecutionControlRequest,
    current_user: str = Depends(get_current_user)
):
    """Control a mission execution."""
    if not execution_service:
        raise HTTPException(status_code=500, detail="Execution service not initialized")
    
    try:
        command = ExecutionCommand(request.command)
    except ValueError:
        raise HTTPException(status_code=400, detail=f"Invalid command: {request.command}")
    
    success = await execution_service.control_execution(execution_id, command, request.params)
    if not success:
        raise HTTPException(status_code=400, detail=f"Failed to execute command: {command}")
    
    status = await execution_service.get_execution_status(execution_id)
    if not status:
        raise HTTPException(status_code=404, detail="Execution not found")
    
    return ExecutionStatusResponse(**status)


@router.get("/{execution_id}/status", response_model=ExecutionStatusResponse)
async def get_execution_status(
    execution_id: str,
    current_user: str = Depends(get_current_user)
):
    """Get the status of a mission execution."""
    if not execution_service:
        raise HTTPException(status_code=500, detail="Execution service not initialized")
    
    status = await execution_service.get_execution_status(execution_id)
    if not status:
        raise HTTPException(status_code=404, detail="Execution not found")
    
    return ExecutionStatusResponse(**status)


@router.get("/active", response_model=List[ExecutionStatusResponse])
async def get_active_executions(
    current_user: str = Depends(get_current_user)
):
    """Get all active mission executions."""
    if not execution_service:
        raise HTTPException(status_code=500, detail="Execution service not initialized")
    
    active_executions = await execution_service.get_active_executions()
    return [ExecutionStatusResponse(**status) for status in active_executions]


@router.websocket("/{execution_id}/telemetry")
async def websocket_telemetry(websocket: WebSocket, execution_id: str):
    """WebSocket endpoint for telemetry updates."""
    if not execution_service:
        await websocket.close(code=1011, reason="Execution service not initialized")
        return
    
    await websocket.accept()
    
    # Add to active connections
    if execution_id not in active_connections:
        active_connections[execution_id] = []
    active_connections[execution_id].append(websocket)
    
    # Create telemetry queue
    telemetry_queue = asyncio.Queue()
    
    # Subscribe to telemetry
    def telemetry_callback(telemetry):
        asyncio.create_task(telemetry_queue.put(telemetry))
    
    subscription_success = execution_service.subscribe_to_telemetry(execution_id, telemetry_callback)
    if not subscription_success:
        await websocket.close(code=1011, reason=f"Execution {execution_id} not found or not active")
        return
    
    try:
        # Send initial status
        status = await execution_service.get_execution_status(execution_id)
        if status:
            await websocket.send_text(json.dumps({"type": "status", "data": status}))
        
        # Handle incoming messages (commands)
        command_task = asyncio.create_task(handle_commands(websocket, execution_id))
        
        # Send telemetry updates
        while True:
            telemetry = await telemetry_queue.get()
            await websocket.send_text(json.dumps({"type": "telemetry", "data": telemetry}))
    
    except WebSocketDisconnect:
        # Remove from active connections
        if execution_id in active_connections and websocket in active_connections[execution_id]:
            active_connections[execution_id].remove(websocket)
        
        # Unsubscribe from telemetry
        execution_service.unsubscribe_from_telemetry(execution_id, telemetry_callback)
        
        # Cancel command task
        if command_task and not command_task.done():
            command_task.cancel()
    
    except Exception as e:
        # Handle other exceptions
        print(f"WebSocket error: {str(e)}")
        
        # Remove from active connections
        if execution_id in active_connections and websocket in active_connections[execution_id]:
            active_connections[execution_id].remove(websocket)
        
        # Unsubscribe from telemetry
        execution_service.unsubscribe_from_telemetry(execution_id, telemetry_callback)
        
        # Cancel command task
        if command_task and not command_task.done():
            command_task.cancel()


async def handle_commands(websocket: WebSocket, execution_id: str):
    """Handle incoming WebSocket commands."""
    try:
        while True:
            message = await websocket.receive_text()
            data = json.loads(message)
            
            if "command" in data:
                try:
                    command = ExecutionCommand(data["command"])
                    params = data.get("params")
                    
                    success = await execution_service.control_execution(execution_id, command, params)
                    
                    # Send response
                    await websocket.send_text(json.dumps({
                        "type": "command_response",
                        "command": data["command"],
                        "success": success
                    }))
                    
                    # Send updated status
                    status = await execution_service.get_execution_status(execution_id)
                    if status:
                        await websocket.send_text(json.dumps({"type": "status", "data": status}))
                
                except ValueError:
                    await websocket.send_text(json.dumps({
                        "type": "command_response",
                        "command": data["command"],
                        "success": False,
                        "error": f"Invalid command: {data['command']}"
                    }))
    
    except WebSocketDisconnect:
        pass
    
    except Exception as e:
        print(f"Error handling commands: {str(e)}")


@router.post("/start-service")
async def start_service(
    background_tasks: BackgroundTasks,
    current_user: str = Depends(get_current_user)
):
    """Start the execution service."""
    if not execution_service:
        raise HTTPException(status_code=500, detail="Execution service not initialized")
    
    background_tasks.add_task(execution_service.start)
    return {"message": "Execution service starting"}


@router.post("/stop-service")
async def stop_service(
    background_tasks: BackgroundTasks,
    current_user: str = Depends(get_current_user)
):
    """Stop the execution service."""
    if not execution_service:
        raise HTTPException(status_code=500, detail="Execution service not initialized")
    
    background_tasks.add_task(execution_service.stop)
    return {"message": "Execution service stopping"}
