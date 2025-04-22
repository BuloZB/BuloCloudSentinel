"""
API endpoints for simulation.

This module provides API endpoints for simulating drone show choreographies.
"""

import logging
from typing import List, Optional
from fastapi import APIRouter, Depends, HTTPException, Query, Path, status, WebSocket, WebSocketDisconnect
from sqlalchemy.ext.asyncio import AsyncSession
import json

from drone_show_service.core.database import get_db
from drone_show_service.models.choreography import SimulationSettings, SimulationResponse
from drone_show_service.services.simulation_service import SimulationService
from drone_show_service.api.main import get_simulation_service

logger = logging.getLogger(__name__)

router = APIRouter()


@router.post("/{choreography_id}", response_model=SimulationResponse)
async def simulate_choreography(
    settings: SimulationSettings,
    choreography_id: str = Path(...),
    db: AsyncSession = Depends(get_db),
    simulation_service: SimulationService = Depends(get_simulation_service),
):
    """
    Simulate a choreography.
    
    Args:
        settings: Simulation settings
        choreography_id: Choreography ID
        db: Database session
        simulation_service: Simulation service
        
    Returns:
        Simulation response
    """
    try:
        return await simulation_service.simulate_choreography(db, choreography_id, settings)
    except ValueError as e:
        logger.error(f"Error simulating choreography {choreography_id}: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=str(e),
        )
    except Exception as e:
        logger.error(f"Error simulating choreography {choreography_id}: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error simulating choreography: {str(e)}",
        )


@router.get("/{simulation_id}", response_model=SimulationResponse)
async def get_simulation(
    simulation_id: str = Path(...),
    db: AsyncSession = Depends(get_db),
    simulation_service: SimulationService = Depends(get_simulation_service),
):
    """
    Get a simulation by ID.
    
    Args:
        simulation_id: Simulation ID
        db: Database session
        simulation_service: Simulation service
        
    Returns:
        Simulation
    """
    try:
        simulation = await simulation_service.get_simulation(db, simulation_id)
        if not simulation:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail=f"Simulation {simulation_id} not found",
            )
        return simulation
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error getting simulation {simulation_id}: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error getting simulation: {str(e)}",
        )


@router.get("/", response_model=List[SimulationResponse])
async def get_simulations(
    choreography_id: Optional[str] = Query(None),
    skip: int = Query(0, ge=0),
    limit: int = Query(100, ge=1, le=1000),
    db: AsyncSession = Depends(get_db),
    simulation_service: SimulationService = Depends(get_simulation_service),
):
    """
    Get all simulations.
    
    Args:
        choreography_id: Filter by choreography ID
        skip: Number of records to skip
        limit: Maximum number of records to return
        db: Database session
        simulation_service: Simulation service
        
    Returns:
        List of simulations
    """
    try:
        return await simulation_service.get_simulations(db, choreography_id, skip, limit)
    except Exception as e:
        logger.error(f"Error getting simulations: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error getting simulations: {str(e)}",
        )


@router.delete("/{simulation_id}", status_code=status.HTTP_204_NO_CONTENT)
async def delete_simulation(
    simulation_id: str = Path(...),
    db: AsyncSession = Depends(get_db),
    simulation_service: SimulationService = Depends(get_simulation_service),
):
    """
    Delete a simulation.
    
    Args:
        simulation_id: Simulation ID
        db: Database session
        simulation_service: Simulation service
    """
    try:
        deleted = await simulation_service.delete_simulation(db, simulation_id)
        if not deleted:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail=f"Simulation {simulation_id} not found",
            )
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error deleting simulation {simulation_id}: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error deleting simulation: {str(e)}",
        )


@router.websocket("/ws/{simulation_id}")
async def simulation_websocket(
    websocket: WebSocket,
    simulation_id: str,
    db: AsyncSession = Depends(get_db),
    simulation_service: SimulationService = Depends(get_simulation_service),
):
    """
    WebSocket endpoint for streaming simulation data.
    
    Args:
        websocket: WebSocket connection
        simulation_id: Simulation ID
        db: Database session
        simulation_service: Simulation service
    """
    try:
        # Accept connection
        await websocket.accept()
        
        # Get simulation
        simulation = await simulation_service.get_simulation(db, simulation_id)
        if not simulation:
            await websocket.close(code=1000, reason=f"Simulation {simulation_id} not found")
            return
        
        # Send initial data
        await websocket.send_json({
            "type": "init",
            "data": {
                "id": simulation.id,
                "choreography_id": simulation.choreography_id,
                "settings": simulation.settings.dict(),
                "duration": simulation.duration,
            }
        })
        
        # Stream frames
        for frame in simulation.frames:
            # Check if client is still connected
            try:
                # Send frame
                await websocket.send_json({
                    "type": "frame",
                    "data": frame.dict(),
                })
                
                # Wait for client to acknowledge
                data = await websocket.receive_text()
                message = json.loads(data)
                
                # Check if client wants to pause, skip, or stop
                if message.get("action") == "stop":
                    break
                
                # Sleep if client wants to pause
                if message.get("action") == "pause":
                    while True:
                        data = await websocket.receive_text()
                        message = json.loads(data)
                        if message.get("action") == "resume":
                            break
                        elif message.get("action") == "stop":
                            return
            except WebSocketDisconnect:
                logger.info(f"Client disconnected from simulation {simulation_id}")
                break
        
        # Send end message
        await websocket.send_json({
            "type": "end",
            "data": {
                "id": simulation.id,
                "choreography_id": simulation.choreography_id,
            }
        })
    except WebSocketDisconnect:
        logger.info(f"Client disconnected from simulation {simulation_id}")
    except Exception as e:
        logger.error(f"Error in simulation WebSocket {simulation_id}: {str(e)}")
        try:
            await websocket.close(code=1011, reason=str(e))
        except:
            pass
