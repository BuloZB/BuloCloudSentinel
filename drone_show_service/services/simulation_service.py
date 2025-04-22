"""
Simulation service for the Drone Show microservice.

This module provides services for simulating choreographies.
"""

import logging
import json
import uuid
import math
import asyncio
from typing import List, Dict, Any, Optional, Tuple
from datetime import datetime
import numpy as np
from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy.future import select

from drone_show_service.models.choreography import (
    Choreography, ChoreographyResponse, SimulationSettings, SimulationFrame, SimulationResponse,
    Position, LEDState, DroneTrajectory
)
from drone_show_service.models.database import ChoreographyDB, SimulationDB
from drone_show_service.core.config import settings
from drone_show_service.services.minio_service import MinioService

logger = logging.getLogger(__name__)


class SimulationService:
    """
    Service for simulating choreographies.
    
    This service provides methods for simulating drone show choreographies,
    including trajectory interpolation and LED state calculation.
    """
    
    def __init__(self, minio_service: Optional[MinioService] = None):
        """
        Initialize the simulation service.
        
        Args:
            minio_service: MinIO service for storing simulation data
        """
        self.minio_service = minio_service or MinioService()
        self._simulations: Dict[str, Dict[str, Any]] = {}
    
    async def simulate_choreography(
        self, db: AsyncSession, choreography_id: str, settings: SimulationSettings
    ) -> SimulationResponse:
        """
        Simulate a choreography.
        
        Args:
            db: Database session
            choreography_id: Choreography ID
            settings: Simulation settings
            
        Returns:
            Simulation response
        """
        # Query database
        result = await db.execute(
            select(ChoreographyDB).where(ChoreographyDB.id == choreography_id)
        )
        db_choreography = result.scalars().first()
        
        # Raise exception if not found
        if not db_choreography:
            raise ValueError(f"Choreography {choreography_id} not found")
        
        # Convert to Choreography model
        choreography = self._db_to_choreography(db_choreography)
        
        # Generate simulation frames
        frames = self._generate_frames(choreography, settings)
        
        # Create simulation ID
        simulation_id = str(uuid.uuid4())
        
        # Store frames in MinIO
        frames_file = f"simulations/{simulation_id}/frames.json"
        await self.minio_service.put_json(frames_file, [frame.dict() for frame in frames])
        
        # Create database model
        db_simulation = SimulationDB(
            id=simulation_id,
            choreography_id=choreography_id,
            settings=json.loads(settings.json()),
            frames_file=frames_file,
            duration=choreography.metadata.duration,
        )
        
        # Add to database
        db.add(db_simulation)
        await db.commit()
        await db.refresh(db_simulation)
        
        # Create response model
        response = SimulationResponse(
            id=simulation_id,
            choreography_id=choreography_id,
            settings=settings,
            frames=frames,
            duration=choreography.metadata.duration,
        )
        
        # Store in memory
        self._simulations[simulation_id] = {
            "response": response,
            "frames": frames,
        }
        
        return response
    
    async def get_simulation(self, db: AsyncSession, simulation_id: str) -> Optional[SimulationResponse]:
        """
        Get a simulation by ID.
        
        Args:
            db: Database session
            simulation_id: Simulation ID
            
        Returns:
            Simulation if found, None otherwise
        """
        # Check in-memory cache
        if simulation_id in self._simulations:
            return self._simulations[simulation_id]["response"]
        
        # Query database
        result = await db.execute(
            select(SimulationDB).where(SimulationDB.id == simulation_id)
        )
        db_simulation = result.scalars().first()
        
        # Return None if not found
        if not db_simulation:
            return None
        
        # Load frames from MinIO
        frames_data = await self.minio_service.get_json(db_simulation.frames_file)
        frames = [SimulationFrame(**frame_data) for frame_data in frames_data]
        
        # Create response model
        response = SimulationResponse(
            id=db_simulation.id,
            choreography_id=db_simulation.choreography_id,
            settings=db_simulation.settings,
            frames=frames,
            duration=db_simulation.duration,
            created_at=db_simulation.created_at,
        )
        
        # Store in memory
        self._simulations[simulation_id] = {
            "response": response,
            "frames": frames,
        }
        
        return response
    
    async def get_simulations(
        self, db: AsyncSession, choreography_id: Optional[str] = None, skip: int = 0, limit: int = 100
    ) -> List[SimulationResponse]:
        """
        Get all simulations.
        
        Args:
            db: Database session
            choreography_id: Filter by choreography ID
            skip: Number of records to skip
            limit: Maximum number of records to return
            
        Returns:
            List of simulations
        """
        # Build query
        query = select(SimulationDB)
        
        # Apply filters
        if choreography_id:
            query = query.where(SimulationDB.choreography_id == choreography_id)
        
        # Apply pagination
        query = query.offset(skip).limit(limit)
        
        # Execute query
        result = await db.execute(query)
        db_simulations = result.scalars().all()
        
        # Convert to response models
        responses = []
        for db_simulation in db_simulations:
            # Load frames from MinIO
            frames_data = await self.minio_service.get_json(db_simulation.frames_file)
            frames = [SimulationFrame(**frame_data) for frame_data in frames_data]
            
            # Create response model
            response = SimulationResponse(
                id=db_simulation.id,
                choreography_id=db_simulation.choreography_id,
                settings=db_simulation.settings,
                frames=frames,
                duration=db_simulation.duration,
                created_at=db_simulation.created_at,
            )
            
            responses.append(response)
        
        return responses
    
    async def delete_simulation(self, db: AsyncSession, simulation_id: str) -> bool:
        """
        Delete a simulation.
        
        Args:
            db: Database session
            simulation_id: Simulation ID
            
        Returns:
            True if deleted, False if not found
        """
        # Query database
        result = await db.execute(
            select(SimulationDB).where(SimulationDB.id == simulation_id)
        )
        db_simulation = result.scalars().first()
        
        # Return False if not found
        if not db_simulation:
            return False
        
        # Delete frames from MinIO
        await self.minio_service.delete(db_simulation.frames_file)
        
        # Delete from database
        await db.delete(db_simulation)
        await db.commit()
        
        # Remove from memory
        if simulation_id in self._simulations:
            del self._simulations[simulation_id]
        
        return True
    
    def _db_to_choreography(self, db_choreography: ChoreographyDB) -> Choreography:
        """
        Convert a database model to a Choreography model.
        
        Args:
            db_choreography: Database model
            
        Returns:
            Choreography model
        """
        # Create metadata
        metadata = {
            "name": db_choreography.name,
            "description": db_choreography.description,
            "author": db_choreography.author,
            "created_at": db_choreography.created_at,
            "updated_at": db_choreography.updated_at,
            "tags": db_choreography.tags,
            "duration": db_choreography.duration,
            "drone_count": db_choreography.drone_count,
            "status": db_choreography.status,
        }
        
        # Create Choreography model
        return Choreography(
            id=db_choreography.id,
            metadata=metadata,
            type=db_choreography.type,
            trajectories=db_choreography.trajectories,
            formations=db_choreography.formations,
            music_file=db_choreography.music_file,
            music_bpm=db_choreography.music_bpm,
            music_offset=db_choreography.music_offset,
            boundary=db_choreography.boundary,
            home_position=db_choreography.home_position,
            notes=db_choreography.notes,
        )
    
    def _generate_frames(self, choreography: Choreography, settings: SimulationSettings) -> List[SimulationFrame]:
        """
        Generate simulation frames for a choreography.
        
        Args:
            choreography: Choreography to simulate
            settings: Simulation settings
            
        Returns:
            List of simulation frames
        """
        # Determine time range
        start_time = settings.start_time
        end_time = settings.end_time or choreography.metadata.duration
        
        # Calculate frame count
        frame_count = int((end_time - start_time) * settings.SIMULATION_UPDATE_RATE / settings.speed_factor)
        
        # Generate frames
        frames = []
        for i in range(frame_count):
            # Calculate time
            time = start_time + i * settings.speed_factor / settings.SIMULATION_UPDATE_RATE
            
            # Create frame
            frame = SimulationFrame(
                time=time,
                drone_states=self._calculate_drone_states(choreography, time),
            )
            
            frames.append(frame)
        
        return frames
    
    def _calculate_drone_states(self, choreography: Choreography, time: float) -> Dict[str, Dict[str, Any]]:
        """
        Calculate drone states at a specific time.
        
        Args:
            choreography: Choreography to simulate
            time: Time in seconds from start of show
            
        Returns:
            Dictionary of drone states
        """
        drone_states = {}
        
        # Process each drone trajectory
        for trajectory in choreography.trajectories:
            # Get drone ID
            drone_id = trajectory.drone_id
            
            # Calculate position and orientation
            position, heading = self._interpolate_position(trajectory, time)
            
            # Calculate LED state
            led_state = self._interpolate_led_state(trajectory, time)
            
            # Create drone state
            drone_states[drone_id] = {
                "position": position.dict() if position else None,
                "heading": heading,
                "led_state": led_state.dict() if led_state else None,
            }
        
        return drone_states
    
    def _interpolate_position(self, trajectory: DroneTrajectory, time: float) -> Tuple[Optional[Position], Optional[float]]:
        """
        Interpolate position and heading at a specific time.
        
        Args:
            trajectory: Drone trajectory
            time: Time in seconds from start of show
            
        Returns:
            Tuple of (position, heading)
        """
        # Sort waypoints by time
        waypoints = sorted(trajectory.waypoints, key=lambda wp: wp.time)
        
        # Handle edge cases
        if not waypoints:
            return None, None
        
        if time <= waypoints[0].time:
            return waypoints[0].position, waypoints[0].heading
        
        if time >= waypoints[-1].time:
            return waypoints[-1].position, waypoints[-1].heading
        
        # Find surrounding waypoints
        for i in range(len(waypoints) - 1):
            if waypoints[i].time <= time < waypoints[i + 1].time:
                # Calculate interpolation factor
                t = (time - waypoints[i].time) / (waypoints[i + 1].time - waypoints[i].time)
                
                # Interpolate position
                pos1 = waypoints[i].position
                pos2 = waypoints[i + 1].position
                
                position = Position(
                    lat=pos1.lat + t * (pos2.lat - pos1.lat),
                    lon=pos1.lon + t * (pos2.lon - pos1.lon),
                    alt=pos1.alt + t * (pos2.alt - pos1.alt),
                )
                
                # Interpolate heading
                heading1 = waypoints[i].heading
                heading2 = waypoints[i + 1].heading
                
                if heading1 is not None and heading2 is not None:
                    # Handle wrap-around
                    if abs(heading2 - heading1) > 180:
                        if heading1 < heading2:
                            heading1 += 360
                        else:
                            heading2 += 360
                    
                    heading = heading1 + t * (heading2 - heading1)
                    
                    # Normalize to 0-360
                    heading = heading % 360
                else:
                    heading = heading1 or heading2
                
                return position, heading
        
        # Fallback
        return waypoints[-1].position, waypoints[-1].heading
    
    def _interpolate_led_state(self, trajectory: DroneTrajectory, time: float) -> Optional[LEDState]:
        """
        Interpolate LED state at a specific time.
        
        Args:
            trajectory: Drone trajectory
            time: Time in seconds from start of show
            
        Returns:
            LED state
        """
        # Sort LED states by time
        led_states = sorted(trajectory.led_states, key=lambda ls: ls.time)
        
        # Handle edge cases
        if not led_states:
            return None
        
        if time <= led_states[0].time:
            return led_states[0]
        
        if time >= led_states[-1].time:
            return led_states[-1]
        
        # Find surrounding LED states
        for i in range(len(led_states) - 1):
            if led_states[i].time <= time < led_states[i + 1].time:
                # For now, just return the previous state
                # In a real implementation, we would interpolate based on effect type
                return led_states[i]
        
        # Fallback
        return led_states[-1]
