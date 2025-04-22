"""
Execution service for the Drone Show microservice.

This module provides services for executing choreographies.
"""

import logging
import json
import uuid
import asyncio
import time
from typing import Dict, Any, List, Optional, Set
from datetime import datetime
from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy.future import select

from drone_show_service.models.choreography import (
    Choreography, ChoreographyResponse, ExecutionSettings, ExecutionStatus, ExecutionResponse,
    DroneStatus, Position, LEDState
)
from drone_show_service.models.database import ChoreographyDB, ExecutionDB
from drone_show_service.core.config import settings
from drone_show_service.services.choreography_service import ChoreographyService
from drone_show_service.services.led_control_service import LEDControlService
from drone_show_service.services.synchronization_service import SynchronizationService
from drone_show_service.services.telemetry_service import TelemetryService
from drone_show_service.services.logging_service import LoggingService
from drone_show_service.services.sentinel_integration import SentinelIntegrationService

logger = logging.getLogger(__name__)


class ExecutionService:
    """
    Service for executing choreographies.
    
    This service provides methods for executing drone show choreographies,
    including trajectory following and LED control.
    """
    
    def __init__(
        self,
        choreography_service: Optional[ChoreographyService] = None,
        led_control_service: Optional[LEDControlService] = None,
        synchronization_service: Optional[SynchronizationService] = None,
        telemetry_service: Optional[TelemetryService] = None,
        logging_service: Optional[LoggingService] = None,
        sentinel_integration: Optional[SentinelIntegrationService] = None,
    ):
        """
        Initialize the execution service.
        
        Args:
            choreography_service: Choreography service
            led_control_service: LED control service
            synchronization_service: Synchronization service
            telemetry_service: Telemetry service
            logging_service: Logging service
            sentinel_integration: Sentinel integration service
        """
        self.choreography_service = choreography_service or ChoreographyService()
        self.led_control_service = led_control_service or LEDControlService()
        self.synchronization_service = synchronization_service or SynchronizationService()
        self.telemetry_service = telemetry_service or TelemetryService()
        self.logging_service = logging_service or LoggingService()
        self.sentinel_integration = sentinel_integration or SentinelIntegrationService()
        
        self._active_executions: Dict[str, Dict[str, Any]] = {}
        self._background_tasks: Set[asyncio.Task] = set()
        self._running = True
    
    async def execute_choreography(
        self, db: AsyncSession, choreography_id: str, settings: ExecutionSettings
    ) -> ExecutionResponse:
        """
        Execute a choreography.
        
        Args:
            db: Database session
            choreography_id: Choreography ID
            settings: Execution settings
            
        Returns:
            Execution response
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
        
        # Get drone IDs
        drone_ids = [trajectory.drone_id for trajectory in choreography.trajectories]
        
        # Create execution ID
        execution_id = str(uuid.uuid4())
        
        # Initialize drone statuses
        drone_statuses = {}
        for drone_id in drone_ids:
            drone_statuses[drone_id] = DroneStatus(
                drone_id=drone_id,
                connected=False,
                battery_level=0.0,
                position=None,
                heading=None,
                led_state=None,
                current_waypoint_index=None,
                status="idle",
                error_message=None,
            )
        
        # Create database model
        db_execution = ExecutionDB(
            id=execution_id,
            choreography_id=choreography_id,
            settings=json.loads(settings.json()),
            status=ExecutionStatus.PENDING.value,
            drone_statuses=json.loads(json.dumps({k: v.dict() for k, v in drone_statuses.items()})),
        )
        
        # Add to database
        db.add(db_execution)
        await db.commit()
        await db.refresh(db_execution)
        
        # Create response model
        response = ExecutionResponse(
            id=execution_id,
            choreography_id=choreography_id,
            settings=settings,
            status=ExecutionStatus.PENDING,
            drone_statuses=drone_statuses,
            created_at=db_execution.created_at,
            updated_at=db_execution.updated_at,
        )
        
        # Store in memory
        self._active_executions[execution_id] = {
            "response": response,
            "choreography": choreography,
            "db_execution": db_execution,
            "db": db,
            "running": True,
            "paused": False,
        }
        
        # Start execution in background
        task = asyncio.create_task(self._execute_choreography_task(execution_id))
        self._background_tasks.add(task)
        task.add_done_callback(self._background_tasks.remove)
        
        return response
    
    async def get_execution(self, db: AsyncSession, execution_id: str) -> Optional[ExecutionResponse]:
        """
        Get an execution by ID.
        
        Args:
            db: Database session
            execution_id: Execution ID
            
        Returns:
            Execution if found, None otherwise
        """
        # Check in-memory cache
        if execution_id in self._active_executions:
            return self._active_executions[execution_id]["response"]
        
        # Query database
        result = await db.execute(
            select(ExecutionDB).where(ExecutionDB.id == execution_id)
        )
        db_execution = result.scalars().first()
        
        # Return None if not found
        if not db_execution:
            return None
        
        # Create response model
        response = ExecutionResponse(
            id=db_execution.id,
            choreography_id=db_execution.choreography_id,
            settings=db_execution.settings,
            status=ExecutionStatus(db_execution.status),
            drone_statuses={k: DroneStatus(**v) for k, v in db_execution.drone_statuses.items()},
            start_time=db_execution.start_time,
            end_time=db_execution.end_time,
            current_time=db_execution.current_time,
            progress=db_execution.progress,
            error_message=db_execution.error_message,
            created_at=db_execution.created_at,
            updated_at=db_execution.updated_at,
        )
        
        return response
    
    async def get_executions(
        self, db: AsyncSession, choreography_id: Optional[str] = None, status: Optional[str] = None,
        skip: int = 0, limit: int = 100
    ) -> List[ExecutionResponse]:
        """
        Get all executions.
        
        Args:
            db: Database session
            choreography_id: Filter by choreography ID
            status: Filter by status
            skip: Number of records to skip
            limit: Maximum number of records to return
            
        Returns:
            List of executions
        """
        # Build query
        query = select(ExecutionDB)
        
        # Apply filters
        if choreography_id:
            query = query.where(ExecutionDB.choreography_id == choreography_id)
        
        if status:
            query = query.where(ExecutionDB.status == status)
        
        # Apply sorting
        query = query.order_by(ExecutionDB.created_at.desc())
        
        # Apply pagination
        query = query.offset(skip).limit(limit)
        
        # Execute query
        result = await db.execute(query)
        db_executions = result.scalars().all()
        
        # Convert to response models
        responses = []
        for db_execution in db_executions:
            # Create response model
            response = ExecutionResponse(
                id=db_execution.id,
                choreography_id=db_execution.choreography_id,
                settings=db_execution.settings,
                status=ExecutionStatus(db_execution.status),
                drone_statuses={k: DroneStatus(**v) for k, v in db_execution.drone_statuses.items()},
                start_time=db_execution.start_time,
                end_time=db_execution.end_time,
                current_time=db_execution.current_time,
                progress=db_execution.progress,
                error_message=db_execution.error_message,
                created_at=db_execution.created_at,
                updated_at=db_execution.updated_at,
            )
            
            responses.append(response)
        
        return responses
    
    async def pause_execution(self, db: AsyncSession, execution_id: str) -> Optional[ExecutionResponse]:
        """
        Pause an execution.
        
        Args:
            db: Database session
            execution_id: Execution ID
            
        Returns:
            Updated execution if found, None otherwise
        """
        # Check if execution exists
        if execution_id not in self._active_executions:
            # Query database
            result = await db.execute(
                select(ExecutionDB).where(ExecutionDB.id == execution_id)
            )
            db_execution = result.scalars().first()
            
            # Return None if not found
            if not db_execution:
                return None
            
            # Check if execution is in progress
            if db_execution.status != ExecutionStatus.IN_PROGRESS.value:
                logger.warning(f"Cannot pause execution {execution_id} with status {db_execution.status}")
                return None
            
            # Update status
            db_execution.status = ExecutionStatus.PAUSED.value
            db_execution.updated_at = datetime.utcnow()
            
            # Commit changes
            await db.commit()
            await db.refresh(db_execution)
            
            # Create response model
            response = ExecutionResponse(
                id=db_execution.id,
                choreography_id=db_execution.choreography_id,
                settings=db_execution.settings,
                status=ExecutionStatus(db_execution.status),
                drone_statuses={k: DroneStatus(**v) for k, v in db_execution.drone_statuses.items()},
                start_time=db_execution.start_time,
                end_time=db_execution.end_time,
                current_time=db_execution.current_time,
                progress=db_execution.progress,
                error_message=db_execution.error_message,
                created_at=db_execution.created_at,
                updated_at=db_execution.updated_at,
            )
            
            return response
        
        # Pause execution
        execution_data = self._active_executions[execution_id]
        execution_data["paused"] = True
        
        # Update status
        db_execution = execution_data["db_execution"]
        db_execution.status = ExecutionStatus.PAUSED.value
        db_execution.updated_at = datetime.utcnow()
        
        # Commit changes
        await db.commit()
        await db.refresh(db_execution)
        
        # Update response
        response = execution_data["response"]
        response.status = ExecutionStatus.PAUSED
        response.updated_at = db_execution.updated_at
        
        return response
    
    async def resume_execution(self, db: AsyncSession, execution_id: str) -> Optional[ExecutionResponse]:
        """
        Resume a paused execution.
        
        Args:
            db: Database session
            execution_id: Execution ID
            
        Returns:
            Updated execution if found, None otherwise
        """
        # Check if execution exists
        if execution_id not in self._active_executions:
            # Query database
            result = await db.execute(
                select(ExecutionDB).where(ExecutionDB.id == execution_id)
            )
            db_execution = result.scalars().first()
            
            # Return None if not found
            if not db_execution:
                return None
            
            # Check if execution is paused
            if db_execution.status != ExecutionStatus.PAUSED.value:
                logger.warning(f"Cannot resume execution {execution_id} with status {db_execution.status}")
                return None
            
            # Update status
            db_execution.status = ExecutionStatus.IN_PROGRESS.value
            db_execution.updated_at = datetime.utcnow()
            
            # Commit changes
            await db.commit()
            await db.refresh(db_execution)
            
            # Create response model
            response = ExecutionResponse(
                id=db_execution.id,
                choreography_id=db_execution.choreography_id,
                settings=db_execution.settings,
                status=ExecutionStatus(db_execution.status),
                drone_statuses={k: DroneStatus(**v) for k, v in db_execution.drone_statuses.items()},
                start_time=db_execution.start_time,
                end_time=db_execution.end_time,
                current_time=db_execution.current_time,
                progress=db_execution.progress,
                error_message=db_execution.error_message,
                created_at=db_execution.created_at,
                updated_at=db_execution.updated_at,
            )
            
            return response
        
        # Resume execution
        execution_data = self._active_executions[execution_id]
        execution_data["paused"] = False
        
        # Update status
        db_execution = execution_data["db_execution"]
        db_execution.status = ExecutionStatus.IN_PROGRESS.value
        db_execution.updated_at = datetime.utcnow()
        
        # Commit changes
        await db.commit()
        await db.refresh(db_execution)
        
        # Update response
        response = execution_data["response"]
        response.status = ExecutionStatus.IN_PROGRESS
        response.updated_at = db_execution.updated_at
        
        return response
    
    async def abort_execution(self, db: AsyncSession, execution_id: str) -> Optional[ExecutionResponse]:
        """
        Abort an execution.
        
        Args:
            db: Database session
            execution_id: Execution ID
            
        Returns:
            Updated execution if found, None otherwise
        """
        # Check if execution exists
        if execution_id not in self._active_executions:
            # Query database
            result = await db.execute(
                select(ExecutionDB).where(ExecutionDB.id == execution_id)
            )
            db_execution = result.scalars().first()
            
            # Return None if not found
            if not db_execution:
                return None
            
            # Check if execution is in progress or paused
            if db_execution.status not in [ExecutionStatus.IN_PROGRESS.value, ExecutionStatus.PAUSED.value]:
                logger.warning(f"Cannot abort execution {execution_id} with status {db_execution.status}")
                return None
            
            # Update status
            db_execution.status = ExecutionStatus.ABORTED.value
            db_execution.updated_at = datetime.utcnow()
            db_execution.end_time = datetime.utcnow()
            
            # Commit changes
            await db.commit()
            await db.refresh(db_execution)
            
            # Create response model
            response = ExecutionResponse(
                id=db_execution.id,
                choreography_id=db_execution.choreography_id,
                settings=db_execution.settings,
                status=ExecutionStatus(db_execution.status),
                drone_statuses={k: DroneStatus(**v) for k, v in db_execution.drone_statuses.items()},
                start_time=db_execution.start_time,
                end_time=db_execution.end_time,
                current_time=db_execution.current_time,
                progress=db_execution.progress,
                error_message=db_execution.error_message,
                created_at=db_execution.created_at,
                updated_at=db_execution.updated_at,
            )
            
            return response
        
        # Abort execution
        execution_data = self._active_executions[execution_id]
        execution_data["running"] = False
        
        # Update status
        db_execution = execution_data["db_execution"]
        db_execution.status = ExecutionStatus.ABORTED.value
        db_execution.updated_at = datetime.utcnow()
        db_execution.end_time = datetime.utcnow()
        
        # Commit changes
        await db.commit()
        await db.refresh(db_execution)
        
        # Update response
        response = execution_data["response"]
        response.status = ExecutionStatus.ABORTED
        response.updated_at = db_execution.updated_at
        response.end_time = db_execution.end_time
        
        return response
    
    async def shutdown(self):
        """Shutdown the execution service."""
        # Stop all executions
        for execution_id in list(self._active_executions.keys()):
            self._active_executions[execution_id]["running"] = False
        
        # Set running flag to False
        self._running = False
        
        # Wait for all background tasks to complete
        if self._background_tasks:
            await asyncio.gather(*self._background_tasks, return_exceptions=True)
    
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
    
    async def _execute_choreography_task(self, execution_id: str):
        """
        Execute a choreography in the background.
        
        Args:
            execution_id: Execution ID
        """
        try:
            # Get execution data
            execution_data = self._active_executions[execution_id]
            choreography = execution_data["choreography"]
            db_execution = execution_data["db_execution"]
            db = execution_data["db"]
            response = execution_data["response"]
            
            # Get drone IDs
            drone_ids = [trajectory.drone_id for trajectory in choreography.trajectories]
            
            # Update status
            db_execution.status = ExecutionStatus.PREPARING.value
            db_execution.updated_at = datetime.utcnow()
            await db.commit()
            await db.refresh(db_execution)
            
            # Update response
            response.status = ExecutionStatus.PREPARING
            response.updated_at = db_execution.updated_at
            
            # Log event
            await self.logging_service.log_info(
                db, execution_id, f"Preparing to execute choreography {choreography.id}"
            )
            
            # Start telemetry
            await self.telemetry_service.start_telemetry(execution_id, drone_ids)
            
            # Register telemetry callback
            def telemetry_callback(drone_id: str, telemetry: Dict[str, Any]):
                # Update drone status
                if drone_id in response.drone_statuses:
                    # Extract position
                    position = None
                    if "lat" in telemetry and "lon" in telemetry and "alt" in telemetry:
                        position = Position(
                            lat=float(telemetry["lat"]),
                            lon=float(telemetry["lon"]),
                            alt=float(telemetry["alt"]),
                        )
                    
                    # Update status
                    status = response.drone_statuses[drone_id]
                    status.connected = telemetry.get("connected", False)
                    status.battery_level = float(telemetry.get("battery", 0))
                    status.position = position
                    status.heading = float(telemetry.get("heading", 0)) if "heading" in telemetry else None
                    status.current_waypoint_index = int(telemetry.get("waypoint_index", 0)) if "waypoint_index" in telemetry else None
                    status.status = telemetry.get("status", "unknown")
                    status.error_message = telemetry.get("error_message")
            
            self.telemetry_service.register_telemetry_callback(execution_id, telemetry_callback)
            
            # Start time synchronization
            await self.synchronization_service.start_synchronization(execution_id, drone_ids)
            
            # Wait for all drones to connect
            connected_drones = set()
            timeout = 60  # seconds
            start_time = time.time()
            
            while len(connected_drones) < len(drone_ids) and time.time() - start_time < timeout:
                # Check if execution is still running
                if not execution_data["running"]:
                    # Execution aborted
                    await self.logging_service.log_warning(
                        db, execution_id, "Execution aborted during preparation"
                    )
                    return
                
                # Check drone statuses
                for drone_id, status in response.drone_statuses.items():
                    if status.connected and drone_id not in connected_drones:
                        connected_drones.add(drone_id)
                        await self.logging_service.log_info(
                            db, execution_id, f"Drone {drone_id} connected"
                        )
                
                # Sleep briefly
                await asyncio.sleep(1.0)
            
            # Check if all drones connected
            if len(connected_drones) < len(drone_ids):
                # Not all drones connected
                missing_drones = set(drone_ids) - connected_drones
                error_message = f"Not all drones connected: {missing_drones}"
                
                # Update status
                db_execution.status = ExecutionStatus.FAILED.value
                db_execution.updated_at = datetime.utcnow()
                db_execution.end_time = datetime.utcnow()
                db_execution.error_message = error_message
                await db.commit()
                await db.refresh(db_execution)
                
                # Update response
                response.status = ExecutionStatus.FAILED
                response.updated_at = db_execution.updated_at
                response.end_time = db_execution.end_time
                response.error_message = error_message
                
                # Log event
                await self.logging_service.log_error(
                    db, execution_id, error_message
                )
                
                # Clean up
                await self.telemetry_service.stop_telemetry(execution_id)
                await self.synchronization_service.stop_synchronization(execution_id)
                
                return
            
            # Update status
            db_execution.status = ExecutionStatus.IN_PROGRESS.value
            db_execution.updated_at = datetime.utcnow()
            db_execution.start_time = datetime.utcnow()
            await db.commit()
            await db.refresh(db_execution)
            
            # Update response
            response.status = ExecutionStatus.IN_PROGRESS
            response.updated_at = db_execution.updated_at
            response.start_time = db_execution.start_time
            
            # Log event
            await self.logging_service.log_info(
                db, execution_id, f"Starting execution of choreography {choreography.id}"
            )
            
            # Prepare LED states
            led_states = {}
            for trajectory in choreography.trajectories:
                led_states[trajectory.drone_id] = trajectory.led_states
            
            # Start LED show
            await self.led_control_service.start_led_show(execution_id, led_states)
            
            # Synchronize start time
            start_time = time.time() + 5.0  # Start in 5 seconds
            await self.synchronization_service.synchronize_start_time(execution_id, start_time)
            
            # Wait for start time
            await asyncio.sleep(max(0, start_time - time.time()))
            
            # Execute choreography
            duration = choreography.metadata.duration
            start_execution_time = time.time()
            
            while time.time() - start_execution_time < duration:
                # Check if execution is still running
                if not execution_data["running"]:
                    # Execution aborted
                    await self.logging_service.log_warning(
                        db, execution_id, "Execution aborted during execution"
                    )
                    break
                
                # Check if execution is paused
                if execution_data["paused"]:
                    # Execution paused
                    await asyncio.sleep(0.1)
                    continue
                
                # Calculate current time
                current_time = time.time() - start_execution_time
                
                # Update progress
                progress = min(100.0, current_time / duration * 100.0)
                
                # Update database
                db_execution.current_time = current_time
                db_execution.progress = progress
                db_execution.updated_at = datetime.utcnow()
                await db.commit()
                
                # Update response
                response.current_time = current_time
                response.progress = progress
                response.updated_at = db_execution.updated_at
                
                # Sleep briefly
                await asyncio.sleep(0.1)
            
            # Execution completed
            db_execution.status = ExecutionStatus.COMPLETED.value
            db_execution.updated_at = datetime.utcnow()
            db_execution.end_time = datetime.utcnow()
            db_execution.current_time = duration
            db_execution.progress = 100.0
            await db.commit()
            await db.refresh(db_execution)
            
            # Update response
            response.status = ExecutionStatus.COMPLETED
            response.updated_at = db_execution.updated_at
            response.end_time = db_execution.end_time
            response.current_time = duration
            response.progress = 100.0
            
            # Log event
            await self.logging_service.log_info(
                db, execution_id, f"Execution of choreography {choreography.id} completed"
            )
            
            # Clean up
            await self.telemetry_service.stop_telemetry(execution_id)
            await self.synchronization_service.stop_synchronization(execution_id)
            await self.led_control_service.stop_led_show(execution_id)
            
            # Remove from active executions
            del self._active_executions[execution_id]
        except Exception as e:
            # Log error
            logger.error(f"Error executing choreography: {str(e)}")
            
            # Update status
            try:
                db_execution = execution_data["db_execution"]
                db = execution_data["db"]
                response = execution_data["response"]
                
                db_execution.status = ExecutionStatus.FAILED.value
                db_execution.updated_at = datetime.utcnow()
                db_execution.end_time = datetime.utcnow()
                db_execution.error_message = str(e)
                await db.commit()
                await db.refresh(db_execution)
                
                # Update response
                response.status = ExecutionStatus.FAILED
                response.updated_at = db_execution.updated_at
                response.end_time = db_execution.end_time
                response.error_message = str(e)
                
                # Log event
                await self.logging_service.log_error(
                    db, execution_id, f"Execution failed: {str(e)}"
                )
                
                # Clean up
                await self.telemetry_service.stop_telemetry(execution_id)
                await self.synchronization_service.stop_synchronization(execution_id)
                await self.led_control_service.stop_led_show(execution_id)
                
                # Remove from active executions
                del self._active_executions[execution_id]
            except Exception as cleanup_error:
                logger.error(f"Error cleaning up after execution failure: {str(cleanup_error)}")
