"""
Mission execution service for Bulo.Cloud Sentinel.

This module provides services for executing missions, monitoring telemetry,
and handling mission control commands.
"""

import asyncio
import logging
import json
import time
import uuid
from datetime import datetime
from typing import Dict, List, Any, Optional, Union, Callable
from enum import Enum

from backend.mission_planning.models import (
    Mission, MissionType, MissionExecution, MissionExecutionStatus,
    Position, Waypoint, CameraAction, ActionTrigger
)
from backend.mission_planning.service import MissionPlanningService
from backend.mission_planning.terrain_service import TerrainService
from dronecore.drone_command_telemetry_hub import DroneCommandHub
from dronecore.telemetry_models import TelemetryData

logger = logging.getLogger(__name__)


class ExecutionCommand(str, Enum):
    """Commands for mission execution control."""
    START = "start"
    PAUSE = "pause"
    RESUME = "resume"
    ABORT = "abort"
    SKIP_WAYPOINT = "skip_waypoint"
    RETURN_HOME = "return_home"
    TAKE_PHOTO = "take_photo"
    START_RECORDING = "start_recording"
    STOP_RECORDING = "stop_recording"
    SET_GIMBAL = "set_gimbal"
    SET_SPEED = "set_speed"


class MissionExecutionService:
    """
    Service for mission execution operations.
    
    This service handles the execution of missions, including sending commands
    to drones, monitoring telemetry, and updating mission status.
    """
    
    def __init__(
        self,
        mission_service: MissionPlanningService,
        drone_command_hub: DroneCommandHub,
        terrain_service: Optional[TerrainService] = None
    ):
        """
        Initialize the mission execution service.
        
        Args:
            mission_service: Mission planning service
            drone_command_hub: Drone command hub
            terrain_service: Optional terrain service
        """
        self.mission_service = mission_service
        self.drone_command_hub = drone_command_hub
        self.terrain_service = terrain_service or TerrainService()
        
        # Active executions
        self._active_executions: Dict[str, Dict[str, Any]] = {}
        
        # Telemetry subscribers
        self._telemetry_subscribers: Dict[str, List[Callable]] = {}
        
        # Background tasks
        self._background_tasks = set()
        self._running = False
    
    async def start(self):
        """Start the execution service."""
        if self._running:
            return
        
        self._running = True
        
        # Start telemetry monitoring
        task = asyncio.create_task(self._monitor_telemetry())
        self._background_tasks.add(task)
        task.add_done_callback(self._background_tasks.remove)
        
        logger.info("Mission execution service started")
    
    async def stop(self):
        """Stop the execution service."""
        self._running = False
        
        # Cancel all background tasks
        for task in self._background_tasks:
            task.cancel()
        
        # Wait for all tasks to complete
        if self._background_tasks:
            await asyncio.gather(*self._background_tasks, return_exceptions=True)
        
        logger.info("Mission execution service stopped")
    
    async def execute_mission(self, mission_id: str, drone_id: str) -> Optional[MissionExecution]:
        """
        Start executing a mission.
        
        Args:
            mission_id: ID of the mission to execute
            drone_id: ID of the drone to use
            
        Returns:
            Mission execution record if successful, None otherwise
        """
        # Get mission
        mission = await self.mission_service.get_mission(mission_id)
        if not mission:
            logger.error(f"Mission {mission_id} not found")
            return None
        
        # Check if drone is available
        if not await self.drone_command_hub.is_drone_available(drone_id):
            logger.error(f"Drone {drone_id} is not available")
            return None
        
        # Create execution record
        execution = await self.mission_service.start_mission_execution(mission_id, drone_id)
        if not execution:
            logger.error(f"Failed to create execution record for mission {mission_id}")
            return None
        
        # Get waypoints
        waypoints = mission.settings.waypoint
        if not waypoints:
            logger.error(f"Mission {mission_id} has no waypoints")
            await self.mission_service.update_mission_execution(
                execution.id,
                {
                    "status": MissionExecutionStatus.FAILED.value,
                    "end_time": datetime.utcnow(),
                    "notes": "Mission has no waypoints"
                }
            )
            return None
        
        # Apply terrain following if enabled
        if mission.terrain_following != "none" and self.terrain_service:
            try:
                # Get AGL altitude
                agl_altitude = mission.settings.mapping.flightAltitude if mission.mission_type == MissionType.MAPPING else 50
                
                # Adjust waypoints for terrain
                waypoints_dict = [wp.dict() for wp in waypoints]
                adjusted_waypoints = await self.terrain_service.adjust_altitude_for_terrain(
                    waypoints_dict,
                    agl_altitude
                )
                
                # Convert back to Waypoint objects
                waypoints = [Waypoint(**wp) for wp in adjusted_waypoints]
                
                logger.info(f"Applied terrain following to {len(waypoints)} waypoints")
            except Exception as e:
                logger.error(f"Error applying terrain following: {str(e)}")
        
        # Update execution record
        await self.mission_service.update_mission_execution(
            execution.id,
            {
                "status": MissionExecutionStatus.IN_PROGRESS.value,
                "total_waypoints": len(waypoints)
            }
        )
        
        # Store execution data
        self._active_executions[execution.id] = {
            "mission": mission,
            "execution": execution,
            "waypoints": waypoints,
            "current_waypoint_index": 0,
            "paused": False,
            "telemetry_log": [],
            "start_time": datetime.utcnow(),
            "last_update_time": datetime.utcnow()
        }
        
        # Start execution in background
        task = asyncio.create_task(self._execute_mission_task(execution.id))
        self._background_tasks.add(task)
        task.add_done_callback(self._background_tasks.remove)
        
        return execution
    
    async def control_execution(self, execution_id: str, command: ExecutionCommand, params: Optional[Dict[str, Any]] = None) -> bool:
        """
        Control an active mission execution.
        
        Args:
            execution_id: ID of the execution to control
            command: Command to execute
            params: Optional parameters for the command
            
        Returns:
            True if command was executed successfully, False otherwise
        """
        if execution_id not in self._active_executions:
            logger.error(f"Execution {execution_id} not found")
            return False
        
        execution_data = self._active_executions[execution_id]
        drone_id = execution_data["execution"].drone_id
        
        try:
            if command == ExecutionCommand.PAUSE:
                # Pause mission
                execution_data["paused"] = True
                
                # Send pause command to drone
                await self.drone_command_hub.pause_mission(drone_id)
                
                # Update execution record
                await self.mission_service.update_mission_execution(
                    execution_id,
                    {"status": MissionExecutionStatus.PAUSED.value}
                )
                
                logger.info(f"Paused execution {execution_id}")
                return True
            
            elif command == ExecutionCommand.RESUME:
                # Resume mission
                execution_data["paused"] = False
                
                # Send resume command to drone
                await self.drone_command_hub.resume_mission(drone_id)
                
                # Update execution record
                await self.mission_service.update_mission_execution(
                    execution_id,
                    {"status": MissionExecutionStatus.IN_PROGRESS.value}
                )
                
                logger.info(f"Resumed execution {execution_id}")
                return True
            
            elif command == ExecutionCommand.ABORT:
                # Abort mission
                execution_data["paused"] = True
                
                # Send abort command to drone
                await self.drone_command_hub.abort_mission(drone_id)
                
                # Update execution record
                await self.mission_service.update_mission_execution(
                    execution_id,
                    {
                        "status": MissionExecutionStatus.ABORTED.value,
                        "end_time": datetime.utcnow(),
                        "notes": "Mission aborted by user"
                    }
                )
                
                # Remove from active executions
                del self._active_executions[execution_id]
                
                logger.info(f"Aborted execution {execution_id}")
                return True
            
            elif command == ExecutionCommand.SKIP_WAYPOINT:
                # Skip current waypoint
                current_index = execution_data["current_waypoint_index"]
                if current_index < len(execution_data["waypoints"]) - 1:
                    execution_data["current_waypoint_index"] = current_index + 1
                    
                    # Send command to drone
                    await self.drone_command_hub.goto_waypoint(
                        drone_id,
                        execution_data["waypoints"][current_index + 1].dict()
                    )
                    
                    logger.info(f"Skipped waypoint {current_index} in execution {execution_id}")
                    return True
                else:
                    logger.error(f"Cannot skip last waypoint in execution {execution_id}")
                    return False
            
            elif command == ExecutionCommand.RETURN_HOME:
                # Return to home
                await self.drone_command_hub.return_to_home(drone_id)
                
                # Update execution record
                await self.mission_service.update_mission_execution(
                    execution_id,
                    {
                        "status": MissionExecutionStatus.ABORTED.value,
                        "end_time": datetime.utcnow(),
                        "notes": "Mission aborted - returned to home"
                    }
                )
                
                # Remove from active executions
                del self._active_executions[execution_id]
                
                logger.info(f"Returned to home for execution {execution_id}")
                return True
            
            elif command == ExecutionCommand.TAKE_PHOTO:
                # Take photo
                await self.drone_command_hub.take_photo(drone_id)
                
                # Update photos taken count
                execution = execution_data["execution"]
                photos_taken = execution.photos_taken or 0
                await self.mission_service.update_mission_execution(
                    execution_id,
                    {"photos_taken": photos_taken + 1}
                )
                
                logger.info(f"Took photo during execution {execution_id}")
                return True
            
            elif command == ExecutionCommand.START_RECORDING:
                # Start recording
                await self.drone_command_hub.start_recording(drone_id)
                
                logger.info(f"Started recording during execution {execution_id}")
                return True
            
            elif command == ExecutionCommand.STOP_RECORDING:
                # Stop recording
                duration = await self.drone_command_hub.stop_recording(drone_id)
                
                # Update video duration
                execution = execution_data["execution"]
                video_duration = execution.video_duration or 0
                await self.mission_service.update_mission_execution(
                    execution_id,
                    {"video_duration": video_duration + (duration or 0)}
                )
                
                logger.info(f"Stopped recording during execution {execution_id}")
                return True
            
            elif command == ExecutionCommand.SET_GIMBAL:
                # Set gimbal
                if not params or "pitch" not in params:
                    logger.error(f"Missing pitch parameter for SET_GIMBAL command")
                    return False
                
                pitch = params["pitch"]
                yaw = params.get("yaw")
                roll = params.get("roll")
                
                await self.drone_command_hub.set_gimbal(drone_id, pitch, yaw, roll)
                
                logger.info(f"Set gimbal during execution {execution_id}")
                return True
            
            elif command == ExecutionCommand.SET_SPEED:
                # Set speed
                if not params or "speed" not in params:
                    logger.error(f"Missing speed parameter for SET_SPEED command")
                    return False
                
                speed = params["speed"]
                
                await self.drone_command_hub.set_speed(drone_id, speed)
                
                logger.info(f"Set speed to {speed} during execution {execution_id}")
                return True
            
            else:
                logger.error(f"Unknown command: {command}")
                return False
        
        except Exception as e:
            logger.error(f"Error executing command {command} for execution {execution_id}: {str(e)}")
            return False
    
    async def get_execution_status(self, execution_id: str) -> Optional[Dict[str, Any]]:
        """
        Get the status of a mission execution.
        
        Args:
            execution_id: ID of the execution to get status for
            
        Returns:
            Execution status if found, None otherwise
        """
        if execution_id in self._active_executions:
            # Get from active executions
            execution_data = self._active_executions[execution_id]
            
            # Get latest telemetry
            latest_telemetry = None
            if execution_data["telemetry_log"]:
                latest_telemetry = execution_data["telemetry_log"][-1]
            
            return {
                "execution_id": execution_id,
                "mission_id": execution_data["mission"].id,
                "drone_id": execution_data["execution"].drone_id,
                "status": execution_data["execution"].status,
                "current_waypoint": execution_data["current_waypoint_index"] + 1,
                "total_waypoints": len(execution_data["waypoints"]),
                "paused": execution_data["paused"],
                "telemetry": latest_telemetry,
                "start_time": execution_data["start_time"].isoformat(),
                "elapsed_time": (datetime.utcnow() - execution_data["start_time"]).total_seconds()
            }
        else:
            # Try to get from database
            execution = await self.mission_service.get_mission_execution(execution_id)
            if not execution:
                return None
            
            return {
                "execution_id": execution_id,
                "mission_id": execution.mission_id,
                "drone_id": execution.drone_id,
                "status": execution.status,
                "current_waypoint": execution.completed_waypoints,
                "total_waypoints": execution.total_waypoints,
                "paused": False,
                "telemetry": None,
                "start_time": execution.start_time.isoformat() if execution.start_time else None,
                "end_time": execution.end_time.isoformat() if execution.end_time else None,
                "elapsed_time": (execution.end_time - execution.start_time).total_seconds() if execution.start_time and execution.end_time else None
            }
    
    async def get_active_executions(self) -> List[Dict[str, Any]]:
        """
        Get all active mission executions.
        
        Returns:
            List of active execution statuses
        """
        result = []
        
        for execution_id in self._active_executions:
            status = await self.get_execution_status(execution_id)
            if status:
                result.append(status)
        
        return result
    
    def subscribe_to_telemetry(self, execution_id: str, callback: Callable[[Dict[str, Any]], None]) -> bool:
        """
        Subscribe to telemetry updates for a mission execution.
        
        Args:
            execution_id: ID of the execution to subscribe to
            callback: Callback function to receive telemetry updates
            
        Returns:
            True if subscription was successful, False otherwise
        """
        if execution_id not in self._active_executions:
            logger.error(f"Cannot subscribe to telemetry for inactive execution {execution_id}")
            return False
        
        if execution_id not in self._telemetry_subscribers:
            self._telemetry_subscribers[execution_id] = []
        
        self._telemetry_subscribers[execution_id].append(callback)
        return True
    
    def unsubscribe_from_telemetry(self, execution_id: str, callback: Callable) -> bool:
        """
        Unsubscribe from telemetry updates for a mission execution.
        
        Args:
            execution_id: ID of the execution to unsubscribe from
            callback: Callback function to remove
            
        Returns:
            True if unsubscription was successful, False otherwise
        """
        if execution_id not in self._telemetry_subscribers:
            return False
        
        if callback in self._telemetry_subscribers[execution_id]:
            self._telemetry_subscribers[execution_id].remove(callback)
            return True
        
        return False
    
    async def _execute_mission_task(self, execution_id: str):
        """
        Background task for executing a mission.
        
        Args:
            execution_id: ID of the execution to run
        """
        if execution_id not in self._active_executions:
            logger.error(f"Execution {execution_id} not found")
            return
        
        execution_data = self._active_executions[execution_id]
        mission = execution_data["mission"]
        drone_id = execution_data["execution"].drone_id
        waypoints = execution_data["waypoints"]
        
        try:
            # Prepare mission
            logger.info(f"Preparing mission {mission.id} for execution {execution_id}")
            
            # Upload waypoints to drone
            await self.drone_command_hub.upload_mission(drone_id, [wp.dict() for wp in waypoints])
            
            # Start mission
            logger.info(f"Starting mission {mission.id} for execution {execution_id}")
            await self.drone_command_hub.start_mission(drone_id)
            
            # Monitor mission progress
            while execution_id in self._active_executions:
                # Check if paused
                if execution_data["paused"]:
                    await asyncio.sleep(1)
                    continue
                
                # Get current waypoint index from drone
                current_index = await self.drone_command_hub.get_current_waypoint_index(drone_id)
                if current_index is not None:
                    # Update current waypoint index
                    old_index = execution_data["current_waypoint_index"]
                    execution_data["current_waypoint_index"] = current_index
                    
                    # Check if waypoint changed
                    if current_index > old_index:
                        # Update completed waypoints
                        await self.mission_service.update_mission_execution(
                            execution_id,
                            {"completed_waypoints": current_index}
                        )
                        
                        logger.info(f"Reached waypoint {current_index} in execution {execution_id}")
                
                # Check if mission is complete
                mission_complete = await self.drone_command_hub.is_mission_complete(drone_id)
                if mission_complete:
                    logger.info(f"Mission {mission.id} completed for execution {execution_id}")
                    
                    # Update execution record
                    await self.mission_service.update_mission_execution(
                        execution_id,
                        {
                            "status": MissionExecutionStatus.COMPLETED.value,
                            "end_time": datetime.utcnow(),
                            "completed_waypoints": len(waypoints),
                            "notes": "Mission completed successfully"
                        }
                    )
                    
                    # Remove from active executions
                    del self._active_executions[execution_id]
                    break
                
                await asyncio.sleep(1)
        
        except Exception as e:
            logger.error(f"Error executing mission {mission.id}: {str(e)}")
            
            # Update execution record
            await self.mission_service.update_mission_execution(
                execution_id,
                {
                    "status": MissionExecutionStatus.FAILED.value,
                    "end_time": datetime.utcnow(),
                    "notes": f"Mission failed: {str(e)}"
                }
            )
            
            # Remove from active executions
            if execution_id in self._active_executions:
                del self._active_executions[execution_id]
    
    async def _monitor_telemetry(self):
        """Background task for monitoring telemetry from all active drones."""
        while self._running:
            try:
                # Get active executions
                for execution_id, execution_data in list(self._active_executions.items()):
                    drone_id = execution_data["execution"].drone_id
                    
                    # Get telemetry
                    telemetry = await self.drone_command_hub.get_telemetry(drone_id)
                    if telemetry:
                        # Add timestamp
                        telemetry_with_time = {
                            "timestamp": datetime.utcnow().isoformat(),
                            **telemetry
                        }
                        
                        # Add to telemetry log
                        execution_data["telemetry_log"].append(telemetry_with_time)
                        
                        # Limit log size
                        if len(execution_data["telemetry_log"]) > 1000:
                            execution_data["telemetry_log"] = execution_data["telemetry_log"][-1000:]
                        
                        # Update last update time
                        execution_data["last_update_time"] = datetime.utcnow()
                        
                        # Update current position in execution record
                        if "position" in telemetry:
                            position = Position(
                                latitude=telemetry["position"]["latitude"],
                                longitude=telemetry["position"]["longitude"],
                                altitude=telemetry["position"]["altitude"]
                            )
                            
                            await self.mission_service.update_mission_execution(
                                execution_id,
                                {"current_position": position.dict()}
                            )
                        
                        # Update battery level
                        if "battery" in telemetry:
                            battery_level = telemetry["battery"]["percentage"]
                            
                            # Update battery start if not set
                            execution = execution_data["execution"]
                            if execution.battery_start is None:
                                await self.mission_service.update_mission_execution(
                                    execution_id,
                                    {"battery_start": battery_level}
                                )
                            
                            # Update battery end
                            await self.mission_service.update_mission_execution(
                                execution_id,
                                {"battery_end": battery_level}
                            )
                        
                        # Update max altitude and speed
                        if "position" in telemetry and "altitude" in telemetry["position"]:
                            altitude = telemetry["position"]["altitude"]
                            execution = execution_data["execution"]
                            
                            if execution.max_altitude is None or altitude > execution.max_altitude:
                                await self.mission_service.update_mission_execution(
                                    execution_id,
                                    {"max_altitude": altitude}
                                )
                        
                        if "speed" in telemetry and "horizontal" in telemetry["speed"]:
                            speed = telemetry["speed"]["horizontal"]
                            execution = execution_data["execution"]
                            
                            if execution.max_speed is None or speed > execution.max_speed:
                                await self.mission_service.update_mission_execution(
                                    execution_id,
                                    {"max_speed": speed}
                                )
                        
                        # Notify subscribers
                        if execution_id in self._telemetry_subscribers:
                            for callback in self._telemetry_subscribers[execution_id]:
                                try:
                                    callback(telemetry_with_time)
                                except Exception as e:
                                    logger.error(f"Error in telemetry subscriber: {str(e)}")
            
            except Exception as e:
                logger.error(f"Error monitoring telemetry: {str(e)}")
            
            await asyncio.sleep(1)
