"""
Enhanced mission planning service for Bulo.Cloud Sentinel.

This module provides comprehensive services for creating, managing, and executing
various mission types including waypoint, mapping, orbit, and facade missions.
"""

import asyncio
import logging
import json
from typing import List, Dict, Any, Optional, Union, Tuple
from datetime import datetime
from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy.future import select
from sqlalchemy import update, delete

from backend.mission_planning.models import (
    Mission, MissionType, Waypoint, Position, CameraAction, 
    MissionTemplate, MissionExecution, MissionExecutionStatus,
    MappingSettings, OrbitSettings, FacadeSettings, PanoSettings, HoverSettings
)
from backend.infrastructure.persistence.models import (
    MissionModel, WaypointModel, MissionTemplateModel, MissionExecutionModel
)
from backend.mission_planning.terrain_service import TerrainService
from backend.mission_planning.path_planning import PathPlanner

logger = logging.getLogger(__name__)


class MissionPlanningService:
    """
    Service for mission planning operations.
    
    This service handles the creation, management, and execution of various
    mission types, including waypoint, mapping, orbit, and facade missions.
    """
    
    def __init__(self, db_session: AsyncSession, terrain_service: Optional[TerrainService] = None):
        """
        Initialize the mission planning service.
        
        Args:
            db_session: Database session for persistence
            terrain_service: Optional terrain service for elevation data
        """
        self.db = db_session
        self.terrain_service = terrain_service or TerrainService()
        self.path_planner = PathPlanner()
        self._active_missions: Dict[str, MissionExecution] = {}
    
    async def create_mission(self, mission: Mission) -> Mission:
        """
        Create a new mission.
        
        Args:
            mission: Mission to create
            
        Returns:
            Created mission with ID
        """
        # Convert to database model
        mission_model = MissionModel(
            id=mission.id,
            name=mission.name,
            description=mission.description,
            mission_type=mission.mission_type.value,
            settings=mission.settings.dict(),
            boundary_points=json.dumps([p.dict() for p in mission.boundary_points]) if mission.boundary_points else None,
            home_position=json.dumps(mission.home_position.dict()) if mission.home_position else None,
            max_altitude=mission.max_altitude,
            max_distance=mission.max_distance,
            auto_takeoff=mission.auto_takeoff,
            auto_land=mission.auto_land,
            fail_safe_return_home=mission.fail_safe_return_home,
            terrain_following=mission.terrain_following.value,
            notes=mission.notes,
            tags=json.dumps(mission.tags),
            is_template=mission.is_template,
            template_category=mission.template_category,
            created_at=mission.created_at,
            updated_at=mission.updated_at
        )
        
        self.db.add(mission_model)
        await self.db.commit()
        await self.db.refresh(mission_model)
        
        # Return the created mission
        return await self._convert_to_mission(mission_model)
    
    async def get_mission(self, mission_id: str) -> Optional[Mission]:
        """
        Get a mission by ID.
        
        Args:
            mission_id: ID of the mission to get
            
        Returns:
            Mission if found, None otherwise
        """
        result = await self.db.execute(select(MissionModel).where(MissionModel.id == mission_id))
        mission_model = result.scalars().first()
        
        if not mission_model:
            return None
        
        return await self._convert_to_mission(mission_model)
    
    async def update_mission(self, mission_id: str, mission: Mission) -> Optional[Mission]:
        """
        Update an existing mission.
        
        Args:
            mission_id: ID of the mission to update
            mission: Updated mission data
            
        Returns:
            Updated mission if found, None otherwise
        """
        # Check if mission exists
        result = await self.db.execute(select(MissionModel).where(MissionModel.id == mission_id))
        mission_model = result.scalars().first()
        
        if not mission_model:
            return None
        
        # Update mission
        mission_model.name = mission.name
        mission_model.description = mission.description
        mission_model.mission_type = mission.mission_type.value
        mission_model.settings = mission.settings.dict()
        mission_model.boundary_points = json.dumps([p.dict() for p in mission.boundary_points]) if mission.boundary_points else None
        mission_model.home_position = json.dumps(mission.home_position.dict()) if mission.home_position else None
        mission_model.max_altitude = mission.max_altitude
        mission_model.max_distance = mission.max_distance
        mission_model.auto_takeoff = mission.auto_takeoff
        mission_model.auto_land = mission.auto_land
        mission_model.fail_safe_return_home = mission.fail_safe_return_home
        mission_model.terrain_following = mission.terrain_following.value
        mission_model.notes = mission.notes
        mission_model.tags = json.dumps(mission.tags)
        mission_model.is_template = mission.is_template
        mission_model.template_category = mission.template_category
        mission_model.updated_at = datetime.utcnow()
        
        await self.db.commit()
        await self.db.refresh(mission_model)
        
        # Return the updated mission
        return await self._convert_to_mission(mission_model)
    
    async def delete_mission(self, mission_id: str) -> bool:
        """
        Delete a mission.
        
        Args:
            mission_id: ID of the mission to delete
            
        Returns:
            True if mission was deleted, False otherwise
        """
        # Check if mission exists
        result = await self.db.execute(select(MissionModel).where(MissionModel.id == mission_id))
        mission_model = result.scalars().first()
        
        if not mission_model:
            return False
        
        # Delete mission
        await self.db.delete(mission_model)
        await self.db.commit()
        
        return True
    
    async def list_missions(self, 
                           mission_type: Optional[MissionType] = None, 
                           is_template: bool = False,
                           tags: Optional[List[str]] = None) -> List[Mission]:
        """
        List missions with optional filtering.
        
        Args:
            mission_type: Optional mission type filter
            is_template: Filter for templates
            tags: Optional tags filter
            
        Returns:
            List of missions matching the filters
        """
        query = select(MissionModel).where(MissionModel.is_template == is_template)
        
        if mission_type:
            query = query.where(MissionModel.mission_type == mission_type.value)
        
        if tags:
            # This is a simplistic approach - in a real implementation, you'd use a proper JSON query
            for tag in tags:
                query = query.where(MissionModel.tags.contains(tag))
        
        result = await self.db.execute(query)
        mission_models = result.scalars().all()
        
        missions = []
        for mission_model in mission_models:
            mission = await self._convert_to_mission(mission_model)
            missions.append(mission)
        
        return missions
    
    async def create_template(self, template: MissionTemplate) -> MissionTemplate:
        """
        Create a new mission template.
        
        Args:
            template: Template to create
            
        Returns:
            Created template with ID
        """
        # Convert to database model
        template_model = MissionTemplateModel(
            id=template.id,
            name=template.name,
            description=template.description,
            mission_type=template.mission_type.value,
            category=template.category,
            thumbnail_url=template.thumbnail_url,
            settings=template.settings.dict(),
            tags=json.dumps(template.tags)
        )
        
        self.db.add(template_model)
        await self.db.commit()
        await self.db.refresh(template_model)
        
        # Return the created template
        return await self._convert_to_template(template_model)
    
    async def get_template(self, template_id: str) -> Optional[MissionTemplate]:
        """
        Get a template by ID.
        
        Args:
            template_id: ID of the template to get
            
        Returns:
            Template if found, None otherwise
        """
        result = await self.db.execute(select(MissionTemplateModel).where(MissionTemplateModel.id == template_id))
        template_model = result.scalars().first()
        
        if not template_model:
            return None
        
        return await self._convert_to_template(template_model)
    
    async def list_templates(self, 
                            mission_type: Optional[MissionType] = None,
                            category: Optional[str] = None,
                            tags: Optional[List[str]] = None) -> List[MissionTemplate]:
        """
        List templates with optional filtering.
        
        Args:
            mission_type: Optional mission type filter
            category: Optional category filter
            tags: Optional tags filter
            
        Returns:
            List of templates matching the filters
        """
        query = select(MissionTemplateModel)
        
        if mission_type:
            query = query.where(MissionTemplateModel.mission_type == mission_type.value)
        
        if category:
            query = query.where(MissionTemplateModel.category == category)
        
        if tags:
            # This is a simplistic approach - in a real implementation, you'd use a proper JSON query
            for tag in tags:
                query = query.where(MissionTemplateModel.tags.contains(tag))
        
        result = await self.db.execute(query)
        template_models = result.scalars().all()
        
        templates = []
        for template_model in template_models:
            template = await self._convert_to_template(template_model)
            templates.append(template)
        
        return templates
    
    async def create_mission_from_template(self, template_id: str, name: str, description: Optional[str] = None) -> Optional[Mission]:
        """
        Create a new mission from a template.
        
        Args:
            template_id: ID of the template to use
            name: Name for the new mission
            description: Optional description for the new mission
            
        Returns:
            Created mission if template found, None otherwise
        """
        # Get template
        template = await self.get_template(template_id)
        if not template:
            return None
        
        # Create mission from template
        mission = Mission(
            name=name,
            description=description or template.description,
            mission_type=template.mission_type,
            settings=template.settings,
            tags=template.tags.copy(),
            is_template=False
        )
        
        # Save mission
        return await self.create_mission(mission)
    
    async def start_mission_execution(self, mission_id: str, drone_id: str) -> Optional[MissionExecution]:
        """
        Start executing a mission.
        
        Args:
            mission_id: ID of the mission to execute
            drone_id: ID of the drone to use
            
        Returns:
            Mission execution record if mission found, None otherwise
        """
        # Get mission
        mission = await self.get_mission(mission_id)
        if not mission:
            return None
        
        # Create execution record
        execution = MissionExecution(
            mission_id=mission_id,
            drone_id=drone_id,
            start_time=datetime.utcnow(),
            status=MissionExecutionStatus.PREPARING,
            total_waypoints=len(mission.settings.waypoint or [])
        )
        
        # Save execution record
        execution_model = MissionExecutionModel(
            id=execution.id,
            mission_id=execution.mission_id,
            drone_id=execution.drone_id,
            start_time=execution.start_time,
            status=execution.status.value,
            total_waypoints=execution.total_waypoints
        )
        
        self.db.add(execution_model)
        await self.db.commit()
        await self.db.refresh(execution_model)
        
        # Add to active missions
        self._active_missions[execution.id] = execution
        
        # Return execution record
        return execution
    
    async def update_mission_execution(self, execution_id: str, updates: Dict[str, Any]) -> Optional[MissionExecution]:
        """
        Update a mission execution record.
        
        Args:
            execution_id: ID of the execution to update
            updates: Updates to apply
            
        Returns:
            Updated execution record if found, None otherwise
        """
        # Check if execution exists
        result = await self.db.execute(select(MissionExecutionModel).where(MissionExecutionModel.id == execution_id))
        execution_model = result.scalars().first()
        
        if not execution_model:
            return None
        
        # Update execution
        for key, value in updates.items():
            if hasattr(execution_model, key):
                setattr(execution_model, key, value)
        
        await self.db.commit()
        await self.db.refresh(execution_model)
        
        # Update active mission if present
        if execution_id in self._active_missions:
            for key, value in updates.items():
                if hasattr(self._active_missions[execution_id], key):
                    setattr(self._active_missions[execution_id], key, value)
        
        # Return updated execution
        return await self._convert_to_execution(execution_model)
    
    async def get_mission_execution(self, execution_id: str) -> Optional[MissionExecution]:
        """
        Get a mission execution record.
        
        Args:
            execution_id: ID of the execution to get
            
        Returns:
            Execution record if found, None otherwise
        """
        result = await self.db.execute(select(MissionExecutionModel).where(MissionExecutionModel.id == execution_id))
        execution_model = result.scalars().first()
        
        if not execution_model:
            return None
        
        return await self._convert_to_execution(execution_model)
    
    async def list_mission_executions(self, 
                                     mission_id: Optional[str] = None,
                                     drone_id: Optional[str] = None,
                                     status: Optional[MissionExecutionStatus] = None) -> List[MissionExecution]:
        """
        List mission executions with optional filtering.
        
        Args:
            mission_id: Optional mission ID filter
            drone_id: Optional drone ID filter
            status: Optional status filter
            
        Returns:
            List of execution records matching the filters
        """
        query = select(MissionExecutionModel)
        
        if mission_id:
            query = query.where(MissionExecutionModel.mission_id == mission_id)
        
        if drone_id:
            query = query.where(MissionExecutionModel.drone_id == drone_id)
        
        if status:
            query = query.where(MissionExecutionModel.status == status.value)
        
        result = await self.db.execute(query)
        execution_models = result.scalars().all()
        
        executions = []
        for execution_model in execution_models:
            execution = await self._convert_to_execution(execution_model)
            executions.append(execution)
        
        return executions
    
    async def generate_waypoint_mission(self, waypoints: List[Position], name: str, description: Optional[str] = None) -> Mission:
        """
        Generate a waypoint mission from a list of positions.
        
        Args:
            waypoints: List of positions for waypoints
            name: Name for the mission
            description: Optional description for the mission
            
        Returns:
            Generated mission
        """
        # Create waypoints
        mission_waypoints = []
        for i, position in enumerate(waypoints):
            waypoint = Waypoint(
                position=position,
                order=i
            )
            mission_waypoints.append(waypoint)
        
        # Create mission
        mission = Mission(
            name=name,
            description=description,
            mission_type=MissionType.WAYPOINT,
            settings={
                "waypoint": mission_waypoints
            }
        )
        
        # Save mission
        return await self.create_mission(mission)
    
    async def generate_mapping_mission(self, 
                                      boundary_points: List[Position], 
                                      mapping_settings: MappingSettings,
                                      name: str, 
                                      description: Optional[str] = None) -> Mission:
        """
        Generate a mapping mission from boundary points and settings.
        
        Args:
            boundary_points: List of positions defining the mapping area
            mapping_settings: Settings for the mapping mission
            name: Name for the mission
            description: Optional description for the mission
            
        Returns:
            Generated mission
        """
        # Create mission
        mission = Mission(
            name=name,
            description=description,
            mission_type=MissionType.MAPPING,
            settings={
                "mapping": mapping_settings
            },
            boundary_points=boundary_points
        )
        
        # Generate waypoints based on mapping settings
        waypoints = await self.path_planner.generate_mapping_waypoints(boundary_points, mapping_settings)
        mission.settings.waypoint = waypoints
        
        # Save mission
        return await self.create_mission(mission)
    
    async def generate_orbit_mission(self,
                                    orbit_settings: OrbitSettings,
                                    name: str,
                                    description: Optional[str] = None) -> Mission:
        """
        Generate an orbit mission from settings.
        
        Args:
            orbit_settings: Settings for the orbit mission
            name: Name for the mission
            description: Optional description for the mission
            
        Returns:
            Generated mission
        """
        # Create mission
        mission = Mission(
            name=name,
            description=description,
            mission_type=MissionType.ORBIT,
            settings={
                "orbit": orbit_settings
            }
        )
        
        # Generate waypoints based on orbit settings
        waypoints = await self.path_planner.generate_orbit_waypoints(orbit_settings)
        mission.settings.waypoint = waypoints
        
        # Save mission
        return await self.create_mission(mission)
    
    async def generate_facade_mission(self,
                                     facade_settings: FacadeSettings,
                                     name: str,
                                     description: Optional[str] = None) -> Mission:
        """
        Generate a facade mission from settings.
        
        Args:
            facade_settings: Settings for the facade mission
            name: Name for the mission
            description: Optional description for the mission
            
        Returns:
            Generated mission
        """
        # Create mission
        mission = Mission(
            name=name,
            description=description,
            mission_type=MissionType.FACADE,
            settings={
                "facade": facade_settings
            }
        )
        
        # Generate waypoints based on facade settings
        waypoints = await self.path_planner.generate_facade_waypoints(facade_settings)
        mission.settings.waypoint = waypoints
        
        # Save mission
        return await self.create_mission(mission)
    
    async def _convert_to_mission(self, mission_model: MissionModel) -> Mission:
        """
        Convert a database mission model to a domain mission.
        
        Args:
            mission_model: Database mission model
            
        Returns:
            Domain mission
        """
        # Parse JSON fields
        settings = mission_model.settings
        boundary_points = json.loads(mission_model.boundary_points) if mission_model.boundary_points else None
        home_position = json.loads(mission_model.home_position) if mission_model.home_position else None
        tags = json.loads(mission_model.tags) if mission_model.tags else []
        
        # Convert boundary points to Position objects
        boundary_positions = None
        if boundary_points:
            boundary_positions = [Position(**point) for point in boundary_points]
        
        # Convert home position to Position object
        home_pos = None
        if home_position:
            home_pos = Position(**home_position)
        
        # Create mission
        mission = Mission(
            id=mission_model.id,
            name=mission_model.name,
            description=mission_model.description,
            mission_type=MissionType(mission_model.mission_type),
            settings=settings,
            boundary_points=boundary_positions,
            home_position=home_pos,
            max_altitude=mission_model.max_altitude,
            max_distance=mission_model.max_distance,
            auto_takeoff=mission_model.auto_takeoff,
            auto_land=mission_model.auto_land,
            fail_safe_return_home=mission_model.fail_safe_return_home,
            terrain_following=mission_model.terrain_following,
            notes=mission_model.notes,
            tags=tags,
            is_template=mission_model.is_template,
            template_category=mission_model.template_category,
            created_at=mission_model.created_at,
            updated_at=mission_model.updated_at
        )
        
        return mission
    
    async def _convert_to_template(self, template_model: MissionTemplateModel) -> MissionTemplate:
        """
        Convert a database template model to a domain template.
        
        Args:
            template_model: Database template model
            
        Returns:
            Domain template
        """
        # Parse JSON fields
        settings = template_model.settings
        tags = json.loads(template_model.tags) if template_model.tags else []
        
        # Create template
        template = MissionTemplate(
            id=template_model.id,
            name=template_model.name,
            description=template_model.description,
            mission_type=MissionType(template_model.mission_type),
            category=template_model.category,
            thumbnail_url=template_model.thumbnail_url,
            settings=settings,
            tags=tags
        )
        
        return template
    
    async def _convert_to_execution(self, execution_model: MissionExecutionModel) -> MissionExecution:
        """
        Convert a database execution model to a domain execution.
        
        Args:
            execution_model: Database execution model
            
        Returns:
            Domain execution
        """
        # Parse JSON fields
        current_position = json.loads(execution_model.current_position) if execution_model.current_position else None
        weather_conditions = json.loads(execution_model.weather_conditions) if execution_model.weather_conditions else None
        
        # Convert current position to Position object
        current_pos = None
        if current_position:
            current_pos = Position(**current_position)
        
        # Create execution
        execution = MissionExecution(
            id=execution_model.id,
            mission_id=execution_model.mission_id,
            drone_id=execution_model.drone_id,
            start_time=execution_model.start_time,
            end_time=execution_model.end_time,
            status=MissionExecutionStatus(execution_model.status),
            telemetry_log=execution_model.telemetry_log,
            completed_waypoints=execution_model.completed_waypoints,
            total_waypoints=execution_model.total_waypoints,
            current_position=current_pos,
            battery_start=execution_model.battery_start,
            battery_end=execution_model.battery_end,
            distance_traveled=execution_model.distance_traveled,
            max_altitude=execution_model.max_altitude,
            max_speed=execution_model.max_speed,
            photos_taken=execution_model.photos_taken,
            video_duration=execution_model.video_duration,
            notes=execution_model.notes,
            weather_conditions=weather_conditions
        )
        
        return execution
