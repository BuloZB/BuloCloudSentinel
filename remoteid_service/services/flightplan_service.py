"""
Flight plan service for the Remote ID & Regulatory Compliance Service.

This module provides functionality for managing flight plans and submitting
them to regulatory authorities.
"""

import asyncio
import logging
import uuid
from datetime import datetime, timedelta
from typing import Dict, List, Optional, Any, Tuple, Union
from uuid import UUID

from sqlalchemy import select, func, delete, update
from sqlalchemy.ext.asyncio import AsyncSession
from geoalchemy2.functions import ST_GeomFromGeoJSON, ST_SetSRID

from remoteid_service.api.schemas.flightplans import (
    CreateFlightPlanRequest,
    FlightPlanResponse,
    FlightPlanStatus,
    FlightPlanSubmissionResponse,
    FlightPlanType,
    GeoPolygon,
    GeoPath,
    Waypoint,
)
from remoteid_service.adapters.easa_sora import EASASoraAdapter
from remoteid_service.adapters.faa_laanc import FAALaancAdapter
from remoteid_service.core.settings import Settings
from remoteid_service.db.models import FlightPlan as FlightPlanModel
from remoteid_service.db.models import Waypoint as WaypointModel

# Configure logging
logger = logging.getLogger(__name__)

class FlightPlanService:
    """
    Flight plan service.
    
    This service manages flight plans and provides functionality for
    submitting them to regulatory authorities.
    """
    
    def __init__(self, settings: Settings):
        """
        Initialize the flight plan service.
        
        Args:
            settings: Application settings
        """
        self.settings = settings
        self.easa_adapter = None
        self.faa_adapter = None
        self.running = False
    
    async def start(self) -> None:
        """
        Start the flight plan service.
        """
        if self.running:
            return
        
        self.running = True
        
        # Initialize adapters
        if self.settings.EASA_API_URL:
            self.easa_adapter = EASASoraAdapter(
                api_url=self.settings.EASA_API_URL,
                api_key=self.settings.EASA_API_KEY,
            )
        
        if self.settings.FAA_LAANC_API_URL:
            self.faa_adapter = FAALaancAdapter(
                api_url=self.settings.FAA_LAANC_API_URL,
                api_key=self.settings.FAA_LAANC_API_KEY,
            )
        
        logger.info("Started flight plan service")
    
    async def stop(self) -> None:
        """
        Stop the flight plan service.
        """
        if not self.running:
            return
        
        self.running = False
        
        # Clean up adapters
        self.easa_adapter = None
        self.faa_adapter = None
        
        logger.info("Stopped flight plan service")
    
    async def create_flight_plan(
        self,
        name: str,
        description: Optional[str],
        operator_id: str,
        drone_id: str,
        plan_type: FlightPlanType,
        start_time: datetime,
        end_time: datetime,
        max_altitude: float,
        area: GeoPolygon,
        path: Optional[GeoPath],
        waypoints: List[Waypoint],
        metadata: Optional[Dict[str, Any]],
        db: AsyncSession,
    ) -> FlightPlanResponse:
        """
        Create a new flight plan.
        
        Args:
            name: Name
            description: Description
            operator_id: Operator ID
            drone_id: Drone ID
            plan_type: Plan type
            start_time: Start time
            end_time: End time
            max_altitude: Maximum altitude
            area: Area
            path: Path
            waypoints: Waypoints
            metadata: Metadata
            db: Database session
            
        Returns:
            FlightPlanResponse: Created flight plan
        """
        try:
            # Create flight plan
            flight_plan = FlightPlanModel(
                id=uuid.uuid4(),
                name=name,
                description=description,
                operator_id=operator_id,
                drone_id=drone_id,
                plan_type=plan_type,
                status=FlightPlanStatus.DRAFT,
                start_time=start_time,
                end_time=end_time,
                max_altitude=max_altitude,
                area=self._create_geography_from_polygon(area),
                path=self._create_geography_from_path(path) if path else None,
                metadata=metadata,
                created_at=datetime.utcnow(),
                updated_at=datetime.utcnow(),
            )
            
            # Add to session
            db.add(flight_plan)
            await db.flush()
            
            # Create waypoints
            for waypoint_data in waypoints:
                waypoint = WaypointModel(
                    id=uuid.uuid4(),
                    flight_plan_id=flight_plan.id,
                    sequence=waypoint_data.sequence,
                    location=self._create_geography_from_position(waypoint_data.position),
                    altitude=waypoint_data.position.altitude,
                    speed=waypoint_data.speed,
                    hold_time=waypoint_data.hold_time,
                    action=waypoint_data.action,
                    parameters=waypoint_data.parameters,
                )
                
                db.add(waypoint)
            
            # Commit changes
            await db.commit()
            await db.refresh(flight_plan)
            
            # Convert to response model
            response = await self._flight_plan_to_response(flight_plan, db)
            
            logger.info(f"Created flight plan {flight_plan.id}")
            
            return response
        except Exception as e:
            await db.rollback()
            logger.error(f"Error creating flight plan: {str(e)}")
            raise
    
    async def get_flight_plan(
        self,
        flight_plan_id: UUID,
        db: AsyncSession,
    ) -> Optional[FlightPlanResponse]:
        """
        Get a flight plan by ID.
        
        Args:
            flight_plan_id: Flight plan ID
            db: Database session
            
        Returns:
            Optional[FlightPlanResponse]: Flight plan or None if not found
        """
        try:
            # Get flight plan
            result = await db.execute(
                select(FlightPlanModel).where(FlightPlanModel.id == flight_plan_id)
            )
            
            flight_plan = result.scalars().first()
            
            if not flight_plan:
                return None
            
            # Convert to response model
            response = await self._flight_plan_to_response(flight_plan, db)
            
            return response
        except Exception as e:
            logger.error(f"Error getting flight plan: {str(e)}")
            raise
    
    async def update_flight_plan(
        self,
        flight_plan_id: UUID,
        name: Optional[str],
        description: Optional[str],
        start_time: Optional[datetime],
        end_time: Optional[datetime],
        max_altitude: Optional[float],
        area: Optional[GeoPolygon],
        path: Optional[GeoPath],
        waypoints: Optional[List[Waypoint]],
        metadata: Optional[Dict[str, Any]],
        db: AsyncSession,
    ) -> Optional[FlightPlanResponse]:
        """
        Update a flight plan.
        
        Args:
            flight_plan_id: Flight plan ID
            name: Name
            description: Description
            start_time: Start time
            end_time: End time
            max_altitude: Maximum altitude
            area: Area
            path: Path
            waypoints: Waypoints
            metadata: Metadata
            db: Database session
            
        Returns:
            Optional[FlightPlanResponse]: Updated flight plan or None if not found
        """
        try:
            # Get flight plan
            result = await db.execute(
                select(FlightPlanModel).where(FlightPlanModel.id == flight_plan_id)
            )
            
            flight_plan = result.scalars().first()
            
            if not flight_plan:
                return None
            
            # Check if flight plan can be updated
            if flight_plan.status not in [FlightPlanStatus.DRAFT, FlightPlanStatus.REJECTED]:
                raise ValueError(f"Cannot update flight plan with status {flight_plan.status}")
            
            # Update fields
            if name is not None:
                flight_plan.name = name
            
            if description is not None:
                flight_plan.description = description
            
            if start_time is not None:
                flight_plan.start_time = start_time
            
            if end_time is not None:
                flight_plan.end_time = end_time
            
            if max_altitude is not None:
                flight_plan.max_altitude = max_altitude
            
            if area is not None:
                flight_plan.area = self._create_geography_from_polygon(area)
            
            if path is not None:
                flight_plan.path = self._create_geography_from_path(path)
            
            if metadata is not None:
                flight_plan.metadata = metadata
            
            flight_plan.updated_at = datetime.utcnow()
            
            # Update waypoints if provided
            if waypoints is not None:
                # Delete existing waypoints
                await db.execute(
                    delete(WaypointModel).where(WaypointModel.flight_plan_id == flight_plan_id)
                )
                
                # Create new waypoints
                for waypoint_data in waypoints:
                    waypoint = WaypointModel(
                        id=uuid.uuid4(),
                        flight_plan_id=flight_plan.id,
                        sequence=waypoint_data.sequence,
                        location=self._create_geography_from_position(waypoint_data.position),
                        altitude=waypoint_data.position.altitude,
                        speed=waypoint_data.speed,
                        hold_time=waypoint_data.hold_time,
                        action=waypoint_data.action,
                        parameters=waypoint_data.parameters,
                    )
                    
                    db.add(waypoint)
            
            # Commit changes
            await db.commit()
            await db.refresh(flight_plan)
            
            # Convert to response model
            response = await self._flight_plan_to_response(flight_plan, db)
            
            logger.info(f"Updated flight plan {flight_plan.id}")
            
            return response
        except Exception as e:
            await db.rollback()
            logger.error(f"Error updating flight plan: {str(e)}")
            raise
    
    async def delete_flight_plan(
        self,
        flight_plan_id: UUID,
        db: AsyncSession,
    ) -> bool:
        """
        Delete a flight plan.
        
        Args:
            flight_plan_id: Flight plan ID
            db: Database session
            
        Returns:
            bool: True if deleted, False if not found
        """
        try:
            # Get flight plan
            result = await db.execute(
                select(FlightPlanModel).where(FlightPlanModel.id == flight_plan_id)
            )
            
            flight_plan = result.scalars().first()
            
            if not flight_plan:
                return False
            
            # Check if flight plan can be deleted
            if flight_plan.status not in [FlightPlanStatus.DRAFT, FlightPlanStatus.REJECTED, FlightPlanStatus.CANCELLED, FlightPlanStatus.COMPLETED]:
                raise ValueError(f"Cannot delete flight plan with status {flight_plan.status}")
            
            # Delete waypoints
            await db.execute(
                delete(WaypointModel).where(WaypointModel.flight_plan_id == flight_plan_id)
            )
            
            # Delete flight plan
            await db.delete(flight_plan)
            
            # Commit changes
            await db.commit()
            
            logger.info(f"Deleted flight plan {flight_plan_id}")
            
            return True
        except Exception as e:
            await db.rollback()
            logger.error(f"Error deleting flight plan: {str(e)}")
            raise
    
    async def search_flight_plans(
        self,
        operator_id: Optional[str],
        drone_id: Optional[str],
        status: Optional[FlightPlanStatus],
        start_time_from: Optional[datetime],
        start_time_to: Optional[datetime],
        limit: Optional[int],
        offset: Optional[int],
        db: AsyncSession,
    ) -> Tuple[List[FlightPlanResponse], int]:
        """
        Search for flight plans.
        
        Args:
            operator_id: Operator ID
            drone_id: Drone ID
            status: Status
            start_time_from: Start time from
            start_time_to: Start time to
            limit: Limit
            offset: Offset
            db: Database session
            
        Returns:
            Tuple[List[FlightPlanResponse], int]: Flight plans and total count
        """
        try:
            # Build query
            query = select(FlightPlanModel)
            count_query = select(func.count()).select_from(FlightPlanModel)
            
            # Add filters
            if operator_id:
                query = query.where(FlightPlanModel.operator_id == operator_id)
                count_query = count_query.where(FlightPlanModel.operator_id == operator_id)
            
            if drone_id:
                query = query.where(FlightPlanModel.drone_id == drone_id)
                count_query = count_query.where(FlightPlanModel.drone_id == drone_id)
            
            if status:
                query = query.where(FlightPlanModel.status == status)
                count_query = count_query.where(FlightPlanModel.status == status)
            
            if start_time_from:
                query = query.where(FlightPlanModel.start_time >= start_time_from)
                count_query = count_query.where(FlightPlanModel.start_time >= start_time_from)
            
            if start_time_to:
                query = query.where(FlightPlanModel.start_time <= start_time_to)
                count_query = count_query.where(FlightPlanModel.start_time <= start_time_to)
            
            # Add order by
            query = query.order_by(FlightPlanModel.start_time.desc())
            
            # Add limit and offset
            if limit is not None:
                query = query.limit(limit)
            
            if offset is not None:
                query = query.offset(offset)
            
            # Execute queries
            result = await db.execute(query)
            count_result = await db.execute(count_query)
            
            # Get results
            flight_plans = result.scalars().all()
            total = count_result.scalar()
            
            # Convert to response models
            responses = []
            for flight_plan in flight_plans:
                response = await self._flight_plan_to_response(flight_plan, db)
                responses.append(response)
            
            return responses, total
        except Exception as e:
            logger.error(f"Error searching flight plans: {str(e)}")
            raise
