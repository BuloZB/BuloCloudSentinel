"""
NOTAM service for the Remote ID & Regulatory Compliance Service.

This module provides functionality for managing NOTAMs and checking flight
plans against them.
"""

import asyncio
import logging
from datetime import datetime, timedelta
from typing import Dict, List, Optional, Any, Tuple, Union
from uuid import UUID

from sqlalchemy import select, func, delete, update
from sqlalchemy.ext.asyncio import AsyncSession
from geoalchemy2.functions import ST_GeomFromGeoJSON, ST_SetSRID, ST_Intersects

from remoteid_service.api.schemas.notams import (
    NOTAMResponse,
    NOTAMImportResponse,
    FlightPlanNOTAMsResponse,
    NOTAMConflict,
    NOTAMSource,
    NOTAMType,
    NOTAMStatus,
)
from remoteid_service.adapters.notam_aixm import NOTAMAIXMAdapter
from remoteid_service.core.settings import Settings
from remoteid_service.db.models import NOTAM as NOTAMModel
from remoteid_service.db.models import FlightPlan as FlightPlanModel

# Configure logging
logger = logging.getLogger(__name__)

class NOTAMService:
    """
    NOTAM service.
    
    This service manages NOTAMs and provides functionality for checking
    flight plans against them.
    """
    
    def __init__(self, settings: Settings):
        """
        Initialize the NOTAM service.
        
        Args:
            settings: Application settings
        """
        self.settings = settings
        self.adapters = {}
        self.update_task = None
        self.running = False
    
    async def start(self) -> None:
        """
        Start the NOTAM service.
        """
        if self.running:
            return
        
        self.running = True
        
        # Initialize adapters
        for source in self.settings.NOTAM_SOURCES:
            self.adapters[source] = NOTAMAIXMAdapter(source)
        
        # Start update task
        self.update_task = asyncio.create_task(self._update_loop())
        logger.info("Started NOTAM service")
    
    async def stop(self) -> None:
        """
        Stop the NOTAM service.
        """
        if not self.running:
            return
        
        self.running = False
        
        # Stop update task
        if self.update_task:
            self.update_task.cancel()
            try:
                await self.update_task
            except asyncio.CancelledError:
                pass
            self.update_task = None
        
        # Clean up adapters
        self.adapters = {}
        
        logger.info("Stopped NOTAM service")
    
    async def _update_loop(self) -> None:
        """
        Update loop for the NOTAM service.
        
        This method runs in a background task and periodically updates
        NOTAMs from all configured sources.
        """
        while self.running:
            try:
                # Update NOTAMs from all sources
                for source in self.settings.NOTAM_SOURCES:
                    try:
                        # Create async session
                        async with AsyncSession() as session:
                            # Import NOTAMs
                            await self.import_notams(
                                source=source,
                                db=session,
                            )
                    except Exception as e:
                        logger.error(f"Error updating NOTAMs from {source}: {str(e)}")
                
                # Wait for next update interval
                await asyncio.sleep(self.settings.NOTAM_UPDATE_INTERVAL)
            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error(f"Error in update loop: {str(e)}")
                await asyncio.sleep(60)  # Avoid tight loop on error
    
    async def import_notams(
        self,
        source: NOTAMSource,
        region: Optional[str] = None,
        start_time: Optional[datetime] = None,
        end_time: Optional[datetime] = None,
        location_code: Optional[str] = None,
        force_update: bool = False,
        db: AsyncSession = None,
    ) -> NOTAMImportResponse:
        """
        Import NOTAMs from a source.
        
        Args:
            source: Source
            region: Region
            start_time: Start time
            end_time: End time
            location_code: Location code
            force_update: Force update
            db: Database session
            
        Returns:
            NOTAMImportResponse: Import response
        """
        try:
            # Check if adapter exists
            if source not in self.adapters:
                raise ValueError(f"NOTAM source {source} not supported")
            
            # Get adapter
            adapter = self.adapters[source]
            
            # Fetch NOTAMs
            notams = await adapter.fetch_notams(
                region=region,
                start_time=start_time,
                end_time=end_time,
                location_code=location_code,
            )
            
            # Import NOTAMs
            imported_count = 0
            updated_count = 0
            skipped_count = 0
            failed_count = 0
            
            for notam_data in notams:
                try:
                    # Check if NOTAM already exists
                    result = await db.execute(
                        select(NOTAMModel).where(NOTAMModel.notam_id == notam_data["notam_id"])
                    )
                    
                    existing_notam = result.scalars().first()
                    
                    if existing_notam and not force_update:
                        # Skip if not forcing update
                        skipped_count += 1
                        continue
                    
                    # Create or update NOTAM
                    if existing_notam:
                        # Update existing NOTAM
                        for key, value in notam_data.items():
                            if key != "notam_id":
                                setattr(existing_notam, key, value)
                        
                        existing_notam.updated_at = datetime.utcnow()
                        updated_count += 1
                    else:
                        # Create new NOTAM
                        notam = NOTAMModel(**notam_data)
                        db.add(notam)
                        imported_count += 1
                except Exception as e:
                    logger.error(f"Error importing NOTAM {notam_data.get('notam_id')}: {str(e)}")
                    failed_count += 1
            
            # Commit changes
            await db.commit()
            
            logger.info(f"Imported {imported_count} NOTAMs, updated {updated_count}, skipped {skipped_count}, failed {failed_count}")
            
            return NOTAMImportResponse(
                imported_count=imported_count,
                updated_count=updated_count,
                skipped_count=skipped_count,
                failed_count=failed_count,
                details={
                    "source": source,
                    "region": region,
                    "location_code": location_code,
                    "force_update": force_update,
                },
            )
        except Exception as e:
            await db.rollback()
            logger.error(f"Error importing NOTAMs: {str(e)}")
            raise
    
    async def get_notam(
        self,
        notam_id: UUID,
        db: AsyncSession,
    ) -> Optional[NOTAMResponse]:
        """
        Get a NOTAM by ID.
        
        Args:
            notam_id: NOTAM ID
            db: Database session
            
        Returns:
            Optional[NOTAMResponse]: NOTAM or None if not found
        """
        try:
            # Get NOTAM
            result = await db.execute(
                select(NOTAMModel).where(NOTAMModel.id == notam_id)
            )
            
            notam = result.scalars().first()
            
            if not notam:
                return None
            
            # Convert to response model
            response = self._notam_to_response(notam)
            
            return response
        except Exception as e:
            logger.error(f"Error getting NOTAM: {str(e)}")
            raise
    
    async def search_notams(
        self,
        source: Optional[NOTAMSource] = None,
        notam_type: Optional[NOTAMType] = None,
        location_code: Optional[str] = None,
        status: Optional[NOTAMStatus] = None,
        effective_start_from: Optional[datetime] = None,
        effective_start_to: Optional[datetime] = None,
        effective_end_from: Optional[datetime] = None,
        effective_end_to: Optional[datetime] = None,
        area: Optional[Dict[str, Any]] = None,
        point: Optional[Dict[str, Any]] = None,
        radius: Optional[float] = None,
        limit: Optional[int] = 100,
        offset: Optional[int] = 0,
        db: AsyncSession = None,
    ) -> Tuple[List[NOTAMResponse], int]:
        """
        Search for NOTAMs.
        
        Args:
            source: Source
            notam_type: NOTAM type
            location_code: Location code
            status: Status
            effective_start_from: Effective start from
            effective_start_to: Effective start to
            effective_end_from: Effective end from
            effective_end_to: Effective end to
            area: Area
            point: Point
            radius: Radius
            limit: Limit
            offset: Offset
            db: Database session
            
        Returns:
            Tuple[List[NOTAMResponse], int]: NOTAMs and total count
        """
        try:
            # Build query
            query = select(NOTAMModel)
            count_query = select(func.count()).select_from(NOTAMModel)
            
            # Add filters
            if source:
                query = query.where(NOTAMModel.source == source)
                count_query = count_query.where(NOTAMModel.source == source)
            
            if notam_type:
                query = query.where(NOTAMModel.notam_type == notam_type)
                count_query = count_query.where(NOTAMModel.notam_type == notam_type)
            
            if location_code:
                query = query.where(NOTAMModel.location == location_code)
                count_query = count_query.where(NOTAMModel.location == location_code)
            
            if status:
                # Calculate status based on effective dates
                now = datetime.utcnow()
                
                if status == NOTAMStatus.ACTIVE:
                    query = query.where(NOTAMModel.effective_start <= now)
                    query = query.where(NOTAMModel.effective_end >= now)
                    count_query = count_query.where(NOTAMModel.effective_start <= now)
                    count_query = count_query.where(NOTAMModel.effective_end >= now)
                elif status == NOTAMStatus.INACTIVE:
                    query = query.where(NOTAMModel.effective_end < now)
                    count_query = count_query.where(NOTAMModel.effective_end < now)
                elif status == NOTAMStatus.UPCOMING:
                    query = query.where(NOTAMModel.effective_start > now)
                    count_query = count_query.where(NOTAMModel.effective_start > now)
            
            if effective_start_from:
                query = query.where(NOTAMModel.effective_start >= effective_start_from)
                count_query = count_query.where(NOTAMModel.effective_start >= effective_start_from)
            
            if effective_start_to:
                query = query.where(NOTAMModel.effective_start <= effective_start_to)
                count_query = count_query.where(NOTAMModel.effective_start <= effective_start_to)
            
            if effective_end_from:
                query = query.where(NOTAMModel.effective_end >= effective_end_from)
                count_query = count_query.where(NOTAMModel.effective_end >= effective_end_from)
            
            if effective_end_to:
                query = query.where(NOTAMModel.effective_end <= effective_end_to)
                count_query = count_query.where(NOTAMModel.effective_end <= effective_end_to)
            
            # Add spatial filters
            if area and area.get("points"):
                # Create polygon from area
                polygon = self._create_geography_from_polygon(area)
                
                # Add spatial filter
                query = query.where(ST_Intersects(NOTAMModel.area, polygon))
                count_query = count_query.where(ST_Intersects(NOTAMModel.area, polygon))
            
            if point and radius:
                # Create point from position
                point_geom = self._create_geography_from_position(point)
                
                # Add spatial filter
                query = query.where(ST_DWithin(NOTAMModel.area, point_geom, radius))
                count_query = count_query.where(ST_DWithin(NOTAMModel.area, point_geom, radius))
            
            # Add order by
            query = query.order_by(NOTAMModel.effective_start.desc())
            
            # Add limit and offset
            if limit is not None:
                query = query.limit(limit)
            
            if offset is not None:
                query = query.offset(offset)
            
            # Execute queries
            result = await db.execute(query)
            count_result = await db.execute(count_query)
            
            # Get results
            notams = result.scalars().all()
            total = count_result.scalar()
            
            # Convert to response models
            responses = []
            for notam in notams:
                response = self._notam_to_response(notam)
                responses.append(response)
            
            return responses, total
        except Exception as e:
            logger.error(f"Error searching NOTAMs: {str(e)}")
            raise
