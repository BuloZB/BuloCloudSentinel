"""
Choreography service for the Drone Show microservice.

This module provides services for managing choreographies.
"""

import logging
import json
from typing import List, Dict, Any, Optional
from datetime import datetime
from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy.future import select
from sqlalchemy import update, delete

from drone_show_service.models.choreography import (
    Choreography, ChoreographyCreate, ChoreographyUpdate, ChoreographyResponse,
    ChoreographyStatus, ChoreographyType, DroneTrajectory, Formation
)
from drone_show_service.models.database import ChoreographyDB
from drone_show_service.core.config import settings

logger = logging.getLogger(__name__)


class ChoreographyService:
    """
    Service for managing choreographies.
    
    This service provides methods for creating, retrieving, updating, and deleting
    choreographies.
    """
    
    async def create_choreography(self, db: AsyncSession, choreography: ChoreographyCreate) -> ChoreographyResponse:
        """
        Create a new choreography.
        
        Args:
            db: Database session
            choreography: Choreography data
            
        Returns:
            Created choreography
        """
        # Create database model
        db_choreography = ChoreographyDB(
            name=choreography.metadata.name,
            description=choreography.metadata.description,
            author=choreography.metadata.author,
            tags=choreography.metadata.tags,
            duration=choreography.metadata.duration,
            drone_count=choreography.metadata.drone_count,
            status=choreography.metadata.status,
            type=choreography.type,
            trajectories=json.loads(choreography.trajectories.json()),
            formations=json.loads(choreography.formations.json()) if choreography.formations else None,
            music_file=choreography.music_file,
            music_bpm=choreography.music_bpm,
            music_offset=choreography.music_offset,
            boundary=json.loads(choreography.boundary.json()) if choreography.boundary else None,
            home_position=json.loads(choreography.home_position.json()) if choreography.home_position else None,
            notes=choreography.notes,
        )
        
        # Add to database
        db.add(db_choreography)
        await db.commit()
        await db.refresh(db_choreography)
        
        # Convert to response model
        return self._db_to_response(db_choreography)
    
    async def get_choreography(self, db: AsyncSession, choreography_id: str) -> Optional[ChoreographyResponse]:
        """
        Get a choreography by ID.
        
        Args:
            db: Database session
            choreography_id: Choreography ID
            
        Returns:
            Choreography if found, None otherwise
        """
        # Query database
        result = await db.execute(
            select(ChoreographyDB).where(ChoreographyDB.id == choreography_id)
        )
        db_choreography = result.scalars().first()
        
        # Return response model if found
        if db_choreography:
            return self._db_to_response(db_choreography)
        
        return None
    
    async def get_choreographies(
        self, db: AsyncSession, skip: int = 0, limit: int = 100, status: Optional[str] = None
    ) -> List[ChoreographyResponse]:
        """
        Get all choreographies.
        
        Args:
            db: Database session
            skip: Number of records to skip
            limit: Maximum number of records to return
            status: Filter by status
            
        Returns:
            List of choreographies
        """
        # Build query
        query = select(ChoreographyDB)
        
        # Apply filters
        if status:
            query = query.where(ChoreographyDB.status == status)
        
        # Apply pagination
        query = query.offset(skip).limit(limit)
        
        # Execute query
        result = await db.execute(query)
        db_choreographies = result.scalars().all()
        
        # Convert to response models
        return [self._db_to_response(db_choreography) for db_choreography in db_choreographies]
    
    async def update_choreography(
        self, db: AsyncSession, choreography_id: str, choreography: ChoreographyUpdate
    ) -> Optional[ChoreographyResponse]:
        """
        Update a choreography.
        
        Args:
            db: Database session
            choreography_id: Choreography ID
            choreography: Updated choreography data
            
        Returns:
            Updated choreography if found, None otherwise
        """
        # Query database
        result = await db.execute(
            select(ChoreographyDB).where(ChoreographyDB.id == choreography_id)
        )
        db_choreography = result.scalars().first()
        
        # Return None if not found
        if not db_choreography:
            return None
        
        # Update fields
        update_data = choreography.dict(exclude_unset=True)
        
        # Handle nested fields
        if "metadata" in update_data:
            metadata = update_data.pop("metadata")
            for key, value in metadata.items():
                setattr(db_choreography, key, value)
        
        # Handle JSON fields
        for field in ["trajectories", "formations", "boundary", "home_position"]:
            if field in update_data and update_data[field] is not None:
                update_data[field] = json.loads(update_data[field].json())
        
        # Update remaining fields
        for key, value in update_data.items():
            setattr(db_choreography, key, value)
        
        # Update timestamp
        db_choreography.updated_at = datetime.utcnow()
        
        # Commit changes
        await db.commit()
        await db.refresh(db_choreography)
        
        # Return updated choreography
        return self._db_to_response(db_choreography)
    
    async def delete_choreography(self, db: AsyncSession, choreography_id: str) -> bool:
        """
        Delete a choreography.
        
        Args:
            db: Database session
            choreography_id: Choreography ID
            
        Returns:
            True if deleted, False if not found
        """
        # Query database
        result = await db.execute(
            select(ChoreographyDB).where(ChoreographyDB.id == choreography_id)
        )
        db_choreography = result.scalars().first()
        
        # Return False if not found
        if not db_choreography:
            return False
        
        # Delete from database
        await db.delete(db_choreography)
        await db.commit()
        
        return True
    
    async def update_choreography_status(
        self, db: AsyncSession, choreography_id: str, status: ChoreographyStatus
    ) -> Optional[ChoreographyResponse]:
        """
        Update a choreography's status.
        
        Args:
            db: Database session
            choreography_id: Choreography ID
            status: New status
            
        Returns:
            Updated choreography if found, None otherwise
        """
        # Query database
        result = await db.execute(
            select(ChoreographyDB).where(ChoreographyDB.id == choreography_id)
        )
        db_choreography = result.scalars().first()
        
        # Return None if not found
        if not db_choreography:
            return None
        
        # Update status
        db_choreography.status = status
        db_choreography.updated_at = datetime.utcnow()
        
        # Commit changes
        await db.commit()
        await db.refresh(db_choreography)
        
        # Return updated choreography
        return self._db_to_response(db_choreography)
    
    def _db_to_response(self, db_choreography: ChoreographyDB) -> ChoreographyResponse:
        """
        Convert a database model to a response model.
        
        Args:
            db_choreography: Database model
            
        Returns:
            Response model
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
        
        # Create response model
        return ChoreographyResponse(
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
