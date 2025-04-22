"""
API endpoints for shows.

This module provides API endpoints for managing drone show choreographies.
"""

import logging
from typing import List, Optional
from fastapi import APIRouter, Depends, HTTPException, Query, Path, status
from sqlalchemy.ext.asyncio import AsyncSession

from drone_show_service.core.database import get_db
from drone_show_service.models.choreography import (
    ChoreographyCreate, ChoreographyUpdate, ChoreographyResponse, ChoreographyStatus
)
from drone_show_service.services.choreography_service import ChoreographyService
from drone_show_service.api.main import get_choreography_service

logger = logging.getLogger(__name__)

router = APIRouter()


@router.post("/", response_model=ChoreographyResponse, status_code=status.HTTP_201_CREATED)
async def create_choreography(
    choreography: ChoreographyCreate,
    db: AsyncSession = Depends(get_db),
    choreography_service: ChoreographyService = Depends(get_choreography_service),
):
    """
    Create a new choreography.
    
    Args:
        choreography: Choreography data
        db: Database session
        choreography_service: Choreography service
        
    Returns:
        Created choreography
    """
    try:
        return await choreography_service.create_choreography(db, choreography)
    except Exception as e:
        logger.error(f"Error creating choreography: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error creating choreography: {str(e)}",
        )


@router.get("/", response_model=List[ChoreographyResponse])
async def get_choreographies(
    skip: int = Query(0, ge=0),
    limit: int = Query(100, ge=1, le=1000),
    status: Optional[str] = Query(None),
    db: AsyncSession = Depends(get_db),
    choreography_service: ChoreographyService = Depends(get_choreography_service),
):
    """
    Get all choreographies.
    
    Args:
        skip: Number of records to skip
        limit: Maximum number of records to return
        status: Filter by status
        db: Database session
        choreography_service: Choreography service
        
    Returns:
        List of choreographies
    """
    try:
        return await choreography_service.get_choreographies(db, skip, limit, status)
    except Exception as e:
        logger.error(f"Error getting choreographies: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error getting choreographies: {str(e)}",
        )


@router.get("/{choreography_id}", response_model=ChoreographyResponse)
async def get_choreography(
    choreography_id: str = Path(...),
    db: AsyncSession = Depends(get_db),
    choreography_service: ChoreographyService = Depends(get_choreography_service),
):
    """
    Get a choreography by ID.
    
    Args:
        choreography_id: Choreography ID
        db: Database session
        choreography_service: Choreography service
        
    Returns:
        Choreography
    """
    try:
        choreography = await choreography_service.get_choreography(db, choreography_id)
        if not choreography:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail=f"Choreography {choreography_id} not found",
            )
        return choreography
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error getting choreography {choreography_id}: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error getting choreography: {str(e)}",
        )


@router.put("/{choreography_id}", response_model=ChoreographyResponse)
async def update_choreography(
    choreography: ChoreographyUpdate,
    choreography_id: str = Path(...),
    db: AsyncSession = Depends(get_db),
    choreography_service: ChoreographyService = Depends(get_choreography_service),
):
    """
    Update a choreography.
    
    Args:
        choreography: Updated choreography data
        choreography_id: Choreography ID
        db: Database session
        choreography_service: Choreography service
        
    Returns:
        Updated choreography
    """
    try:
        updated_choreography = await choreography_service.update_choreography(db, choreography_id, choreography)
        if not updated_choreography:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail=f"Choreography {choreography_id} not found",
            )
        return updated_choreography
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error updating choreography {choreography_id}: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error updating choreography: {str(e)}",
        )


@router.delete("/{choreography_id}", status_code=status.HTTP_204_NO_CONTENT)
async def delete_choreography(
    choreography_id: str = Path(...),
    db: AsyncSession = Depends(get_db),
    choreography_service: ChoreographyService = Depends(get_choreography_service),
):
    """
    Delete a choreography.
    
    Args:
        choreography_id: Choreography ID
        db: Database session
        choreography_service: Choreography service
    """
    try:
        deleted = await choreography_service.delete_choreography(db, choreography_id)
        if not deleted:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail=f"Choreography {choreography_id} not found",
            )
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error deleting choreography {choreography_id}: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error deleting choreography: {str(e)}",
        )


@router.patch("/{choreography_id}/status", response_model=ChoreographyResponse)
async def update_choreography_status(
    status: ChoreographyStatus,
    choreography_id: str = Path(...),
    db: AsyncSession = Depends(get_db),
    choreography_service: ChoreographyService = Depends(get_choreography_service),
):
    """
    Update a choreography's status.
    
    Args:
        status: New status
        choreography_id: Choreography ID
        db: Database session
        choreography_service: Choreography service
        
    Returns:
        Updated choreography
    """
    try:
        updated_choreography = await choreography_service.update_choreography_status(db, choreography_id, status)
        if not updated_choreography:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail=f"Choreography {choreography_id} not found",
            )
        return updated_choreography
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error updating choreography status {choreography_id}: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error updating choreography status: {str(e)}",
        )
