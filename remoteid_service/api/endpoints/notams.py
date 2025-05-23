"""
NOTAM endpoints for the Remote ID & Regulatory Compliance Service.

This module provides API endpoints for NOTAM management and integration.
"""

from datetime import datetime
from typing import Dict, List, Optional, Any
from uuid import UUID

from fastapi import APIRouter, Depends, HTTPException, status, Request, Security
from sqlalchemy.ext.asyncio import AsyncSession

from remoteid_service.api.schemas.notams import (
    ImportNOTAMsRequest,
    GetNOTAMsRequest,
    CheckFlightPlanNOTAMsRequest,
    NOTAMResponse,
    NOTAMsResponse,
    NOTAMImportResponse,
    FlightPlanNOTAMsResponse,
)
from remoteid_service.core.security import User, get_current_user
from remoteid_service.core.settings import Settings, get_settings
from remoteid_service.db.session import get_db_session
from remoteid_service.services.notam_service import NOTAMService

router = APIRouter()

# Helper functions
async def get_notam_service(request: Request) -> NOTAMService:
    """
    Get the NOTAM service.
    
    Args:
        request: FastAPI request
        
    Returns:
        NOTAMService: NOTAM service
        
    Raises:
        HTTPException: If the service is not available
    """
    if not hasattr(request.app.state, "notam_service"):
        raise HTTPException(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail="NOTAM service is not available",
        )
    
    return request.app.state.notam_service

# Endpoints
@router.post("/import", response_model=NOTAMImportResponse)
async def import_notams(
    request: ImportNOTAMsRequest,
    fastapi_request: Request,
    current_user: User = Security(get_current_user, scopes=["notam:read"]),
    notam_service: NOTAMService = Depends(get_notam_service),
    db: AsyncSession = Depends(get_db_session),
) -> NOTAMImportResponse:
    """
    Import NOTAMs from a source.
    
    Args:
        request: Import NOTAMs request
        fastapi_request: FastAPI request
        current_user: Current user
        notam_service: NOTAM service
        db: Database session
        
    Returns:
        NOTAMImportResponse: Import response
    """
    try:
        # Import NOTAMs
        response = await notam_service.import_notams(
            source=request.source,
            region=request.region,
            start_time=request.start_time,
            end_time=request.end_time,
            location_code=request.location_code,
            force_update=request.force_update,
            db=db,
        )
        
        return response
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error importing NOTAMs: {str(e)}",
        )

@router.get("/{notam_id}", response_model=NOTAMResponse)
async def get_notam(
    notam_id: UUID,
    fastapi_request: Request,
    current_user: User = Security(get_current_user, scopes=["notam:read"]),
    notam_service: NOTAMService = Depends(get_notam_service),
    db: AsyncSession = Depends(get_db_session),
) -> NOTAMResponse:
    """
    Get a NOTAM by ID.
    
    Args:
        notam_id: NOTAM ID
        fastapi_request: FastAPI request
        current_user: Current user
        notam_service: NOTAM service
        db: Database session
        
    Returns:
        NOTAMResponse: NOTAM
    """
    try:
        # Get NOTAM
        notam = await notam_service.get_notam(
            notam_id=notam_id,
            db=db,
        )
        
        if not notam:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail=f"NOTAM with ID {notam_id} not found",
            )
        
        return notam
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error getting NOTAM: {str(e)}",
        )

@router.post("/search", response_model=NOTAMsResponse)
async def search_notams(
    request: GetNOTAMsRequest,
    fastapi_request: Request,
    current_user: User = Security(get_current_user, scopes=["notam:read"]),
    notam_service: NOTAMService = Depends(get_notam_service),
    db: AsyncSession = Depends(get_db_session),
) -> NOTAMsResponse:
    """
    Search for NOTAMs.
    
    Args:
        request: Get NOTAMs request
        fastapi_request: FastAPI request
        current_user: Current user
        notam_service: NOTAM service
        db: Database session
        
    Returns:
        NOTAMsResponse: NOTAMs
    """
    try:
        # Search NOTAMs
        notams, total = await notam_service.search_notams(
            source=request.source,
            notam_type=request.notam_type,
            location_code=request.location_code,
            status=request.status,
            effective_start_from=request.effective_start_from,
            effective_start_to=request.effective_start_to,
            effective_end_from=request.effective_end_from,
            effective_end_to=request.effective_end_to,
            area=request.area,
            point=request.point,
            radius=request.radius,
            limit=request.limit,
            offset=request.offset,
            db=db,
        )
        
        return NOTAMsResponse(
            notams=notams,
            total=total,
            limit=request.limit or 100,
            offset=request.offset or 0,
        )
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error searching NOTAMs: {str(e)}",
        )

@router.post("/check-flight-plan", response_model=FlightPlanNOTAMsResponse)
async def check_flight_plan_notams(
    request: CheckFlightPlanNOTAMsRequest,
    fastapi_request: Request,
    current_user: User = Security(get_current_user, scopes=["notam:read", "flightplan:read"]),
    notam_service: NOTAMService = Depends(get_notam_service),
    db: AsyncSession = Depends(get_db_session),
) -> FlightPlanNOTAMsResponse:
    """
    Check a flight plan for NOTAM conflicts.
    
    Args:
        request: Check flight plan NOTAMs request
        fastapi_request: FastAPI request
        current_user: Current user
        notam_service: NOTAM service
        db: Database session
        
    Returns:
        FlightPlanNOTAMsResponse: NOTAM conflicts
    """
    try:
        # Check flight plan NOTAMs
        response = await notam_service.check_flight_plan_notams(
            flight_plan_id=request.flight_plan_id,
            db=db,
        )
        
        return response
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error checking flight plan NOTAMs: {str(e)}",
        )

@router.get("/sources", response_model=List[str])
async def get_notam_sources(
    fastapi_request: Request,
    current_user: User = Security(get_current_user, scopes=["notam:read"]),
    notam_service: NOTAMService = Depends(get_notam_service),
) -> List[str]:
    """
    Get available NOTAM sources.
    
    Args:
        fastapi_request: FastAPI request
        current_user: Current user
        notam_service: NOTAM service
        
    Returns:
        List[str]: NOTAM sources
    """
    try:
        # Get NOTAM sources
        sources = await notam_service.get_sources()
        
        return sources
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error getting NOTAM sources: {str(e)}",
        )

@router.get("/regions/{source}", response_model=List[str])
async def get_notam_regions(
    source: str,
    fastapi_request: Request,
    current_user: User = Security(get_current_user, scopes=["notam:read"]),
    notam_service: NOTAMService = Depends(get_notam_service),
) -> List[str]:
    """
    Get available NOTAM regions for a source.
    
    Args:
        source: NOTAM source
        fastapi_request: FastAPI request
        current_user: Current user
        notam_service: NOTAM service
        
    Returns:
        List[str]: NOTAM regions
    """
    try:
        # Get NOTAM regions
        regions = await notam_service.get_regions(source)
        
        return regions
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error getting NOTAM regions: {str(e)}",
        )
