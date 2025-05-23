"""
Flight plan endpoints for the Remote ID & Regulatory Compliance Service.

This module provides API endpoints for flight plan management and submission.
"""

from datetime import datetime
from typing import Dict, List, Optional, Any
from uuid import UUID

from fastapi import APIRouter, Depends, HTTPException, status, Request, Security
from sqlalchemy.ext.asyncio import AsyncSession

from remoteid_service.api.schemas.flightplans import (
    CreateFlightPlanRequest,
    UpdateFlightPlanRequest,
    SubmitFlightPlanRequest,
    CancelFlightPlanRequest,
    GetFlightPlansRequest,
    FlightPlanResponse,
    FlightPlansResponse,
    FlightPlanSubmissionResponse,
    EASASoraSubmission,
    FAALaancSubmission,
)
from remoteid_service.core.security import User, get_current_user
from remoteid_service.core.settings import Settings, get_settings
from remoteid_service.db.session import get_db_session
from remoteid_service.services.flightplan_service import FlightPlanService

router = APIRouter()

# Helper functions
async def get_flightplan_service(request: Request) -> FlightPlanService:
    """
    Get the flight plan service.
    
    Args:
        request: FastAPI request
        
    Returns:
        FlightPlanService: Flight plan service
        
    Raises:
        HTTPException: If the service is not available
    """
    if not hasattr(request.app.state, "flightplan_service"):
        raise HTTPException(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail="Flight plan service is not available",
        )
    
    return request.app.state.flightplan_service

# Endpoints
@router.post("", response_model=FlightPlanResponse)
async def create_flight_plan(
    request: CreateFlightPlanRequest,
    fastapi_request: Request,
    current_user: User = Security(get_current_user, scopes=["flightplan:write"]),
    flightplan_service: FlightPlanService = Depends(get_flightplan_service),
    db: AsyncSession = Depends(get_db_session),
) -> FlightPlanResponse:
    """
    Create a new flight plan.
    
    Args:
        request: Create flight plan request
        fastapi_request: FastAPI request
        current_user: Current user
        flightplan_service: Flight plan service
        db: Database session
        
    Returns:
        FlightPlanResponse: Created flight plan
    """
    try:
        # Create flight plan
        flight_plan = await flightplan_service.create_flight_plan(
            name=request.name,
            description=request.description,
            operator_id=request.operator_id,
            drone_id=request.drone_id,
            plan_type=request.plan_type,
            start_time=request.start_time,
            end_time=request.end_time,
            max_altitude=request.max_altitude,
            area=request.area,
            path=request.path,
            waypoints=request.waypoints,
            metadata=request.metadata,
            db=db,
        )
        
        return flight_plan
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error creating flight plan: {str(e)}",
        )

@router.get("/{flight_plan_id}", response_model=FlightPlanResponse)
async def get_flight_plan(
    flight_plan_id: UUID,
    fastapi_request: Request,
    current_user: User = Security(get_current_user, scopes=["flightplan:read"]),
    flightplan_service: FlightPlanService = Depends(get_flightplan_service),
    db: AsyncSession = Depends(get_db_session),
) -> FlightPlanResponse:
    """
    Get a flight plan by ID.
    
    Args:
        flight_plan_id: Flight plan ID
        fastapi_request: FastAPI request
        current_user: Current user
        flightplan_service: Flight plan service
        db: Database session
        
    Returns:
        FlightPlanResponse: Flight plan
    """
    try:
        # Get flight plan
        flight_plan = await flightplan_service.get_flight_plan(
            flight_plan_id=flight_plan_id,
            db=db,
        )
        
        if not flight_plan:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail=f"Flight plan with ID {flight_plan_id} not found",
            )
        
        return flight_plan
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error getting flight plan: {str(e)}",
        )

@router.put("/{flight_plan_id}", response_model=FlightPlanResponse)
async def update_flight_plan(
    flight_plan_id: UUID,
    request: UpdateFlightPlanRequest,
    fastapi_request: Request,
    current_user: User = Security(get_current_user, scopes=["flightplan:write"]),
    flightplan_service: FlightPlanService = Depends(get_flightplan_service),
    db: AsyncSession = Depends(get_db_session),
) -> FlightPlanResponse:
    """
    Update a flight plan.
    
    Args:
        flight_plan_id: Flight plan ID
        request: Update flight plan request
        fastapi_request: FastAPI request
        current_user: Current user
        flightplan_service: Flight plan service
        db: Database session
        
    Returns:
        FlightPlanResponse: Updated flight plan
    """
    try:
        # Update flight plan
        flight_plan = await flightplan_service.update_flight_plan(
            flight_plan_id=flight_plan_id,
            name=request.name,
            description=request.description,
            start_time=request.start_time,
            end_time=request.end_time,
            max_altitude=request.max_altitude,
            area=request.area,
            path=request.path,
            waypoints=request.waypoints,
            metadata=request.metadata,
            db=db,
        )
        
        if not flight_plan:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail=f"Flight plan with ID {flight_plan_id} not found",
            )
        
        return flight_plan
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error updating flight plan: {str(e)}",
        )

@router.delete("/{flight_plan_id}", response_model=Dict[str, Any])
async def delete_flight_plan(
    flight_plan_id: UUID,
    fastapi_request: Request,
    current_user: User = Security(get_current_user, scopes=["flightplan:write"]),
    flightplan_service: FlightPlanService = Depends(get_flightplan_service),
    db: AsyncSession = Depends(get_db_session),
) -> Dict[str, Any]:
    """
    Delete a flight plan.
    
    Args:
        flight_plan_id: Flight plan ID
        fastapi_request: FastAPI request
        current_user: Current user
        flightplan_service: Flight plan service
        db: Database session
        
    Returns:
        dict: Success message
    """
    try:
        # Delete flight plan
        success = await flightplan_service.delete_flight_plan(
            flight_plan_id=flight_plan_id,
            db=db,
        )
        
        if not success:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail=f"Flight plan with ID {flight_plan_id} not found",
            )
        
        return {"message": f"Flight plan with ID {flight_plan_id} deleted successfully"}
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error deleting flight plan: {str(e)}",
        )

@router.post("/search", response_model=FlightPlansResponse)
async def search_flight_plans(
    request: GetFlightPlansRequest,
    fastapi_request: Request,
    current_user: User = Security(get_current_user, scopes=["flightplan:read"]),
    flightplan_service: FlightPlanService = Depends(get_flightplan_service),
    db: AsyncSession = Depends(get_db_session),
) -> FlightPlansResponse:
    """
    Search for flight plans.
    
    Args:
        request: Get flight plans request
        fastapi_request: FastAPI request
        current_user: Current user
        flightplan_service: Flight plan service
        db: Database session
        
    Returns:
        FlightPlansResponse: Flight plans
    """
    try:
        # Search flight plans
        flight_plans, total = await flightplan_service.search_flight_plans(
            operator_id=request.operator_id,
            drone_id=request.drone_id,
            status=request.status,
            start_time_from=request.start_time_from,
            start_time_to=request.start_time_to,
            limit=request.limit,
            offset=request.offset,
            db=db,
        )
        
        return FlightPlansResponse(
            flight_plans=flight_plans,
            total=total,
            limit=request.limit or 100,
            offset=request.offset or 0,
        )
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error searching flight plans: {str(e)}",
        )

@router.post("/submit", response_model=FlightPlanSubmissionResponse)
async def submit_flight_plan(
    request: SubmitFlightPlanRequest,
    fastapi_request: Request,
    current_user: User = Security(get_current_user, scopes=["flightplan:write"]),
    flightplan_service: FlightPlanService = Depends(get_flightplan_service),
    db: AsyncSession = Depends(get_db_session),
) -> FlightPlanSubmissionResponse:
    """
    Submit a flight plan to regulatory authorities.
    
    Args:
        request: Submit flight plan request
        fastapi_request: FastAPI request
        current_user: Current user
        flightplan_service: Flight plan service
        db: Database session
        
    Returns:
        FlightPlanSubmissionResponse: Submission response
    """
    try:
        # Submit flight plan
        response = await flightplan_service.submit_flight_plan(
            flight_plan_id=request.flight_plan_id,
            submission_type=request.submission_type,
            additional_data=request.additional_data,
            db=db,
        )
        
        return response
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error submitting flight plan: {str(e)}",
        )

@router.post("/cancel", response_model=FlightPlanResponse)
async def cancel_flight_plan(
    request: CancelFlightPlanRequest,
    fastapi_request: Request,
    current_user: User = Security(get_current_user, scopes=["flightplan:write"]),
    flightplan_service: FlightPlanService = Depends(get_flightplan_service),
    db: AsyncSession = Depends(get_db_session),
) -> FlightPlanResponse:
    """
    Cancel a submitted flight plan.
    
    Args:
        request: Cancel flight plan request
        fastapi_request: FastAPI request
        current_user: Current user
        flightplan_service: Flight plan service
        db: Database session
        
    Returns:
        FlightPlanResponse: Updated flight plan
    """
    try:
        # Cancel flight plan
        flight_plan = await flightplan_service.cancel_flight_plan(
            flight_plan_id=request.flight_plan_id,
            reason=request.reason,
            db=db,
        )
        
        if not flight_plan:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail=f"Flight plan with ID {request.flight_plan_id} not found",
            )
        
        return flight_plan
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error cancelling flight plan: {str(e)}",
        )
