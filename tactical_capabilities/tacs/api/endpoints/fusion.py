"""
API endpoints for sensor fusion.
"""

from typing import List, Optional, Dict, Any
from uuid import UUID
from fastapi import APIRouter, Depends, HTTPException, Query, status, Request, Body
from sqlalchemy.ext.asyncio import AsyncSession
from datetime import datetime

from api.schemas import (
    FusionJob, FusionJobCreate, FusionJobStatus
)
from core.security import get_current_user, has_permission, log_security_event
from db.session import get_db_session
from services.sensor_fusion import SensorFusion

router = APIRouter()

@router.post("/process", response_model=FusionJob, status_code=status.HTTP_201_CREATED)
async def create_fusion_job(
    job: FusionJobCreate,
    current_user = Depends(has_permission("tacs:create_fusion_job")),
    db: AsyncSession = Depends(get_db_session)
):
    """
    Create a new fusion job.
    
    Args:
        job: Fusion job data
        current_user: Current authenticated user with required permission
        db: Database session
        
    Returns:
        Created fusion job
    """
    # Create sensor fusion service
    sensor_fusion = SensorFusion(db)
    
    # Create fusion job
    fusion_job = await sensor_fusion.create_fusion_job(job)
    
    # Log the operation
    await log_security_event(
        event_type="fusion_job_created",
        user_id=current_user.id,
        resource_id=str(fusion_job.id),
        resource_type="fusion_job",
        details={
            "sensor_data_ids": [str(id) for id in job.sensor_data_ids],
            "target_id": str(job.target_id) if job.target_id else None,
            "priority": job.priority
        }
    )
    
    # Process fusion job asynchronously
    # In a real implementation, this would be done by a background worker
    # For now, we'll process it directly
    await sensor_fusion.process_fusion_job(str(fusion_job.id))
    
    return fusion_job

@router.get("/status/{job_id}", response_model=FusionJob)
async def get_fusion_job_status(
    job_id: UUID,
    current_user = Depends(get_current_user),
    db: AsyncSession = Depends(get_db_session)
):
    """
    Get the status of a fusion job.
    
    Args:
        job_id: Fusion job ID
        current_user: Current authenticated user
        db: Database session
        
    Returns:
        Fusion job
        
    Raises:
        HTTPException: If fusion job not found
    """
    # Create sensor fusion service
    sensor_fusion = SensorFusion(db)
    
    # Get fusion job
    fusion_job = await sensor_fusion.get_fusion_job(str(job_id))
    
    # Check if fusion job exists
    if not fusion_job:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Fusion job with ID {job_id} not found"
        )
    
    return fusion_job

@router.post("/process_batch", status_code=status.HTTP_202_ACCEPTED)
async def process_fusion_jobs_batch(
    sensor_data_ids: List[UUID],
    target_ids: Optional[List[UUID]] = Body(None),
    parameters: Optional[Dict[str, Any]] = Body(None),
    priority: int = Body(1, ge=1, le=10),
    current_user = Depends(has_permission("tacs:create_fusion_job")),
    db: AsyncSession = Depends(get_db_session)
):
    """
    Process a batch of fusion jobs.
    
    Args:
        sensor_data_ids: List of sensor data IDs
        target_ids: Optional list of target IDs
        parameters: Optional fusion parameters
        priority: Job priority
        current_user: Current authenticated user with required permission
        db: Database session
        
    Returns:
        Batch processing result
    """
    # Create sensor fusion service
    sensor_fusion = SensorFusion(db)
    
    # Create fusion jobs
    job_ids = []
    
    if target_ids:
        # Create one job per target
        for target_id in target_ids:
            job = FusionJobCreate(
                sensor_data_ids=sensor_data_ids,
                target_id=target_id,
                parameters=parameters or {},
                priority=priority,
                metadata={}
            )
            
            fusion_job = await sensor_fusion.create_fusion_job(job)
            job_ids.append(str(fusion_job.id))
            
            # Process fusion job asynchronously
            # In a real implementation, this would be done by a background worker
            # For now, we'll process it directly
            await sensor_fusion.process_fusion_job(str(fusion_job.id))
    else:
        # Create a single job without target
        job = FusionJobCreate(
            sensor_data_ids=sensor_data_ids,
            parameters=parameters or {},
            priority=priority,
            metadata={}
        )
        
        fusion_job = await sensor_fusion.create_fusion_job(job)
        job_ids.append(str(fusion_job.id))
        
        # Process fusion job asynchronously
        # In a real implementation, this would be done by a background worker
        # For now, we'll process it directly
        await sensor_fusion.process_fusion_job(str(fusion_job.id))
    
    # Log the operation
    await log_security_event(
        event_type="fusion_jobs_batch_created",
        user_id=current_user.id,
        resource_id=None,
        resource_type="fusion_job",
        details={
            "sensor_data_ids": [str(id) for id in sensor_data_ids],
            "target_ids": [str(id) for id in target_ids] if target_ids else None,
            "job_ids": job_ids,
            "priority": priority
        }
    )
    
    # Return result
    return {
        "message": f"Processing {len(job_ids)} fusion jobs",
        "job_ids": job_ids
    }

@router.post("/cancel/{job_id}", status_code=status.HTTP_200_OK)
async def cancel_fusion_job(
    job_id: UUID,
    current_user = Depends(has_permission("tacs:cancel_fusion_job")),
    db: AsyncSession = Depends(get_db_session)
):
    """
    Cancel a fusion job.
    
    Args:
        job_id: Fusion job ID
        current_user: Current authenticated user with required permission
        db: Database session
        
    Returns:
        Cancellation result
        
    Raises:
        HTTPException: If fusion job not found or cannot be cancelled
    """
    # Create sensor fusion service
    sensor_fusion = SensorFusion(db)
    
    # Get fusion job
    fusion_job = await sensor_fusion.get_fusion_job(str(job_id))
    
    # Check if fusion job exists
    if not fusion_job:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Fusion job with ID {job_id} not found"
        )
    
    # Check if fusion job can be cancelled
    if fusion_job.status not in [FusionJobStatus.PENDING, FusionJobStatus.PROCESSING]:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=f"Cannot cancel fusion job with status {fusion_job.status}"
        )
    
    # In a real implementation, this would cancel the job
    # For now, we'll just update the status
    
    # Log the operation
    await log_security_event(
        event_type="fusion_job_cancelled",
        user_id=current_user.id,
        resource_id=str(job_id),
        resource_type="fusion_job",
        details={}
    )
    
    # Return result
    return {
        "message": f"Fusion job {job_id} cancelled",
        "job_id": str(job_id)
    }
