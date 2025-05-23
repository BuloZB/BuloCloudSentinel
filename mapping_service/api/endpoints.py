"""
API endpoints for the Mapping Service.
"""

import uuid
from typing import Any, Dict, List, Optional

from fastapi import APIRouter, Depends, File, Form, HTTPException, UploadFile, status
from sqlalchemy.ext.asyncio import AsyncSession

from mapping_service.api.schemas import (
    AssetResponse, ErrorResponse, ProcessingJobCreate, ProcessingJobResponse,
    ProjectCreate, ProjectDetailResponse, ProjectResponse, ProjectUpdate,
    SourceImageResponse, StartProcessingRequest, SuccessResponse
)
from mapping_service.db.session import get_db
from mapping_service.services.mapping_service import MappingService
from mapping_service.services.storage_service import StorageService

router = APIRouter()


@router.post(
    "/projects",
    response_model=ProjectResponse,
    status_code=status.HTTP_201_CREATED,
    responses={
        status.HTTP_400_BAD_REQUEST: {"model": ErrorResponse},
        status.HTTP_401_UNAUTHORIZED: {"model": ErrorResponse},
        status.HTTP_500_INTERNAL_SERVER_ERROR: {"model": ErrorResponse}
    }
)
async def create_project(
    project: ProjectCreate,
    db: AsyncSession = Depends(get_db),
    user_id: uuid.UUID = None  # This would come from auth middleware
):
    """
    Create a new mapping project.
    """
    # For now, use a dummy user ID if not provided
    if user_id is None:
        user_id = uuid.uuid4()
    
    mapping_service = MappingService(db)
    success, project_id, error = await mapping_service.create_project(
        name=project.name,
        description=project.description,
        user_id=user_id
    )
    
    if not success:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=error or "Failed to create project"
        )
    
    # Get the created project
    project = await mapping_service.get_project(project_id)
    
    return project


@router.get(
    "/projects",
    response_model=List[ProjectResponse],
    responses={
        status.HTTP_401_UNAUTHORIZED: {"model": ErrorResponse},
        status.HTTP_500_INTERNAL_SERVER_ERROR: {"model": ErrorResponse}
    }
)
async def list_projects(
    db: AsyncSession = Depends(get_db),
    user_id: Optional[uuid.UUID] = None  # This would come from auth middleware
):
    """
    List mapping projects.
    """
    mapping_service = MappingService(db)
    projects = await mapping_service.list_projects(user_id)
    
    return projects


@router.get(
    "/projects/{project_id}",
    response_model=ProjectDetailResponse,
    responses={
        status.HTTP_404_NOT_FOUND: {"model": ErrorResponse},
        status.HTTP_401_UNAUTHORIZED: {"model": ErrorResponse},
        status.HTTP_500_INTERNAL_SERVER_ERROR: {"model": ErrorResponse}
    }
)
async def get_project(
    project_id: uuid.UUID,
    db: AsyncSession = Depends(get_db)
):
    """
    Get a mapping project with all related data.
    """
    mapping_service = MappingService(db)
    project = await mapping_service.get_project(project_id)
    
    if project is None:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Project {project_id} not found"
        )
    
    # Get related data
    source_images = await mapping_service.get_project_source_images(project_id)
    assets = await mapping_service.get_project_assets(project_id)
    processing_jobs = await mapping_service.get_project_processing_jobs(project_id)
    
    # Add URLs to assets
    storage_service = StorageService()
    for asset in assets:
        asset.url = storage_service.get_public_url(asset.storage_path)
    
    # Create response
    response = ProjectDetailResponse(
        **project.__dict__,
        source_images=source_images,
        assets=assets,
        processing_jobs=processing_jobs
    )
    
    return response


@router.put(
    "/projects/{project_id}",
    response_model=ProjectResponse,
    responses={
        status.HTTP_404_NOT_FOUND: {"model": ErrorResponse},
        status.HTTP_401_UNAUTHORIZED: {"model": ErrorResponse},
        status.HTTP_500_INTERNAL_SERVER_ERROR: {"model": ErrorResponse}
    }
)
async def update_project(
    project_id: uuid.UUID,
    project_update: ProjectUpdate,
    db: AsyncSession = Depends(get_db)
):
    """
    Update a mapping project.
    """
    # Get project
    project = await db.get(ProjectResponse.__config__.orm_mode.__getattribute__("model"), project_id)
    
    if project is None:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Project {project_id} not found"
        )
    
    # Update fields
    if project_update.name is not None:
        project.name = project_update.name
    
    if project_update.description is not None:
        project.description = project_update.description
    
    # Save changes
    await db.commit()
    await db.refresh(project)
    
    return project


@router.post(
    "/projects/{project_id}/images",
    response_model=SuccessResponse,
    responses={
        status.HTTP_404_NOT_FOUND: {"model": ErrorResponse},
        status.HTTP_400_BAD_REQUEST: {"model": ErrorResponse},
        status.HTTP_401_UNAUTHORIZED: {"model": ErrorResponse},
        status.HTTP_500_INTERNAL_SERVER_ERROR: {"model": ErrorResponse}
    }
)
async def upload_images(
    project_id: uuid.UUID,
    files: List[UploadFile] = File(...),
    db: AsyncSession = Depends(get_db)
):
    """
    Upload images to a project.
    """
    mapping_service = MappingService(db)
    
    # Check if project exists
    project = await mapping_service.get_project(project_id)
    if project is None:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Project {project_id} not found"
        )
    
    # Prepare files
    file_data = []
    for file in files:
        if not file.content_type.startswith("image/"):
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail=f"File {file.filename} is not an image"
            )
        
        file_data.append({
            "filename": file.filename,
            "content_type": file.content_type,
            "file": await file.read()
        })
    
    # Upload images
    success, count, error = await mapping_service.upload_images(project_id, file_data)
    
    if not success:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=error or "Failed to upload images"
        )
    
    return {"message": f"Successfully uploaded {count} images"}


@router.post(
    "/projects/{project_id}/process",
    response_model=ProcessingJobResponse,
    responses={
        status.HTTP_404_NOT_FOUND: {"model": ErrorResponse},
        status.HTTP_400_BAD_REQUEST: {"model": ErrorResponse},
        status.HTTP_401_UNAUTHORIZED: {"model": ErrorResponse},
        status.HTTP_500_INTERNAL_SERVER_ERROR: {"model": ErrorResponse}
    }
)
async def start_processing(
    project_id: uuid.UUID,
    request: StartProcessingRequest,
    db: AsyncSession = Depends(get_db)
):
    """
    Start processing a project.
    """
    mapping_service = MappingService(db)
    
    # Check if project exists
    project = await mapping_service.get_project(project_id)
    if project is None:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Project {project_id} not found"
        )
    
    # Start processing
    success, job_id, error = await mapping_service.start_processing(
        project_id=project_id,
        parameters=request.options.dict()
    )
    
    if not success:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=error or "Failed to start processing"
        )
    
    # Get the created job
    jobs = await mapping_service.get_project_processing_jobs(project_id)
    job = next((j for j in jobs if j.id == job_id), None)
    
    if job is None:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to retrieve processing job"
        )
    
    return job


@router.get(
    "/projects/{project_id}/images",
    response_model=List[SourceImageResponse],
    responses={
        status.HTTP_404_NOT_FOUND: {"model": ErrorResponse},
        status.HTTP_401_UNAUTHORIZED: {"model": ErrorResponse},
        status.HTTP_500_INTERNAL_SERVER_ERROR: {"model": ErrorResponse}
    }
)
async def list_images(
    project_id: uuid.UUID,
    db: AsyncSession = Depends(get_db)
):
    """
    List images for a project.
    """
    mapping_service = MappingService(db)
    
    # Check if project exists
    project = await mapping_service.get_project(project_id)
    if project is None:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Project {project_id} not found"
        )
    
    # Get images
    images = await mapping_service.get_project_source_images(project_id)
    
    return images


@router.get(
    "/projects/{project_id}/assets",
    response_model=List[AssetResponse],
    responses={
        status.HTTP_404_NOT_FOUND: {"model": ErrorResponse},
        status.HTTP_401_UNAUTHORIZED: {"model": ErrorResponse},
        status.HTTP_500_INTERNAL_SERVER_ERROR: {"model": ErrorResponse}
    }
)
async def list_assets(
    project_id: uuid.UUID,
    db: AsyncSession = Depends(get_db)
):
    """
    List assets for a project.
    """
    mapping_service = MappingService(db)
    
    # Check if project exists
    project = await mapping_service.get_project(project_id)
    if project is None:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Project {project_id} not found"
        )
    
    # Get assets
    assets = await mapping_service.get_project_assets(project_id)
    
    # Add URLs
    storage_service = StorageService()
    for asset in assets:
        asset.url = storage_service.get_public_url(asset.storage_path)
    
    return assets


@router.get(
    "/projects/{project_id}/jobs",
    response_model=List[ProcessingJobResponse],
    responses={
        status.HTTP_404_NOT_FOUND: {"model": ErrorResponse},
        status.HTTP_401_UNAUTHORIZED: {"model": ErrorResponse},
        status.HTTP_500_INTERNAL_SERVER_ERROR: {"model": ErrorResponse}
    }
)
async def list_jobs(
    project_id: uuid.UUID,
    db: AsyncSession = Depends(get_db)
):
    """
    List processing jobs for a project.
    """
    mapping_service = MappingService(db)
    
    # Check if project exists
    project = await mapping_service.get_project(project_id)
    if project is None:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Project {project_id} not found"
        )
    
    # Get jobs
    jobs = await mapping_service.get_project_processing_jobs(project_id)
    
    return jobs
