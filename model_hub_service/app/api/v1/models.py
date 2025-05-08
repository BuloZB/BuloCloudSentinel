"""
API endpoints for model management.

This module provides API endpoints for managing models in the Model Hub.
"""

import os
import logging
import tempfile
import shutil
from typing import Dict, List, Any, Optional
from datetime import datetime

from fastapi import APIRouter, Depends, HTTPException, status, UploadFile, File, Form, Query
from fastapi.responses import JSONResponse
from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy.future import select
from sqlalchemy import update, delete

from app.db.database import get_db
from app.models.model import (
    Model,
    ModelCreate,
    ModelUpdate,
    ModelResponse,
    ModelStage,
)
from app.services.mlflow_service import MLflowService

# Setup logging
logger = logging.getLogger(__name__)

# Create router
router = APIRouter()

@router.get("/", response_model=List[ModelResponse])
async def get_models(
    name: Optional[str] = None,
    model_type: Optional[str] = None,
    stage: Optional[ModelStage] = None,
    is_active: Optional[bool] = None,
    skip: int = 0,
    limit: int = 100,
    db: AsyncSession = Depends(get_db),
):
    """
    Get all models.
    
    Args:
        name: Filter by model name
        model_type: Filter by model type
        stage: Filter by model stage
        is_active: Filter by active status
        skip: Number of models to skip
        limit: Maximum number of models to return
        db: Database session
        
    Returns:
        List of models
    """
    try:
        # Build query
        query = select(Model)
        
        # Apply filters
        if name:
            query = query.filter(Model.name == name)
        if model_type:
            query = query.filter(Model.model_type == model_type)
        if stage:
            query = query.filter(Model.stage == stage)
        if is_active is not None:
            query = query.filter(Model.is_active == is_active)
        
        # Apply pagination
        query = query.offset(skip).limit(limit)
        
        # Execute query
        result = await db.execute(query)
        models = result.scalars().all()
        
        return models
    except Exception as e:
        logger.error(f"Error getting models: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error getting models: {str(e)}",
        )

@router.get("/{model_id}", response_model=ModelResponse)
async def get_model(
    model_id: str,
    db: AsyncSession = Depends(get_db),
):
    """
    Get a model by ID.
    
    Args:
        model_id: ID of the model
        db: Database session
        
    Returns:
        Model
    """
    try:
        # Get model
        result = await db.execute(select(Model).filter(Model.id == model_id))
        model = result.scalars().first()
        
        if not model:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail=f"Model with ID {model_id} not found",
            )
        
        return model
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error getting model {model_id}: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error getting model: {str(e)}",
        )

@router.post("/", response_model=ModelResponse, status_code=status.HTTP_201_CREATED)
async def create_model(
    file: UploadFile = File(...),
    name: str = Form(...),
    version: str = Form(...),
    description: Optional[str] = Form(None),
    model_type: str = Form(...),
    framework: str = Form(...),
    tags: Optional[str] = Form(None),
    stage: ModelStage = Form(ModelStage.DEVELOPMENT),
    db: AsyncSession = Depends(get_db),
    mlflow_service: MLflowService = Depends(lambda: MLflowService()),
):
    """
    Create a new model.
    
    Args:
        file: Model file
        name: Name of the model
        version: Version of the model
        description: Description of the model
        model_type: Type of the model
        framework: Framework of the model
        tags: Tags for the model (JSON string)
        stage: Stage of the model
        db: Database session
        mlflow_service: MLflow service
        
    Returns:
        Created model
    """
    try:
        # Parse tags
        parsed_tags = {}
        if tags:
            import json
            parsed_tags = json.loads(tags)
        
        # Save file to temporary location
        with tempfile.NamedTemporaryFile(delete=False) as temp_file:
            shutil.copyfileobj(file.file, temp_file)
            temp_file_path = temp_file.name
        
        try:
            # Log model to MLflow
            run_id, experiment_id, model_hash = await mlflow_service.log_model(
                model_path=temp_file_path,
                model_name=name,
                model_version=version,
                model_type=model_type,
                framework=framework,
                tags=parsed_tags,
                metadata={
                    "description": description,
                    "stage": stage.value,
                },
            )
            
            # Get file size
            size_bytes = os.path.getsize(temp_file_path)
            
            # Create model
            model = Model(
                name=name,
                version=version,
                description=description,
                model_type=model_type,
                framework=framework,
                tags=parsed_tags,
                accuracy=None,  # To be updated later
                size_bytes=size_bytes,
                hash=model_hash,
                signature=None,  # To be updated later
                compatible_hardware=None,  # To be updated later
                min_memory_mb=None,  # To be updated later
                mlflow_run_id=run_id,
                mlflow_experiment_id=experiment_id,
                stage=stage.value,
                is_active=False,
            )
            
            # Save model to database
            db.add(model)
            await db.commit()
            await db.refresh(model)
            
            logger.info(f"Created model {name} version {version}")
            
            return model
        finally:
            # Clean up temporary file
            if os.path.exists(temp_file_path):
                os.unlink(temp_file_path)
    except Exception as e:
        logger.error(f"Error creating model: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error creating model: {str(e)}",
        )

@router.put("/{model_id}", response_model=ModelResponse)
async def update_model(
    model_id: str,
    model_update: ModelUpdate,
    db: AsyncSession = Depends(get_db),
):
    """
    Update a model.
    
    Args:
        model_id: ID of the model
        model_update: Model update data
        db: Database session
        
    Returns:
        Updated model
    """
    try:
        # Check if model exists
        result = await db.execute(select(Model).filter(Model.id == model_id))
        model = result.scalars().first()
        
        if not model:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail=f"Model with ID {model_id} not found",
            )
        
        # Update model
        update_data = model_update.dict(exclude_unset=True)
        
        if "stage" in update_data:
            update_data["stage"] = update_data["stage"].value
        
        await db.execute(
            update(Model)
            .where(Model.id == model_id)
            .values(**update_data)
        )
        
        await db.commit()
        
        # Get updated model
        result = await db.execute(select(Model).filter(Model.id == model_id))
        updated_model = result.scalars().first()
        
        logger.info(f"Updated model {model_id}")
        
        return updated_model
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error updating model {model_id}: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error updating model: {str(e)}",
        )

@router.delete("/{model_id}", status_code=status.HTTP_204_NO_CONTENT)
async def delete_model(
    model_id: str,
    db: AsyncSession = Depends(get_db),
):
    """
    Delete a model.
    
    Args:
        model_id: ID of the model
        db: Database session
    """
    try:
        # Check if model exists
        result = await db.execute(select(Model).filter(Model.id == model_id))
        model = result.scalars().first()
        
        if not model:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail=f"Model with ID {model_id} not found",
            )
        
        # Delete model
        await db.execute(delete(Model).where(Model.id == model_id))
        await db.commit()
        
        logger.info(f"Deleted model {model_id}")
        
        return None
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error deleting model {model_id}: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error deleting model: {str(e)}",
        )

@router.post("/{model_id}/activate", response_model=ModelResponse)
async def activate_model(
    model_id: str,
    db: AsyncSession = Depends(get_db),
):
    """
    Activate a model.
    
    Args:
        model_id: ID of the model
        db: Database session
        
    Returns:
        Activated model
    """
    try:
        # Check if model exists
        result = await db.execute(select(Model).filter(Model.id == model_id))
        model = result.scalars().first()
        
        if not model:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail=f"Model with ID {model_id} not found",
            )
        
        # Deactivate all other models of the same type
        await db.execute(
            update(Model)
            .where(Model.model_type == model.model_type)
            .values(is_active=False)
        )
        
        # Activate this model
        await db.execute(
            update(Model)
            .where(Model.id == model_id)
            .values(is_active=True)
        )
        
        await db.commit()
        
        # Get updated model
        result = await db.execute(select(Model).filter(Model.id == model_id))
        updated_model = result.scalars().first()
        
        logger.info(f"Activated model {model_id}")
        
        return updated_model
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error activating model {model_id}: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error activating model: {str(e)}",
        )
