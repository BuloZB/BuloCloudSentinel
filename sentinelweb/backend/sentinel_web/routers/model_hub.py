"""
Model Hub router for SentinelWeb.

This module provides API endpoints for interacting with the Model Hub service.
"""

import os
import logging
import json
import tempfile
import shutil
from typing import Dict, List, Any, Optional
from datetime import datetime

from fastapi import APIRouter, Depends, HTTPException, status, UploadFile, File, Form, Query, Request
from fastapi.responses import JSONResponse
import httpx

from sentinel_web.auth.users import get_verified_user, get_admin_user
from sentinel_web.models.users import UserResponse

# Setup logging
log = logging.getLogger(__name__)

# Create router
router = APIRouter()

# Model Hub API URL
MODEL_HUB_API_URL = os.environ.get("MODEL_HUB_API_URL", "http://model-hub-api:8070")

@router.get("/models")
async def get_models(
    request: Request,
    name: Optional[str] = None,
    model_type: Optional[str] = None,
    stage: Optional[str] = None,
    is_active: Optional[bool] = None,
    skip: int = 0,
    limit: int = 100,
    user=Depends(get_verified_user),
):
    """
    Get all models from the Model Hub.
    
    Args:
        request: FastAPI request
        name: Filter by model name
        model_type: Filter by model type
        stage: Filter by model stage
        is_active: Filter by active status
        skip: Number of models to skip
        limit: Maximum number of models to return
        user: Authenticated user
        
    Returns:
        List of models
    """
    try:
        # Prepare query parameters
        params = {}
        if name:
            params["name"] = name
        if model_type:
            params["model_type"] = model_type
        if stage:
            params["stage"] = stage
        if is_active is not None:
            params["is_active"] = str(is_active).lower()
        if skip:
            params["skip"] = skip
        if limit:
            params["limit"] = limit
        
        # Send request to Model Hub API
        async with httpx.AsyncClient() as client:
            response = await client.get(
                f"{MODEL_HUB_API_URL}/api/v1/models",
                params=params,
            )
            
            # Check response
            response.raise_for_status()
            
            # Return models
            return response.json()
    except httpx.HTTPStatusError as e:
        log.error(f"Error getting models from Model Hub: {e}")
        raise HTTPException(
            status_code=e.response.status_code,
            detail=e.response.text,
        )
    except Exception as e:
        log.error(f"Error getting models from Model Hub: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error getting models: {str(e)}",
        )

@router.get("/models/{model_id}")
async def get_model(
    request: Request,
    model_id: str,
    user=Depends(get_verified_user),
):
    """
    Get a model by ID from the Model Hub.
    
    Args:
        request: FastAPI request
        model_id: ID of the model
        user: Authenticated user
        
    Returns:
        Model
    """
    try:
        # Send request to Model Hub API
        async with httpx.AsyncClient() as client:
            response = await client.get(
                f"{MODEL_HUB_API_URL}/api/v1/models/{model_id}",
            )
            
            # Check response
            response.raise_for_status()
            
            # Return model
            return response.json()
    except httpx.HTTPStatusError as e:
        log.error(f"Error getting model from Model Hub: {e}")
        raise HTTPException(
            status_code=e.response.status_code,
            detail=e.response.text,
        )
    except Exception as e:
        log.error(f"Error getting model from Model Hub: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error getting model: {str(e)}",
        )

@router.post("/models")
async def create_model(
    request: Request,
    file: UploadFile = File(...),
    name: str = Form(...),
    version: str = Form(...),
    description: Optional[str] = Form(None),
    model_type: str = Form(...),
    framework: str = Form(...),
    tags: Optional[str] = Form(None),
    stage: str = Form("development"),
    user=Depends(get_admin_user),
):
    """
    Create a new model in the Model Hub.
    
    Args:
        request: FastAPI request
        file: Model file
        name: Name of the model
        version: Version of the model
        description: Description of the model
        model_type: Type of the model
        framework: Framework of the model
        tags: Tags for the model (JSON string)
        stage: Stage of the model
        user: Authenticated user
        
    Returns:
        Created model
    """
    try:
        # Save file to temporary location
        with tempfile.NamedTemporaryFile(delete=False) as temp_file:
            shutil.copyfileobj(file.file, temp_file)
            temp_file_path = temp_file.name
        
        try:
            # Prepare form data
            form_data = {
                "name": name,
                "version": version,
                "description": description,
                "model_type": model_type,
                "framework": framework,
                "stage": stage,
            }
            
            # Add tags if provided
            if tags:
                form_data["tags"] = tags
            
            # Prepare file
            files = {
                "file": (file.filename, open(temp_file_path, "rb")),
            }
            
            # Send request to Model Hub API
            async with httpx.AsyncClient() as client:
                response = await client.post(
                    f"{MODEL_HUB_API_URL}/api/v1/models",
                    data=form_data,
                    files=files,
                )
                
                # Check response
                response.raise_for_status()
                
                # Return created model
                return response.json()
        finally:
            # Clean up temporary file
            if os.path.exists(temp_file_path):
                os.unlink(temp_file_path)
    except httpx.HTTPStatusError as e:
        log.error(f"Error creating model in Model Hub: {e}")
        raise HTTPException(
            status_code=e.response.status_code,
            detail=e.response.text,
        )
    except Exception as e:
        log.error(f"Error creating model in Model Hub: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error creating model: {str(e)}",
        )

@router.get("/deployments")
async def get_deployments(
    request: Request,
    model_id: Optional[str] = None,
    environment: Optional[str] = None,
    status: Optional[str] = None,
    skip: int = 0,
    limit: int = 100,
    user=Depends(get_verified_user),
):
    """
    Get all deployments from the Model Hub.
    
    Args:
        request: FastAPI request
        model_id: Filter by model ID
        environment: Filter by environment
        status: Filter by status
        skip: Number of deployments to skip
        limit: Maximum number of deployments to return
        user: Authenticated user
        
    Returns:
        List of deployments
    """
    try:
        # Prepare query parameters
        params = {}
        if model_id:
            params["model_id"] = model_id
        if environment:
            params["environment"] = environment
        if status:
            params["status"] = status
        if skip:
            params["skip"] = skip
        if limit:
            params["limit"] = limit
        
        # Send request to Model Hub API
        async with httpx.AsyncClient() as client:
            response = await client.get(
                f"{MODEL_HUB_API_URL}/api/v1/deployments",
                params=params,
            )
            
            # Check response
            response.raise_for_status()
            
            # Return deployments
            return response.json()
    except httpx.HTTPStatusError as e:
        log.error(f"Error getting deployments from Model Hub: {e}")
        raise HTTPException(
            status_code=e.response.status_code,
            detail=e.response.text,
        )
    except Exception as e:
        log.error(f"Error getting deployments from Model Hub: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error getting deployments: {str(e)}",
        )

@router.post("/deployments")
async def create_deployment(
    request: Request,
    deployment_data: Dict[str, Any],
    user=Depends(get_admin_user),
):
    """
    Create a new deployment in the Model Hub.
    
    Args:
        request: FastAPI request
        deployment_data: Deployment data
        user: Authenticated user
        
    Returns:
        Created deployment
    """
    try:
        # Send request to Model Hub API
        async with httpx.AsyncClient() as client:
            response = await client.post(
                f"{MODEL_HUB_API_URL}/api/v1/deployments",
                json=deployment_data,
            )
            
            # Check response
            response.raise_for_status()
            
            # Return created deployment
            return response.json()
    except httpx.HTTPStatusError as e:
        log.error(f"Error creating deployment in Model Hub: {e}")
        raise HTTPException(
            status_code=e.response.status_code,
            detail=e.response.text,
        )
    except Exception as e:
        log.error(f"Error creating deployment in Model Hub: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error creating deployment: {str(e)}",
        )

@router.post("/deployments/{deployment_id}/promote")
async def promote_deployment(
    request: Request,
    deployment_id: str,
    user=Depends(get_admin_user),
):
    """
    Promote a deployment in the Model Hub.
    
    Args:
        request: FastAPI request
        deployment_id: ID of the deployment
        user: Authenticated user
        
    Returns:
        Promoted deployment
    """
    try:
        # Send request to Model Hub API
        async with httpx.AsyncClient() as client:
            response = await client.post(
                f"{MODEL_HUB_API_URL}/api/v1/deployments/{deployment_id}/promote",
            )
            
            # Check response
            response.raise_for_status()
            
            # Return promoted deployment
            return response.json()
    except httpx.HTTPStatusError as e:
        log.error(f"Error promoting deployment in Model Hub: {e}")
        raise HTTPException(
            status_code=e.response.status_code,
            detail=e.response.text,
        )
    except Exception as e:
        log.error(f"Error promoting deployment in Model Hub: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error promoting deployment: {str(e)}",
        )

@router.post("/deployments/{deployment_id}/rollback")
async def rollback_deployment(
    request: Request,
    deployment_id: str,
    user=Depends(get_admin_user),
):
    """
    Rollback a deployment in the Model Hub.
    
    Args:
        request: FastAPI request
        deployment_id: ID of the deployment
        user: Authenticated user
        
    Returns:
        Rolled back deployment
    """
    try:
        # Send request to Model Hub API
        async with httpx.AsyncClient() as client:
            response = await client.post(
                f"{MODEL_HUB_API_URL}/api/v1/deployments/{deployment_id}/rollback",
            )
            
            # Check response
            response.raise_for_status()
            
            # Return rolled back deployment
            return response.json()
    except httpx.HTTPStatusError as e:
        log.error(f"Error rolling back deployment in Model Hub: {e}")
        raise HTTPException(
            status_code=e.response.status_code,
            detail=e.response.text,
        )
    except Exception as e:
        log.error(f"Error rolling back deployment in Model Hub: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error rolling back deployment: {str(e)}",
        )
