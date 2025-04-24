"""
API endpoints for waveform management.
"""

from typing import List, Optional, Dict, Any
from uuid import UUID
from fastapi import APIRouter, Depends, HTTPException, Query, status, Request, Body, UploadFile, File, Form
from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy.future import select
from sqlalchemy import update, delete
from datetime import datetime
import json
import os
import tempfile

from api.schemas import (
    WaveformTemplate, WaveformTemplateCreate, WaveformTemplateUpdate
)
from core.security import get_current_user, has_permission, check_clearance, log_security_event
from db.session import get_db_session
from services.waveform_manager import WaveformManager

router = APIRouter()

@router.get("/", response_model=List[WaveformTemplate])
async def get_waveform_templates(
    skip: int = 0,
    limit: int = 100,
    waveform_type: Optional[str] = None,
    current_user = Depends(get_current_user),
    request: Request = None
):
    """
    Get all waveform templates with optional filtering.
    
    Args:
        skip: Number of templates to skip
        limit: Maximum number of templates to return
        waveform_type: Filter by waveform type
        current_user: Current authenticated user
        request: FastAPI request
        
    Returns:
        List of waveform templates
    """
    # Get waveform manager
    waveform_manager = request.app.state.waveform_manager
    
    # Get all templates
    templates = await waveform_manager.get_all_waveform_templates()
    
    # Apply filters
    if waveform_type:
        templates = [t for t in templates if t["waveform_type"] == waveform_type]
    
    # Apply pagination
    templates = templates[skip:skip+limit]
    
    return templates

@router.post("/", response_model=WaveformTemplate, status_code=status.HTTP_201_CREATED)
async def create_waveform_template(
    name: str = Form(...),
    description: str = Form(...),
    waveform_type: str = Form(...),
    parameters: str = Form(...),
    preview: bool = Form(True),
    metadata: Optional[str] = Form(None),
    file: Optional[UploadFile] = File(None),
    current_user = Depends(has_permission("ew:create_waveform")),
    request: Request = None
):
    """
    Create a new waveform template.
    
    Args:
        name: Waveform template name
        description: Waveform template description
        waveform_type: Type of waveform
        parameters: Waveform parameters (JSON string)
        preview: Whether to generate a preview
        metadata: Optional metadata (JSON string)
        file: Optional waveform file
        current_user: Current authenticated user with required permission
        request: FastAPI request
        
    Returns:
        Created waveform template
    """
    # Parse parameters and metadata
    try:
        parameters_dict = json.loads(parameters)
    except json.JSONDecodeError:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Invalid parameters JSON"
        )
    
    metadata_dict = None
    if metadata:
        try:
            metadata_dict = json.loads(metadata)
        except json.JSONDecodeError:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="Invalid metadata JSON"
            )
    
    # Get waveform manager
    waveform_manager = request.app.state.waveform_manager
    
    try:
        # Create waveform template
        template = await waveform_manager.create_waveform_template(
            name=name,
            description=description,
            waveform_type=waveform_type,
            parameters=parameters_dict,
            preview=preview,
            metadata=metadata_dict
        )
        
        # Upload file if provided
        if file and template:
            # Save file to temporary location
            temp_file = tempfile.NamedTemporaryFile(delete=False)
            try:
                # Write file content
                content = await file.read()
                temp_file.write(content)
                temp_file.close()
                
                # Upload file
                waveform_id = template["id"]
                file_path = f"waveforms/{waveform_id}.bin"
                
                # Get storage manager
                storage_manager = request.app.state.storage_manager
                
                # Upload file
                file_url = await storage_manager.upload_file(
                    temp_file.name,
                    file_path,
                    file.content_type
                )
                
                # Update template with file URL
                if file_url:
                    template = await waveform_manager.update_waveform_template(
                        waveform_id=waveform_id,
                        update_data={"file_url": file_url}
                    )
            
            finally:
                # Clean up temporary file
                os.unlink(temp_file.name)
        
        # Log the operation
        await log_security_event(
            event_type="waveform_template_created",
            user_id=current_user.id,
            resource_id=template["id"],
            resource_type="waveform_template",
            details={
                "name": name,
                "waveform_type": waveform_type
            }
        )
        
        return template
    
    except ValueError as e:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=str(e)
        )
    
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Failed to create waveform template: {str(e)}"
        )

@router.get("/{waveform_id}", response_model=WaveformTemplate)
async def get_waveform_template(
    waveform_id: str,
    current_user = Depends(get_current_user),
    request: Request = None
):
    """
    Get a specific waveform template by ID.
    
    Args:
        waveform_id: Waveform template ID
        current_user: Current authenticated user
        request: FastAPI request
        
    Returns:
        Waveform template
        
    Raises:
        HTTPException: If template not found
    """
    # Get waveform manager
    waveform_manager = request.app.state.waveform_manager
    
    # Get template
    template = await waveform_manager.get_waveform_template(waveform_id)
    
    # Check if template exists
    if not template:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Waveform template with ID {waveform_id} not found"
        )
    
    return template

@router.put("/{waveform_id}", response_model=WaveformTemplate)
async def update_waveform_template(
    waveform_id: str,
    template_update: WaveformTemplateUpdate,
    current_user = Depends(has_permission("ew:update_waveform")),
    request: Request = None
):
    """
    Update a waveform template.
    
    Args:
        waveform_id: Waveform template ID
        template_update: Waveform template update data
        current_user: Current authenticated user with required permission
        request: FastAPI request
        
    Returns:
        Updated waveform template
        
    Raises:
        HTTPException: If template not found
    """
    # Get waveform manager
    waveform_manager = request.app.state.waveform_manager
    
    # Get template
    template = await waveform_manager.get_waveform_template(waveform_id)
    
    # Check if template exists
    if not template:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Waveform template with ID {waveform_id} not found"
        )
    
    try:
        # Update template
        updated_template = await waveform_manager.update_waveform_template(
            waveform_id=waveform_id,
            update_data=template_update.dict(exclude_unset=True)
        )
        
        # Log the operation
        await log_security_event(
            event_type="waveform_template_updated",
            user_id=current_user.id,
            resource_id=waveform_id,
            resource_type="waveform_template",
            details={
                "updated_fields": [k for k, v in template_update.dict(exclude_unset=True).items() if v is not None]
            }
        )
        
        return updated_template
    
    except ValueError as e:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=str(e)
        )
    
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Failed to update waveform template: {str(e)}"
        )

@router.delete("/{waveform_id}", status_code=status.HTTP_204_NO_CONTENT)
async def delete_waveform_template(
    waveform_id: str,
    current_user = Depends(has_permission("ew:delete_waveform")),
    request: Request = None
):
    """
    Delete a waveform template.
    
    Args:
        waveform_id: Waveform template ID
        current_user: Current authenticated user with required permission
        request: FastAPI request
        
    Raises:
        HTTPException: If template not found or deletion failed
    """
    # Get waveform manager
    waveform_manager = request.app.state.waveform_manager
    
    # Get template
    template = await waveform_manager.get_waveform_template(waveform_id)
    
    # Check if template exists
    if not template:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Waveform template with ID {waveform_id} not found"
        )
    
    # Delete template
    success = await waveform_manager.delete_waveform_template(waveform_id)
    
    if not success:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Failed to delete waveform template"
        )
    
    # Log the operation
    await log_security_event(
        event_type="waveform_template_deleted",
        user_id=current_user.id,
        resource_id=waveform_id,
        resource_type="waveform_template",
        details={
            "name": template["name"],
            "waveform_type": template["waveform_type"]
        }
    )
    
    return None

@router.post("/{waveform_id}/generate")
async def generate_waveform(
    waveform_id: str,
    duration: float = 1.0,
    sample_rate: int = 44100,
    parameters_override: Optional[Dict[str, Any]] = Body(None),
    current_user = Depends(has_permission("ew:generate_waveform")),
    request: Request = None
):
    """
    Generate a waveform from a template.
    
    Args:
        waveform_id: Waveform template ID
        duration: Duration in seconds
        sample_rate: Sample rate in Hz
        parameters_override: Optional parameters to override
        current_user: Current authenticated user with required permission
        request: FastAPI request
        
    Returns:
        Waveform metadata
        
    Raises:
        HTTPException: If template not found or generation failed
    """
    # Get waveform manager
    waveform_manager = request.app.state.waveform_manager
    
    try:
        # Generate waveform
        samples, metadata = await waveform_manager.generate_waveform(
            waveform_id=waveform_id,
            duration=duration,
            sample_rate=sample_rate,
            parameters_override=parameters_override
        )
        
        # Return metadata
        return {
            "waveform_id": waveform_id,
            "duration": duration,
            "sample_rate": sample_rate,
            "num_samples": len(samples),
            "metadata": metadata
        }
    
    except ValueError as e:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=str(e)
        )
    
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Failed to generate waveform: {str(e)}"
        )
