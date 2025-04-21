"""
Recognition API routes for the AI Analytics module.

This module provides endpoints for face and license plate recognition.
"""

from fastapi import APIRouter, Depends, HTTPException, Request, BackgroundTasks, UploadFile, File
from typing import List, Dict, Any, Optional
from pydantic import BaseModel

# Import local modules
from services.recognition import RecognitionService
from api.schemas.recognition import (
    FaceProfile,
    LicensePlate,
    RecognitionConfig,
    RecognitionResult
)

router = APIRouter()

# Models
class UpdateRecognitionConfigRequest(BaseModel):
    """Request model for updating recognition configuration."""
    face_recognition_enabled: Optional[bool] = None
    license_plate_recognition_enabled: Optional[bool] = None
    face_recognition_threshold: Optional[float] = None
    license_plate_confidence_threshold: Optional[float] = None

class CreateFaceProfileRequest(BaseModel):
    """Request model for creating a face profile."""
    name: str
    description: Optional[str] = None
    tags: Optional[List[str]] = None
    alert_on_recognition: Optional[bool] = None

class UpdateFaceProfileRequest(BaseModel):
    """Request model for updating a face profile."""
    name: Optional[str] = None
    description: Optional[str] = None
    tags: Optional[List[str]] = None
    alert_on_recognition: Optional[bool] = None

class CreateLicensePlateRequest(BaseModel):
    """Request model for creating a license plate entry."""
    plate_number: str
    description: Optional[str] = None
    tags: Optional[List[str]] = None
    alert_on_recognition: Optional[bool] = None

class UpdateLicensePlateRequest(BaseModel):
    """Request model for updating a license plate entry."""
    plate_number: Optional[str] = None
    description: Optional[str] = None
    tags: Optional[List[str]] = None
    alert_on_recognition: Optional[bool] = None

# Helper functions
def get_recognition_service(request: Request) -> RecognitionService:
    """Get the recognition service from the app state."""
    return RecognitionService(
        request.app.state.video_manager,
        request.app.state.event_publisher,
        request.app.state.config["recognition"]
    )

# Routes
@router.get("/config")
async def get_recognition_config(
    camera_id: Optional[str] = None,
    recognition_service: RecognitionService = Depends(get_recognition_service)
):
    """Get the current recognition configuration."""
    try:
        config = await recognition_service.get_config(camera_id)
        return config
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error getting recognition config: {str(e)}")

@router.put("/config")
async def update_recognition_config(
    request: UpdateRecognitionConfigRequest,
    camera_id: Optional[str] = None,
    recognition_service: RecognitionService = Depends(get_recognition_service)
):
    """Update the recognition configuration."""
    try:
        updated_config = await recognition_service.update_config(
            camera_id=camera_id,
            face_recognition_enabled=request.face_recognition_enabled,
            license_plate_recognition_enabled=request.license_plate_recognition_enabled,
            face_recognition_threshold=request.face_recognition_threshold,
            license_plate_confidence_threshold=request.license_plate_confidence_threshold
        )
        return updated_config
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error updating recognition config: {str(e)}")

# Face recognition routes
@router.get("/faces")
async def get_face_profiles(
    tags: Optional[List[str]] = None,
    recognition_service: RecognitionService = Depends(get_recognition_service)
):
    """Get all face profiles, optionally filtered by tags."""
    try:
        profiles = await recognition_service.get_face_profiles(tags)
        return profiles
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error getting face profiles: {str(e)}")

@router.get("/faces/{profile_id}")
async def get_face_profile(
    profile_id: str,
    recognition_service: RecognitionService = Depends(get_recognition_service)
):
    """Get a specific face profile by ID."""
    try:
        profile = await recognition_service.get_face_profile(profile_id)
        if not profile:
            raise HTTPException(status_code=404, detail=f"Face profile {profile_id} not found")
        return profile
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error getting face profile: {str(e)}")

@router.post("/faces")
async def create_face_profile(
    request: CreateFaceProfileRequest,
    recognition_service: RecognitionService = Depends(get_recognition_service)
):
    """Create a new face profile."""
    try:
        profile = await recognition_service.create_face_profile(
            name=request.name,
            description=request.description,
            tags=request.tags,
            alert_on_recognition=request.alert_on_recognition
        )
        return profile
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error creating face profile: {str(e)}")

@router.put("/faces/{profile_id}")
async def update_face_profile(
    profile_id: str,
    request: UpdateFaceProfileRequest,
    recognition_service: RecognitionService = Depends(get_recognition_service)
):
    """Update a face profile."""
    try:
        profile = await recognition_service.update_face_profile(
            profile_id=profile_id,
            name=request.name,
            description=request.description,
            tags=request.tags,
            alert_on_recognition=request.alert_on_recognition
        )
        if not profile:
            raise HTTPException(status_code=404, detail=f"Face profile {profile_id} not found")
        return profile
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error updating face profile: {str(e)}")

@router.delete("/faces/{profile_id}")
async def delete_face_profile(
    profile_id: str,
    recognition_service: RecognitionService = Depends(get_recognition_service)
):
    """Delete a face profile."""
    try:
        success = await recognition_service.delete_face_profile(profile_id)
        if not success:
            raise HTTPException(status_code=404, detail=f"Face profile {profile_id} not found")
        return {"success": True}
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error deleting face profile: {str(e)}")

@router.post("/faces/{profile_id}/images")
async def add_face_image(
    profile_id: str,
    image: UploadFile = File(...),
    recognition_service: RecognitionService = Depends(get_recognition_service)
):
    """Add a face image to a profile."""
    try:
        image_content = await image.read()
        image_id = await recognition_service.add_face_image(profile_id, image_content)
        return {"image_id": image_id}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error adding face image: {str(e)}")

@router.delete("/faces/{profile_id}/images/{image_id}")
async def delete_face_image(
    profile_id: str,
    image_id: str,
    recognition_service: RecognitionService = Depends(get_recognition_service)
):
    """Delete a face image from a profile."""
    try:
        success = await recognition_service.delete_face_image(profile_id, image_id)
        if not success:
            raise HTTPException(status_code=404, detail=f"Image {image_id} not found in profile {profile_id}")
        return {"success": True}
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error deleting face image: {str(e)}")

# License plate recognition routes
@router.get("/license-plates")
async def get_license_plates(
    tags: Optional[List[str]] = None,
    recognition_service: RecognitionService = Depends(get_recognition_service)
):
    """Get all license plates, optionally filtered by tags."""
    try:
        plates = await recognition_service.get_license_plates(tags)
        return plates
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error getting license plates: {str(e)}")

@router.get("/license-plates/{plate_id}")
async def get_license_plate(
    plate_id: str,
    recognition_service: RecognitionService = Depends(get_recognition_service)
):
    """Get a specific license plate by ID."""
    try:
        plate = await recognition_service.get_license_plate(plate_id)
        if not plate:
            raise HTTPException(status_code=404, detail=f"License plate {plate_id} not found")
        return plate
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error getting license plate: {str(e)}")

@router.post("/license-plates")
async def create_license_plate(
    request: CreateLicensePlateRequest,
    recognition_service: RecognitionService = Depends(get_recognition_service)
):
    """Create a new license plate entry."""
    try:
        plate = await recognition_service.create_license_plate(
            plate_number=request.plate_number,
            description=request.description,
            tags=request.tags,
            alert_on_recognition=request.alert_on_recognition
        )
        return plate
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error creating license plate: {str(e)}")

@router.put("/license-plates/{plate_id}")
async def update_license_plate(
    plate_id: str,
    request: UpdateLicensePlateRequest,
    recognition_service: RecognitionService = Depends(get_recognition_service)
):
    """Update a license plate entry."""
    try:
        plate = await recognition_service.update_license_plate(
            plate_id=plate_id,
            plate_number=request.plate_number,
            description=request.description,
            tags=request.tags,
            alert_on_recognition=request.alert_on_recognition
        )
        if not plate:
            raise HTTPException(status_code=404, detail=f"License plate {plate_id} not found")
        return plate
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error updating license plate: {str(e)}")

@router.delete("/license-plates/{plate_id}")
async def delete_license_plate(
    plate_id: str,
    recognition_service: RecognitionService = Depends(get_recognition_service)
):
    """Delete a license plate entry."""
    try:
        success = await recognition_service.delete_license_plate(plate_id)
        if not success:
            raise HTTPException(status_code=404, detail=f"License plate {plate_id} not found")
        return {"success": True}
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error deleting license plate: {str(e)}")

@router.get("/results")
async def get_recent_recognition_results(
    camera_id: Optional[str] = None,
    type: Optional[str] = None,  # "face" or "license_plate"
    limit: int = 10,
    recognition_service: RecognitionService = Depends(get_recognition_service)
):
    """Get recent recognition results."""
    try:
        results = await recognition_service.get_recent_results(camera_id, type, limit)
        return results
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error getting recognition results: {str(e)}")

@router.post("/test")
async def test_recognition(
    camera_id: str,
    type: str,  # "face" or "license_plate"
    background_tasks: BackgroundTasks,
    recognition_service: RecognitionService = Depends(get_recognition_service)
):
    """Run a test recognition on a single frame from the camera."""
    try:
        # Run recognition in background to avoid blocking
        background_tasks.add_task(recognition_service.run_test_recognition, camera_id, type)
        return {"message": f"Test {type} recognition started for camera {camera_id}"}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error starting test recognition: {str(e)}")
