"""
Video API router for SentinelWeb.
"""

from fastapi import APIRouter, Depends, HTTPException, status, Response
from typing import List, Dict, Any, Optional
from datetime import datetime

from sentinel_web.internal.auth import get_current_user
from sentinel_web.models.users import User
from sentinel_web.drone_adapter.adapter import get_drone_adapter
from sentinel_web.drone_adapter.video_client import VideoStream, VideoRecording

router = APIRouter()

@router.get("/streams", response_model=List[VideoStream])
async def list_streams(
    drone_id: Optional[str] = None,
    current_user: User = Depends(get_current_user)
):
    """
    List all available video streams.
    """
    adapter = get_drone_adapter()
    streams = await adapter.get_video_client().list_streams(drone_id)
    return streams

@router.get("/streams/{stream_id}", response_model=VideoStream)
async def get_stream(stream_id: str, current_user: User = Depends(get_current_user)):
    """
    Get information about a specific video stream.
    """
    adapter = get_drone_adapter()
    stream = await adapter.get_video_client().get_stream(stream_id)
    
    if not stream:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Stream with ID {stream_id} not found"
        )
    
    return stream

@router.get("/drones/{drone_id}/stream")
async def get_stream_url(
    drone_id: str,
    stream_type: str = "main",
    current_user: User = Depends(get_current_user)
):
    """
    Get the URL for a video stream.
    """
    adapter = get_drone_adapter()
    url = await adapter.get_video_client().get_stream_url(drone_id, stream_type)
    
    return {"url": url}

@router.get("/recordings", response_model=List[VideoRecording])
async def list_recordings(
    drone_id: Optional[str] = None,
    start_date: Optional[datetime] = None,
    end_date: Optional[datetime] = None,
    current_user: User = Depends(get_current_user)
):
    """
    List all available video recordings.
    """
    adapter = get_drone_adapter()
    recordings = await adapter.get_video_client().list_recordings(drone_id, start_date, end_date)
    return recordings

@router.get("/recordings/{recording_id}", response_model=VideoRecording)
async def get_recording(recording_id: str, current_user: User = Depends(get_current_user)):
    """
    Get information about a specific video recording.
    """
    adapter = get_drone_adapter()
    recording = await adapter.get_video_client().get_recording(recording_id)
    
    if not recording:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Recording with ID {recording_id} not found"
        )
    
    return recording

@router.post("/recordings/start")
async def start_recording(
    data: Dict[str, Any],
    current_user: User = Depends(get_current_user)
):
    """
    Start recording video from a drone.
    """
    adapter = get_drone_adapter()
    
    drone_id = data.get("drone_id")
    name = data.get("name")
    
    if not drone_id:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="drone_id is required"
        )
    
    result = await adapter.get_video_client().start_recording(drone_id, name)
    
    if "error" in result:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=result["error"]
        )
    
    return result

@router.post("/recordings/{recording_id}/stop")
async def stop_recording(recording_id: str, current_user: User = Depends(get_current_user)):
    """
    Stop recording video.
    """
    adapter = get_drone_adapter()
    result = await adapter.get_video_client().stop_recording(recording_id)
    
    if "error" in result:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=result["error"]
        )
    
    return result

@router.delete("/recordings/{recording_id}")
async def delete_recording(recording_id: str, current_user: User = Depends(get_current_user)):
    """
    Delete a video recording.
    """
    adapter = get_drone_adapter()
    success = await adapter.get_video_client().delete_recording(recording_id)
    
    if not success:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to delete recording"
        )
    
    return {"message": "Recording deleted successfully"}

@router.get("/recordings/{recording_id}/url")
async def get_recording_url(recording_id: str, current_user: User = Depends(get_current_user)):
    """
    Get the URL for a video recording.
    """
    adapter = get_drone_adapter()
    url = await adapter.get_video_client().get_recording_url(recording_id)
    
    if not url:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"URL for recording {recording_id} not found"
        )
    
    return {"url": url}
