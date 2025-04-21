"""
Streams API routes for the Vision System.

This module provides endpoints for managing video streams.
"""

from fastapi import APIRouter, Depends, HTTPException, Request, BackgroundTasks
from typing import List, Dict, Any, Optional
from pydantic import BaseModel

# Import local modules
from services.stream_service import StreamService
from api.schemas.stream import (
    Stream,
    StreamCreate,
    StreamUpdate,
    StreamStatus,
    StreamSource
)

router = APIRouter()

# Models
class StreamSourceConfig(BaseModel):
    """Request model for configuring a stream source."""
    source_type: str
    url: str
    credentials: Optional[Dict[str, str]] = None
    parameters: Optional[Dict[str, Any]] = None

# Helper functions
def get_stream_service(request: Request) -> StreamService:
    """Get the stream service from the app state."""
    return request.app.state.stream_service

# Routes
@router.get("/")
async def get_all_streams(
    status: Optional[str] = None,
    stream_service: StreamService = Depends(get_stream_service)
):
    """Get all streams, optionally filtered by status."""
    try:
        streams = await stream_service.get_all_streams(status)
        return streams
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error getting streams: {str(e)}")

@router.get("/{stream_id}")
async def get_stream(
    stream_id: str,
    stream_service: StreamService = Depends(get_stream_service)
):
    """Get a specific stream by ID."""
    try:
        stream = await stream_service.get_stream(stream_id)
        if not stream:
            raise HTTPException(status_code=404, detail=f"Stream {stream_id} not found")
        return stream
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error getting stream: {str(e)}")

@router.post("/")
async def create_stream(
    stream: StreamCreate,
    stream_service: StreamService = Depends(get_stream_service)
):
    """Create a new stream."""
    try:
        new_stream = await stream_service.create_stream(stream)
        return new_stream
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error creating stream: {str(e)}")

@router.put("/{stream_id}")
async def update_stream(
    stream_id: str,
    stream: StreamUpdate,
    stream_service: StreamService = Depends(get_stream_service)
):
    """Update a stream."""
    try:
        updated_stream = await stream_service.update_stream(stream_id, stream)
        if not updated_stream:
            raise HTTPException(status_code=404, detail=f"Stream {stream_id} not found")
        return updated_stream
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error updating stream: {str(e)}")

@router.delete("/{stream_id}")
async def delete_stream(
    stream_id: str,
    stream_service: StreamService = Depends(get_stream_service)
):
    """Delete a stream."""
    try:
        success = await stream_service.delete_stream(stream_id)
        if not success:
            raise HTTPException(status_code=404, detail=f"Stream {stream_id} not found")
        return {"success": True}
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error deleting stream: {str(e)}")

@router.post("/{stream_id}/start")
async def start_stream(
    stream_id: str,
    background_tasks: BackgroundTasks,
    stream_service: StreamService = Depends(get_stream_service)
):
    """Start a stream."""
    try:
        # Check if stream exists
        stream = await stream_service.get_stream(stream_id)
        if not stream:
            raise HTTPException(status_code=404, detail=f"Stream {stream_id} not found")
        
        # Start stream in background
        background_tasks.add_task(stream_service.start_stream, stream_id)
        
        return {"message": f"Stream {stream_id} starting"}
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error starting stream: {str(e)}")

@router.post("/{stream_id}/stop")
async def stop_stream(
    stream_id: str,
    background_tasks: BackgroundTasks,
    stream_service: StreamService = Depends(get_stream_service)
):
    """Stop a stream."""
    try:
        # Check if stream exists
        stream = await stream_service.get_stream(stream_id)
        if not stream:
            raise HTTPException(status_code=404, detail=f"Stream {stream_id} not found")
        
        # Stop stream in background
        background_tasks.add_task(stream_service.stop_stream, stream_id)
        
        return {"message": f"Stream {stream_id} stopping"}
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error stopping stream: {str(e)}")

@router.get("/{stream_id}/status")
async def get_stream_status(
    stream_id: str,
    stream_service: StreamService = Depends(get_stream_service)
):
    """Get the status of a stream."""
    try:
        status = await stream_service.get_stream_status(stream_id)
        if not status:
            raise HTTPException(status_code=404, detail=f"Stream {stream_id} not found")
        return status
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error getting stream status: {str(e)}")

@router.get("/{stream_id}/snapshot")
async def get_stream_snapshot(
    stream_id: str,
    stream_service: StreamService = Depends(get_stream_service)
):
    """Get a snapshot from a stream."""
    try:
        snapshot = await stream_service.get_stream_snapshot(stream_id)
        if not snapshot:
            raise HTTPException(status_code=404, detail=f"Stream {stream_id} not found or not active")
        
        # Return snapshot as image
        return Response(content=snapshot, media_type="image/jpeg")
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error getting stream snapshot: {str(e)}")

@router.get("/{stream_id}/recordings")
async def get_stream_recordings(
    stream_id: str,
    start_time: Optional[datetime] = None,
    end_time: Optional[datetime] = None,
    limit: int = 10,
    stream_service: StreamService = Depends(get_stream_service)
):
    """Get recordings for a stream."""
    try:
        recordings = await stream_service.get_stream_recordings(
            stream_id=stream_id,
            start_time=start_time,
            end_time=end_time,
            limit=limit
        )
        return recordings
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error getting stream recordings: {str(e)}")

@router.post("/{stream_id}/record/start")
async def start_recording(
    stream_id: str,
    duration: Optional[int] = None,  # seconds, None for continuous
    stream_service: StreamService = Depends(get_stream_service)
):
    """Start recording a stream."""
    try:
        recording_id = await stream_service.start_recording(stream_id, duration)
        return {"recording_id": recording_id}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error starting recording: {str(e)}")

@router.post("/{stream_id}/record/stop")
async def stop_recording(
    stream_id: str,
    stream_service: StreamService = Depends(get_stream_service)
):
    """Stop recording a stream."""
    try:
        success = await stream_service.stop_recording(stream_id)
        return {"success": success}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error stopping recording: {str(e)}")

@router.get("/sources")
async def get_stream_sources(
    stream_service: StreamService = Depends(get_stream_service)
):
    """Get all available stream sources."""
    try:
        sources = await stream_service.get_stream_sources()
        return sources
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error getting stream sources: {str(e)}")

@router.post("/sources/test")
async def test_stream_source(
    source: StreamSourceConfig,
    stream_service: StreamService = Depends(get_stream_service)
):
    """Test a stream source configuration."""
    try:
        result = await stream_service.test_stream_source(
            source_type=source.source_type,
            url=source.url,
            credentials=source.credentials,
            parameters=source.parameters
        )
        return result
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error testing stream source: {str(e)}")
