"""
Voice recognition API routes for Voice & Gesture Co-Pilot.

This module provides API endpoints for voice recognition and command processing.
"""

import os
import time
from typing import Dict, Any, List, Optional
from datetime import datetime

from fastapi import APIRouter, Depends, HTTPException, status, Request, UploadFile, File, Form
from fastapi.responses import JSONResponse

from voice_gesture_copilot.core.security import get_current_user
from voice_gesture_copilot.services.recognition_service import RecognitionService
from voice_gesture_copilot.services.command_service import CommandService
from voice_gesture_copilot.services.feedback_service import FeedbackService

router = APIRouter()

def get_recognition_service(request: Request) -> RecognitionService:
    """Get the recognition service from the request state."""
    return request.app.state.recognition_service

def get_command_service(request: Request) -> CommandService:
    """Get the command service from the request state."""
    return request.app.state.command_service

def get_feedback_service(request: Request) -> FeedbackService:
    """Get the feedback service from the request state."""
    return request.app.state.feedback_service

@router.post("/recognize")
async def recognize_voice(
    request: Request,
    audio_file: UploadFile = File(...),
    user: Dict[str, Any] = Depends(get_current_user),
):
    """
    Recognize voice commands from audio data.
    
    Args:
        request: FastAPI request object
        audio_file: Audio file to process
        user: Current authenticated user
        
    Returns:
        Recognition results
    """
    # Get services
    recognition_service = get_recognition_service(request)
    
    try:
        # Read audio data
        audio_data = await audio_file.read()
        
        # Process audio with recognition service
        result = await recognition_service.recognize_voice(audio_data)
        
        return result
    
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error recognizing voice: {str(e)}",
        )

@router.post("/process")
async def process_voice_command(
    request: Request,
    command_text: str = Form(...),
    user: Dict[str, Any] = Depends(get_current_user),
):
    """
    Process a voice command.
    
    Args:
        request: FastAPI request object
        command_text: Text of the voice command
        user: Current authenticated user
        
    Returns:
        Command processing results
    """
    # Get services
    command_service = get_command_service(request)
    feedback_service = get_feedback_service(request)
    
    try:
        # Process command
        result = await command_service.process_voice_command(command_text)
        
        # Generate feedback
        feedback = await feedback_service.generate_visual_feedback(
            "voice_command_processing",
            {
                "command_text": command_text,
                "result": result,
            },
        )
        
        # Add feedback to result
        result["feedback"] = feedback
        
        return result
    
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error processing voice command: {str(e)}",
        )

@router.post("/recognize-and-process")
async def recognize_and_process_voice(
    request: Request,
    audio_file: UploadFile = File(...),
    user: Dict[str, Any] = Depends(get_current_user),
):
    """
    Recognize and process voice commands from audio data.
    
    Args:
        request: FastAPI request object
        audio_file: Audio file to process
        user: Current authenticated user
        
    Returns:
        Recognition and processing results
    """
    # Get services
    recognition_service = get_recognition_service(request)
    command_service = get_command_service(request)
    feedback_service = get_feedback_service(request)
    
    try:
        # Read audio data
        audio_data = await audio_file.read()
        
        # Process audio with recognition service
        recognition_result = await recognition_service.recognize_voice(audio_data)
        
        # Extract recognized text
        command_text = recognition_result.get("text", "")
        
        # If text was recognized, process command
        if command_text:
            # Process command
            command_result = await command_service.process_voice_command(command_text)
            
            # Generate feedback
            feedback = await feedback_service.generate_visual_feedback(
                "voice_recognition_and_processing",
                {
                    "recognition_result": recognition_result,
                    "command_result": command_result,
                },
            )
            
            # Combine results
            result = {
                "recognition": recognition_result,
                "command": command_result,
                "feedback": feedback,
            }
        else:
            # No text recognized
            result = {
                "recognition": recognition_result,
                "command": {
                    "success": False,
                    "message": "No text recognized",
                },
            }
        
        return result
    
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error recognizing and processing voice: {str(e)}",
        )

@router.get("/history")
async def get_voice_command_history(
    request: Request,
    limit: int = 100,
    user: Dict[str, Any] = Depends(get_current_user),
):
    """
    Get voice command history.
    
    Args:
        request: FastAPI request object
        limit: Maximum number of history items to return
        user: Current authenticated user
        
    Returns:
        Voice command history
    """
    # Get services
    command_service = get_command_service(request)
    
    try:
        # Get command history
        history = command_service.command_history
        
        # Filter for voice commands only
        voice_history = [item for item in history if item.get("type") == "voice"]
        
        # Limit results
        voice_history = voice_history[-limit:]
        
        return {"history": voice_history}
    
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error getting voice command history: {str(e)}",
        )

@router.get("/status")
async def get_voice_recognition_status(
    request: Request,
    user: Dict[str, Any] = Depends(get_current_user),
):
    """
    Get voice recognition status.
    
    Args:
        request: FastAPI request object
        user: Current authenticated user
        
    Returns:
        Voice recognition status
    """
    # Get services
    recognition_service = get_recognition_service(request)
    
    try:
        # Get status
        status_info = await recognition_service.status()
        
        # Filter for voice recognition status
        voice_status = {
            "initialized": status_info.get("initialized", False),
            "voice_recognition": status_info.get("voice_recognition", {}),
            "performance": {
                "voice_recognition": status_info.get("performance", {}).get("voice_recognition", {}),
            },
        }
        
        return voice_status
    
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error getting voice recognition status: {str(e)}",
        )
