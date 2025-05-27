"""
Gesture recognition API routes for Voice & Gesture Co-Pilot.

This module provides API endpoints for gesture recognition and command processing.
"""

import os
import time
import base64
from typing import Dict, Any, List, Optional
from datetime import datetime

from fastapi import APIRouter, Depends, HTTPException, status, Request, UploadFile, File, Form, Body
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
async def recognize_gesture(
    request: Request,
    image_file: UploadFile = File(...),
    user: Dict[str, Any] = Depends(get_current_user),
):
    """
    Recognize gestures from image data.

    Args:
        request: FastAPI request object
        image_file: Image file to process
        user: Current authenticated user

    Returns:
        Recognition results
    """
    # Get services
    recognition_service = get_recognition_service(request)

    try:
        # Read image data
        image_data = await image_file.read()

        # Process image with recognition service
        result = await recognition_service.recognize_gesture(image_data)

        return result

    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error recognizing gesture: {str(e)}",
        )

@router.post("/recognize-base64")
async def recognize_gesture_base64(
    request: Request,
    data: Dict[str, Any] = Body(...),
    user: Dict[str, Any] = Depends(get_current_user),
):
    """
    Recognize gestures from base64-encoded image data.

    Args:
        request: FastAPI request object
        data: Request body containing base64-encoded image data
        user: Current authenticated user

    Returns:
        Recognition results
    """
    # Get services
    recognition_service = get_recognition_service(request)

    try:
        # Extract base64 image data
        base64_image = data.get("image", "")
        if not base64_image:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="Base64 image data is required",
            )

        # Decode base64 image data
        try:
            # Remove data URL prefix if present
            if base64_image.startswith("data:image"):
                base64_image = base64_image.split(",")[1]

            image_data = base64.b64decode(base64_image)
        except Exception as e:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="Invalid base64 image data",
            )

        # Process image with recognition service
        result = await recognition_service.recognize_gesture(image_data)

        return result

    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error recognizing gesture: {str(e)}",
        )

@router.post("/process")
async def process_gesture(
    request: Request,
    gesture_data: Dict[str, Any] = Body(...),
    user: Dict[str, Any] = Depends(get_current_user),
):
    """
    Process a recognized gesture.

    Args:
        request: FastAPI request object
        gesture_data: Gesture recognition data
        user: Current authenticated user

    Returns:
        Command processing results
    """
    # Get services
    command_service = get_command_service(request)
    feedback_service = get_feedback_service(request)

    try:
        # Process gesture
        result = await command_service.process_gesture(gesture_data)

        # Generate feedback
        feedback = await feedback_service.generate_visual_feedback(
            "gesture_processing",
            {
                "gesture_data": gesture_data,
                "result": result,
            },
        )

        # Add feedback to result
        result["feedback"] = feedback

        return result

    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error processing gesture: {str(e)}",
        )

@router.post("/recognize-and-process")
async def recognize_and_process_gesture(
    request: Request,
    image_file: UploadFile = File(...),
    user: Dict[str, Any] = Depends(get_current_user),
):
    """
    Recognize and process gestures from image data.

    Args:
        request: FastAPI request object
        image_file: Image file to process
        user: Current authenticated user

    Returns:
        Recognition and processing results
    """
    # Get services
    recognition_service = get_recognition_service(request)
    command_service = get_command_service(request)
    feedback_service = get_feedback_service(request)

    try:
        # Read image data
        image_data = await image_file.read()

        # Process image with recognition service
        recognition_result = await recognition_service.recognize_gesture(image_data)

        # Extract recognized gesture
        gesture = recognition_result.get("gesture", "")

        # If gesture was recognized, process command
        if gesture:
            # Process gesture
            command_result = await command_service.process_gesture(recognition_result)

            # Generate feedback
            feedback = await feedback_service.generate_visual_feedback(
                "gesture_recognition_and_processing",
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
            # No gesture recognized
            result = {
                "recognition": recognition_result,
                "command": {
                    "success": False,
                    "message": "No gesture recognized",
                },
            }

        return result

    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Error recognizing and processing gesture",
        )

@router.post("/recognize-and-process-base64")
async def recognize_and_process_gesture_base64(
    request: Request,
    data: Dict[str, Any] = Body(...),
    user: Dict[str, Any] = Depends(get_current_user),
):
    """
    Recognize and process gestures from base64-encoded image data.

    Args:
        request: FastAPI request object
        data: Request body containing base64-encoded image data
        user: Current authenticated user

    Returns:
        Recognition and processing results
    """
    # Get services
    recognition_service = get_recognition_service(request)
    command_service = get_command_service(request)
    feedback_service = get_feedback_service(request)

    try:
        # Extract base64 image data
        base64_image = data.get("image", "")
        if not base64_image:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="Base64 image data is required",
            )

        # Decode base64 image data
        try:
            # Remove data URL prefix if present
            if base64_image.startswith("data:image"):
                base64_image = base64_image.split(",")[1]

            image_data = base64.b64decode(base64_image)
        except Exception as e:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail=f"Invalid base64 image data: {str(e)}",
            )

        # Process image with recognition service
        recognition_result = await recognition_service.recognize_gesture(image_data)

        # Extract recognized gesture
        gesture = recognition_result.get("gesture", "")

        # If gesture was recognized, process command
        if gesture:
            # Process gesture
            command_result = await command_service.process_gesture(recognition_result)

            # Generate feedback
            feedback = await feedback_service.generate_visual_feedback(
                "gesture_recognition_and_processing",
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
            # No gesture recognized
            result = {
                "recognition": recognition_result,
                "command": {
                    "success": False,
                    "message": "No gesture recognized",
                },
            }

        return result

    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error recognizing and processing gesture: {str(e)}",
        )

@router.get("/history")
async def get_gesture_history(
    request: Request,
    limit: int = 100,
    user: Dict[str, Any] = Depends(get_current_user),
):
    """
    Get gesture command history.

    Args:
        request: FastAPI request object
        limit: Maximum number of history items to return
        user: Current authenticated user

    Returns:
        Gesture command history
    """
    # Get services
    command_service = get_command_service(request)

    try:
        # Get command history
        history = command_service.command_history

        # Filter for gesture commands only
        gesture_history = [item for item in history if item.get("type") == "gesture"]

        # Limit results
        gesture_history = gesture_history[-limit:]

        return {"history": gesture_history}

    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error getting gesture history: {str(e)}",
        )

@router.get("/status")
async def get_gesture_recognition_status(
    request: Request,
    user: Dict[str, Any] = Depends(get_current_user),
):
    """
    Get gesture recognition status.

    Args:
        request: FastAPI request object
        user: Current authenticated user

    Returns:
        Gesture recognition status
    """
    # Get services
    recognition_service = get_recognition_service(request)

    try:
        # Get status
        status_info = await recognition_service.status()

        # Filter for gesture recognition status
        gesture_status = {
            "initialized": status_info.get("initialized", False),
            "gesture_recognition": status_info.get("gesture_recognition", {}),
            "performance": {
                "gesture_recognition": status_info.get("performance", {}).get("gesture_recognition", {}),
            },
        }

        return gesture_status

    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error getting gesture recognition status: {str(e)}",
        )
