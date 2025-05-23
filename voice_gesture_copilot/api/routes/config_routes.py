"""
Configuration API routes for Voice & Gesture Co-Pilot.

This module provides API endpoints for configuration management.
"""

import os
import time
from typing import Dict, Any, List, Optional
from datetime import datetime

from fastapi import APIRouter, Depends, HTTPException, status, Request, Body
from fastapi.responses import JSONResponse

from voice_gesture_copilot.core.security import get_current_user
from voice_gesture_copilot.core.config import settings

router = APIRouter()

@router.get("/voice-commands")
async def get_voice_commands(
    request: Request,
    user: Dict[str, Any] = Depends(get_current_user),
):
    """
    Get voice command definitions.
    
    Args:
        request: FastAPI request object
        user: Current authenticated user
        
    Returns:
        Voice command definitions
    """
    try:
        return {"voice_commands": settings.VOICE_COMMANDS}
    
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error getting voice command definitions: {str(e)}",
        )

@router.post("/voice-commands")
async def update_voice_commands(
    request: Request,
    voice_commands: Dict[str, Dict[str, Any]] = Body(...),
    user: Dict[str, Any] = Depends(get_current_user),
):
    """
    Update voice command definitions.
    
    Args:
        request: FastAPI request object
        voice_commands: Updated voice command definitions
        user: Current authenticated user
        
    Returns:
        Updated voice command definitions
    """
    try:
        # Validate voice commands
        for command_name, command_config in voice_commands.items():
            if "phrases" not in command_config:
                raise HTTPException(
                    status_code=status.HTTP_400_BAD_REQUEST,
                    detail=f"Command '{command_name}' is missing 'phrases' field",
                )
            
            if not isinstance(command_config["phrases"], list):
                raise HTTPException(
                    status_code=status.HTTP_400_BAD_REQUEST,
                    detail=f"Command '{command_name}' 'phrases' field must be a list",
                )
            
            if "confidence_threshold" not in command_config:
                raise HTTPException(
                    status_code=status.HTTP_400_BAD_REQUEST,
                    detail=f"Command '{command_name}' is missing 'confidence_threshold' field",
                )
            
            if not isinstance(command_config["confidence_threshold"], (int, float)):
                raise HTTPException(
                    status_code=status.HTTP_400_BAD_REQUEST,
                    detail=f"Command '{command_name}' 'confidence_threshold' field must be a number",
                )
            
            if "requires_confirmation" not in command_config:
                raise HTTPException(
                    status_code=status.HTTP_400_BAD_REQUEST,
                    detail=f"Command '{command_name}' is missing 'requires_confirmation' field",
                )
            
            if not isinstance(command_config["requires_confirmation"], bool):
                raise HTTPException(
                    status_code=status.HTTP_400_BAD_REQUEST,
                    detail=f"Command '{command_name}' 'requires_confirmation' field must be a boolean",
                )
        
        # Update voice commands
        settings.VOICE_COMMANDS = voice_commands
        
        return {"voice_commands": settings.VOICE_COMMANDS}
    
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error updating voice command definitions: {str(e)}",
        )

@router.get("/gestures")
async def get_gestures(
    request: Request,
    user: Dict[str, Any] = Depends(get_current_user),
):
    """
    Get gesture definitions.
    
    Args:
        request: FastAPI request object
        user: Current authenticated user
        
    Returns:
        Gesture definitions
    """
    try:
        return {"gestures": settings.GESTURE_DEFINITIONS}
    
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error getting gesture definitions: {str(e)}",
        )

@router.post("/gestures")
async def update_gestures(
    request: Request,
    gestures: Dict[str, Dict[str, Any]] = Body(...),
    user: Dict[str, Any] = Depends(get_current_user),
):
    """
    Update gesture definitions.
    
    Args:
        request: FastAPI request object
        gestures: Updated gesture definitions
        user: Current authenticated user
        
    Returns:
        Updated gesture definitions
    """
    try:
        # Validate gestures
        for gesture_name, gesture_config in gestures.items():
            if "description" not in gesture_config:
                raise HTTPException(
                    status_code=status.HTTP_400_BAD_REQUEST,
                    detail=f"Gesture '{gesture_name}' is missing 'description' field",
                )
            
            if "confidence_threshold" not in gesture_config:
                raise HTTPException(
                    status_code=status.HTTP_400_BAD_REQUEST,
                    detail=f"Gesture '{gesture_name}' is missing 'confidence_threshold' field",
                )
            
            if not isinstance(gesture_config["confidence_threshold"], (int, float)):
                raise HTTPException(
                    status_code=status.HTTP_400_BAD_REQUEST,
                    detail=f"Gesture '{gesture_name}' 'confidence_threshold' field must be a number",
                )
            
            if "cooldown_seconds" not in gesture_config:
                raise HTTPException(
                    status_code=status.HTTP_400_BAD_REQUEST,
                    detail=f"Gesture '{gesture_name}' is missing 'cooldown_seconds' field",
                )
            
            if not isinstance(gesture_config["cooldown_seconds"], (int, float)):
                raise HTTPException(
                    status_code=status.HTTP_400_BAD_REQUEST,
                    detail=f"Gesture '{gesture_name}' 'cooldown_seconds' field must be a number",
                )
        
        # Update gestures
        settings.GESTURE_DEFINITIONS = gestures
        
        return {"gestures": settings.GESTURE_DEFINITIONS}
    
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error updating gesture definitions: {str(e)}",
        )

@router.get("/performance")
async def get_performance_settings(
    request: Request,
    user: Dict[str, Any] = Depends(get_current_user),
):
    """
    Get performance settings.
    
    Args:
        request: FastAPI request object
        user: Current authenticated user
        
    Returns:
        Performance settings
    """
    try:
        return {
            "max_latency_ms": settings.MAX_LATENCY_MS,
            "target_accuracy": settings.TARGET_ACCURACY,
        }
    
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error getting performance settings: {str(e)}",
        )

@router.post("/performance")
async def update_performance_settings(
    request: Request,
    performance_settings: Dict[str, Any] = Body(...),
    user: Dict[str, Any] = Depends(get_current_user),
):
    """
    Update performance settings.
    
    Args:
        request: FastAPI request object
        performance_settings: Updated performance settings
        user: Current authenticated user
        
    Returns:
        Updated performance settings
    """
    try:
        # Validate performance settings
        if "max_latency_ms" in performance_settings:
            if not isinstance(performance_settings["max_latency_ms"], (int, float)):
                raise HTTPException(
                    status_code=status.HTTP_400_BAD_REQUEST,
                    detail="'max_latency_ms' field must be a number",
                )
            
            settings.MAX_LATENCY_MS = performance_settings["max_latency_ms"]
        
        if "target_accuracy" in performance_settings:
            if not isinstance(performance_settings["target_accuracy"], (int, float)):
                raise HTTPException(
                    status_code=status.HTTP_400_BAD_REQUEST,
                    detail="'target_accuracy' field must be a number",
                )
            
            settings.TARGET_ACCURACY = performance_settings["target_accuracy"]
        
        return {
            "max_latency_ms": settings.MAX_LATENCY_MS,
            "target_accuracy": settings.TARGET_ACCURACY,
        }
    
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error updating performance settings: {str(e)}",
        )

@router.get("/all")
async def get_all_settings(
    request: Request,
    user: Dict[str, Any] = Depends(get_current_user),
):
    """
    Get all configuration settings.
    
    Args:
        request: FastAPI request object
        user: Current authenticated user
        
    Returns:
        All configuration settings
    """
    try:
        # Convert settings to dictionary
        settings_dict = {
            "voice_commands": settings.VOICE_COMMANDS,
            "gesture_definitions": settings.GESTURE_DEFINITIONS,
            "max_latency_ms": settings.MAX_LATENCY_MS,
            "target_accuracy": settings.TARGET_ACCURACY,
            "whisper_model": settings.WHISPER_MODEL,
            "whisper_language": settings.WHISPER_LANGUAGE,
            "whisper_beam_size": settings.WHISPER_BEAM_SIZE,
            "whisper_vad_filter": settings.WHISPER_VAD_FILTER,
            "audio_sample_rate": settings.AUDIO_SAMPLE_RATE,
            "audio_channels": settings.AUDIO_CHANNELS,
            "mediapipe_model_complexity": settings.MEDIAPIPE_MODEL_COMPLEXITY,
            "mediapipe_min_detection_confidence": settings.MEDIAPIPE_MIN_DETECTION_CONFIDENCE,
            "mediapipe_min_tracking_confidence": settings.MEDIAPIPE_MIN_TRACKING_CONFIDENCE,
            "command_confidence_threshold": settings.COMMAND_CONFIDENCE_THRESHOLD,
        }
        
        return settings_dict
    
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error getting all settings: {str(e)}",
        )
