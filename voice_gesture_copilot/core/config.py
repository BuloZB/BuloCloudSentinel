"""
Configuration module for Voice & Gesture Co-Pilot.

This module provides configuration settings for the Voice & Gesture Co-Pilot module,
loaded from environment variables with sensible defaults.
"""

import os
from typing import List, Dict, Any, Optional
from pydantic import BaseSettings, Field, validator

class Settings(BaseSettings):
    """
    Configuration settings for the Voice & Gesture Co-Pilot module.
    """
    # General settings
    DEBUG: bool = Field(False, env="DEBUG")
    HOST: str = Field("0.0.0.0", env="HOST")
    PORT: int = Field(8080, env="PORT")
    LOG_LEVEL: str = Field("INFO", env="LOG_LEVEL")
    
    # CORS settings
    CORS_ORIGINS: List[str] = Field(
        ["http://localhost", "http://localhost:3000", "http://localhost:8000"],
        env="CORS_ORIGINS"
    )
    
    # API settings
    SENTINEL_API_URL: str = Field("http://bulocloud-sentinel-api:8000", env="SENTINEL_API_URL")
    SENTINEL_API_TOKEN: str = Field("", env="SENTINEL_API_TOKEN")
    
    # Voice recognition settings
    WHISPER_MODEL: str = Field("tiny-int8", env="WHISPER_MODEL")
    WHISPER_LANGUAGE: str = Field("en", env="WHISPER_LANGUAGE")
    WHISPER_BEAM_SIZE: int = Field(5, env="WHISPER_BEAM_SIZE")
    WHISPER_VAD_FILTER: bool = Field(True, env="WHISPER_VAD_FILTER")
    AUDIO_SAMPLE_RATE: int = Field(16000, env="AUDIO_SAMPLE_RATE")
    AUDIO_CHANNELS: int = Field(1, env="AUDIO_CHANNELS")
    
    # Gesture recognition settings
    MEDIAPIPE_MODEL_COMPLEXITY: int = Field(1, env="MEDIAPIPE_MODEL_COMPLEXITY")
    MEDIAPIPE_MIN_DETECTION_CONFIDENCE: float = Field(0.5, env="MEDIAPIPE_MIN_DETECTION_CONFIDENCE")
    MEDIAPIPE_MIN_TRACKING_CONFIDENCE: float = Field(0.5, env="MEDIAPIPE_MIN_TRACKING_CONFIDENCE")
    
    # Command processing settings
    RASA_MODEL_DIR: str = Field("models/rasa", env="RASA_MODEL_DIR")
    COMMAND_CONFIDENCE_THRESHOLD: float = Field(0.7, env="COMMAND_CONFIDENCE_THRESHOLD")
    
    # Performance settings
    MAX_LATENCY_MS: int = Field(400, env="MAX_LATENCY_MS")
    TARGET_ACCURACY: float = Field(0.95, env="TARGET_ACCURACY")
    
    # Security settings
    JWT_SECRET_KEY: str = Field("", env="JWT_SECRET_KEY")
    JWT_ALGORITHM: str = Field("HS256", env="JWT_ALGORITHM")
    JWT_EXPIRATION_MINUTES: int = Field(60, env="JWT_EXPIRATION_MINUTES")
    
    # Predefined gestures
    GESTURE_DEFINITIONS: Dict[str, Dict[str, Any]] = {
        "takeoff": {
            "description": "Raise both hands above shoulders with palms facing up",
            "confidence_threshold": 0.8,
            "cooldown_seconds": 3.0,
        },
        "land": {
            "description": "Lower both hands to waist level with palms facing down",
            "confidence_threshold": 0.8,
            "cooldown_seconds": 3.0,
        },
        "return_home": {
            "description": "Cross arms over chest in X pattern",
            "confidence_threshold": 0.85,
            "cooldown_seconds": 5.0,
        },
        "move_forward": {
            "description": "Right arm extended forward with palm facing forward",
            "confidence_threshold": 0.7,
            "cooldown_seconds": 1.0,
        },
        "move_backward": {
            "description": "Right arm extended backward with palm facing backward",
            "confidence_threshold": 0.7,
            "cooldown_seconds": 1.0,
        },
        "move_left": {
            "description": "Right arm extended to the left with palm facing left",
            "confidence_threshold": 0.7,
            "cooldown_seconds": 1.0,
        },
        "move_right": {
            "description": "Right arm extended to the right with palm facing right",
            "confidence_threshold": 0.7,
            "cooldown_seconds": 1.0,
        },
        "move_up": {
            "description": "Right arm extended upward with palm facing up",
            "confidence_threshold": 0.7,
            "cooldown_seconds": 1.0,
        },
        "move_down": {
            "description": "Right arm extended downward with palm facing down",
            "confidence_threshold": 0.7,
            "cooldown_seconds": 1.0,
        },
        "stop": {
            "description": "Open palm facing forward (traffic stop gesture)",
            "confidence_threshold": 0.9,
            "cooldown_seconds": 0.5,
        },
        "emergency_stop": {
            "description": "Both arms crossed in front of face",
            "confidence_threshold": 0.95,
            "cooldown_seconds": 0.0,  # No cooldown for emergency stop
        },
    }
    
    # Predefined voice commands
    VOICE_COMMANDS: Dict[str, Dict[str, Any]] = {
        "takeoff": {
            "phrases": ["take off", "launch", "start flight", "lift off"],
            "confidence_threshold": 0.8,
            "requires_confirmation": True,
        },
        "land": {
            "phrases": ["land", "land now", "touch down", "return to ground"],
            "confidence_threshold": 0.8,
            "requires_confirmation": True,
        },
        "return_home": {
            "phrases": ["return home", "return to base", "come back", "rtb"],
            "confidence_threshold": 0.85,
            "requires_confirmation": True,
        },
        "move_forward": {
            "phrases": ["move forward", "go forward", "advance", "forward"],
            "confidence_threshold": 0.7,
            "requires_confirmation": False,
        },
        "move_backward": {
            "phrases": ["move backward", "go backward", "back up", "reverse", "backward"],
            "confidence_threshold": 0.7,
            "requires_confirmation": False,
        },
        "move_left": {
            "phrases": ["move left", "go left", "left"],
            "confidence_threshold": 0.7,
            "requires_confirmation": False,
        },
        "move_right": {
            "phrases": ["move right", "go right", "right"],
            "confidence_threshold": 0.7,
            "requires_confirmation": False,
        },
        "move_up": {
            "phrases": ["move up", "go up", "ascend", "increase altitude", "up"],
            "confidence_threshold": 0.7,
            "requires_confirmation": False,
        },
        "move_down": {
            "phrases": ["move down", "go down", "descend", "decrease altitude", "down"],
            "confidence_threshold": 0.7,
            "requires_confirmation": False,
        },
        "stop": {
            "phrases": ["stop", "halt", "hover", "hold position", "stay"],
            "confidence_threshold": 0.9,
            "requires_confirmation": False,
        },
        "emergency_stop": {
            "phrases": ["emergency stop", "abort", "emergency abort", "stop immediately"],
            "confidence_threshold": 0.95,
            "requires_confirmation": False,
        },
    }
    
    class Config:
        env_file = ".env"
        env_file_encoding = "utf-8"
        case_sensitive = True

# Create settings instance
settings = Settings()
