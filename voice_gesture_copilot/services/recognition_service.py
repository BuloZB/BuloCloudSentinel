"""
Recognition Service for Voice & Gesture Co-Pilot.

This service provides voice and gesture recognition capabilities.
"""

import os
import time
import logging
import asyncio
from typing import Dict, Any, List, Optional, Tuple
from pathlib import Path

import numpy as np
import cv2
import mediapipe as mp
from loguru import logger

from voice_gesture_copilot.core.config import settings
from voice_gesture_copilot.models.whisper_asr.whisper_cpp import WhisperASR
from voice_gesture_copilot.models.mediapipe.gesture_detector import GestureDetector

class RecognitionService:
    """
    Service for voice and gesture recognition.
    
    This service provides methods for recognizing voice commands and gestures
    using Whisper-cpp and MediaPipe.
    """
    
    def __init__(self):
        """Initialize the recognition service."""
        self.whisper_asr = None
        self.gesture_detector = None
        self.is_initialized = False
        self.performance_metrics = {
            "voice_recognition": {
                "latency_ms": [],
                "accuracy": [],
            },
            "gesture_recognition": {
                "latency_ms": [],
                "accuracy": [],
            },
        }
    
    async def load_models(self):
        """
        Load the voice and gesture recognition models.
        
        This method initializes the Whisper-cpp ASR model and MediaPipe
        gesture detection model.
        """
        logger.info("Loading voice and gesture recognition models")
        
        # Create models directory if it doesn't exist
        models_dir = Path("models")
        models_dir.mkdir(exist_ok=True)
        
        # Initialize Whisper ASR
        self.whisper_asr = WhisperASR(
            model_name=settings.WHISPER_MODEL,
            language=settings.WHISPER_LANGUAGE,
            beam_size=settings.WHISPER_BEAM_SIZE,
            vad_filter=settings.WHISPER_VAD_FILTER,
        )
        await self.whisper_asr.initialize()
        
        # Initialize MediaPipe gesture detector
        self.gesture_detector = GestureDetector(
            model_complexity=settings.MEDIAPIPE_MODEL_COMPLEXITY,
            min_detection_confidence=settings.MEDIAPIPE_MIN_DETECTION_CONFIDENCE,
            min_tracking_confidence=settings.MEDIAPIPE_MIN_TRACKING_CONFIDENCE,
            gesture_definitions=settings.GESTURE_DEFINITIONS,
        )
        await self.gesture_detector.initialize()
        
        self.is_initialized = True
        logger.info("Voice and gesture recognition models loaded successfully")
    
    async def cleanup(self):
        """
        Clean up resources used by the recognition service.
        
        This method releases resources used by the Whisper-cpp ASR model
        and MediaPipe gesture detection model.
        """
        logger.info("Cleaning up recognition service resources")
        
        if self.whisper_asr:
            await self.whisper_asr.cleanup()
        
        if self.gesture_detector:
            await self.gesture_detector.cleanup()
        
        self.is_initialized = False
        logger.info("Recognition service resources cleaned up")
    
    async def recognize_voice(self, audio_data: bytes) -> Dict[str, Any]:
        """
        Recognize voice commands from audio data.
        
        Args:
            audio_data: Raw audio data in bytes
            
        Returns:
            Dictionary containing recognition results
        """
        if not self.is_initialized:
            logger.error("Recognition service not initialized")
            return {"error": "Recognition service not initialized"}
        
        try:
            # Measure latency
            start_time = time.time()
            
            # Process audio with Whisper ASR
            result = await self.whisper_asr.transcribe(audio_data)
            
            # Calculate latency
            latency_ms = (time.time() - start_time) * 1000
            self.performance_metrics["voice_recognition"]["latency_ms"].append(latency_ms)
            
            # Log result
            logger.debug(f"Voice recognition result: {result}")
            logger.debug(f"Voice recognition latency: {latency_ms:.2f} ms")
            
            # Check if latency exceeds maximum
            if latency_ms > settings.MAX_LATENCY_MS:
                logger.warning(f"Voice recognition latency ({latency_ms:.2f} ms) exceeds maximum ({settings.MAX_LATENCY_MS} ms)")
            
            # Add latency to result
            result["latency_ms"] = latency_ms
            
            return result
        
        except Exception as e:
            logger.error(f"Error recognizing voice: {str(e)}")
            return {"error": str(e)}
    
    async def recognize_gesture(self, image_data: bytes) -> Dict[str, Any]:
        """
        Recognize gestures from image data.
        
        Args:
            image_data: Raw image data in bytes
            
        Returns:
            Dictionary containing recognition results
        """
        if not self.is_initialized:
            logger.error("Recognition service not initialized")
            return {"error": "Recognition service not initialized"}
        
        try:
            # Measure latency
            start_time = time.time()
            
            # Process image with MediaPipe
            result = await self.gesture_detector.detect_gesture(image_data)
            
            # Calculate latency
            latency_ms = (time.time() - start_time) * 1000
            self.performance_metrics["gesture_recognition"]["latency_ms"].append(latency_ms)
            
            # Log result
            logger.debug(f"Gesture recognition result: {result}")
            logger.debug(f"Gesture recognition latency: {latency_ms:.2f} ms")
            
            # Check if latency exceeds maximum
            if latency_ms > settings.MAX_LATENCY_MS:
                logger.warning(f"Gesture recognition latency ({latency_ms:.2f} ms) exceeds maximum ({settings.MAX_LATENCY_MS} ms)")
            
            # Add latency to result
            result["latency_ms"] = latency_ms
            
            return result
        
        except Exception as e:
            logger.error(f"Error recognizing gesture: {str(e)}")
            return {"error": str(e)}
    
    async def status(self) -> Dict[str, Any]:
        """
        Get the status of the recognition service.
        
        Returns:
            Dictionary containing service status
        """
        status_info = {
            "initialized": self.is_initialized,
            "voice_recognition": {
                "available": self.whisper_asr is not None and self.whisper_asr.is_initialized,
                "model": settings.WHISPER_MODEL if self.whisper_asr else None,
            },
            "gesture_recognition": {
                "available": self.gesture_detector is not None and self.gesture_detector.is_initialized,
                "model_complexity": settings.MEDIAPIPE_MODEL_COMPLEXITY if self.gesture_detector else None,
            },
            "performance": {
                "voice_recognition": {
                    "avg_latency_ms": np.mean(self.performance_metrics["voice_recognition"]["latency_ms"]).item() if self.performance_metrics["voice_recognition"]["latency_ms"] else None,
                    "max_latency_ms": np.max(self.performance_metrics["voice_recognition"]["latency_ms"]).item() if self.performance_metrics["voice_recognition"]["latency_ms"] else None,
                },
                "gesture_recognition": {
                    "avg_latency_ms": np.mean(self.performance_metrics["gesture_recognition"]["latency_ms"]).item() if self.performance_metrics["gesture_recognition"]["latency_ms"] else None,
                    "max_latency_ms": np.max(self.performance_metrics["gesture_recognition"]["latency_ms"]).item() if self.performance_metrics["gesture_recognition"]["latency_ms"] else None,
                },
            },
        }
        
        return status_info
