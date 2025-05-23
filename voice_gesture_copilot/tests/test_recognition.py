"""
Tests for the recognition service.

This module contains tests for the voice and gesture recognition service.
"""

import os
import pytest
import asyncio
from unittest.mock import patch, MagicMock

from voice_gesture_copilot.services.recognition_service import RecognitionService
from voice_gesture_copilot.models.whisper_asr.whisper_cpp import WhisperASR
from voice_gesture_copilot.models.mediapipe.gesture_detector import GestureDetector

# Sample test data
SAMPLE_AUDIO_DATA = b"SAMPLE_AUDIO_DATA"
SAMPLE_IMAGE_DATA = b"SAMPLE_IMAGE_DATA"

@pytest.fixture
async def recognition_service():
    """Create a recognition service for testing."""
    service = RecognitionService()
    
    # Mock the models
    service.whisper_asr = MagicMock(spec=WhisperASR)
    service.whisper_asr.is_initialized = True
    service.whisper_asr.transcribe.return_value = {
        "text": "take off",
        "language": "en",
        "confidence": 0.95,
        "latency_ms": 150.0,
    }
    
    service.gesture_detector = MagicMock(spec=GestureDetector)
    service.gesture_detector.is_initialized = True
    service.gesture_detector.detect_gesture.return_value = {
        "gesture": "takeoff",
        "confidence": 0.9,
        "has_pose": True,
        "has_left_hand": True,
        "has_right_hand": True,
        "has_face": True,
        "latency_ms": 50.0,
    }
    
    service.is_initialized = True
    
    yield service
    
    # Clean up
    await service.cleanup()

@pytest.mark.asyncio
async def test_recognize_voice(recognition_service):
    """Test voice recognition."""
    # Call the service
    result = await recognition_service.recognize_voice(SAMPLE_AUDIO_DATA)
    
    # Check that the model was called
    recognition_service.whisper_asr.transcribe.assert_called_once_with(SAMPLE_AUDIO_DATA)
    
    # Check the result
    assert result["text"] == "take off"
    assert result["language"] == "en"
    assert result["confidence"] == 0.95
    assert "latency_ms" in result

@pytest.mark.asyncio
async def test_recognize_gesture(recognition_service):
    """Test gesture recognition."""
    # Call the service
    result = await recognition_service.recognize_gesture(SAMPLE_IMAGE_DATA)
    
    # Check that the model was called
    recognition_service.gesture_detector.detect_gesture.assert_called_once_with(SAMPLE_IMAGE_DATA)
    
    # Check the result
    assert result["gesture"] == "takeoff"
    assert result["confidence"] == 0.9
    assert result["has_pose"] is True
    assert "latency_ms" in result

@pytest.mark.asyncio
async def test_status(recognition_service):
    """Test getting service status."""
    # Call the service
    status = await recognition_service.status()
    
    # Check the status
    assert status["initialized"] is True
    assert "voice_recognition" in status
    assert "gesture_recognition" in status
    assert "performance" in status
