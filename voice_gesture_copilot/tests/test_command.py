"""
Tests for the command service.

This module contains tests for the command processing service.
"""

import os
import pytest
import asyncio
from unittest.mock import patch, MagicMock

from voice_gesture_copilot.services.command_service import CommandService
from voice_gesture_copilot.services.drone_service import DroneService
from voice_gesture_copilot.models.rasa_nlu.intent_classifier import IntentClassifier
from voice_gesture_copilot.models.mission_graph.mission_graph import MissionGraph

@pytest.fixture
def drone_service():
    """Create a drone service for testing."""
    service = MagicMock(spec=DroneService)
    service.is_initialized = True
    service.send_command.return_value = {
        "success": True,
        "message": "Command executed successfully",
    }
    return service

@pytest.fixture
async def command_service(drone_service):
    """Create a command service for testing."""
    service = CommandService(drone_service)
    
    # Mock the models
    service.intent_classifier = MagicMock(spec=IntentClassifier)
    service.intent_classifier.is_initialized = True
    service.intent_classifier.classify.return_value = {
        "intent": "takeoff",
        "confidence": 0.95,
        "entities": [],
        "latency_ms": 50.0,
    }
    
    service.mission_graph = MagicMock(spec=MissionGraph)
    service.mission_graph.is_initialized = True
    service.mission_graph.process_command.return_value = {
        "success": True,
        "message": "Command processed successfully",
        "command_type": "basic",
        "requires_confirmation": True,
        "drone_command": {
            "drone_id": "drone1",
            "command": "takeoff",
            "parameters": {
                "altitude": 5.0,
            },
        },
        "latency_ms": 20.0,
    }
    
    service.is_initialized = True
    
    yield service
    
    # Clean up
    await service.cleanup()

@pytest.mark.asyncio
async def test_process_voice_command(command_service):
    """Test processing a voice command."""
    # Call the service
    result = await command_service.process_voice_command("take off")
    
    # Check that the models were called
    command_service.intent_classifier.classify.assert_called_once_with("take off")
    command_service.mission_graph.process_command.assert_called_once()
    
    # Check the result
    assert result["success"] is True
    assert "message" in result
    assert "command_type" in result
    assert "requires_confirmation" in result
    assert "drone_command" in result
    assert "latency_ms" in result

@pytest.mark.asyncio
async def test_process_gesture(command_service):
    """Test processing a gesture."""
    # Gesture data
    gesture_data = {
        "gesture": "takeoff",
        "confidence": 0.9,
        "has_pose": True,
        "has_left_hand": True,
        "has_right_hand": True,
        "has_face": True,
    }
    
    # Call the service
    result = await command_service.process_gesture(gesture_data)
    
    # Check that the models were called
    command_service.mission_graph.process_command.assert_called_once()
    
    # Check the result
    assert result["success"] is True
    assert "message" in result
    assert "command_type" in result
    assert "requires_confirmation" in result
    assert "drone_command" in result
    assert "latency_ms" in result

@pytest.mark.asyncio
async def test_status(command_service):
    """Test getting service status."""
    # Call the service
    status = await command_service.status()
    
    # Check the status
    assert status["initialized"] is True
    assert "intent_classifier" in status
    assert "mission_graph" in status
    assert "performance" in status
    assert "command_history_count" in status
