"""
Tests for utility functions.

This module provides tests for utility functions in the Drone Show microservice.
"""

import pytest
import json
import os
import tempfile
from datetime import datetime

from drone_show_service.models.choreography import (
    Choreography, ChoreographyMetadata, ChoreographyType, ChoreographyStatus,
    DroneTrajectory, Waypoint, Position, LEDState, LEDColor, LEDEffect
)
from drone_show_service.utils.blender_converter import (
    convert_blender_animation, save_choreography_to_file, load_choreography_from_file
)


def test_save_and_load_choreography():
    """Test saving and loading a choreography to/from a file."""
    # Create a simple choreography
    metadata = ChoreographyMetadata(
        name="Test Choreography",
        description="Test description",
        author="Test Author",
        tags=["test", "demo"],
        duration=60.0,
        drone_count=1,
        status=ChoreographyStatus.DRAFT,
    )
    
    waypoints = [
        Waypoint(
            time=0.0,
            position=Position(lat=37.7749, lon=-122.4194, alt=10.0),
            heading=0.0,
        ),
        Waypoint(
            time=60.0,
            position=Position(lat=37.7751, lon=-122.4196, alt=10.0),
            heading=180.0,
        ),
    ]
    
    led_states = [
        LEDState(
            time=0.0,
            color=LEDColor(r=255, g=0, b=0),
            effect=LEDEffect.SOLID,
        ),
        LEDState(
            time=60.0,
            color=LEDColor(r=0, g=0, b=255),
            effect=LEDEffect.SOLID,
        ),
    ]
    
    trajectories = [
        DroneTrajectory(
            drone_id="drone_1",
            waypoints=waypoints,
            led_states=led_states,
        ),
    ]
    
    choreography = Choreography(
        id="test-choreography-id",
        metadata=metadata,
        type=ChoreographyType.WAYPOINT,
        trajectories=trajectories,
    )
    
    # Create a temporary file
    with tempfile.NamedTemporaryFile(delete=False, suffix=".json") as temp_file:
        file_path = temp_file.name
    
    try:
        # Save choreography to file
        result = save_choreography_to_file(choreography, file_path)
        assert result is True
        
        # Check that file exists
        assert os.path.exists(file_path)
        
        # Load choreography from file
        loaded_choreography = load_choreography_from_file(file_path)
        
        # Check that loaded choreography matches original
        assert loaded_choreography is not None
        assert loaded_choreography.id == choreography.id
        assert loaded_choreography.metadata.name == choreography.metadata.name
        assert loaded_choreography.metadata.description == choreography.metadata.description
        assert loaded_choreography.metadata.author == choreography.metadata.author
        assert loaded_choreography.metadata.tags == choreography.metadata.tags
        assert loaded_choreography.metadata.duration == choreography.metadata.duration
        assert loaded_choreography.metadata.drone_count == choreography.metadata.drone_count
        assert loaded_choreography.metadata.status == choreography.metadata.status
        assert loaded_choreography.type == choreography.type
        assert len(loaded_choreography.trajectories) == len(choreography.trajectories)
        assert loaded_choreography.trajectories[0].drone_id == choreography.trajectories[0].drone_id
        assert len(loaded_choreography.trajectories[0].waypoints) == len(choreography.trajectories[0].waypoints)
        assert len(loaded_choreography.trajectories[0].led_states) == len(choreography.trajectories[0].led_states)
    finally:
        # Clean up
        if os.path.exists(file_path):
            os.unlink(file_path)


def test_convert_blender_animation():
    """Test converting a Blender animation to a choreography."""
    # Create sample Blender animation data
    animation_data = {
        "metadata": {
            "blender_version": "3.0.0",
            "export_time": "2023-06-01 12:00:00",
            "reference_lat": 37.7749,
            "reference_lon": -122.4194,
            "reference_alt": 0.0,
            "frame_rate": 30.0,
            "time_scale": 1.0,
            "frame_start": 1,
            "frame_end": 100,
        },
        "objects": [
            {
                "name": "drone_1",
                "is_drone": True,
                "frames": [
                    {
                        "time": 0.0,
                        "lat": 37.7749,
                        "lon": -122.4194,
                        "alt": 10.0,
                        "heading": 0.0,
                        "color": {
                            "r": 255,
                            "g": 0,
                            "b": 0
                        },
                        "effect": "solid"
                    },
                    {
                        "time": 60.0,
                        "lat": 37.7751,
                        "lon": -122.4196,
                        "alt": 10.0,
                        "heading": 180.0,
                        "color": {
                            "r": 0,
                            "g": 0,
                            "b": 255
                        },
                        "effect": "solid"
                    }
                ]
            },
            {
                "name": "camera",
                "is_drone": False,
                "frames": [
                    {
                        "time": 0.0,
                        "lat": 37.7749,
                        "lon": -122.4194,
                        "alt": 20.0,
                        "heading": 0.0,
                    }
                ]
            }
        ]
    }
    
    # Convert to choreography
    choreography = convert_blender_animation(
        animation_data=animation_data,
        name="Blender Test",
        description="Test description",
        author="Test Author",
    )
    
    # Check values
    assert choreography.metadata.name == "Blender Test"
    assert choreography.metadata.description == "Test description"
    assert choreography.metadata.author == "Test Author"
    assert choreography.metadata.tags == ["blender", "imported"]
    assert choreography.metadata.duration == 60.0
    assert choreography.metadata.drone_count == 1
    assert choreography.metadata.status == ChoreographyStatus.DRAFT
    assert choreography.type == ChoreographyType.BLENDER
    assert len(choreography.trajectories) == 1
    assert choreography.trajectories[0].drone_id == "drone_1"
    assert len(choreography.trajectories[0].waypoints) == 2
    assert len(choreography.trajectories[0].led_states) == 2
