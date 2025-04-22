"""
Tests for the choreography models and services.

This module provides tests for the choreography models and services.
"""

import pytest
import json
from datetime import datetime
from drone_show_service.models.choreography import (
    Choreography, ChoreographyMetadata, ChoreographyType, ChoreographyStatus,
    DroneTrajectory, Waypoint, Position, LEDState, LEDColor, LEDEffect
)
from drone_show_service.utils.blender_converter import convert_blender_animation


def test_choreography_model():
    """Test the Choreography model."""
    # Create metadata
    metadata = ChoreographyMetadata(
        name="Test Choreography",
        description="Test description",
        author="Test Author",
        tags=["test", "demo"],
        duration=60.0,
        drone_count=5,
        status=ChoreographyStatus.DRAFT,
    )
    
    # Create waypoints
    waypoints = [
        Waypoint(
            time=0.0,
            position=Position(lat=37.7749, lon=-122.4194, alt=10.0),
            heading=0.0,
        ),
        Waypoint(
            time=30.0,
            position=Position(lat=37.7750, lon=-122.4195, alt=15.0),
            heading=90.0,
        ),
        Waypoint(
            time=60.0,
            position=Position(lat=37.7751, lon=-122.4196, alt=10.0),
            heading=180.0,
        ),
    ]
    
    # Create LED states
    led_states = [
        LEDState(
            time=0.0,
            color=LEDColor(r=255, g=0, b=0),
            effect=LEDEffect.SOLID,
        ),
        LEDState(
            time=30.0,
            color=LEDColor(r=0, g=255, b=0),
            effect=LEDEffect.BLINK,
            effect_params={"frequency": 2.0},
        ),
        LEDState(
            time=60.0,
            color=LEDColor(r=0, g=0, b=255),
            effect=LEDEffect.SOLID,
        ),
    ]
    
    # Create trajectories
    trajectories = [
        DroneTrajectory(
            drone_id="drone_1",
            waypoints=waypoints,
            led_states=led_states,
        ),
    ]
    
    # Create choreography
    choreography = Choreography(
        metadata=metadata,
        type=ChoreographyType.WAYPOINT,
        trajectories=trajectories,
    )
    
    # Check values
    assert choreography.metadata.name == "Test Choreography"
    assert choreography.metadata.description == "Test description"
    assert choreography.metadata.author == "Test Author"
    assert choreography.metadata.tags == ["test", "demo"]
    assert choreography.metadata.duration == 60.0
    assert choreography.metadata.drone_count == 5
    assert choreography.metadata.status == ChoreographyStatus.DRAFT
    assert choreography.type == ChoreographyType.WAYPOINT
    assert len(choreography.trajectories) == 1
    assert choreography.trajectories[0].drone_id == "drone_1"
    assert len(choreography.trajectories[0].waypoints) == 3
    assert len(choreography.trajectories[0].led_states) == 3


def test_blender_converter():
    """Test the Blender animation converter."""
    # Create sample Blender animation data
    animation_data = {
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
                        "color": {"r": 255, "g": 0, "b": 0},
                        "effect": "solid",
                    },
                    {
                        "time": 30.0,
                        "lat": 37.7750,
                        "lon": -122.4195,
                        "alt": 15.0,
                        "heading": 90.0,
                        "color": {"r": 0, "g": 255, "b": 0},
                        "effect": "blink",
                        "effect_params": {"frequency": 2.0},
                    },
                    {
                        "time": 60.0,
                        "lat": 37.7751,
                        "lon": -122.4196,
                        "alt": 10.0,
                        "heading": 180.0,
                        "color": {"r": 0, "g": 0, "b": 255},
                        "effect": "solid",
                    },
                ],
            },
            {
                "name": "drone_2",
                "is_drone": True,
                "frames": [
                    {
                        "time": 0.0,
                        "lat": 37.7749,
                        "lon": -122.4194,
                        "alt": 10.0,
                        "heading": 0.0,
                        "color": {"r": 255, "g": 0, "b": 0},
                        "effect": "solid",
                    },
                    {
                        "time": 30.0,
                        "lat": 37.7750,
                        "lon": -122.4195,
                        "alt": 15.0,
                        "heading": 90.0,
                        "color": {"r": 0, "g": 255, "b": 0},
                        "effect": "blink",
                        "effect_params": {"frequency": 2.0},
                    },
                    {
                        "time": 60.0,
                        "lat": 37.7751,
                        "lon": -122.4196,
                        "alt": 10.0,
                        "heading": 180.0,
                        "color": {"r": 0, "g": 0, "b": 255},
                        "effect": "solid",
                    },
                ],
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
                    },
                ],
            },
        ],
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
    assert choreography.metadata.drone_count == 2
    assert choreography.metadata.status == ChoreographyStatus.DRAFT
    assert choreography.type == ChoreographyType.BLENDER
    assert len(choreography.trajectories) == 2
    assert choreography.trajectories[0].drone_id == "drone_1"
    assert choreography.trajectories[1].drone_id == "drone_2"
    assert len(choreography.trajectories[0].waypoints) == 3
    assert len(choreography.trajectories[0].led_states) == 3
    assert len(choreography.trajectories[1].waypoints) == 3
    assert len(choreography.trajectories[1].led_states) == 3
