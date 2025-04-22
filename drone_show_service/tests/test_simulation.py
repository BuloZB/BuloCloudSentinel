"""
Tests for the simulation service.

This module provides tests for the simulation service.
"""

import pytest
import json
from datetime import datetime
from unittest.mock import AsyncMock, MagicMock, patch

from drone_show_service.models.choreography import (
    Choreography, ChoreographyMetadata, ChoreographyType, ChoreographyStatus,
    DroneTrajectory, Waypoint, Position, LEDState, LEDColor, LEDEffect,
    SimulationSettings, SimulationFrame
)
from drone_show_service.services.simulation_service import SimulationService
from drone_show_service.services.minio_service import MinioService


@pytest.fixture
def sample_choreography():
    """Create a sample choreography for testing."""
    # Create metadata
    metadata = ChoreographyMetadata(
        name="Test Choreography",
        description="Test description",
        author="Test Author",
        tags=["test", "demo"],
        duration=60.0,
        drone_count=2,
        status=ChoreographyStatus.DRAFT,
    )
    
    # Create waypoints for drone 1
    waypoints1 = [
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
    
    # Create LED states for drone 1
    led_states1 = [
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
    
    # Create waypoints for drone 2
    waypoints2 = [
        Waypoint(
            time=0.0,
            position=Position(lat=37.7749, lon=-122.4194, alt=10.0),
            heading=0.0,
        ),
        Waypoint(
            time=30.0,
            position=Position(lat=37.7748, lon=-122.4193, alt=15.0),
            heading=270.0,
        ),
        Waypoint(
            time=60.0,
            position=Position(lat=37.7747, lon=-122.4192, alt=10.0),
            heading=180.0,
        ),
    ]
    
    # Create LED states for drone 2
    led_states2 = [
        LEDState(
            time=0.0,
            color=LEDColor(r=0, g=0, b=255),
            effect=LEDEffect.SOLID,
        ),
        LEDState(
            time=30.0,
            color=LEDColor(r=0, g=255, b=0),
            effect=LEDEffect.PULSE,
            effect_params={"frequency": 1.0},
        ),
        LEDState(
            time=60.0,
            color=LEDColor(r=255, g=0, b=0),
            effect=LEDEffect.SOLID,
        ),
    ]
    
    # Create trajectories
    trajectories = [
        DroneTrajectory(
            drone_id="drone_1",
            waypoints=waypoints1,
            led_states=led_states1,
        ),
        DroneTrajectory(
            drone_id="drone_2",
            waypoints=waypoints2,
            led_states=led_states2,
        ),
    ]
    
    # Create choreography
    return Choreography(
        id="test-choreography-id",
        metadata=metadata,
        type=ChoreographyType.WAYPOINT,
        trajectories=trajectories,
    )


@pytest.fixture
def mock_minio_service():
    """Create a mock MinIO service."""
    mock_service = AsyncMock(spec=MinioService)
    mock_service.put_json.return_value = True
    mock_service.get_json.return_value = []
    return mock_service


@pytest.mark.asyncio
async def test_simulate_choreography(sample_choreography, mock_minio_service):
    """Test simulating a choreography."""
    # Create simulation service with mock MinIO service
    simulation_service = SimulationService(mock_minio_service)
    
    # Create mock database session
    mock_db = AsyncMock()
    mock_db.execute = AsyncMock()
    mock_db.execute.return_value.scalars.return_value.first.return_value = MagicMock(
        id=sample_choreography.id,
        name=sample_choreography.metadata.name,
        description=sample_choreography.metadata.description,
        author=sample_choreography.metadata.author,
        created_at=datetime.utcnow(),
        updated_at=datetime.utcnow(),
        tags=sample_choreography.metadata.tags,
        duration=sample_choreography.metadata.duration,
        drone_count=sample_choreography.metadata.drone_count,
        status=sample_choreography.metadata.status,
        type=sample_choreography.type,
        trajectories=json.loads(json.dumps([t.dict() for t in sample_choreography.trajectories])),
        formations=None,
        music_file=None,
        music_bpm=None,
        music_offset=None,
        boundary=None,
        home_position=None,
        notes=None,
    )
    
    # Create simulation settings
    settings = SimulationSettings(
        start_time=0.0,
        end_time=60.0,
        speed_factor=1.0,
        include_takeoff_landing=True,
        visualize_led=True,
        visualize_trajectories=True,
        drone_model="generic",
    )
    
    # Mock the _generate_frames method to return a fixed set of frames
    with patch.object(simulation_service, '_generate_frames') as mock_generate_frames:
        # Create sample frames
        frames = [
            SimulationFrame(
                time=0.0,
                drone_states={
                    "drone_1": {
                        "position": {"lat": 37.7749, "lon": -122.4194, "alt": 10.0},
                        "heading": 0.0,
                        "led_state": {"time": 0.0, "color": {"r": 255, "g": 0, "b": 0}, "effect": "solid"},
                    },
                    "drone_2": {
                        "position": {"lat": 37.7749, "lon": -122.4194, "alt": 10.0},
                        "heading": 0.0,
                        "led_state": {"time": 0.0, "color": {"r": 0, "g": 0, "b": 255}, "effect": "solid"},
                    },
                },
            ),
            SimulationFrame(
                time=30.0,
                drone_states={
                    "drone_1": {
                        "position": {"lat": 37.7750, "lon": -122.4195, "alt": 15.0},
                        "heading": 90.0,
                        "led_state": {"time": 30.0, "color": {"r": 0, "g": 255, "b": 0}, "effect": "blink", "effect_params": {"frequency": 2.0}},
                    },
                    "drone_2": {
                        "position": {"lat": 37.7748, "lon": -122.4193, "alt": 15.0},
                        "heading": 270.0,
                        "led_state": {"time": 30.0, "color": {"r": 0, "g": 255, "b": 0}, "effect": "pulse", "effect_params": {"frequency": 1.0}},
                    },
                },
            ),
            SimulationFrame(
                time=60.0,
                drone_states={
                    "drone_1": {
                        "position": {"lat": 37.7751, "lon": -122.4196, "alt": 10.0},
                        "heading": 180.0,
                        "led_state": {"time": 60.0, "color": {"r": 0, "g": 0, "b": 255}, "effect": "solid"},
                    },
                    "drone_2": {
                        "position": {"lat": 37.7747, "lon": -122.4192, "alt": 10.0},
                        "heading": 180.0,
                        "led_state": {"time": 60.0, "color": {"r": 255, "g": 0, "b": 0}, "effect": "solid"},
                    },
                },
            ),
        ]
        mock_generate_frames.return_value = frames
        
        # Simulate choreography
        result = await simulation_service.simulate_choreography(mock_db, sample_choreography.id, settings)
        
        # Check that the simulation was created
        assert result is not None
        assert result.choreography_id == sample_choreography.id
        assert result.settings == settings
        assert result.duration == sample_choreography.metadata.duration
        assert len(result.frames) == 3
        
        # Check that frames were stored in MinIO
        mock_minio_service.put_json.assert_called_once()
        
        # Check that the simulation was stored in the database
        mock_db.add.assert_called_once()
        mock_db.commit.assert_called_once()
        mock_db.refresh.assert_called_once()


@pytest.mark.asyncio
async def test_get_simulation(mock_minio_service):
    """Test getting a simulation."""
    # Create simulation service with mock MinIO service
    simulation_service = SimulationService(mock_minio_service)
    
    # Create mock database session
    mock_db = AsyncMock()
    mock_db.execute = AsyncMock()
    
    # Create sample frames
    frames = [
        SimulationFrame(
            time=0.0,
            drone_states={
                "drone_1": {
                    "position": {"lat": 37.7749, "lon": -122.4194, "alt": 10.0},
                    "heading": 0.0,
                    "led_state": {"time": 0.0, "color": {"r": 255, "g": 0, "b": 0}, "effect": "solid"},
                },
            },
        ),
    ]
    
    # Set up mock MinIO service to return frames
    mock_minio_service.get_json.return_value = [frame.dict() for frame in frames]
    
    # Set up mock database to return a simulation
    mock_db.execute.return_value.scalars.return_value.first.return_value = MagicMock(
        id="test-simulation-id",
        choreography_id="test-choreography-id",
        settings={"start_time": 0.0, "end_time": 60.0, "speed_factor": 1.0},
        frames_file="simulations/test-simulation-id/frames.json",
        duration=60.0,
        created_at=datetime.utcnow(),
    )
    
    # Get simulation
    result = await simulation_service.get_simulation(mock_db, "test-simulation-id")
    
    # Check that the simulation was retrieved
    assert result is not None
    assert result.id == "test-simulation-id"
    assert result.choreography_id == "test-choreography-id"
    assert result.duration == 60.0
    assert len(result.frames) == 1
    
    # Check that frames were retrieved from MinIO
    mock_minio_service.get_json.assert_called_once_with("simulations/test-simulation-id/frames.json")


@pytest.mark.asyncio
async def test_delete_simulation(mock_minio_service):
    """Test deleting a simulation."""
    # Create simulation service with mock MinIO service
    simulation_service = SimulationService(mock_minio_service)
    
    # Create mock database session
    mock_db = AsyncMock()
    mock_db.execute = AsyncMock()
    
    # Set up mock database to return a simulation
    mock_db.execute.return_value.scalars.return_value.first.return_value = MagicMock(
        id="test-simulation-id",
        choreography_id="test-choreography-id",
        settings={"start_time": 0.0, "end_time": 60.0, "speed_factor": 1.0},
        frames_file="simulations/test-simulation-id/frames.json",
        duration=60.0,
        created_at=datetime.utcnow(),
    )
    
    # Delete simulation
    result = await simulation_service.delete_simulation(mock_db, "test-simulation-id")
    
    # Check that the simulation was deleted
    assert result is True
    
    # Check that frames were deleted from MinIO
    mock_minio_service.delete.assert_called_once_with("simulations/test-simulation-id/frames.json")
    
    # Check that the simulation was deleted from the database
    mock_db.delete.assert_called_once()
    mock_db.commit.assert_called_once()
