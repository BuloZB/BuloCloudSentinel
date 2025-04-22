"""
Performance tests for the Drone Show microservice.

This module provides performance tests for the Drone Show microservice.
"""

import pytest
import json
import asyncio
from datetime import datetime
from unittest.mock import AsyncMock, MagicMock, patch

from drone_show_service.models.choreography import (
    Choreography, ChoreographyMetadata, ChoreographyType, ChoreographyStatus,
    DroneTrajectory, Waypoint, Position, LEDState, LEDColor, LEDEffect,
    SimulationSettings
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
        drone_count=100,  # Large number of drones for performance testing
        status=ChoreographyStatus.DRAFT,
    )
    
    # Create trajectories
    trajectories = []
    for i in range(100):  # 100 drones
        # Create waypoints
        waypoints = []
        led_states = []
        
        # Add waypoints and LED states
        for t in range(0, 61, 10):  # 7 waypoints per drone
            # Add waypoint
            waypoints.append(
                Waypoint(
                    time=float(t),
                    position=Position(
                        lat=37.7749 + (i % 10) * 0.0001,
                        lon=-122.4194 + (i // 10) * 0.0001,
                        alt=10.0 + t / 10.0,
                    ),
                    heading=float(t * 6),
                )
            )
            
            # Add LED state
            led_states.append(
                LEDState(
                    time=float(t),
                    color=LEDColor(
                        r=int(255 * (t / 60.0)),
                        g=int(255 * (1 - t / 60.0)),
                        b=int(128),
                    ),
                    effect=LEDEffect.SOLID,
                )
            )
        
        # Create trajectory
        trajectories.append(
            DroneTrajectory(
                drone_id=f"drone_{i+1}",
                waypoints=waypoints,
                led_states=led_states,
            )
        )
    
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


@pytest.mark.benchmark(
    group="simulation",
    min_time=0.1,
    max_time=0.5,
    min_rounds=5,
    timer=time.time,
    disable_gc=True,
    warmup=False
)
@pytest.mark.asyncio
async def test_simulation_performance(benchmark, sample_choreography, mock_minio_service):
    """Test the performance of the simulation service."""
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
    
    # Benchmark the _generate_frames method
    def run_benchmark():
        return simulation_service._generate_frames(sample_choreography, settings)
    
    # Run the benchmark
    frames = benchmark(run_benchmark)
    
    # Check that frames were generated
    assert frames is not None
    assert len(frames) > 0


@pytest.mark.benchmark(
    group="interpolation",
    min_time=0.1,
    max_time=0.5,
    min_rounds=5,
    timer=time.time,
    disable_gc=True,
    warmup=False
)
def test_interpolation_performance(benchmark, sample_choreography):
    """Test the performance of the position interpolation."""
    # Create simulation service
    simulation_service = SimulationService()
    
    # Get a trajectory
    trajectory = sample_choreography.trajectories[0]
    
    # Benchmark the _interpolate_position method
    def run_benchmark():
        return simulation_service._interpolate_position(trajectory, 15.0)
    
    # Run the benchmark
    position, heading = benchmark(run_benchmark)
    
    # Check that position and heading were interpolated
    assert position is not None
    assert heading is not None


@pytest.mark.benchmark(
    group="led_interpolation",
    min_time=0.1,
    max_time=0.5,
    min_rounds=5,
    timer=time.time,
    disable_gc=True,
    warmup=False
)
def test_led_interpolation_performance(benchmark, sample_choreography):
    """Test the performance of the LED state interpolation."""
    # Create simulation service
    simulation_service = SimulationService()
    
    # Get a trajectory
    trajectory = sample_choreography.trajectories[0]
    
    # Benchmark the _interpolate_led_state method
    def run_benchmark():
        return simulation_service._interpolate_led_state(trajectory, 15.0)
    
    # Run the benchmark
    led_state = benchmark(run_benchmark)
    
    # Check that LED state was interpolated
    assert led_state is not None


@pytest.mark.benchmark(
    group="drone_states",
    min_time=0.1,
    max_time=0.5,
    min_rounds=5,
    timer=time.time,
    disable_gc=True,
    warmup=False
)
def test_calculate_drone_states_performance(benchmark, sample_choreography):
    """Test the performance of calculating drone states."""
    # Create simulation service
    simulation_service = SimulationService()
    
    # Benchmark the _calculate_drone_states method
    def run_benchmark():
        return simulation_service._calculate_drone_states(sample_choreography, 15.0)
    
    # Run the benchmark
    drone_states = benchmark(run_benchmark)
    
    # Check that drone states were calculated
    assert drone_states is not None
    assert len(drone_states) == sample_choreography.metadata.drone_count
