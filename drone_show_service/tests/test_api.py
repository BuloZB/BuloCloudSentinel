"""
Tests for the API endpoints.

This module provides tests for the API endpoints.
"""

import pytest
import json
from datetime import datetime
from unittest.mock import AsyncMock, MagicMock, patch
from fastapi.testclient import TestClient
from fastapi import FastAPI

from drone_show_service.api.main import app
from drone_show_service.models.choreography import (
    Choreography, ChoreographyMetadata, ChoreographyType, ChoreographyStatus,
    DroneTrajectory, Waypoint, Position, LEDState, LEDColor, LEDEffect,
    ChoreographyCreate, ChoreographyResponse, SimulationSettings, SimulationResponse,
    ExecutionSettings, ExecutionResponse, ExecutionStatus
)
from drone_show_service.services.choreography_service import ChoreographyService
from drone_show_service.services.simulation_service import SimulationService
from drone_show_service.services.execution_service import ExecutionService


@pytest.fixture
def test_client():
    """Create a test client for the FastAPI app."""
    return TestClient(app)


@pytest.fixture
def mock_choreography_service():
    """Create a mock choreography service."""
    return AsyncMock(spec=ChoreographyService)


@pytest.fixture
def mock_simulation_service():
    """Create a mock simulation service."""
    return AsyncMock(spec=SimulationService)


@pytest.fixture
def mock_execution_service():
    """Create a mock execution service."""
    return AsyncMock(spec=ExecutionService)


@pytest.fixture
def sample_choreography_response():
    """Create a sample choreography response."""
    return ChoreographyResponse(
        id="test-choreography-id",
        metadata=ChoreographyMetadata(
            name="Test Choreography",
            description="Test description",
            author="Test Author",
            tags=["test", "demo"],
            duration=60.0,
            drone_count=2,
            status=ChoreographyStatus.DRAFT,
        ),
        type=ChoreographyType.WAYPOINT,
        trajectories=[
            DroneTrajectory(
                drone_id="drone_1",
                waypoints=[
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
                ],
                led_states=[
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
                ],
            ),
        ],
    )


@pytest.fixture
def sample_simulation_response():
    """Create a sample simulation response."""
    return SimulationResponse(
        id="test-simulation-id",
        choreography_id="test-choreography-id",
        settings=SimulationSettings(
            start_time=0.0,
            end_time=60.0,
            speed_factor=1.0,
            include_takeoff_landing=True,
            visualize_led=True,
            visualize_trajectories=True,
            drone_model="generic",
        ),
        frames=[],
        duration=60.0,
    )


@pytest.fixture
def sample_execution_response():
    """Create a sample execution response."""
    return ExecutionResponse(
        id="test-execution-id",
        choreography_id="test-choreography-id",
        settings=ExecutionSettings(
            include_takeoff_landing=True,
            use_rtk=True,
            safety_checks=True,
            geofence_enabled=True,
            return_home_on_low_battery=True,
            return_home_on_connection_loss=True,
            led_enabled=True,
        ),
        status=ExecutionStatus.PENDING,
        drone_statuses={},
        current_time=0.0,
        progress=0.0,
    )


def test_health_check(test_client):
    """Test the health check endpoint."""
    response = test_client.get("/health")
    assert response.status_code == 200
    assert response.json() == {"status": "ok"}


def test_create_choreography(test_client, mock_choreography_service, sample_choreography_response):
    """Test creating a choreography."""
    # Mock the choreography service
    app.state.choreography_service = mock_choreography_service
    mock_choreography_service.create_choreography.return_value = sample_choreography_response
    
    # Create choreography data
    choreography_data = {
        "metadata": {
            "name": "Test Choreography",
            "description": "Test description",
            "author": "Test Author",
            "tags": ["test", "demo"],
            "duration": 60.0,
            "drone_count": 2,
            "status": "draft",
        },
        "type": "waypoint",
        "trajectories": [
            {
                "drone_id": "drone_1",
                "waypoints": [
                    {
                        "time": 0.0,
                        "position": {"lat": 37.7749, "lon": -122.4194, "alt": 10.0},
                        "heading": 0.0,
                    },
                    {
                        "time": 60.0,
                        "position": {"lat": 37.7751, "lon": -122.4196, "alt": 10.0},
                        "heading": 180.0,
                    },
                ],
                "led_states": [
                    {
                        "time": 0.0,
                        "color": {"r": 255, "g": 0, "b": 0},
                        "effect": "solid",
                    },
                    {
                        "time": 60.0,
                        "color": {"r": 0, "g": 0, "b": 255},
                        "effect": "solid",
                    },
                ],
            },
        ],
    }
    
    # Send request
    response = test_client.post("/shows/", json=choreography_data)
    
    # Check response
    assert response.status_code == 201
    assert response.json()["id"] == "test-choreography-id"
    assert response.json()["metadata"]["name"] == "Test Choreography"
    
    # Check that the choreography service was called
    mock_choreography_service.create_choreography.assert_called_once()


def test_get_choreography(test_client, mock_choreography_service, sample_choreography_response):
    """Test getting a choreography."""
    # Mock the choreography service
    app.state.choreography_service = mock_choreography_service
    mock_choreography_service.get_choreography.return_value = sample_choreography_response
    
    # Send request
    response = test_client.get("/shows/test-choreography-id")
    
    # Check response
    assert response.status_code == 200
    assert response.json()["id"] == "test-choreography-id"
    assert response.json()["metadata"]["name"] == "Test Choreography"
    
    # Check that the choreography service was called
    mock_choreography_service.get_choreography.assert_called_once_with(
        mock_choreography_service.get_choreography.call_args[0][0],  # db
        "test-choreography-id",
    )


def test_get_choreographies(test_client, mock_choreography_service, sample_choreography_response):
    """Test getting all choreographies."""
    # Mock the choreography service
    app.state.choreography_service = mock_choreography_service
    mock_choreography_service.get_choreographies.return_value = [sample_choreography_response]
    
    # Send request
    response = test_client.get("/shows/")
    
    # Check response
    assert response.status_code == 200
    assert len(response.json()) == 1
    assert response.json()[0]["id"] == "test-choreography-id"
    assert response.json()[0]["metadata"]["name"] == "Test Choreography"
    
    # Check that the choreography service was called
    mock_choreography_service.get_choreographies.assert_called_once()


def test_simulate_choreography(test_client, mock_simulation_service, sample_simulation_response):
    """Test simulating a choreography."""
    # Mock the simulation service
    app.state.simulation_service = mock_simulation_service
    mock_simulation_service.simulate_choreography.return_value = sample_simulation_response
    
    # Create simulation settings
    settings = {
        "start_time": 0.0,
        "end_time": 60.0,
        "speed_factor": 1.0,
        "include_takeoff_landing": True,
        "visualize_led": True,
        "visualize_trajectories": True,
        "drone_model": "generic",
    }
    
    # Send request
    response = test_client.post("/simulation/test-choreography-id", json=settings)
    
    # Check response
    assert response.status_code == 200
    assert response.json()["id"] == "test-simulation-id"
    assert response.json()["choreography_id"] == "test-choreography-id"
    
    # Check that the simulation service was called
    mock_simulation_service.simulate_choreography.assert_called_once()


def test_execute_choreography(test_client, mock_execution_service, sample_execution_response):
    """Test executing a choreography."""
    # Mock the execution service
    app.state.execution_service = mock_execution_service
    mock_execution_service.execute_choreography.return_value = sample_execution_response
    
    # Create execution settings
    settings = {
        "include_takeoff_landing": True,
        "use_rtk": True,
        "safety_checks": True,
        "geofence_enabled": True,
        "return_home_on_low_battery": True,
        "return_home_on_connection_loss": True,
        "led_enabled": True,
    }
    
    # Send request
    response = test_client.post("/execution/test-choreography-id", json=settings)
    
    # Check response
    assert response.status_code == 200
    assert response.json()["id"] == "test-execution-id"
    assert response.json()["choreography_id"] == "test-choreography-id"
    assert response.json()["status"] == "pending"
    
    # Check that the execution service was called
    mock_execution_service.execute_choreography.assert_called_once()
