"""
Tests for the Remote ID functionality.

This module contains tests for the Remote ID functionality.
"""

import pytest
from fastapi.testclient import TestClient
from sqlalchemy.ext.asyncio import AsyncSession

from remoteid_service.api.schemas.remoteid import (
    RemoteIDMode,
    BroadcastMethod,
    Position,
    Velocity,
)

# Test Remote ID broadcasting
def test_start_broadcast(client: TestClient):
    """
    Test starting Remote ID broadcasting.
    
    Args:
        client: Test client
    """
    # Create request data
    request_data = {
        "drone_id": "TEST-001",
        "mode": "faa",
        "methods": ["wifi_nan", "bluetooth_le"],
        "operator_id": {
            "id": "OP-001",
            "type": "other",
        },
        "serial_number": "SN-001",
        "initial_position": {
            "latitude": 37.7749,
            "longitude": -122.4194,
            "altitude": 100,
        },
        "initial_velocity": {
            "speed_horizontal": 5,
            "heading": 90,
        },
    }
    
    # Send request
    response = client.post(
        "/api/v1/remoteid/broadcast/start",
        json=request_data,
    )
    
    # Check response
    assert response.status_code == 200
    assert response.json()["drone_id"] == "TEST-001"
    assert response.json()["broadcasting"] is True
    assert response.json()["mode"] == "faa"
    assert "wifi_nan" in response.json()["methods"]
    assert "bluetooth_le" in response.json()["methods"]
    assert "session_id" in response.json()
    assert "last_update" in response.json()
    assert "position" in response.json()
    assert "velocity" in response.json()

# Test stopping Remote ID broadcasting
def test_stop_broadcast(client: TestClient):
    """
    Test stopping Remote ID broadcasting.
    
    Args:
        client: Test client
    """
    # Start broadcasting first
    start_request_data = {
        "drone_id": "TEST-002",
        "mode": "faa",
        "methods": ["wifi_nan", "bluetooth_le"],
    }
    
    start_response = client.post(
        "/api/v1/remoteid/broadcast/start",
        json=start_request_data,
    )
    
    assert start_response.status_code == 200
    session_id = start_response.json()["session_id"]
    
    # Create stop request data
    stop_request_data = {
        "drone_id": "TEST-002",
        "session_id": session_id,
    }
    
    # Send request
    stop_response = client.post(
        "/api/v1/remoteid/broadcast/stop",
        json=stop_request_data,
    )
    
    # Check response
    assert stop_response.status_code == 200
    assert stop_response.json()["drone_id"] == "TEST-002"
    assert stop_response.json()["broadcasting"] is False
    assert stop_response.json()["session_id"] == session_id

# Test updating Remote ID broadcast data
def test_update_broadcast(client: TestClient):
    """
    Test updating Remote ID broadcast data.
    
    Args:
        client: Test client
    """
    # Start broadcasting first
    start_request_data = {
        "drone_id": "TEST-003",
        "mode": "faa",
        "methods": ["wifi_nan", "bluetooth_le"],
        "initial_position": {
            "latitude": 37.7749,
            "longitude": -122.4194,
            "altitude": 100,
        },
    }
    
    start_response = client.post(
        "/api/v1/remoteid/broadcast/start",
        json=start_request_data,
    )
    
    assert start_response.status_code == 200
    session_id = start_response.json()["session_id"]
    
    # Create update request data
    update_request_data = {
        "drone_id": "TEST-003",
        "position": {
            "latitude": 37.7750,
            "longitude": -122.4195,
            "altitude": 110,
        },
        "velocity": {
            "speed_horizontal": 10,
            "heading": 180,
        },
        "session_id": session_id,
    }
    
    # Send request
    update_response = client.post(
        "/api/v1/remoteid/broadcast/update",
        json=update_request_data,
    )
    
    # Check response
    assert update_response.status_code == 200
    assert update_response.json()["drone_id"] == "TEST-003"
    assert update_response.json()["broadcasting"] is True
    assert update_response.json()["session_id"] == session_id
    assert update_response.json()["position"]["latitude"] == 37.7750
    assert update_response.json()["position"]["longitude"] == -122.4195
    assert update_response.json()["position"]["altitude"] == 110
    assert update_response.json()["velocity"]["speed_horizontal"] == 10
    assert update_response.json()["velocity"]["heading"] == 180

# Test getting Remote ID broadcast status
def test_get_broadcast_status(client: TestClient):
    """
    Test getting Remote ID broadcast status.
    
    Args:
        client: Test client
    """
    # Start broadcasting first
    start_request_data = {
        "drone_id": "TEST-004",
        "mode": "faa",
        "methods": ["wifi_nan", "bluetooth_le"],
    }
    
    start_response = client.post(
        "/api/v1/remoteid/broadcast/start",
        json=start_request_data,
    )
    
    assert start_response.status_code == 200
    
    # Send request
    status_response = client.get(
        "/api/v1/remoteid/broadcast/status/TEST-004",
    )
    
    # Check response
    assert status_response.status_code == 200
    assert status_response.json()["drone_id"] == "TEST-004"
    assert status_response.json()["broadcasting"] is True
    assert status_response.json()["mode"] == "faa"
    assert "wifi_nan" in status_response.json()["methods"]
    assert "bluetooth_le" in status_response.json()["methods"]
