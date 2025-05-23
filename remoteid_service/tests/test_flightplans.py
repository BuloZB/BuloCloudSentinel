"""
Tests for the flight plan functionality.

This module contains tests for the flight plan functionality.
"""

import pytest
from datetime import datetime, timedelta
from fastapi.testclient import TestClient
from sqlalchemy.ext.asyncio import AsyncSession

from remoteid_service.api.schemas.flightplans import (
    FlightPlanType,
    FlightPlanStatus,
)

# Test creating a flight plan
def test_create_flight_plan(client: TestClient):
    """
    Test creating a flight plan.
    
    Args:
        client: Test client
    """
    # Create request data
    request_data = {
        "name": "Test Flight Plan",
        "description": "Test flight plan for testing",
        "operator_id": "OP-001",
        "drone_id": "TEST-001",
        "plan_type": "faa_laanc",
        "start_time": (datetime.utcnow() + timedelta(hours=1)).isoformat(),
        "end_time": (datetime.utcnow() + timedelta(hours=2)).isoformat(),
        "max_altitude": 100,
        "area": {
            "points": [
                {
                    "latitude": 37.7749,
                    "longitude": -122.4194,
                    "altitude": 0,
                },
                {
                    "latitude": 37.7749,
                    "longitude": -122.4094,
                    "altitude": 0,
                },
                {
                    "latitude": 37.7649,
                    "longitude": -122.4094,
                    "altitude": 0,
                },
                {
                    "latitude": 37.7649,
                    "longitude": -122.4194,
                    "altitude": 0,
                },
                {
                    "latitude": 37.7749,
                    "longitude": -122.4194,
                    "altitude": 0,
                },
            ]
        },
        "waypoints": [
            {
                "sequence": 1,
                "position": {
                    "latitude": 37.7749,
                    "longitude": -122.4194,
                    "altitude": 50,
                },
                "speed": 5,
                "hold_time": 0,
                "action": "takeoff",
            },
            {
                "sequence": 2,
                "position": {
                    "latitude": 37.7749,
                    "longitude": -122.4094,
                    "altitude": 100,
                },
                "speed": 10,
                "hold_time": 0,
                "action": None,
            },
            {
                "sequence": 3,
                "position": {
                    "latitude": 37.7649,
                    "longitude": -122.4094,
                    "altitude": 100,
                },
                "speed": 10,
                "hold_time": 0,
                "action": None,
            },
            {
                "sequence": 4,
                "position": {
                    "latitude": 37.7649,
                    "longitude": -122.4194,
                    "altitude": 100,
                },
                "speed": 10,
                "hold_time": 0,
                "action": None,
            },
            {
                "sequence": 5,
                "position": {
                    "latitude": 37.7749,
                    "longitude": -122.4194,
                    "altitude": 50,
                },
                "speed": 5,
                "hold_time": 0,
                "action": "land",
            },
        ],
    }
    
    # Send request
    response = client.post(
        "/api/v1/flightplans",
        json=request_data,
    )
    
    # Check response
    assert response.status_code == 200
    assert response.json()["name"] == "Test Flight Plan"
    assert response.json()["description"] == "Test flight plan for testing"
    assert response.json()["operator_id"] == "OP-001"
    assert response.json()["drone_id"] == "TEST-001"
    assert response.json()["plan_type"] == "faa_laanc"
    assert response.json()["status"] == "draft"
    assert "id" in response.json()
    assert "waypoints" in response.json()
    assert len(response.json()["waypoints"]) == 5

# Test getting a flight plan
def test_get_flight_plan(client: TestClient):
    """
    Test getting a flight plan.
    
    Args:
        client: Test client
    """
    # Create a flight plan first
    create_request_data = {
        "name": "Test Flight Plan 2",
        "description": "Test flight plan for testing",
        "operator_id": "OP-001",
        "drone_id": "TEST-001",
        "plan_type": "faa_laanc",
        "start_time": (datetime.utcnow() + timedelta(hours=1)).isoformat(),
        "end_time": (datetime.utcnow() + timedelta(hours=2)).isoformat(),
        "max_altitude": 100,
        "area": {
            "points": [
                {
                    "latitude": 37.7749,
                    "longitude": -122.4194,
                    "altitude": 0,
                },
                {
                    "latitude": 37.7749,
                    "longitude": -122.4094,
                    "altitude": 0,
                },
                {
                    "latitude": 37.7649,
                    "longitude": -122.4094,
                    "altitude": 0,
                },
                {
                    "latitude": 37.7649,
                    "longitude": -122.4194,
                    "altitude": 0,
                },
                {
                    "latitude": 37.7749,
                    "longitude": -122.4194,
                    "altitude": 0,
                },
            ]
        },
        "waypoints": [
            {
                "sequence": 1,
                "position": {
                    "latitude": 37.7749,
                    "longitude": -122.4194,
                    "altitude": 50,
                },
                "speed": 5,
                "hold_time": 0,
                "action": "takeoff",
            },
        ],
    }
    
    create_response = client.post(
        "/api/v1/flightplans",
        json=create_request_data,
    )
    
    assert create_response.status_code == 200
    flight_plan_id = create_response.json()["id"]
    
    # Send request
    get_response = client.get(
        f"/api/v1/flightplans/{flight_plan_id}",
    )
    
    # Check response
    assert get_response.status_code == 200
    assert get_response.json()["id"] == flight_plan_id
    assert get_response.json()["name"] == "Test Flight Plan 2"
    assert get_response.json()["status"] == "draft"
