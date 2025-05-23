"""
Tests for the NOTAM functionality.

This module contains tests for the NOTAM functionality.
"""

import pytest
from datetime import datetime, timedelta
from fastapi.testclient import TestClient
from sqlalchemy.ext.asyncio import AsyncSession

from remoteid_service.api.schemas.notams import (
    NOTAMSource,
    NOTAMType,
    NOTAMStatus,
)

# Test importing NOTAMs
def test_import_notams(client: TestClient):
    """
    Test importing NOTAMs.
    
    Args:
        client: Test client
    """
    # Create request data
    request_data = {
        "source": "faa",
        "region": "us-east",
        "force_update": True,
    }
    
    # Send request
    response = client.post(
        "/api/v1/notams/import",
        json=request_data,
    )
    
    # Check response
    assert response.status_code == 200
    assert "imported_count" in response.json()
    assert "updated_count" in response.json()
    assert "skipped_count" in response.json()
    assert "failed_count" in response.json()
    assert "details" in response.json()
    assert response.json()["details"]["source"] == "faa"
    assert response.json()["details"]["region"] == "us-east"

# Test searching NOTAMs
def test_search_notams(client: TestClient):
    """
    Test searching NOTAMs.
    
    Args:
        client: Test client
    """
    # Import NOTAMs first
    import_request_data = {
        "source": "faa",
        "region": "us-east",
        "force_update": True,
    }
    
    import_response = client.post(
        "/api/v1/notams/import",
        json=import_request_data,
    )
    
    assert import_response.status_code == 200
    
    # Create search request data
    search_request_data = {
        "source": "faa",
        "notam_type": "airspace",
        "status": "active",
        "limit": 10,
        "offset": 0,
    }
    
    # Send request
    search_response = client.post(
        "/api/v1/notams/search",
        json=search_request_data,
    )
    
    # Check response
    assert search_response.status_code == 200
    assert "notams" in search_response.json()
    assert "total" in search_response.json()
    assert "limit" in search_response.json()
    assert "offset" in search_response.json()
    assert search_response.json()["limit"] == 10
    assert search_response.json()["offset"] == 0

# Test checking flight plan NOTAMs
def test_check_flight_plan_notams(client: TestClient):
    """
    Test checking flight plan NOTAMs.
    
    Args:
        client: Test client
    """
    # Import NOTAMs first
    import_request_data = {
        "source": "faa",
        "region": "us-east",
        "force_update": True,
    }
    
    import_response = client.post(
        "/api/v1/notams/import",
        json=import_request_data,
    )
    
    assert import_response.status_code == 200
    
    # Create a flight plan
    flight_plan_request_data = {
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
        ],
    }
    
    flight_plan_response = client.post(
        "/api/v1/flightplans",
        json=flight_plan_request_data,
    )
    
    assert flight_plan_response.status_code == 200
    flight_plan_id = flight_plan_response.json()["id"]
    
    # Create check request data
    check_request_data = {
        "flight_plan_id": flight_plan_id,
    }
    
    # Send request
    check_response = client.post(
        "/api/v1/notams/check-flight-plan",
        json=check_request_data,
    )
    
    # Check response
    assert check_response.status_code == 200
    assert "flight_plan_id" in check_response.json()
    assert "conflicts" in check_response.json()
    assert "has_critical_conflicts" in check_response.json()
    assert check_response.json()["flight_plan_id"] == flight_plan_id
