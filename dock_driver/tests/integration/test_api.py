"""
Integration Tests for Dock Driver API

This module contains integration tests for the Dock Driver API.
"""

import pytest
import asyncio
from fastapi.testclient import TestClient
from unittest.mock import patch, MagicMock, AsyncMock

from dock_driver.main import app
from dock_driver.services.dock_manager import DockManager
from dock_driver.adapters.interface import DockStatus, ChargingStatus


# Mock DockManager
class MockDockManager:
    """Mock DockManager for testing."""
    
    def __init__(self):
        """Initialize the mock dock manager."""
        self.adapters = {
            "dji": {
                "dji-dock-1": MockDJIAdapter()
            },
            "heisha": {
                "heisha-dock-1": MockHeishaAdapter()
            },
            "esp32": {
                "esp32-dock-1": MockESP32Adapter()
            }
        }
    
    def get_adapter(self, dock_type, dock_id):
        """Get a dock adapter by type and ID."""
        return self.adapters.get(dock_type, {}).get(dock_id)
    
    def get_all_docks(self):
        """Get a list of all configured docks."""
        return [
            {
                "dock_id": "dji-dock-1",
                "dock_type": "dji",
                "name": "DJI Dock 1",
                "status": DockStatus.ONLINE,
                "charging_status": ChargingStatus.IDLE,
                "last_updated": None
            },
            {
                "dock_id": "heisha-dock-1",
                "dock_type": "heisha",
                "name": "Heisha Dock 1",
                "status": DockStatus.ONLINE,
                "charging_status": ChargingStatus.IDLE,
                "last_updated": None
            },
            {
                "dock_id": "esp32-dock-1",
                "dock_type": "esp32",
                "name": "ESP32 Dock 1",
                "status": DockStatus.ONLINE,
                "charging_status": ChargingStatus.IDLE,
                "last_updated": None
            }
        ]


# Mock DJI Adapter
class MockDJIAdapter:
    """Mock DJI Adapter for testing."""
    
    def __init__(self):
        """Initialize the mock DJI adapter."""
        self.status_cache = {
            "dock_id": "dji-dock-1",
            "status": DockStatus.ONLINE,
            "charging_status": ChargingStatus.IDLE,
            "door_state": "closed",
            "drone_connected": False,
            "error_code": 0,
            "error_message": "",
            "timestamp": "2023-01-01T00:00:00"
        }
        self.telemetry_cache = {
            "dock_id": "dji-dock-1",
            "temperature": 25.0,
            "humidity": 50.0,
            "charging_voltage": 12.0,
            "charging_current": 2.0,
            "battery_level": 80,
            "network_signal": 90,
            "timestamp": "2023-01-01T00:00:00"
        }
    
    async def get_status(self):
        """Get the current status of the dock."""
        return self.status_cache
    
    async def get_telemetry(self):
        """Get telemetry data from the dock."""
        return self.telemetry_cache
    
    async def open_dock(self):
        """Open the dock."""
        return True
    
    async def close_dock(self):
        """Close the dock."""
        return True
    
    async def start_charging(self):
        """Start charging the drone."""
        return True
    
    async def stop_charging(self):
        """Stop charging the drone."""
        return True


# Mock Heisha Adapter
class MockHeishaAdapter:
    """Mock Heisha Adapter for testing."""
    
    def __init__(self):
        """Initialize the mock Heisha adapter."""
        self.status_cache = {
            "dock_id": "heisha-dock-1",
            "status": DockStatus.ONLINE,
            "charging_status": ChargingStatus.IDLE,
            "door_state": "closed",
            "drone_connected": False,
            "error_code": 0,
            "error_message": "",
            "timestamp": "2023-01-01T00:00:00"
        }
        self.telemetry_cache = {
            "dock_id": "heisha-dock-1",
            "temperature": 25.0,
            "humidity": 50.0,
            "charging_voltage": 12.0,
            "charging_current": 2.0,
            "battery_level": 80,
            "network_signal": 90,
            "fan_speed": 30,
            "timestamp": "2023-01-01T00:00:00"
        }
    
    async def get_status(self):
        """Get the current status of the dock."""
        return self.status_cache
    
    async def get_telemetry(self):
        """Get telemetry data from the dock."""
        return self.telemetry_cache
    
    async def open_dock(self):
        """Open the dock."""
        return True
    
    async def close_dock(self):
        """Close the dock."""
        return True
    
    async def start_charging(self):
        """Start charging the drone."""
        return True
    
    async def stop_charging(self):
        """Stop charging the drone."""
        return True
    
    async def set_fan_speed(self, speed):
        """Set the fan speed."""
        return True
    
    async def set_heater_state(self, state):
        """Set the heater state."""
        return True


# Mock ESP32 Adapter
class MockESP32Adapter:
    """Mock ESP32 Adapter for testing."""
    
    def __init__(self):
        """Initialize the mock ESP32 adapter."""
        self.status_cache = {
            "dock_id": "esp32-dock-1",
            "status": DockStatus.ONLINE,
            "charging_status": ChargingStatus.IDLE,
            "relay_state": False,
            "available": True,
            "timestamp": "2023-01-01T00:00:00"
        }
        self.telemetry_cache = {
            "dock_id": "esp32-dock-1",
            "voltage": 12.0,
            "current": 0.0,
            "temperature": 25.0,
            "relay_state": False,
            "timestamp": "2023-01-01T00:00:00"
        }
    
    async def get_status(self):
        """Get the current status of the dock."""
        return self.status_cache
    
    async def get_telemetry(self):
        """Get telemetry data from the dock."""
        return self.telemetry_cache
    
    async def open_dock(self):
        """Open the dock."""
        return True
    
    async def close_dock(self):
        """Close the dock."""
        return True
    
    async def start_charging(self):
        """Start charging the drone."""
        return True
    
    async def stop_charging(self):
        """Stop charging the drone."""
        return True
    
    async def set_relay_state(self, state):
        """Set the relay state."""
        return True


# Mock authentication
def mock_verify_token(token):
    """Mock token verification."""
    return {
        "sub": "test_user",
        "roles": ["admin"]
    }


# Create test client
@pytest.fixture
def client():
    """Create a test client."""
    # Mock DockManager.get_instance
    DockManager.get_instance = MagicMock(return_value=MockDockManager())
    
    # Mock verify_token
    with patch("dock_driver.api.endpoints.verify_token", side_effect=mock_verify_token):
        yield TestClient(app)


# Test health check
def test_health_check(client):
    """Test the health check endpoint."""
    response = client.get("/health")
    assert response.status_code == 200
    assert response.json() == {"status": "ok"}


# Test get docks
def test_get_docks(client):
    """Test the get docks endpoint."""
    response = client.get(
        "/api/docks",
        headers={"Authorization": "Bearer test_token"}
    )
    assert response.status_code == 200
    data = response.json()
    assert "docks" in data
    assert len(data["docks"]) == 3


# Test get dock status
def test_get_dock_status(client):
    """Test the get dock status endpoint."""
    # Test DJI dock
    response = client.get(
        "/api/docks/dji/dji-dock-1/status",
        headers={"Authorization": "Bearer test_token"}
    )
    assert response.status_code == 200
    data = response.json()
    assert data["dock_id"] == "dji-dock-1"
    assert data["status"] == "online"
    assert data["charging_status"] == "idle"
    
    # Test Heisha dock
    response = client.get(
        "/api/docks/heisha/heisha-dock-1/status",
        headers={"Authorization": "Bearer test_token"}
    )
    assert response.status_code == 200
    data = response.json()
    assert data["dock_id"] == "heisha-dock-1"
    assert data["status"] == "online"
    assert data["charging_status"] == "idle"
    
    # Test ESP32 dock
    response = client.get(
        "/api/docks/esp32/esp32-dock-1/status",
        headers={"Authorization": "Bearer test_token"}
    )
    assert response.status_code == 200
    data = response.json()
    assert data["dock_id"] == "esp32-dock-1"
    assert data["status"] == "online"
    assert data["charging_status"] == "idle"
    
    # Test non-existent dock
    response = client.get(
        "/api/docks/dji/non-existent-dock/status",
        headers={"Authorization": "Bearer test_token"}
    )
    assert response.status_code == 404


# Test get dock telemetry
def test_get_dock_telemetry(client):
    """Test the get dock telemetry endpoint."""
    # Test DJI dock
    response = client.get(
        "/api/docks/dji/dji-dock-1/telemetry",
        headers={"Authorization": "Bearer test_token"}
    )
    assert response.status_code == 200
    data = response.json()
    assert data["dock_id"] == "dji-dock-1"
    assert data["temperature"] == 25.0
    assert data["humidity"] == 50.0
    assert data["charging_voltage"] == 12.0
    assert data["charging_current"] == 2.0
    assert data["battery_level"] == 80
    assert data["network_signal"] == 90
    
    # Test Heisha dock
    response = client.get(
        "/api/docks/heisha/heisha-dock-1/telemetry",
        headers={"Authorization": "Bearer test_token"}
    )
    assert response.status_code == 200
    data = response.json()
    assert data["dock_id"] == "heisha-dock-1"
    assert data["temperature"] == 25.0
    assert data["humidity"] == 50.0
    assert data["charging_voltage"] == 12.0
    assert data["charging_current"] == 2.0
    assert data["battery_level"] == 80
    assert data["network_signal"] == 90
    assert data["fan_speed"] == 30
    
    # Test ESP32 dock
    response = client.get(
        "/api/docks/esp32/esp32-dock-1/telemetry",
        headers={"Authorization": "Bearer test_token"}
    )
    assert response.status_code == 200
    data = response.json()
    assert data["dock_id"] == "esp32-dock-1"
    assert data["voltage"] == 12.0
    assert data["current"] == 0.0
    assert data["temperature"] == 25.0
    assert data["relay_state"] is False
    
    # Test non-existent dock
    response = client.get(
        "/api/docks/dji/non-existent-dock/telemetry",
        headers={"Authorization": "Bearer test_token"}
    )
    assert response.status_code == 404


# Test dock control endpoints
def test_dock_control(client):
    """Test the dock control endpoints."""
    # Test open dock
    response = client.post(
        "/api/docks/dji/dji-dock-1/open",
        headers={"Authorization": "Bearer test_token"}
    )
    assert response.status_code == 200
    assert response.json()["message"] == "Dock dji-dock-1 opened successfully"
    
    # Test close dock
    response = client.post(
        "/api/docks/dji/dji-dock-1/close",
        headers={"Authorization": "Bearer test_token"}
    )
    assert response.status_code == 200
    assert response.json()["message"] == "Dock dji-dock-1 closed successfully"
    
    # Test start charging
    response = client.post(
        "/api/docks/dji/dji-dock-1/charge/start",
        headers={"Authorization": "Bearer test_token"}
    )
    assert response.status_code == 200
    assert response.json()["message"] == "Charging started at dock dji-dock-1"
    
    # Test stop charging
    response = client.post(
        "/api/docks/dji/dji-dock-1/charge/stop",
        headers={"Authorization": "Bearer test_token"}
    )
    assert response.status_code == 200
    assert response.json()["message"] == "Charging stopped at dock dji-dock-1"
    
    # Test non-existent dock
    response = client.post(
        "/api/docks/dji/non-existent-dock/open",
        headers={"Authorization": "Bearer test_token"}
    )
    assert response.status_code == 404


# Test Heisha-specific endpoints
def test_heisha_specific_endpoints(client):
    """Test the Heisha-specific endpoints."""
    # Test set fan speed
    response = client.post(
        "/api/docks/heisha/heisha-dock-1/fan/50",
        headers={"Authorization": "Bearer test_token"}
    )
    assert response.status_code == 200
    assert response.json()["message"] == "Fan speed set to 50% for dock heisha-dock-1"
    
    # Test set heater state
    response = client.post(
        "/api/docks/heisha/heisha-dock-1/heater/on",
        headers={"Authorization": "Bearer test_token"}
    )
    assert response.status_code == 200
    assert response.json()["message"] == "Heater on for dock heisha-dock-1"
    
    # Test invalid fan speed
    response = client.post(
        "/api/docks/heisha/heisha-dock-1/fan/101",
        headers={"Authorization": "Bearer test_token"}
    )
    assert response.status_code == 400
    
    # Test invalid heater state
    response = client.post(
        "/api/docks/heisha/heisha-dock-1/heater/invalid",
        headers={"Authorization": "Bearer test_token"}
    )
    assert response.status_code == 400
    
    # Test non-existent dock
    response = client.post(
        "/api/docks/heisha/non-existent-dock/fan/50",
        headers={"Authorization": "Bearer test_token"}
    )
    assert response.status_code == 404


# Test ESP32-specific endpoints
def test_esp32_specific_endpoints(client):
    """Test the ESP32-specific endpoints."""
    # Test set relay state
    response = client.post(
        "/api/docks/esp32/esp32-dock-1/relay/on",
        headers={"Authorization": "Bearer test_token"}
    )
    assert response.status_code == 200
    assert response.json()["message"] == "Relay on for dock esp32-dock-1"
    
    # Test invalid relay state
    response = client.post(
        "/api/docks/esp32/esp32-dock-1/relay/invalid",
        headers={"Authorization": "Bearer test_token"}
    )
    assert response.status_code == 400
    
    # Test non-existent dock
    response = client.post(
        "/api/docks/esp32/non-existent-dock/relay/on",
        headers={"Authorization": "Bearer test_token"}
    )
    assert response.status_code == 404
