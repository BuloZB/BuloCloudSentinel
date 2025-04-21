"""
Tests for API endpoints.

This module provides tests for the API endpoints of the Anti-Jamming Service.
"""

import pytest
from fastapi.testclient import TestClient
from unittest.mock import MagicMock, AsyncMock, patch

from anti_jamming_service.main import AntiJammingService
from anti_jamming_service.api.schemas import HardwareType, ProcessingType


@pytest.fixture
def app():
    """Create FastAPI app for testing."""
    # Create service
    service = AntiJammingService()
    
    # Mock startup event
    async def mock_startup():
        # Mock hardware
        service.hardware = {
            "kraken_sdr": AsyncMock(),
            "hackrf": AsyncMock(),
            "lora_sx127x": AsyncMock()
        }
        
        # Mock processors
        service.processors = {
            "gnss_mitigation": AsyncMock(),
            "doa_estimation": AsyncMock(),
            "jamming_detection": AsyncMock(),
            "fhss": AsyncMock()
        }
        
        # Set up mock status responses
        service.hardware["kraken_sdr"].get_status.return_value = {
            "device_index": 0,
            "initialized": True,
            "running": False,
            "coherent_mode": True,
            "reference_channel": 0,
            "center_frequency": 1575420000,
            "sample_rate": 2400000,
            "gain": 30.0,
            "doa": {"azimuth": 45.0, "elevation": 0.0, "confidence": 0.8}
        }
        
        service.processors["jamming_detection"].get_status.return_value = {
            "enabled": True,
            "sample_size": 1024,
            "detection_interval": 1.0,
            "energy_detector_enabled": True,
            "spectral_detector_enabled": True,
            "cyclostationary_detector_enabled": False,
            "last_detection": {
                "is_jamming": True,
                "jamming_type": "CONTINUOUS_WAVE",
                "confidence": 0.8,
                "snr_db": 15.0,
                "timestamp": 1234567890.0
            }
        }
        
        service.processors["jamming_detection"].last_detection = {
            "is_jamming": True,
            "jamming_type": "CONTINUOUS_WAVE",
            "confidence": 0.8,
            "snr_db": 15.0,
            "timestamp": 1234567890.0
        }
        
        service.processors["doa_estimation"].estimate.return_value = {
            "azimuth": 45.0,
            "elevation": 0.0,
            "confidence": 0.8
        }
        
        service.processors["fhss"].send_message.return_value = True
        service.processors["fhss"].receive_message.return_value = b"Test message"
        
        # Store in app state
        service.app.state.hardware = service.hardware
        service.app.state.processors = service.processors
    
    # Set up startup event
    service.app.router.on_startup = [mock_startup]
    
    return service.app


@pytest.fixture
def client(app):
    """Create test client."""
    return TestClient(app)


def test_get_status(client):
    """Test GET /api/status endpoint."""
    response = client.get("/api/status")
    
    # Check response
    assert response.status_code == 200
    assert "status" in response.json()
    assert "version" in response.json()
    assert "uptime" in response.json()
    assert "hardware" in response.json()
    assert "processing" in response.json()
    assert response.json()["status"] == "running"


def test_get_hardware_status(client):
    """Test GET /api/hardware endpoint."""
    response = client.get("/api/hardware")
    
    # Check response
    assert response.status_code == 200
    assert isinstance(response.json(), list)
    assert len(response.json()) == 3
    
    # Check hardware types
    hardware_types = [hw["type"] for hw in response.json()]
    assert "kraken_sdr" in hardware_types
    assert "hackrf" in hardware_types
    assert "lora_sx127x" in hardware_types


def test_configure_hardware(client):
    """Test POST /api/hardware/configure endpoint."""
    # Configure KrakenSDR
    response = client.post(
        "/api/hardware/configure",
        json={
            "type": "kraken_sdr",
            "config": {
                "center_frequency": 1575420000,
                "sample_rate": 2400000,
                "gain": 30.0,
                "coherent_mode": True,
                "reference_channel": 0
            }
        }
    )
    
    # Check response
    assert response.status_code == 200
    assert "success" in response.json()
    assert "type" in response.json()
    assert "status" in response.json()
    assert response.json()["success"] is True
    assert response.json()["type"] == "kraken_sdr"


def test_get_processing_status(client):
    """Test GET /api/processing endpoint."""
    response = client.get("/api/processing")
    
    # Check response
    assert response.status_code == 200
    assert isinstance(response.json(), list)
    assert len(response.json()) == 4
    
    # Check processing types
    processing_types = [proc["type"] for proc in response.json()]
    assert "gnss_mitigation" in processing_types
    assert "doa_estimation" in processing_types
    assert "jamming_detection" in processing_types
    assert "fhss" in processing_types


def test_configure_processing(client):
    """Test POST /api/processing/configure endpoint."""
    # Configure jamming detection
    response = client.post(
        "/api/processing/configure",
        json={
            "type": "jamming_detection",
            "config": {
                "enabled": True,
                "sample_size": 1024,
                "detection_interval": 1.0,
                "energy_detector": {
                    "enabled": True,
                    "threshold": 10.0
                },
                "spectral_detector": {
                    "enabled": True,
                    "threshold": 10.0
                }
            }
        }
    )
    
    # Check response
    assert response.status_code == 200
    assert "success" in response.json()
    assert "type" in response.json()
    assert "status" in response.json()
    assert response.json()["success"] is True
    assert response.json()["type"] == "jamming_detection"


def test_detect_jamming(client):
    """Test GET /api/jamming/detect endpoint."""
    response = client.get("/api/jamming/detect")
    
    # Check response
    assert response.status_code == 200
    assert "is_jamming" in response.json()
    assert "jamming_type" in response.json()
    assert "confidence" in response.json()
    assert "snr_db" in response.json()
    assert "timestamp" in response.json()
    assert response.json()["is_jamming"] is True
    assert response.json()["jamming_type"] == "CONTINUOUS_WAVE"
    assert response.json()["confidence"] == 0.8
    assert response.json()["snr_db"] == 15.0


def test_estimate_doa(client):
    """Test GET /api/jamming/doa endpoint."""
    response = client.get("/api/jamming/doa")
    
    # Check response
    assert response.status_code == 200
    assert "azimuth" in response.json()
    assert "elevation" in response.json()
    assert "confidence" in response.json()
    assert "timestamp" in response.json()
    assert response.json()["azimuth"] == 45.0
    assert response.json()["elevation"] == 0.0
    assert response.json()["confidence"] == 0.8


def test_send_message(client):
    """Test POST /api/fhss/send endpoint."""
    response = client.post(
        "/api/fhss/send",
        json={
            "message": "Test message"
        }
    )
    
    # Check response
    assert response.status_code == 200
    assert "success" in response.json()
    assert "timestamp" in response.json()
    assert response.json()["success"] is True


def test_receive_message(client):
    """Test GET /api/fhss/receive endpoint."""
    response = client.get("/api/fhss/receive?timeout=1.0")
    
    # Check response
    assert response.status_code == 200
    assert "success" in response.json()
    assert "message" in response.json()
    assert "timestamp" in response.json()
    assert response.json()["success"] is True
    assert response.json()["message"] == "Test message"
