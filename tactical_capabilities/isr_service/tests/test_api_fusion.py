"""
Tests for the fusion API endpoints.
"""

import pytest
from datetime import datetime
from fastapi.testclient import TestClient
from unittest.mock import AsyncMock, MagicMock, patch

from main import app
from services.data_fusion_engine import DataFusionEngine
from services.event_publisher import EventPublisher

@pytest.fixture
def client():
    """Create a test client."""
    # Create mock services
    app.state.data_fusion_engine = MagicMock(spec=DataFusionEngine)
    app.state.event_publisher = MagicMock(spec=EventPublisher)
    
    # Setup async mock methods
    app.state.data_fusion_engine.fuse_observations = AsyncMock()
    app.state.data_fusion_engine.fuse_detections = AsyncMock()
    app.state.data_fusion_engine.correlate_detections = AsyncMock()
    app.state.data_fusion_engine.generate_intelligence_product = AsyncMock()
    app.state.event_publisher.publish_intelligence_product = AsyncMock()
    
    # Create test client
    return TestClient(app)

@pytest.mark.asyncio
async def test_fuse_observations(client):
    """Test fusing observations endpoint."""
    # Mock response
    app.state.data_fusion_engine.fuse_observations.return_value = {
        "id": "fused-obs",
        "timestamp": datetime.utcnow().isoformat(),
        "metadata": {"fusion": {"method": "test"}}
    }
    
    # Test data
    observations = [
        {"id": "obs1", "timestamp": "2023-01-01T00:00:00Z"},
        {"id": "obs2", "timestamp": "2023-01-01T00:01:00Z"}
    ]
    
    # Mock JWT token validation
    with patch("core.security.decode_token", return_value=MagicMock(
        sub="test-user",
        roles=["admin"],
        permissions=["fusion:process"]
    )):
        # Make request
        response = client.post(
            "/api/v1/fusion/observations",
            json=observations,
            headers={"Authorization": "Bearer test-token"}
        )
    
    # Check response
    assert response.status_code == 200
    assert response.json()["id"] == "fused-obs"
    assert "fusion" in response.json()["metadata"]
    
    # Check that service was called
    app.state.data_fusion_engine.fuse_observations.assert_called_once_with(observations)

@pytest.mark.asyncio
async def test_fuse_detections(client):
    """Test fusing detections endpoint."""
    # Mock response
    app.state.data_fusion_engine.fuse_detections.return_value = {
        "id": "fused-det",
        "confidence": 0.9,
        "metadata": {"fusion": {"method": "test"}}
    }
    
    # Test data
    detections = [
        {"id": "det1", "confidence": 0.7},
        {"id": "det2", "confidence": 0.8}
    ]
    
    # Mock JWT token validation
    with patch("core.security.decode_token", return_value=MagicMock(
        sub="test-user",
        roles=["admin"],
        permissions=["fusion:process"]
    )):
        # Make request
        response = client.post(
            "/api/v1/fusion/detections",
            json=detections,
            headers={"Authorization": "Bearer test-token"}
        )
    
    # Check response
    assert response.status_code == 200
    assert response.json()["id"] == "fused-det"
    assert response.json()["confidence"] == 0.9
    assert "fusion" in response.json()["metadata"]
    
    # Check that service was called
    app.state.data_fusion_engine.fuse_detections.assert_called_once_with(detections)

@pytest.mark.asyncio
async def test_correlate_detections(client):
    """Test correlating detections endpoint."""
    # Mock response
    app.state.data_fusion_engine.correlate_detections.return_value = [
        [{"id": "det1"}, {"id": "det2"}],
        [{"id": "det3"}]
    ]
    
    # Test data
    detections = [
        {"id": "det1"},
        {"id": "det2"},
        {"id": "det3"}
    ]
    
    # Mock JWT token validation
    with patch("core.security.decode_token", return_value=MagicMock(
        sub="test-user",
        roles=["admin"],
        permissions=["fusion:process"]
    )):
        # Make request
        response = client.post(
            "/api/v1/fusion/correlate",
            json=detections,
            headers={"Authorization": "Bearer test-token"}
        )
    
    # Check response
    assert response.status_code == 200
    assert len(response.json()) == 2
    assert len(response.json()[0]) == 2
    assert len(response.json()[1]) == 1
    assert response.json()[0][0]["id"] == "det1"
    assert response.json()[0][1]["id"] == "det2"
    assert response.json()[1][0]["id"] == "det3"
    
    # Check that service was called
    app.state.data_fusion_engine.correlate_detections.assert_called_once_with(detections)

@pytest.mark.asyncio
async def test_generate_intelligence_product(client):
    """Test generating intelligence product endpoint."""
    # Mock response
    app.state.data_fusion_engine.generate_intelligence_product.return_value = {
        "id": "intel-1",
        "target_count": 2,
        "confidence": 0.85
    }
    
    # Test data
    targets = [
        {"id": "tgt1", "confidence": 0.8},
        {"id": "tgt2", "confidence": 0.9}
    ]
    
    # Mock JWT token validation
    with patch("core.security.decode_token", return_value=MagicMock(
        sub="test-user",
        roles=["admin"],
        permissions=["fusion:process"]
    )):
        # Make request
        response = client.post(
            "/api/v1/fusion/intelligence",
            json=targets,
            headers={"Authorization": "Bearer test-token"}
        )
    
    # Check response
    assert response.status_code == 200
    assert response.json()["id"] == "intel-1"
    assert response.json()["target_count"] == 2
    assert response.json()["confidence"] == 0.85
    
    # Check that services were called
    app.state.data_fusion_engine.generate_intelligence_product.assert_called_once_with(targets)
    app.state.event_publisher.publish_intelligence_product.assert_called_once()
