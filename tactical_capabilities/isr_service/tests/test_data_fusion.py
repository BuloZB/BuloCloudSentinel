"""
Tests for the data fusion engine.
"""

import pytest
from datetime import datetime, timedelta
from services.data_fusion_engine import DataFusionEngine

@pytest.fixture
def data_fusion_engine():
    """Create a data fusion engine for testing."""
    return DataFusionEngine()

@pytest.mark.asyncio
async def test_fuse_observations(data_fusion_engine):
    """Test fusing observations."""
    # Create test observations
    observations = [
        {
            "id": "obs1",
            "timestamp": datetime.utcnow() - timedelta(seconds=10),
            "data_type": "image",
            "location": {"lat": 40.0, "lon": -74.0, "alt": 100.0}
        },
        {
            "id": "obs2",
            "timestamp": datetime.utcnow(),
            "data_type": "image",
            "location": {"lat": 40.01, "lon": -74.01, "alt": 110.0}
        }
    ]
    
    # Fuse observations
    fused = await data_fusion_engine.fuse_observations(observations)
    
    # Check that the most recent observation was used as the base
    assert fused["id"] == "obs2"
    assert fused["location"]["lat"] == 40.01
    assert fused["location"]["lon"] == -74.01
    assert fused["location"]["alt"] == 110.0
    
    # Check that fusion metadata was added
    assert "fusion" in fused["metadata"]
    assert fused["metadata"]["fusion"]["method"] == "most_recent"
    assert fused["metadata"]["fusion"]["source_count"] == 2
    assert "obs1" in fused["metadata"]["fusion"]["source_ids"]
    assert "obs2" in fused["metadata"]["fusion"]["source_ids"]

@pytest.mark.asyncio
async def test_fuse_detections(data_fusion_engine):
    """Test fusing detections."""
    # Create test detections
    detections = [
        {
            "id": "det1",
            "object_type": "person",
            "confidence": 0.7,
            "location": {"lat": 40.0, "lon": -74.0, "alt": 0.0},
            "velocity": {"x": 1.0, "y": 0.0, "z": 0.0}
        },
        {
            "id": "det2",
            "object_type": "person",
            "confidence": 0.8,
            "location": {"lat": 40.01, "lon": -74.01, "alt": 0.0},
            "velocity": {"x": 1.1, "y": 0.1, "z": 0.0}
        }
    ]
    
    # Fuse detections
    fused = await data_fusion_engine.fuse_detections(detections)
    
    # Check that weighted average was used for location and velocity
    assert fused["location"]["lat"] > 40.0 and fused["location"]["lat"] < 40.01
    assert fused["location"]["lon"] > -74.01 and fused["location"]["lon"] < -74.0
    assert fused["velocity"]["x"] > 1.0 and fused["velocity"]["x"] < 1.1
    assert fused["velocity"]["y"] > 0.0 and fused["velocity"]["y"] < 0.1
    
    # Check that maximum confidence was used
    assert fused["confidence"] == 0.8
    
    # Check that fusion metadata was added
    assert "fusion" in fused["metadata"]
    assert fused["metadata"]["fusion"]["method"] == "weighted_average"
    assert fused["metadata"]["fusion"]["source_count"] == 2
    assert "det1" in fused["metadata"]["fusion"]["source_ids"]
    assert "det2" in fused["metadata"]["fusion"]["source_ids"]

@pytest.mark.asyncio
async def test_correlate_detections(data_fusion_engine):
    """Test correlating detections."""
    # Create test detections
    detections = [
        {
            "id": "det1",
            "object_type": "person",
            "confidence": 0.7,
            "location": {"lat": 40.0, "lon": -74.0, "alt": 0.0}
        },
        {
            "id": "det2",
            "object_type": "person",
            "confidence": 0.8,
            "location": {"lat": 40.0001, "lon": -74.0001, "alt": 0.0}
        },
        {
            "id": "det3",
            "object_type": "vehicle",
            "confidence": 0.9,
            "location": {"lat": 41.0, "lon": -75.0, "alt": 0.0}
        }
    ]
    
    # Correlate detections
    groups = await data_fusion_engine.correlate_detections(detections)
    
    # Check that detections were grouped correctly
    assert len(groups) == 2
    
    # Check that close detections were grouped together
    assert len(groups[0]) == 2
    assert groups[0][0]["id"] in ["det1", "det2"]
    assert groups[0][1]["id"] in ["det1", "det2"]
    
    # Check that distant detection is in its own group
    assert len(groups[1]) == 1
    assert groups[1][0]["id"] == "det3"

@pytest.mark.asyncio
async def test_generate_intelligence_product(data_fusion_engine):
    """Test generating intelligence product."""
    # Create test targets
    targets = [
        {
            "id": "tgt1",
            "track_id": "TRK-1",
            "object_type": "person",
            "confidence": 0.7,
            "location": {"lat": 40.0, "lon": -74.0, "alt": 0.0}
        },
        {
            "id": "tgt2",
            "track_id": "TRK-2",
            "object_type": "person",
            "confidence": 0.8,
            "location": {"lat": 40.1, "lon": -74.1, "alt": 0.0}
        },
        {
            "id": "tgt3",
            "track_id": "TRK-3",
            "object_type": "vehicle",
            "confidence": 0.9,
            "location": {"lat": 40.2, "lon": -74.2, "alt": 0.0}
        }
    ]
    
    # Generate intelligence product
    product = await data_fusion_engine.generate_intelligence_product(targets)
    
    # Check product fields
    assert "id" in product
    assert "timestamp" in product
    assert product["target_count"] == 3
    assert product["target_types"]["person"] == 2
    assert product["target_types"]["vehicle"] == 1
    
    # Check area of interest
    assert product["area_of_interest"]["min_lat"] == 40.0
    assert product["area_of_interest"]["max_lat"] == 40.2
    assert product["area_of_interest"]["min_lon"] == -74.2
    assert product["area_of_interest"]["max_lon"] == -74.0
    
    # Check average confidence
    assert product["confidence"] == (0.7 + 0.8 + 0.9) / 3
    
    # Check targets in product
    assert len(product["targets"]) == 3
    assert product["targets"][0]["id"] == "tgt1"
    assert product["targets"][1]["id"] == "tgt2"
    assert product["targets"][2]["id"] == "tgt3"
