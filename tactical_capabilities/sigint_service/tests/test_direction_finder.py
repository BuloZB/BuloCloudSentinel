"""
Tests for the direction finder service.
"""

import pytest
import numpy as np
from datetime import datetime, timedelta
from services.direction_finder import DirectionFinder

@pytest.fixture
def direction_finder():
    """Create a direction finder for testing."""
    return DirectionFinder()

@pytest.mark.asyncio
async def test_triangulate(direction_finder):
    """Test triangulating a signal source."""
    # Create test bearings
    bearings = [
        {
            "lat": 40.0,
            "lon": -74.0,
            "azimuth": 45.0,
            "timestamp": datetime.utcnow().isoformat()
        },
        {
            "lat": 40.1,
            "lon": -74.1,
            "azimuth": 135.0,
            "timestamp": datetime.utcnow().isoformat()
        },
        {
            "lat": 40.05,
            "lon": -73.95,
            "azimuth": 225.0,
            "timestamp": datetime.utcnow().isoformat()
        }
    ]
    
    # Triangulate
    result = await direction_finder.triangulate(bearings)
    
    # Check result
    assert result["success"] is True
    assert "source_location" in result
    assert "lat" in result["source_location"]
    assert "lon" in result["source_location"]
    assert "accuracy" in result
    assert result["method"] == "triangulation"
    assert result["bearing_count"] == 3

@pytest.mark.asyncio
async def test_triangulate_insufficient_bearings(direction_finder):
    """Test triangulating with insufficient bearings."""
    # Create test bearings
    bearings = [
        {
            "lat": 40.0,
            "lon": -74.0,
            "azimuth": 45.0,
            "timestamp": datetime.utcnow().isoformat()
        }
    ]
    
    # Triangulate
    result = await direction_finder.triangulate(bearings)
    
    # Check result
    assert result["success"] is False
    assert "error" in result
    assert "Insufficient bearings" in result["error"]
    assert "bearings" in result

@pytest.mark.asyncio
async def test_triangulate_old_bearings(direction_finder):
    """Test triangulating with old bearings."""
    # Create test bearings
    bearings = [
        {
            "lat": 40.0,
            "lon": -74.0,
            "azimuth": 45.0,
            "timestamp": (datetime.utcnow() - timedelta(seconds=100)).isoformat()
        },
        {
            "lat": 40.1,
            "lon": -74.1,
            "azimuth": 135.0,
            "timestamp": (datetime.utcnow() - timedelta(seconds=100)).isoformat()
        }
    ]
    
    # Override max_age for testing
    direction_finder.max_age = 60
    
    # Triangulate
    result = await direction_finder.triangulate(bearings)
    
    # Check result
    assert result["success"] is False
    assert "error" in result
    assert "Insufficient recent bearings" in result["error"]
    assert "bearings" in result

@pytest.mark.asyncio
async def test_locate_signal_source(direction_finder):
    """Test locating a signal source."""
    # Create test detections
    detections = [
        {
            "id": "det1",
            "location": {"lat": 40.0, "lon": -74.0},
            "direction": {"azimuth": 45.0},
            "timestamp": datetime.utcnow().isoformat(),
            "signal_strength": -70.0
        },
        {
            "id": "det2",
            "location": {"lat": 40.1, "lon": -74.1},
            "direction": {"azimuth": 135.0},
            "timestamp": datetime.utcnow().isoformat(),
            "signal_strength": -75.0
        },
        {
            "id": "det3",
            "location": {"lat": 40.05, "lon": -73.95},
            "direction": {"azimuth": 225.0},
            "timestamp": datetime.utcnow().isoformat(),
            "signal_strength": -80.0
        }
    ]
    
    # Locate signal source
    result = await direction_finder.locate_signal_source(detections)
    
    # Check result
    assert result["success"] is True
    assert "source_location" in result
    assert "lat" in result["source_location"]
    assert "lon" in result["source_location"]
    assert "accuracy" in result
    assert result["method"] == "triangulation"
    assert result["bearing_count"] == 3

@pytest.mark.asyncio
async def test_locate_signal_source_no_direction(direction_finder):
    """Test locating a signal source without direction information."""
    # Create test detections
    detections = [
        {
            "id": "det1",
            "location": {"lat": 40.0, "lon": -74.0},
            "timestamp": datetime.utcnow().isoformat(),
            "signal_strength": -70.0
        },
        {
            "id": "det2",
            "location": {"lat": 40.1, "lon": -74.1},
            "timestamp": datetime.utcnow().isoformat(),
            "signal_strength": -75.0
        },
        {
            "id": "det3",
            "location": {"lat": 40.05, "lon": -73.95},
            "timestamp": datetime.utcnow().isoformat(),
            "signal_strength": -80.0
        }
    ]
    
    # Locate signal source
    result = await direction_finder.locate_signal_source(detections)
    
    # Check result
    assert result["success"] is True
    assert "source_location" in result
    assert "lat" in result["source_location"]
    assert "lon" in result["source_location"]
    assert "accuracy" in result
    assert result["method"] == "signal_strength"
    assert result["detection_count"] == 3
