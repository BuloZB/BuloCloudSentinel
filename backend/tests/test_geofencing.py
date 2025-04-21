"""
Unit tests for the geofencing service.
"""

import pytest
import asyncio
from datetime import datetime
from shapely.geometry import Point, Polygon

from backend.geofencing.models import (
    GeofenceZone, GeofenceZoneType, GeofenceZoneSource, GeofenceRestrictionLevel
)
from backend.geofencing.service import GeofencingService


@pytest.fixture
async def geofencing_service():
    """Create a geofencing service for testing."""
    service = GeofencingService()
    await service.start()
    yield service
    await service.stop()


@pytest.mark.asyncio
async def test_create_custom_zone(geofencing_service):
    """Test creating a custom geofence zone."""
    # Create a custom zone
    zone = GeofenceZone(
        name="Test Zone",
        description="Test zone for unit tests",
        zone_type=GeofenceZoneType.CUSTOM,
        source=GeofenceZoneSource.CUSTOM,
        restriction_level=GeofenceRestrictionLevel.NO_FLY,
        geometry_type="polygon",
        geometry={
            "coordinates": [
                [
                    [-122.5, 47.5],
                    [-122.5, 47.7],
                    [-122.3, 47.7],
                    [-122.3, 47.5],
                    [-122.5, 47.5]
                ]
            ]
        }
    )
    
    created_zone = await geofencing_service.create_custom_zone(zone)
    
    # Check that zone was created
    assert created_zone.id is not None
    assert created_zone.name == "Test Zone"
    assert created_zone.zone_type == GeofenceZoneType.CUSTOM
    assert created_zone.source == GeofenceZoneSource.CUSTOM
    assert created_zone.restriction_level == GeofenceRestrictionLevel.NO_FLY
    
    # Check that zone is in the service
    zones = await geofencing_service.list_zones()
    assert any(z.id == created_zone.id for z in zones)
    
    # Get zone by ID
    retrieved_zone = await geofencing_service.get_zone(created_zone.id)
    assert retrieved_zone is not None
    assert retrieved_zone.id == created_zone.id
    
    # Delete zone
    success = await geofencing_service.delete_custom_zone(created_zone.id)
    assert success
    
    # Check that zone is no longer in the service
    zones = await geofencing_service.list_zones()
    assert not any(z.id == created_zone.id for z in zones)


@pytest.mark.asyncio
async def test_validate_waypoint(geofencing_service):
    """Test validating a waypoint against geofence zones."""
    # Create a test zone
    zone = GeofenceZone(
        name="Test No-Fly Zone",
        description="Test no-fly zone for unit tests",
        zone_type=GeofenceZoneType.CUSTOM,
        source=GeofenceZoneSource.CUSTOM,
        restriction_level=GeofenceRestrictionLevel.NO_FLY,
        geometry_type="circle",
        geometry={
            "center": [-122.4, 47.6],
            "radius": 5000  # 5km radius
        }
    )
    
    created_zone = await geofencing_service.create_custom_zone(zone)
    
    # Test point inside zone
    valid, violations = await geofencing_service.validate_waypoint(47.6, -122.4)
    assert not valid
    assert len(violations) == 1
    assert violations[0].zone_id == created_zone.id
    
    # Test point outside zone
    valid, violations = await geofencing_service.validate_waypoint(47.7, -122.7)
    assert valid
    assert len(violations) == 0
    
    # Clean up
    await geofencing_service.delete_custom_zone(created_zone.id)


@pytest.mark.asyncio
async def test_validate_mission(geofencing_service):
    """Test validating a mission against geofence zones."""
    # Create a test zone
    zone = GeofenceZone(
        name="Test No-Fly Zone",
        description="Test no-fly zone for unit tests",
        zone_type=GeofenceZoneType.CUSTOM,
        source=GeofenceZoneSource.CUSTOM,
        restriction_level=GeofenceRestrictionLevel.NO_FLY,
        geometry_type="circle",
        geometry={
            "center": [-122.4, 47.6],
            "radius": 5000  # 5km radius
        }
    )
    
    created_zone = await geofencing_service.create_custom_zone(zone)
    
    # Create a mission that passes through the zone
    waypoints = [
        {"latitude": 47.5, "longitude": -122.5},
        {"latitude": 47.6, "longitude": -122.4},  # Inside zone
        {"latitude": 47.7, "longitude": -122.3}
    ]
    
    valid, violations = await geofencing_service.validate_mission(waypoints=waypoints)
    assert not valid
    assert len(violations) >= 1  # At least one violation (waypoint inside zone)
    
    # Create a mission that avoids the zone
    waypoints = [
        {"latitude": 47.5, "longitude": -122.5},
        {"latitude": 47.5, "longitude": -122.3},
        {"latitude": 47.7, "longitude": -122.3}
    ]
    
    valid, violations = await geofencing_service.validate_mission(waypoints=waypoints)
    assert valid
    assert len(violations) == 0
    
    # Clean up
    await geofencing_service.delete_custom_zone(created_zone.id)


@pytest.mark.asyncio
async def test_zone_to_shapely(geofencing_service):
    """Test converting a zone to a Shapely geometry."""
    # Polygon zone
    polygon_zone = GeofenceZone(
        name="Test Polygon",
        zone_type=GeofenceZoneType.CUSTOM,
        source=GeofenceZoneSource.CUSTOM,
        restriction_level=GeofenceRestrictionLevel.NO_FLY,
        geometry_type="polygon",
        geometry={
            "coordinates": [
                [
                    [-122.5, 47.5],
                    [-122.5, 47.7],
                    [-122.3, 47.7],
                    [-122.3, 47.5],
                    [-122.5, 47.5]
                ]
            ]
        }
    )
    
    polygon = geofencing_service._zone_to_shapely(polygon_zone)
    assert isinstance(polygon, Polygon)
    assert polygon.contains(Point(-122.4, 47.6))  # Inside
    assert not polygon.contains(Point(-122.6, 47.6))  # Outside
    
    # Circle zone
    circle_zone = GeofenceZone(
        name="Test Circle",
        zone_type=GeofenceZoneType.CUSTOM,
        source=GeofenceZoneSource.CUSTOM,
        restriction_level=GeofenceRestrictionLevel.NO_FLY,
        geometry_type="circle",
        geometry={
            "center": [-122.4, 47.6],
            "radius": 5000  # 5km radius
        }
    )
    
    circle = geofencing_service._zone_to_shapely(circle_zone)
    assert isinstance(circle, Polygon)  # Circles are approximated as polygons
    assert circle.contains(Point(-122.4, 47.6))  # Center
    
    # Convert 5km to degrees (approximate)
    deg_5km = 5000 / 111320.0
    assert circle.contains(Point(-122.4 + deg_5km/2, 47.6))  # Inside
    assert not circle.contains(Point(-122.4 + deg_5km*2, 47.6))  # Outside
