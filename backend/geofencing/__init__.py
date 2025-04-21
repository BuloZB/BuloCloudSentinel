"""
Geofencing module for Bulo.Cloud Sentinel.

This module provides functionality for managing geofence zones and
validating missions against them.
"""

from backend.geofencing.models import (
    GeofenceZone, GeofenceProvider, GeofenceZoneType, 
    GeofenceZoneSource, GeofenceRestrictionLevel,
    GeofenceViolation
)
from backend.geofencing.service import GeofencingService
