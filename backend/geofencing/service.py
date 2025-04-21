"""
Geofencing Service for Bulo.Cloud Sentinel.

This module provides services for managing geofence zones and validating
missions and waypoints against these zones.
"""

import asyncio
import logging
import json
import math
import time
from typing import Dict, List, Any, Optional, Set, Tuple, Union
from datetime import datetime, timedelta
import httpx
from shapely.geometry import Point, Polygon, MultiPolygon, LineString
import rtree
import aioredis

from backend.geofencing.models import (
    GeofenceZone, GeofenceProvider, GeofenceZoneType, 
    GeofenceZoneSource, GeofenceRestrictionLevel,
    GeofenceViolation
)
from backend.mission_planning.models import Mission, Position, Waypoint

logger = logging.getLogger(__name__)

class GeofencingService:
    """
    Service for managing geofence zones and validating missions against them.
    
    This service handles the retrieval, storage, and querying of geofence zones
    from various sources, and provides validation of missions and waypoints
    against these zones.
    """
    
    def __init__(self, db_session=None, redis_url=None):
        """
        Initialize the geofencing service.
        
        Args:
            db_session: Database session for persistence
            redis_url: URL for Redis cache
        """
        self.db = db_session
        self.redis_url = redis_url
        self.redis = None
        
        # In-memory cache of geofence zones
        self._zones: Dict[str, GeofenceZone] = {}
        
        # Spatial index for efficient querying
        self._spatial_index = rtree.index.Index()
        self._index_counter = 0
        self._index_to_zone_id: Dict[int, str] = {}
        self._zone_id_to_index: Dict[str, int] = {}
        
        # Providers
        self._providers: Dict[str, GeofenceProvider] = {}
        
        # Background tasks
        self._background_tasks = set()
        self._running = False
    
    async def start(self):
        """Start the geofencing service."""
        if self._running:
            return
        
        self._running = True
        
        # Connect to Redis if URL provided
        if self.redis_url:
            self.redis = await aioredis.from_url(self.redis_url)
        
        # Load providers
        await self._load_providers()
        
        # Load zones
        await self._load_zones()
        
        # Start background update task
        task = asyncio.create_task(self._background_update())
        self._background_tasks.add(task)
        task.add_done_callback(self._background_tasks.remove)
        
        logger.info("Geofencing service started")
    
    async def stop(self):
        """Stop the geofencing service."""
        self._running = False
        
        # Cancel all background tasks
        for task in self._background_tasks:
            task.cancel()
        
        # Wait for all tasks to complete
        if self._background_tasks:
            await asyncio.gather(*self._background_tasks, return_exceptions=True)
        
        # Close Redis connection
        if self.redis:
            await self.redis.close()
        
        logger.info("Geofencing service stopped")
    
    async def list_zones(
        self,
        latitude: Optional[float] = None,
        longitude: Optional[float] = None,
        radius: Optional[float] = None,
        zone_type: Optional[GeofenceZoneType] = None
    ) -> List[GeofenceZone]:
        """
        List geofence zones, optionally filtered by location and type.
        
        Args:
            latitude: Optional latitude for location-based filtering
            longitude: Optional longitude for location-based filtering
            radius: Optional radius (in meters) for location-based filtering
            zone_type: Optional zone type for filtering
            
        Returns:
            List of geofence zones matching the filters
        """
        # If location filtering is requested, both lat and lon must be provided
        if (latitude is not None or longitude is not None) and (latitude is None or longitude is None):
            raise ValueError("Both latitude and longitude must be provided for location filtering")
        
        # If location filtering is requested, radius must be provided
        if latitude is not None and longitude is not None and radius is None:
            raise ValueError("Radius must be provided for location filtering")
        
        # Apply filters
        if latitude is not None and longitude is not None and radius is not None:
            # Location-based filtering
            # Convert radius from meters to degrees (approximate)
            radius_deg = radius / 111320.0  # 1 degree is approximately 111.32 km at the equator
            
            # Query spatial index
            min_x = longitude - radius_deg
            min_y = latitude - radius_deg
            max_x = longitude + radius_deg
            max_y = latitude + radius_deg
            
            # Get candidate zones from spatial index
            candidate_indices = list(self._spatial_index.intersection((min_x, min_y, max_x, max_y)))
            
            # Filter candidates by exact distance
            point = Point(longitude, latitude)
            result = []
            
            for idx in candidate_indices:
                zone_id = self._index_to_zone_id.get(idx)
                if not zone_id:
                    continue
                
                zone = self._zones.get(zone_id)
                if not zone:
                    continue
                
                # Apply zone type filter if provided
                if zone_type and zone.zone_type != zone_type:
                    continue
                
                # Check if zone is within radius
                try:
                    shapely_geom = self._zone_to_shapely(zone)
                    if shapely_geom.distance(point) * 111320.0 <= radius:
                        result.append(zone)
                except Exception as e:
                    logger.error(f"Error checking distance for zone {zone_id}: {str(e)}")
            
            return result
        elif zone_type:
            # Filter by zone type only
            return [zone for zone in self._zones.values() if zone.zone_type == zone_type]
        else:
            # No filtering
            return list(self._zones.values())
    
    async def get_zone(self, zone_id: str) -> Optional[GeofenceZone]:
        """
        Get a geofence zone by ID.
        
        Args:
            zone_id: ID of the zone to get
            
        Returns:
            Geofence zone if found, None otherwise
        """
        return self._zones.get(zone_id)
    
    async def create_custom_zone(self, zone: GeofenceZone) -> GeofenceZone:
        """
        Create a custom geofence zone.
        
        Args:
            zone: Zone to create
            
        Returns:
            Created zone
        """
        # Ensure zone is marked as custom
        zone.zone_type = GeofenceZoneType.CUSTOM
        zone.source = GeofenceZoneSource.CUSTOM
        
        # Add to in-memory cache
        self._zones[zone.id] = zone
        
        # Add to spatial index
        self._add_zone_to_spatial_index(zone)
        
        # Persist to database if available
        if self.db:
            await self._save_zone_to_db(zone)
        
        logger.info(f"Created custom geofence zone {zone.id}: {zone.name}")
        
        return zone
    
    async def update_custom_zone(self, zone_id: str, updates: Dict[str, Any]) -> Optional[GeofenceZone]:
        """
        Update a custom geofence zone.
        
        Args:
            zone_id: ID of the zone to update
            updates: Updates to apply
            
        Returns:
            Updated zone if found, None otherwise
        """
        # Get existing zone
        zone = self._zones.get(zone_id)
        if not zone:
            return None
        
        # Ensure zone is custom
        if zone.zone_type != GeofenceZoneType.CUSTOM or zone.source != GeofenceZoneSource.CUSTOM:
            raise ValueError("Only custom geofence zones can be updated")
        
        # Apply updates
        for key, value in updates.items():
            if hasattr(zone, key):
                setattr(zone, key, value)
        
        # Update timestamp
        zone.updated_at = datetime.utcnow()
        
        # Update spatial index
        self._remove_zone_from_spatial_index(zone_id)
        self._add_zone_to_spatial_index(zone)
        
        # Persist to database if available
        if self.db:
            await self._update_zone_in_db(zone)
        
        logger.info(f"Updated custom geofence zone {zone_id}: {zone.name}")
        
        return zone
    
    async def delete_custom_zone(self, zone_id: str) -> bool:
        """
        Delete a custom geofence zone.
        
        Args:
            zone_id: ID of the zone to delete
            
        Returns:
            True if zone was deleted, False otherwise
        """
        # Get existing zone
        zone = self._zones.get(zone_id)
        if not zone:
            return False
        
        # Ensure zone is custom
        if zone.zone_type != GeofenceZoneType.CUSTOM or zone.source != GeofenceZoneSource.CUSTOM:
            raise ValueError("Only custom geofence zones can be deleted")
        
        # Remove from in-memory cache
        del self._zones[zone_id]
        
        # Remove from spatial index
        self._remove_zone_from_spatial_index(zone_id)
        
        # Remove from database if available
        if self.db:
            await self._delete_zone_from_db(zone_id)
        
        logger.info(f"Deleted custom geofence zone {zone_id}: {zone.name}")
        
        return True
    
    async def validate_mission(self, mission_id: Optional[str] = None, waypoints: Optional[List[Dict[str, Any]]] = None) -> Tuple[bool, List[GeofenceViolation]]:
        """
        Validate a mission against geofence zones.
        
        Args:
            mission_id: Optional ID of the mission to validate
            waypoints: Optional list of waypoints to validate
            
        Returns:
            Tuple of (valid, violations)
        """
        # Get waypoints
        if mission_id:
            # Get mission from database
            if not self.db:
                raise ValueError("Database session required to validate mission by ID")
            
            # This is a placeholder - in a real implementation, you would fetch the mission from the database
            # mission = await self.db.get_mission(mission_id)
            # waypoints = mission.waypoints
            
            # For now, we'll just raise an error
            raise NotImplementedError("Validation by mission ID not implemented")
        
        if not waypoints:
            raise ValueError("Either mission_id or waypoints must be provided")
        
        # Validate each waypoint
        violations = []
        
        for i, waypoint in enumerate(waypoints):
            # Extract coordinates
            latitude = waypoint.get("latitude") or waypoint.get("position", {}).get("latitude")
            longitude = waypoint.get("longitude") or waypoint.get("position", {}).get("longitude")
            altitude = waypoint.get("altitude") or waypoint.get("position", {}).get("altitude")
            
            if latitude is None or longitude is None:
                raise ValueError(f"Waypoint {i} is missing latitude or longitude")
            
            # Validate waypoint
            valid, waypoint_violations = await self.validate_waypoint(latitude, longitude, altitude)
            
            # Add waypoint index to violations
            for violation in waypoint_violations:
                violation.waypoint_index = i
                violation.latitude = latitude
                violation.longitude = longitude
                violations.append(violation)
        
        # Check path between waypoints
        if len(waypoints) >= 2:
            path_violations = await self._validate_path(waypoints)
            violations.extend(path_violations)
        
        # Mission is valid if there are no violations
        valid = len(violations) == 0
        
        return valid, violations
    
    async def validate_waypoint(self, latitude: float, longitude: float, altitude: Optional[float] = None) -> Tuple[bool, List[GeofenceViolation]]:
        """
        Validate a waypoint against geofence zones.
        
        Args:
            latitude: Latitude of the waypoint
            longitude: Longitude of the waypoint
            altitude: Optional altitude of the waypoint
            
        Returns:
            Tuple of (valid, violations)
        """
        # Create point
        point = Point(longitude, latitude)
        
        # Find zones that contain the point
        violations = []
        
        for zone_id, zone in self._zones.items():
            try:
                # Check if zone contains point
                shapely_geom = self._zone_to_shapely(zone)
                if shapely_geom.contains(point):
                    # Check altitude restrictions if applicable
                    if altitude is not None and zone.altitude_min is not None and altitude < zone.altitude_min:
                        # Below minimum altitude, no violation
                        continue
                    
                    if altitude is not None and zone.altitude_max is not None and altitude > zone.altitude_max:
                        # Above maximum altitude, no violation
                        continue
                    
                    # Check if zone is currently active
                    if not self._is_zone_active(zone):
                        continue
                    
                    # Check restriction level
                    if zone.restriction_level in [GeofenceRestrictionLevel.NO_FLY, GeofenceRestrictionLevel.RESTRICTED]:
                        # Create violation
                        violation = GeofenceViolation(
                            zone_id=zone.id,
                            zone_name=zone.name,
                            zone_type=zone.zone_type,
                            restriction_level=zone.restriction_level,
                            description=f"Waypoint is inside {zone.restriction_level} zone: {zone.name}"
                        )
                        violations.append(violation)
            except Exception as e:
                logger.error(f"Error validating waypoint against zone {zone_id}: {str(e)}")
        
        # Waypoint is valid if there are no violations
        valid = len(violations) == 0
        
        return valid, violations
    
    async def list_providers(self) -> List[GeofenceProvider]:
        """
        List available geofence data providers.
        
        Returns:
            List of geofence providers
        """
        return list(self._providers.values())
    
    async def update_database(self) -> bool:
        """
        Update the geofence database from external providers.
        
        Returns:
            True if update was successful, False otherwise
        """
        success = True
        
        # Update from each enabled provider
        for provider_id, provider in self._providers.items():
            if not provider.enabled:
                continue
            
            try:
                # Update from provider
                await self._update_from_provider(provider)
            except Exception as e:
                logger.error(f"Error updating from provider {provider_id}: {str(e)}")
                success = False
        
        return success
    
    async def _load_providers(self):
        """Load geofence providers from database or initialize defaults."""
        if self.db:
            # Load from database
            # This is a placeholder - in a real implementation, you would load providers from the database
            pass
        
        # Initialize default providers if none loaded
        if not self._providers:
            # FAA
            faa_provider = GeofenceProvider(
                id="faa",
                name="FAA",
                description="Federal Aviation Administration airspace data",
                url="https://api.faa.gov/",
                api_key_required=True,
                api_key_configured=False,
                zone_types=[
                    GeofenceZoneType.AIRPORT,
                    GeofenceZoneType.HELIPORT,
                    GeofenceZoneType.CONTROLLED_AIRSPACE,
                    GeofenceZoneType.SPECIAL_USE_AIRSPACE
                ],
                enabled=True
            )
            self._providers[faa_provider.id] = faa_provider
            
            # AirMap
            airmap_provider = GeofenceProvider(
                id="airmap",
                name="AirMap",
                description="AirMap airspace data",
                url="https://api.airmap.com/",
                api_key_required=True,
                api_key_configured=False,
                zone_types=[
                    GeofenceZoneType.AIRPORT,
                    GeofenceZoneType.HELIPORT,
                    GeofenceZoneType.CONTROLLED_AIRSPACE,
                    GeofenceZoneType.SPECIAL_USE_AIRSPACE,
                    GeofenceZoneType.NATIONAL_PARK,
                    GeofenceZoneType.MILITARY,
                    GeofenceZoneType.TEMPORARY_FLIGHT_RESTRICTION
                ],
                enabled=True
            )
            self._providers[airmap_provider.id] = airmap_provider
            
            # OpenStreetMap
            osm_provider = GeofenceProvider(
                id="openstreetmap",
                name="OpenStreetMap",
                description="OpenStreetMap data for airports and other restricted areas",
                url="https://www.openstreetmap.org/",
                api_key_required=False,
                api_key_configured=True,
                zone_types=[
                    GeofenceZoneType.AIRPORT,
                    GeofenceZoneType.HELIPORT,
                    GeofenceZoneType.NATIONAL_PARK
                ],
                enabled=True
            )
            self._providers[osm_provider.id] = osm_provider
    
    async def _load_zones(self):
        """Load geofence zones from database or initialize defaults."""
        if self.db:
            # Load from database
            # This is a placeholder - in a real implementation, you would load zones from the database
            pass
        
        # Initialize default zones if none loaded
        if not self._zones:
            # Example airport zone
            airport_zone = GeofenceZone(
                id="example_airport",
                name="Example International Airport",
                description="Example airport with 5km no-fly zone",
                zone_type=GeofenceZoneType.AIRPORT,
                source=GeofenceZoneSource.CUSTOM,
                restriction_level=GeofenceRestrictionLevel.NO_FLY,
                geometry_type="circle",
                geometry={
                    "center": [-122.3, 47.6],  # Example coordinates
                    "radius": 5000  # 5km radius
                },
                altitude_max=400  # 400m max altitude
            )
            self._zones[airport_zone.id] = airport_zone
            self._add_zone_to_spatial_index(airport_zone)
            
            # Example national park
            park_zone = GeofenceZone(
                id="example_park",
                name="Example National Park",
                description="Example national park with no-fly zone",
                zone_type=GeofenceZoneType.NATIONAL_PARK,
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
            self._zones[park_zone.id] = park_zone
            self._add_zone_to_spatial_index(park_zone)
    
    async def _background_update(self):
        """Background task for updating geofence data."""
        while self._running:
            try:
                # Check if update is needed
                update_needed = False
                
                for provider in self._providers.values():
                    if not provider.enabled:
                        continue
                    
                    if provider.last_update is None:
                        update_needed = True
                        break
                    
                    # Update if last update was more than 24 hours ago
                    if datetime.utcnow() - provider.last_update > timedelta(hours=24):
                        update_needed = True
                        break
                
                if update_needed:
                    logger.info("Updating geofence database...")
                    await self.update_database()
                    logger.info("Geofence database update completed")
            except Exception as e:
                logger.error(f"Error in background update: {str(e)}")
            
            # Sleep for 1 hour
            await asyncio.sleep(3600)
    
    async def _update_from_provider(self, provider: GeofenceProvider):
        """
        Update geofence data from a provider.
        
        Args:
            provider: Provider to update from
        """
        # This is a placeholder - in a real implementation, you would fetch data from the provider's API
        logger.info(f"Updating from provider: {provider.name}")
        
        # Update provider last update timestamp
        provider.last_update = datetime.utcnow()
        
        # Persist provider to database if available
        if self.db:
            # This is a placeholder - in a real implementation, you would update the provider in the database
            pass
    
    def _add_zone_to_spatial_index(self, zone: GeofenceZone):
        """
        Add a zone to the spatial index.
        
        Args:
            zone: Zone to add
        """
        try:
            # Get bounding box
            shapely_geom = self._zone_to_shapely(zone)
            minx, miny, maxx, maxy = shapely_geom.bounds
            
            # Add to spatial index
            self._index_counter += 1
            idx = self._index_counter
            self._spatial_index.insert(idx, (minx, miny, maxx, maxy))
            
            # Store mapping
            self._index_to_zone_id[idx] = zone.id
            self._zone_id_to_index[zone.id] = idx
        except Exception as e:
            logger.error(f"Error adding zone {zone.id} to spatial index: {str(e)}")
    
    def _remove_zone_from_spatial_index(self, zone_id: str):
        """
        Remove a zone from the spatial index.
        
        Args:
            zone_id: ID of the zone to remove
        """
        # Get index
        idx = self._zone_id_to_index.get(zone_id)
        if idx is None:
            return
        
        # Remove from spatial index
        self._spatial_index.delete(idx, None)
        
        # Remove mappings
        del self._index_to_zone_id[idx]
        del self._zone_id_to_index[zone_id]
    
    def _zone_to_shapely(self, zone: GeofenceZone) -> Union[Polygon, MultiPolygon]:
        """
        Convert a zone to a Shapely geometry.
        
        Args:
            zone: Zone to convert
            
        Returns:
            Shapely geometry
        """
        if zone.geometry_type == "polygon":
            return Polygon(zone.geometry["coordinates"][0])
        elif zone.geometry_type == "multipolygon":
            return MultiPolygon([Polygon(poly[0]) for poly in zone.geometry["coordinates"]])
        elif zone.geometry_type == "circle":
            # For circles, we approximate with a polygon
            center = Point(zone.geometry["center"])
            return center.buffer(zone.geometry["radius"] / 111320.0)  # Convert meters to degrees
        else:
            raise ValueError(f"Unsupported geometry type: {zone.geometry_type}")
    
    def _is_zone_active(self, zone: GeofenceZone) -> bool:
        """
        Check if a zone is currently active.
        
        Args:
            zone: Zone to check
            
        Returns:
            True if zone is active, False otherwise
        """
        now = datetime.utcnow()
        
        # Check effective from
        if zone.effective_from and now < zone.effective_from:
            return False
        
        # Check effective to
        if zone.effective_to and now > zone.effective_to:
            return False
        
        return True
    
    async def _validate_path(self, waypoints: List[Dict[str, Any]]) -> List[GeofenceViolation]:
        """
        Validate a path between waypoints against geofence zones.
        
        Args:
            waypoints: List of waypoints
            
        Returns:
            List of violations
        """
        violations = []
        
        # Create path
        path = []
        for waypoint in waypoints:
            latitude = waypoint.get("latitude") or waypoint.get("position", {}).get("latitude")
            longitude = waypoint.get("longitude") or waypoint.get("position", {}).get("longitude")
            
            if latitude is None or longitude is None:
                continue
            
            path.append((longitude, latitude))
        
        if len(path) < 2:
            return violations
        
        # Create line
        line = LineString(path)
        
        # Check intersection with each zone
        for zone_id, zone in self._zones.items():
            try:
                # Check if zone intersects path
                shapely_geom = self._zone_to_shapely(zone)
                if shapely_geom.intersects(line):
                    # Check if zone is currently active
                    if not self._is_zone_active(zone):
                        continue
                    
                    # Check restriction level
                    if zone.restriction_level in [GeofenceRestrictionLevel.NO_FLY, GeofenceRestrictionLevel.RESTRICTED]:
                        # Create violation
                        violation = GeofenceViolation(
                            zone_id=zone.id,
                            zone_name=zone.name,
                            zone_type=zone.zone_type,
                            restriction_level=zone.restriction_level,
                            description=f"Flight path intersects {zone.restriction_level} zone: {zone.name}"
                        )
                        violations.append(violation)
            except Exception as e:
                logger.error(f"Error validating path against zone {zone_id}: {str(e)}")
        
        return violations
    
    async def _save_zone_to_db(self, zone: GeofenceZone):
        """
        Save a zone to the database.
        
        Args:
            zone: Zone to save
        """
        # This is a placeholder - in a real implementation, you would save the zone to the database
        pass
    
    async def _update_zone_in_db(self, zone: GeofenceZone):
        """
        Update a zone in the database.
        
        Args:
            zone: Zone to update
        """
        # This is a placeholder - in a real implementation, you would update the zone in the database
        pass
    
    async def _delete_zone_from_db(self, zone_id: str):
        """
        Delete a zone from the database.
        
        Args:
            zone_id: ID of the zone to delete
        """
        # This is a placeholder - in a real implementation, you would delete the zone from the database
        pass
