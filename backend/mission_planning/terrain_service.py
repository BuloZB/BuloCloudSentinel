"""
Terrain service for Bulo.Cloud Sentinel.

This module provides terrain elevation data for mission planning and execution.
"""

import aiohttp
import asyncio
import math
import numpy as np
from typing import List, Dict, Any, Optional, Tuple
import logging

from backend.mission_planning.models import Position

logger = logging.getLogger(__name__)


class TerrainService:
    """
    Service for retrieving terrain elevation data.
    
    This service provides methods for retrieving elevation data for specific
    locations or areas, enabling terrain-following capabilities for missions.
    """
    
    def __init__(self, api_key: Optional[str] = None):
        """
        Initialize the terrain service.
        
        Args:
            api_key: Optional API key for elevation data provider
        """
        self.api_key = api_key
        self.elevation_cache = {}
    
    async def get_elevation(self, latitude: float, longitude: float) -> Optional[float]:
        """
        Get elevation for a specific location.
        
        Args:
            latitude: Latitude in degrees
            longitude: Longitude in degrees
            
        Returns:
            Elevation in meters above sea level, or None if not available
        """
        # Check cache first
        cache_key = f"{latitude:.6f},{longitude:.6f}"
        if cache_key in self.elevation_cache:
            return self.elevation_cache[cache_key]
        
        try:
            # Use Open-Elevation API (free, but limited)
            async with aiohttp.ClientSession() as session:
                async with session.get(
                    "https://api.open-elevation.com/api/v1/lookup",
                    params={"locations": f"{latitude},{longitude}"}
                ) as response:
                    if response.status == 200:
                        data = await response.json()
                        if "results" in data and len(data["results"]) > 0:
                            elevation = data["results"][0].get("elevation")
                            
                            # Cache result
                            self.elevation_cache[cache_key] = elevation
                            
                            return elevation
            
            # Fallback to alternative API if needed
            # This is a placeholder - you would implement your preferred elevation API
            
            return None
        except Exception as e:
            logger.error(f"Error getting elevation data: {str(e)}")
            return None
    
    async def get_elevations(self, positions: List[Position]) -> List[Optional[float]]:
        """
        Get elevations for multiple positions.
        
        Args:
            positions: List of positions
            
        Returns:
            List of elevations in meters above sea level, or None for positions where
            elevation data is not available
        """
        # Batch positions into groups of 100 (API limit)
        batch_size = 100
        batches = [positions[i:i + batch_size] for i in range(0, len(positions), batch_size)]
        
        results = []
        
        for batch in batches:
            # Check cache first
            cached_batch = []
            uncached_positions = []
            uncached_indices = []
            
            for i, position in enumerate(batch):
                cache_key = f"{position.latitude:.6f},{position.longitude:.6f}"
                if cache_key in self.elevation_cache:
                    cached_batch.append(self.elevation_cache[cache_key])
                else:
                    cached_batch.append(None)
                    uncached_positions.append(position)
                    uncached_indices.append(i)
            
            # Fetch uncached positions
            if uncached_positions:
                try:
                    # Use Open-Elevation API (free, but limited)
                    locations = "|".join([f"{p.latitude},{p.longitude}" for p in uncached_positions])
                    
                    async with aiohttp.ClientSession() as session:
                        async with session.get(
                            "https://api.open-elevation.com/api/v1/lookup",
                            params={"locations": locations}
                        ) as response:
                            if response.status == 200:
                                data = await response.json()
                                if "results" in data:
                                    for i, result in enumerate(data["results"]):
                                        elevation = result.get("elevation")
                                        batch_index = uncached_indices[i]
                                        cached_batch[batch_index] = elevation
                                        
                                        # Cache result
                                        position = uncached_positions[i]
                                        cache_key = f"{position.latitude:.6f},{position.longitude:.6f}"
                                        self.elevation_cache[cache_key] = elevation
                
                except Exception as e:
                    logger.error(f"Error getting elevation data: {str(e)}")
            
            results.extend(cached_batch)
        
        return results
    
    async def get_elevation_profile(self, start: Position, end: Position, num_points: int = 100) -> List[Optional[float]]:
        """
        Get elevation profile between two positions.
        
        Args:
            start: Start position
            end: End position
            num_points: Number of points to sample
            
        Returns:
            List of elevations along the path
        """
        # Generate positions along the path
        positions = []
        
        for i in range(num_points):
            t = i / (num_points - 1)
            lat = start.latitude + t * (end.latitude - start.latitude)
            lon = start.longitude + t * (end.longitude - start.longitude)
            positions.append(Position(latitude=lat, longitude=lon))
        
        # Get elevations
        return await self.get_elevations(positions)
    
    async def adjust_altitude_for_terrain(self, 
                                         waypoints: List[Dict[str, Any]], 
                                         agl_altitude: float) -> List[Dict[str, Any]]:
        """
        Adjust waypoint altitudes for terrain following.
        
        Args:
            waypoints: List of waypoints
            agl_altitude: Desired altitude above ground level
            
        Returns:
            List of waypoints with adjusted altitudes
        """
        # Extract positions
        positions = [Position(latitude=wp["position"]["latitude"], 
                             longitude=wp["position"]["longitude"]) 
                    for wp in waypoints]
        
        # Get elevations
        elevations = await self.get_elevations(positions)
        
        # Adjust altitudes
        adjusted_waypoints = []
        
        for i, waypoint in enumerate(waypoints):
            elevation = elevations[i]
            
            if elevation is not None:
                # Create a copy of the waypoint
                adjusted_waypoint = waypoint.copy()
                
                # Adjust altitude
                adjusted_waypoint["position"] = adjusted_waypoint["position"].copy()
                adjusted_waypoint["position"]["altitude"] = elevation + agl_altitude
                adjusted_waypoint["position"]["relative_altitude"] = agl_altitude
                
                adjusted_waypoints.append(adjusted_waypoint)
            else:
                # If elevation data is not available, keep original altitude
                adjusted_waypoints.append(waypoint)
        
        return adjusted_waypoints
