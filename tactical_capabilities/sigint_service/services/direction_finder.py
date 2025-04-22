"""
Direction finder service for the SIGINT service.

This service is responsible for finding the direction of signal sources, including:
- Triangulating signal sources from multiple collectors
- Calculating bearing accuracy
- Estimating source location
"""

import logging
import numpy as np
from typing import Dict, List, Optional, Any, Tuple
from datetime import datetime, timedelta

from core.config import settings

logger = logging.getLogger(__name__)

class DirectionFinder:
    """Direction finder service."""
    
    def __init__(self):
        """Initialize the direction finder."""
        self.min_collectors = settings.DIRECTION_FINDING_MIN_COLLECTORS
        self.max_age = settings.DIRECTION_FINDING_MAX_AGE
    
    async def triangulate(self, bearings: List[Dict[str, Any]]) -> Dict[str, Any]:
        """
        Triangulate a signal source from multiple bearings.
        
        Args:
            bearings: List of bearings, each with lat, lon, and azimuth
            
        Returns:
            Triangulation results
        """
        # Check if we have enough bearings
        if len(bearings) < self.min_collectors:
            return {
                "success": False,
                "error": f"Insufficient bearings for triangulation. Need at least {self.min_collectors}, got {len(bearings)}.",
                "bearings": bearings
            }
        
        # Check if bearings are recent enough
        now = datetime.utcnow()
        recent_bearings = [
            b for b in bearings
            if "timestamp" not in b or (now - datetime.fromisoformat(b["timestamp"])).total_seconds() <= self.max_age
        ]
        
        if len(recent_bearings) < self.min_collectors:
            return {
                "success": False,
                "error": f"Insufficient recent bearings for triangulation. Need at least {self.min_collectors}, got {len(recent_bearings)}.",
                "bearings": bearings
            }
        
        # Perform triangulation
        try:
            source_location, accuracy = self._triangulate_bearings(recent_bearings)
            
            return {
                "success": True,
                "source_location": source_location,
                "accuracy": accuracy,
                "method": "triangulation",
                "bearing_count": len(recent_bearings)
            }
        except Exception as e:
            logger.exception(f"Error during triangulation: {e}")
            return {
                "success": False,
                "error": f"Triangulation failed: {str(e)}",
                "bearings": bearings
            }
    
    async def locate_signal_source(self, detections: List[Dict[str, Any]]) -> Dict[str, Any]:
        """
        Locate a signal source from multiple detections.
        
        Args:
            detections: List of signal detections
            
        Returns:
            Location results
        """
        # Extract bearings from detections
        bearings = []
        for detection in detections:
            if detection.get("location") and detection.get("direction"):
                bearings.append({
                    "lat": detection["location"]["lat"],
                    "lon": detection["location"]["lon"],
                    "azimuth": detection["direction"]["azimuth"],
                    "timestamp": detection.get("timestamp")
                })
        
        # Perform triangulation
        triangulation_result = await self.triangulate(bearings)
        
        # If triangulation failed, try signal strength method
        if not triangulation_result["success"] and len(detections) > 0:
            try:
                source_location, accuracy = self._locate_by_signal_strength(detections)
                
                return {
                    "success": True,
                    "source_location": source_location,
                    "accuracy": accuracy,
                    "method": "signal_strength",
                    "detection_count": len(detections)
                }
            except Exception as e:
                logger.exception(f"Error during signal strength location: {e}")
                return triangulation_result
        
        return triangulation_result
    
    def _triangulate_bearings(self, bearings: List[Dict[str, Any]]) -> Tuple[Dict[str, float], float]:
        """
        Triangulate a signal source from multiple bearings.
        
        Args:
            bearings: List of bearings, each with lat, lon, and azimuth
            
        Returns:
            Tuple of (source location, accuracy in meters)
        """
        # In a real implementation, this would use proper geospatial calculations
        # For now, we'll use a simplified approach
        
        # Convert lat/lon to a local Cartesian coordinate system
        # This is a simplification and only works for small areas
        x0, y0 = bearings[0]["lon"], bearings[0]["lat"]
        earth_radius = 6371000  # meters
        
        points = []
        directions = []
        
        for bearing in bearings:
            # Convert to local Cartesian coordinates (in meters)
            x = (bearing["lon"] - x0) * np.cos(np.radians(y0)) * earth_radius * np.pi / 180
            y = (bearing["lat"] - y0) * earth_radius * np.pi / 180
            
            # Convert azimuth to radians
            azimuth_rad = np.radians(bearing["azimuth"])
            
            # Calculate direction vector
            dx = np.sin(azimuth_rad)
            dy = np.cos(azimuth_rad)
            
            points.append((x, y))
            directions.append((dx, dy))
        
        # Solve the system of equations to find the intersection point
        A = np.zeros((len(bearings) * 2, 2))
        b = np.zeros(len(bearings) * 2)
        
        for i, ((x, y), (dx, dy)) in enumerate(zip(points, directions)):
            # Each bearing gives us a line equation
            A[i*2, 0] = dy
            A[i*2, 1] = -dx
            A[i*2+1, 0] = dx
            A[i*2+1, 1] = dy
            
            b[i*2] = dy * x - dx * y
            b[i*2+1] = dx * x + dy * y
        
        # Solve using least squares
        solution, residuals, rank, s = np.linalg.lstsq(A, b, rcond=None)
        
        # Convert back to lat/lon
        x_source, y_source = solution
        lon_source = x0 + x_source / (earth_radius * np.cos(np.radians(y0)) * np.pi / 180)
        lat_source = y0 + y_source / (earth_radius * np.pi / 180)
        
        # Calculate accuracy (in meters)
        if len(residuals) > 0:
            accuracy = np.sqrt(residuals[0])
        else:
            # If residuals is empty, use a default accuracy
            accuracy = 1000.0  # 1 km
        
        return {"lat": float(lat_source), "lon": float(lon_source)}, float(accuracy)
    
    def _locate_by_signal_strength(self, detections: List[Dict[str, Any]]) -> Tuple[Dict[str, float], float]:
        """
        Locate a signal source using signal strength measurements.
        
        Args:
            detections: List of signal detections
            
        Returns:
            Tuple of (source location, accuracy in meters)
        """
        # In a real implementation, this would use proper propagation models
        # For now, we'll use a simplified approach
        
        # Extract locations and signal strengths
        locations = []
        weights = []
        
        for detection in detections:
            if detection.get("location") and detection.get("signal_strength"):
                locations.append((
                    detection["location"]["lat"],
                    detection["location"]["lon"]
                ))
                
                # Convert dBm to linear scale and use as weight
                signal_strength_linear = 10 ** (detection["signal_strength"] / 10)
                weights.append(signal_strength_linear)
        
        if not locations:
            raise ValueError("No valid locations with signal strength")
        
        # Normalize weights
        total_weight = sum(weights)
        if total_weight == 0:
            raise ValueError("Total weight is zero")
        
        weights = [w / total_weight for w in weights]
        
        # Calculate weighted average
        lat = sum(lat * weight for (lat, _), weight in zip(locations, weights))
        lon = sum(lon * weight for (_, lon), weight in zip(locations, weights))
        
        # Calculate accuracy (standard deviation of distances)
        earth_radius = 6371000  # meters
        distances = []
        
        for (det_lat, det_lon) in locations:
            # Calculate distance using Haversine formula
            dlat = np.radians(det_lat - lat)
            dlon = np.radians(det_lon - lon)
            a = np.sin(dlat/2)**2 + np.cos(np.radians(lat)) * np.cos(np.radians(det_lat)) * np.sin(dlon/2)**2
            c = 2 * np.arctan2(np.sqrt(a), np.sqrt(1-a))
            distance = earth_radius * c
            
            distances.append(distance)
        
        accuracy = np.std(distances) if len(distances) > 1 else 1000.0  # 1 km default
        
        return {"lat": float(lat), "lon": float(lon)}, float(accuracy)
