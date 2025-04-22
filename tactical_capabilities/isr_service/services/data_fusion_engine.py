"""
Data fusion engine for the ISR service.

This service is responsible for fusing data from multiple sensors, including:
- Combining observations from different sensors
- Correlating detections across sensors
- Improving target tracking through sensor fusion
- Generating fused intelligence products
"""

import logging
import uuid
from datetime import datetime
from typing import Dict, List, Optional, Any, Tuple
import numpy as np

from core.config import settings

logger = logging.getLogger(__name__)

class DataFusionEngine:
    """Data fusion engine service."""
    
    def __init__(self):
        """Initialize the data fusion engine."""
        self.observations = {}
        self.detections = {}
        self.confidence_threshold = settings.DATA_FUSION_CONFIDENCE_THRESHOLD
    
    async def fuse_observations(self, observations: List[Dict[str, Any]]) -> Dict[str, Any]:
        """
        Fuse multiple observations into a single observation.
        
        Args:
            observations: List of observations to fuse
            
        Returns:
            Fused observation
        """
        if not observations:
            logger.warning("No observations to fuse")
            return {}
        
        # Store observations
        for observation in observations:
            observation_id = observation.get("id")
            if observation_id:
                self.observations[observation_id] = observation
        
        # In a real implementation, this would use sophisticated fusion algorithms
        # For now, we'll just take the most recent observation as the fused result
        
        # Sort observations by timestamp
        sorted_observations = sorted(
            observations,
            key=lambda x: x.get("timestamp", datetime.min),
            reverse=True
        )
        
        # Take the most recent observation as the base
        fused = sorted_observations[0].copy()
        
        # Add metadata about fusion
        fused["metadata"] = fused.get("metadata", {})
        fused["metadata"]["fusion"] = {
            "method": "most_recent",
            "source_count": len(observations),
            "source_ids": [obs.get("id") for obs in observations],
            "timestamp": datetime.utcnow().isoformat()
        }
        
        return fused
    
    async def fuse_detections(self, detections: List[Dict[str, Any]]) -> Dict[str, Any]:
        """
        Fuse multiple detections into a single detection.
        
        Args:
            detections: List of detections to fuse
            
        Returns:
            Fused detection
        """
        if not detections:
            logger.warning("No detections to fuse")
            return {}
        
        # Store detections
        for detection in detections:
            detection_id = detection.get("id")
            if detection_id:
                self.detections[detection_id] = detection
        
        # In a real implementation, this would use sophisticated fusion algorithms
        # For now, we'll use a simple weighted average based on confidence
        
        # Calculate total confidence
        total_confidence = sum(detection.get("confidence", 0) for detection in detections)
        
        if total_confidence <= 0:
            logger.warning("Total confidence is zero or negative")
            return detections[0].copy()
        
        # Initialize fused detection with first detection
        fused = detections[0].copy()
        
        # Fuse location if available
        locations = [d.get("location") for d in detections if d.get("location")]
        confidences = [d.get("confidence", 0) for d in detections if d.get("location")]
        
        if locations and sum(confidences) > 0:
            fused["location"] = self._weighted_average_location(locations, confidences)
        
        # Fuse velocity if available
        velocities = [d.get("velocity") for d in detections if d.get("velocity")]
        confidences = [d.get("confidence", 0) for d in detections if d.get("velocity")]
        
        if velocities and sum(confidences) > 0:
            fused["velocity"] = self._weighted_average_velocity(velocities, confidences)
        
        # Fuse confidence (use maximum confidence)
        fused["confidence"] = max(detection.get("confidence", 0) for detection in detections)
        
        # Add metadata about fusion
        fused["metadata"] = fused.get("metadata", {})
        fused["metadata"]["fusion"] = {
            "method": "weighted_average",
            "source_count": len(detections),
            "source_ids": [det.get("id") for det in detections],
            "timestamp": datetime.utcnow().isoformat()
        }
        
        return fused
    
    def _weighted_average_location(self, locations: List[Dict[str, float]], confidences: List[float]) -> Dict[str, float]:
        """
        Calculate weighted average of locations.
        
        Args:
            locations: List of locations
            confidences: List of confidences
            
        Returns:
            Weighted average location
        """
        total_confidence = sum(confidences)
        
        # Calculate weighted average for each coordinate
        lat = sum(loc.get("lat", 0) * conf for loc, conf in zip(locations, confidences)) / total_confidence
        lon = sum(loc.get("lon", 0) * conf for loc, conf in zip(locations, confidences)) / total_confidence
        
        # Calculate weighted average for altitude if available
        alts = [loc.get("alt") for loc in locations]
        if all(alt is not None for alt in alts):
            alt = sum(loc.get("alt", 0) * conf for loc, conf in zip(locations, confidences)) / total_confidence
        else:
            # If any altitude is missing, use the first available one
            alt = next((alt for alt in alts if alt is not None), None)
        
        return {"lat": lat, "lon": lon, "alt": alt}
    
    def _weighted_average_velocity(self, velocities: List[Dict[str, float]], confidences: List[float]) -> Dict[str, float]:
        """
        Calculate weighted average of velocities.
        
        Args:
            velocities: List of velocities
            confidences: List of confidences
            
        Returns:
            Weighted average velocity
        """
        total_confidence = sum(confidences)
        
        # Calculate weighted average for each component
        x = sum(vel.get("x", 0) * conf for vel, conf in zip(velocities, confidences)) / total_confidence
        y = sum(vel.get("y", 0) * conf for vel, conf in zip(velocities, confidences)) / total_confidence
        
        # Calculate weighted average for z if available
        zs = [vel.get("z") for vel in velocities]
        if all(z is not None for z in zs):
            z = sum(vel.get("z", 0) * conf for vel, conf in zip(velocities, confidences)) / total_confidence
        else:
            # If any z is missing, use the first available one
            z = next((z for z in zs if z is not None), None)
        
        return {"x": x, "y": y, "z": z}
    
    async def correlate_detections(self, detections: List[Dict[str, Any]]) -> List[List[Dict[str, Any]]]:
        """
        Correlate detections across sensors.
        
        Args:
            detections: List of detections to correlate
            
        Returns:
            List of correlated detection groups
        """
        if not detections:
            return []
        
        # In a real implementation, this would use sophisticated correlation algorithms
        # For now, we'll use a simple distance-based approach
        
        # Initialize correlation groups
        groups = []
        
        # Process each detection
        for detection in detections:
            # Get detection location
            location = detection.get("location")
            if not location:
                # If no location, create a new group with just this detection
                groups.append([detection])
                continue
            
            # Find closest group
            closest_group = None
            closest_distance = float('inf')
            
            for group in groups:
                # Calculate average location of group
                group_locations = [d.get("location") for d in group if d.get("location")]
                if not group_locations:
                    continue
                
                # Calculate average location
                avg_lat = sum(loc.get("lat", 0) for loc in group_locations) / len(group_locations)
                avg_lon = sum(loc.get("lon", 0) for loc in group_locations) / len(group_locations)
                avg_alt = sum((loc.get("alt", 0) or 0) for loc in group_locations) / len(group_locations)
                
                avg_location = {"lat": avg_lat, "lon": avg_lon, "alt": avg_alt}
                
                # Calculate distance
                distance = self._calculate_distance(location, avg_location)
                
                # Update closest if this group is closer
                if distance < closest_distance:
                    closest_group = group
                    closest_distance = distance
            
            # If close enough to a group, add to that group
            if closest_group is not None and closest_distance < 0.001:  # ~100m at equator
                closest_group.append(detection)
            else:
                # Otherwise, create a new group
                groups.append([detection])
        
        return groups
    
    def _calculate_distance(self, loc1: Dict[str, float], loc2: Dict[str, float]) -> float:
        """
        Calculate distance between two locations.
        
        Args:
            loc1: First location
            loc2: Second location
            
        Returns:
            Distance in degrees (approximate)
        """
        # In a real implementation, this would use proper geospatial calculations
        # For now, we'll just use Euclidean distance
        dx = loc1.get("lon", 0) - loc2.get("lon", 0)
        dy = loc1.get("lat", 0) - loc2.get("lat", 0)
        dz = (loc1.get("alt", 0) or 0) - (loc2.get("alt", 0) or 0) / 111000  # Convert meters to approximate degrees
        
        return (dx**2 + dy**2 + dz**2)**0.5
    
    async def generate_intelligence_product(self, targets: List[Dict[str, Any]]) -> Dict[str, Any]:
        """
        Generate an intelligence product from fused data.
        
        Args:
            targets: List of targets
            
        Returns:
            Intelligence product
        """
        if not targets:
            logger.warning("No targets for intelligence product")
            return {}
        
        # In a real implementation, this would generate a comprehensive intelligence product
        # For now, we'll just create a simple summary
        
        # Count targets by type
        target_types = {}
        for target in targets:
            target_type = target.get("object_type", "unknown")
            target_types[target_type] = target_types.get(target_type, 0) + 1
        
        # Calculate area of interest
        locations = [t.get("location") for t in targets if t.get("location")]
        if locations:
            min_lat = min(loc.get("lat", 0) for loc in locations)
            max_lat = max(loc.get("lat", 0) for loc in locations)
            min_lon = min(loc.get("lon", 0) for loc in locations)
            max_lon = max(loc.get("lon", 0) for loc in locations)
            
            area_of_interest = {
                "min_lat": min_lat,
                "max_lat": max_lat,
                "min_lon": min_lon,
                "max_lon": max_lon
            }
        else:
            area_of_interest = {}
        
        # Create intelligence product
        product = {
            "id": str(uuid.uuid4()),
            "timestamp": datetime.utcnow().isoformat(),
            "target_count": len(targets),
            "target_types": target_types,
            "area_of_interest": area_of_interest,
            "confidence": sum(t.get("confidence", 0) for t in targets) / len(targets),
            "targets": [
                {
                    "id": t.get("id"),
                    "track_id": t.get("track_id"),
                    "object_type": t.get("object_type"),
                    "location": t.get("location"),
                    "velocity": t.get("velocity"),
                    "confidence": t.get("confidence")
                }
                for t in targets
            ]
        }
        
        return product
