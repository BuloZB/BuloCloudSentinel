"""
Target tracker service for the ISR service.

This service is responsible for tracking targets, including:
- Creating and updating targets
- Associating detections with targets
- Tracking target movement
- Managing target lifecycle
"""

import logging
import uuid
from datetime import datetime, timedelta
from typing import Dict, List, Optional, Any, Tuple
import numpy as np

from core.config import settings

logger = logging.getLogger(__name__)

class TargetTracker:
    """Target tracker service."""
    
    def __init__(self):
        """Initialize the target tracker."""
        self.targets = {}
        self.confidence_threshold = settings.TARGET_TRACKING_CONFIDENCE_THRESHOLD
        self.max_age = settings.TARGET_TRACKING_MAX_AGE
    
    async def process_detection(self, detection: Dict[str, Any]) -> Optional[str]:
        """
        Process a detection and associate it with a target.
        
        Args:
            detection: Detection data
            
        Returns:
            Target ID if detection was associated with a target, None otherwise
        """
        # Check if detection meets confidence threshold
        if detection.get("confidence", 0) < self.confidence_threshold:
            logger.debug(f"Detection confidence {detection.get('confidence')} below threshold {self.confidence_threshold}")
            return None
        
        # Get detection location
        location = detection.get("location")
        if not location:
            logger.warning("Detection has no location")
            return None
        
        # Find closest target
        target_id, distance = self._find_closest_target(location)
        
        # If no target found or distance too large, create new target
        if target_id is None:
            target_id = self._create_target(detection)
            logger.info(f"Created new target {target_id}")
        else:
            # Update existing target
            self._update_target(target_id, detection)
            logger.debug(f"Updated target {target_id}")
        
        return target_id
    
    def _find_closest_target(self, location: Dict[str, float]) -> Tuple[Optional[str], float]:
        """
        Find the closest target to a location.
        
        Args:
            location: Location to find closest target to
            
        Returns:
            Tuple of (target_id, distance) or (None, float('inf')) if no targets
        """
        if not self.targets:
            return None, float('inf')
        
        closest_id = None
        closest_distance = float('inf')
        
        for target_id, target in self.targets.items():
            target_location = target.get("location")
            if not target_location:
                continue
            
            # Calculate distance
            distance = self._calculate_distance(location, target_location)
            
            # Update closest if this target is closer
            if distance < closest_distance:
                closest_id = target_id
                closest_distance = distance
        
        return closest_id, closest_distance
    
    def _calculate_distance(self, loc1: Dict[str, float], loc2: Dict[str, float]) -> float:
        """
        Calculate distance between two locations.
        
        Args:
            loc1: First location
            loc2: Second location
            
        Returns:
            Distance in meters
        """
        # In a real implementation, this would use proper geospatial calculations
        # For now, we'll just use Euclidean distance
        dx = loc1.get("lon", 0) - loc2.get("lon", 0)
        dy = loc1.get("lat", 0) - loc2.get("lat", 0)
        dz = (loc1.get("alt", 0) or 0) - (loc2.get("alt", 0) or 0)
        
        return (dx**2 + dy**2 + dz**2)**0.5
    
    def _create_target(self, detection: Dict[str, Any]) -> str:
        """
        Create a new target from a detection.
        
        Args:
            detection: Detection data
            
        Returns:
            Target ID
        """
        # Generate target ID
        target_id = str(uuid.uuid4())
        
        # Create target
        self.targets[target_id] = {
            "track_id": f"TRK-{target_id[:8]}",
            "object_type": detection.get("object_type", "unknown"),
            "first_seen": detection.get("timestamp", datetime.utcnow()),
            "last_seen": detection.get("timestamp", datetime.utcnow()),
            "status": "active",
            "confidence": detection.get("confidence", 0),
            "location": detection.get("location"),
            "velocity": detection.get("velocity"),
            "attributes": detection.get("attributes", {}),
            "metadata": detection.get("metadata", {}),
            "detections": [detection]
        }
        
        return target_id
    
    def _update_target(self, target_id: str, detection: Dict[str, Any]):
        """
        Update a target with a new detection.
        
        Args:
            target_id: Target ID
            detection: Detection data
        """
        target = self.targets.get(target_id)
        if not target:
            logger.warning(f"Target {target_id} not found")
            return
        
        # Update target fields
        target["last_seen"] = detection.get("timestamp", datetime.utcnow())
        target["status"] = "active"
        target["confidence"] = max(target["confidence"], detection.get("confidence", 0))
        target["location"] = detection.get("location")
        target["velocity"] = detection.get("velocity")
        
        # Update attributes and metadata
        if detection.get("attributes"):
            target["attributes"].update(detection["attributes"])
        if detection.get("metadata"):
            target["metadata"].update(detection["metadata"])
        
        # Add detection to target
        target["detections"].append(detection)
    
    async def update_targets(self):
        """Update all targets, marking old ones as lost or archived."""
        now = datetime.utcnow()
        
        for target_id, target in list(self.targets.items()):
            # Calculate age
            last_seen = target["last_seen"]
            age = (now - last_seen).total_seconds()
            
            # Update status based on age
            if age > self.max_age:
                if target["status"] == "lost":
                    # Archive target
                    target["status"] = "archived"
                    logger.info(f"Archived target {target_id}")
                    
                    # Remove from active tracking
                    del self.targets[target_id]
                else:
                    # Mark as lost
                    target["status"] = "lost"
                    logger.info(f"Lost target {target_id}")
    
    async def get_active_targets(self) -> List[Dict[str, Any]]:
        """
        Get all active targets.
        
        Returns:
            List of active targets
        """
        return [
            target for target in self.targets.values()
            if target["status"] == "active"
        ]
    
    async def get_target(self, target_id: str) -> Optional[Dict[str, Any]]:
        """
        Get a target by ID.
        
        Args:
            target_id: Target ID
            
        Returns:
            Target data or None if not found
        """
        return self.targets.get(target_id)
