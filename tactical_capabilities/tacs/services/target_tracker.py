"""
Target tracker service for the TACS module.

This service is responsible for tracking targets, including:
- Creating and updating targets
- Associating sensor detections with targets
- Maintaining target tracks
- Predicting target movement
"""

import logging
import uuid
import numpy as np
from datetime import datetime, timedelta
from typing import Dict, List, Optional, Any, Tuple
from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy.future import select
from sqlalchemy import update, delete

from db.models import Target, Track, TrackPoint, Sensor
from api.schemas import (
    TargetCreate, TargetUpdate, Target as TargetSchema,
    TrackCreate, TrackUpdate, Track as TrackSchema,
    TrackPoint as TrackPointSchema,
    GeoLocation, Velocity, Acceleration
)
from services.kalman_filter import KalmanFilter

logger = logging.getLogger(__name__)

class TargetTracker:
    """Target tracker service."""
    
    def __init__(self, db: AsyncSession):
        """
        Initialize the target tracker.
        
        Args:
            db: Database session
        """
        self.db = db
        self.kalman_filters = {}  # target_id -> KalmanFilter
    
    async def create_target(self, target_data: TargetCreate) -> TargetSchema:
        """
        Create a new target.
        
        Args:
            target_data: Target data
            
        Returns:
            Created target
        """
        # Create target
        target = Target(
            name=target_data.name,
            type=target_data.type.value,
            classification=target_data.classification,
            confidence=target_data.confidence,
            location=target_data.location.dict(),
            velocity=target_data.velocity.dict() if target_data.velocity else None,
            dimensions=target_data.dimensions.dict() if target_data.dimensions else None,
            metadata=target_data.metadata,
            priority=target_data.priority,
            status=target_data.status.value,
            first_detected=datetime.utcnow(),
            last_updated=datetime.utcnow()
        )
        
        # Add to database
        self.db.add(target)
        await self.db.commit()
        await self.db.refresh(target)
        
        # Initialize Kalman filter for this target
        self._init_kalman_filter(str(target.id), target_data.location, target_data.velocity)
        
        # Convert to schema
        return self._convert_to_target_schema(target)
    
    async def get_target(self, target_id: str) -> Optional[TargetSchema]:
        """
        Get a target by ID.
        
        Args:
            target_id: Target ID
            
        Returns:
            Target or None if not found
        """
        # Get target
        result = await self.db.execute(select(Target).filter(Target.id == target_id))
        target = result.scalars().first()
        
        if not target:
            return None
        
        # Convert to schema
        return self._convert_to_target_schema(target)
    
    async def get_targets(
        self,
        skip: int = 0,
        limit: int = 100,
        target_type: Optional[str] = None,
        status: Optional[str] = None,
        min_confidence: Optional[float] = None,
        min_priority: Optional[int] = None,
        sensor_id: Optional[str] = None
    ) -> List[TargetSchema]:
        """
        Get all targets with optional filtering.
        
        Args:
            skip: Number of targets to skip
            limit: Maximum number of targets to return
            target_type: Filter by target type
            status: Filter by status
            min_confidence: Filter by minimum confidence
            min_priority: Filter by minimum priority
            sensor_id: Filter by sensor ID
            
        Returns:
            List of targets
        """
        # Build query
        query = select(Target)
        
        # Apply filters
        if target_type:
            query = query.filter(Target.type == target_type)
        if status:
            query = query.filter(Target.status == status)
        if min_confidence is not None:
            query = query.filter(Target.confidence >= min_confidence)
        if min_priority is not None:
            query = query.filter(Target.priority >= min_priority)
        if sensor_id:
            query = query.join(Target.sensors).filter(Sensor.id == sensor_id)
        
        # Apply pagination
        query = query.offset(skip).limit(limit)
        
        # Execute query
        result = await self.db.execute(query)
        targets = result.scalars().all()
        
        # Convert to schemas
        return [self._convert_to_target_schema(target) for target in targets]
    
    async def update_target(self, target_id: str, target_data: TargetUpdate) -> Optional[TargetSchema]:
        """
        Update a target.
        
        Args:
            target_id: Target ID
            target_data: Target update data
            
        Returns:
            Updated target or None if not found
        """
        # Get target
        result = await self.db.execute(select(Target).filter(Target.id == target_id))
        target = result.scalars().first()
        
        if not target:
            return None
        
        # Update fields
        update_data = target_data.dict(exclude_unset=True)
        
        # Handle nested objects
        if "location" in update_data and update_data["location"]:
            update_data["location"] = update_data["location"].dict()
        if "velocity" in update_data and update_data["velocity"]:
            update_data["velocity"] = update_data["velocity"].dict()
        if "dimensions" in update_data and update_data["dimensions"]:
            update_data["dimensions"] = update_data["dimensions"].dict()
        
        # Update enum values
        if "type" in update_data and update_data["type"]:
            update_data["type"] = update_data["type"].value
        if "status" in update_data and update_data["status"]:
            update_data["status"] = update_data["status"].value
        
        # Update target
        for key, value in update_data.items():
            setattr(target, key, value)
        
        # Update last_updated timestamp
        target.last_updated = datetime.utcnow()
        
        # Commit changes
        await self.db.commit()
        await self.db.refresh(target)
        
        # Update Kalman filter if location or velocity changed
        if "location" in update_data or "velocity" in update_data:
            location = GeoLocation(**target.location)
            velocity = Velocity(**target.velocity) if target.velocity else None
            self._update_kalman_filter(str(target.id), location, velocity)
        
        # Convert to schema
        return self._convert_to_target_schema(target)
    
    async def delete_target(self, target_id: str) -> bool:
        """
        Delete a target.
        
        Args:
            target_id: Target ID
            
        Returns:
            True if successful, False if target not found
        """
        # Get target
        result = await self.db.execute(select(Target).filter(Target.id == target_id))
        target = result.scalars().first()
        
        if not target:
            return False
        
        # Delete target
        await self.db.delete(target)
        await self.db.commit()
        
        # Remove Kalman filter
        if str(target_id) in self.kalman_filters:
            del self.kalman_filters[str(target_id)]
        
        return True
    
    async def create_track(self, track_data: TrackCreate) -> TrackSchema:
        """
        Create a new track.
        
        Args:
            track_data: Track data
            
        Returns:
            Created track
        """
        # Create track
        track = Track(
            target_id=track_data.target_id,
            start_time=track_data.start_time,
            quality=track_data.quality,
            status="active",
            metadata=track_data.metadata
        )
        
        # Add to database
        self.db.add(track)
        await self.db.commit()
        await self.db.refresh(track)
        
        # Create track points
        for point_data in track_data.points:
            track_point = TrackPoint(
                track_id=track.id,
                timestamp=point_data.timestamp,
                location=point_data.location.dict(),
                altitude=point_data.altitude,
                velocity=point_data.velocity.dict() if point_data.velocity else None,
                acceleration=point_data.acceleration.dict() if point_data.acceleration else None,
                heading=point_data.heading,
                confidence=point_data.confidence,
                sensor_id=point_data.sensor_id
            )
            self.db.add(track_point)
        
        await self.db.commit()
        await self.db.refresh(track)
        
        # Convert to schema
        return self._convert_to_track_schema(track)
    
    async def get_track(self, track_id: str) -> Optional[TrackSchema]:
        """
        Get a track by ID.
        
        Args:
            track_id: Track ID
            
        Returns:
            Track or None if not found
        """
        # Get track
        result = await self.db.execute(select(Track).filter(Track.id == track_id))
        track = result.scalars().first()
        
        if not track:
            return None
        
        # Convert to schema
        return self._convert_to_track_schema(track)
    
    async def get_tracks(
        self,
        skip: int = 0,
        limit: int = 100,
        target_id: Optional[str] = None,
        status: Optional[str] = None,
        min_quality: Optional[float] = None,
        start_time_after: Optional[datetime] = None,
        end_time_before: Optional[datetime] = None
    ) -> List[TrackSchema]:
        """
        Get all tracks with optional filtering.
        
        Args:
            skip: Number of tracks to skip
            limit: Maximum number of tracks to return
            target_id: Filter by target ID
            status: Filter by status
            min_quality: Filter by minimum quality
            start_time_after: Filter by start time after
            end_time_before: Filter by end time before
            
        Returns:
            List of tracks
        """
        # Build query
        query = select(Track)
        
        # Apply filters
        if target_id:
            query = query.filter(Track.target_id == target_id)
        if status:
            query = query.filter(Track.status == status)
        if min_quality is not None:
            query = query.filter(Track.quality >= min_quality)
        if start_time_after:
            query = query.filter(Track.start_time >= start_time_after)
        if end_time_before:
            query = query.filter(Track.end_time <= end_time_before)
        
        # Apply pagination
        query = query.offset(skip).limit(limit)
        
        # Execute query
        result = await self.db.execute(query)
        tracks = result.scalars().all()
        
        # Convert to schemas
        return [self._convert_to_track_schema(track) for track in tracks]
    
    async def update_track(self, track_id: str, track_data: TrackUpdate) -> Optional[TrackSchema]:
        """
        Update a track.
        
        Args:
            track_id: Track ID
            track_data: Track update data
            
        Returns:
            Updated track or None if not found
        """
        # Get track
        result = await self.db.execute(select(Track).filter(Track.id == track_id))
        track = result.scalars().first()
        
        if not track:
            return None
        
        # Update fields
        update_data = track_data.dict(exclude_unset=True)
        
        # Update enum values
        if "status" in update_data and update_data["status"]:
            update_data["status"] = update_data["status"].value
        
        # Handle points separately
        points = update_data.pop("points", None)
        
        # Update track
        for key, value in update_data.items():
            setattr(track, key, value)
        
        # Add new points if provided
        if points:
            for point_data in points:
                track_point = TrackPoint(
                    track_id=track.id,
                    timestamp=point_data.timestamp,
                    location=point_data.location.dict(),
                    altitude=point_data.altitude,
                    velocity=point_data.velocity.dict() if point_data.velocity else None,
                    acceleration=point_data.acceleration.dict() if point_data.acceleration else None,
                    heading=point_data.heading,
                    confidence=point_data.confidence,
                    sensor_id=point_data.sensor_id
                )
                self.db.add(track_point)
        
        # Commit changes
        await self.db.commit()
        await self.db.refresh(track)
        
        # Convert to schema
        return self._convert_to_track_schema(track)
    
    async def delete_track(self, track_id: str) -> bool:
        """
        Delete a track.
        
        Args:
            track_id: Track ID
            
        Returns:
            True if successful, False if track not found
        """
        # Get track
        result = await self.db.execute(select(Track).filter(Track.id == track_id))
        track = result.scalars().first()
        
        if not track:
            return False
        
        # Delete track
        await self.db.delete(track)
        await self.db.commit()
        
        return True
    
    async def add_track_point(
        self,
        track_id: str,
        timestamp: datetime,
        location: GeoLocation,
        sensor_id: str,
        altitude: Optional[float] = None,
        velocity: Optional[Velocity] = None,
        acceleration: Optional[Acceleration] = None,
        heading: Optional[float] = None,
        confidence: float = 0.8
    ) -> Optional[TrackPointSchema]:
        """
        Add a point to a track.
        
        Args:
            track_id: Track ID
            timestamp: Timestamp
            location: Location
            sensor_id: Sensor ID
            altitude: Optional altitude
            velocity: Optional velocity
            acceleration: Optional acceleration
            heading: Optional heading
            confidence: Confidence (0.0 to 1.0)
            
        Returns:
            Created track point or None if track not found
        """
        # Get track
        result = await self.db.execute(select(Track).filter(Track.id == track_id))
        track = result.scalars().first()
        
        if not track:
            return None
        
        # Create track point
        track_point = TrackPoint(
            track_id=track.id,
            timestamp=timestamp,
            location=location.dict(),
            altitude=altitude,
            velocity=velocity.dict() if velocity else None,
            acceleration=acceleration.dict() if acceleration else None,
            heading=heading,
            confidence=confidence,
            sensor_id=sensor_id
        )
        
        # Add to database
        self.db.add(track_point)
        await self.db.commit()
        await self.db.refresh(track_point)
        
        # Update track
        track.last_updated = datetime.utcnow()
        await self.db.commit()
        
        # Update target location and velocity
        await self._update_target_from_track_point(track.target_id, track_point)
        
        # Convert to schema
        return self._convert_to_track_point_schema(track_point)
    
    async def process_detection(
        self,
        location: GeoLocation,
        sensor_id: str,
        detection_type: str,
        confidence: float,
        timestamp: Optional[datetime] = None,
        velocity: Optional[Velocity] = None,
        dimensions: Optional[Dict[str, float]] = None,
        classification: Optional[str] = None,
        metadata: Optional[Dict[str, Any]] = None
    ) -> Tuple[TargetSchema, bool]:
        """
        Process a new detection and associate it with a target.
        
        Args:
            location: Detection location
            sensor_id: Sensor ID
            detection_type: Type of detection
            confidence: Detection confidence
            timestamp: Detection timestamp
            velocity: Optional velocity
            dimensions: Optional dimensions
            classification: Optional classification
            metadata: Optional metadata
            
        Returns:
            Tuple of (target, is_new_target)
        """
        # Set timestamp if not provided
        if not timestamp:
            timestamp = datetime.utcnow()
        
        # Try to associate with existing target
        target, is_new = await self._associate_detection(
            location=location,
            sensor_id=sensor_id,
            detection_type=detection_type,
            confidence=confidence,
            timestamp=timestamp,
            velocity=velocity,
            dimensions=dimensions,
            classification=classification,
            metadata=metadata
        )
        
        return target, is_new
    
    async def predict_target_location(
        self,
        target_id: str,
        time_delta: float  # seconds
    ) -> Optional[GeoLocation]:
        """
        Predict a target's location after a time delta.
        
        Args:
            target_id: Target ID
            time_delta: Time delta in seconds
            
        Returns:
            Predicted location or None if target not found
        """
        # Get target
        result = await self.db.execute(select(Target).filter(Target.id == target_id))
        target = result.scalars().first()
        
        if not target:
            return None
        
        # Get Kalman filter for this target
        kalman_filter = self.kalman_filters.get(str(target_id))
        
        if not kalman_filter:
            # Initialize Kalman filter if not exists
            location = GeoLocation(**target.location)
            velocity = Velocity(**target.velocity) if target.velocity else None
            kalman_filter = self._init_kalman_filter(str(target_id), location, velocity)
        
        # Predict location
        predicted_state = kalman_filter.predict(time_delta)
        
        # Convert to GeoLocation
        predicted_location = GeoLocation(
            latitude=predicted_state[0],
            longitude=predicted_state[1],
            altitude=target.location.get("altitude")
        )
        
        return predicted_location
    
    def _init_kalman_filter(
        self,
        target_id: str,
        location: GeoLocation,
        velocity: Optional[Velocity] = None
    ) -> KalmanFilter:
        """
        Initialize a Kalman filter for a target.
        
        Args:
            target_id: Target ID
            location: Initial location
            velocity: Initial velocity
            
        Returns:
            Initialized Kalman filter
        """
        # Initial state [lat, lon, lat_vel, lon_vel]
        initial_state = np.array([
            location.latitude,
            location.longitude,
            velocity.x if velocity else 0.0,
            velocity.y if velocity else 0.0
        ])
        
        # Create Kalman filter
        kalman_filter = KalmanFilter(
            initial_state=initial_state,
            process_noise=1e-5,
            measurement_noise=1e-3
        )
        
        # Store Kalman filter
        self.kalman_filters[target_id] = kalman_filter
        
        return kalman_filter
    
    def _update_kalman_filter(
        self,
        target_id: str,
        location: GeoLocation,
        velocity: Optional[Velocity] = None
    ):
        """
        Update a Kalman filter with new measurements.
        
        Args:
            target_id: Target ID
            location: New location
            velocity: New velocity
        """
        # Get Kalman filter
        kalman_filter = self.kalman_filters.get(target_id)
        
        if not kalman_filter:
            # Initialize if not exists
            kalman_filter = self._init_kalman_filter(target_id, location, velocity)
        
        # Measurement [lat, lon]
        measurement = np.array([
            location.latitude,
            location.longitude
        ])
        
        # Update Kalman filter
        kalman_filter.update(measurement)
    
    async def _update_target_from_track_point(self, target_id: str, track_point: TrackPoint):
        """
        Update a target's location and velocity from a track point.
        
        Args:
            target_id: Target ID
            track_point: Track point
        """
        # Get target
        result = await self.db.execute(select(Target).filter(Target.id == target_id))
        target = result.scalars().first()
        
        if not target:
            return
        
        # Update location
        target.location = track_point.location
        
        # Update velocity if available
        if track_point.velocity:
            target.velocity = track_point.velocity
        
        # Update last_updated timestamp
        target.last_updated = datetime.utcnow()
        
        # Commit changes
        await self.db.commit()
        
        # Update Kalman filter
        location = GeoLocation(**track_point.location)
        velocity = Velocity(**track_point.velocity) if track_point.velocity else None
        self._update_kalman_filter(str(target_id), location, velocity)
    
    async def _associate_detection(
        self,
        location: GeoLocation,
        sensor_id: str,
        detection_type: str,
        confidence: float,
        timestamp: datetime,
        velocity: Optional[Velocity] = None,
        dimensions: Optional[Dict[str, float]] = None,
        classification: Optional[str] = None,
        metadata: Optional[Dict[str, Any]] = None
    ) -> Tuple[TargetSchema, bool]:
        """
        Associate a detection with a target.
        
        Args:
            location: Detection location
            sensor_id: Sensor ID
            detection_type: Type of detection
            confidence: Detection confidence
            timestamp: Detection timestamp
            velocity: Optional velocity
            dimensions: Optional dimensions
            classification: Optional classification
            metadata: Optional metadata
            
        Returns:
            Tuple of (target, is_new_target)
        """
        # Get recent active targets
        recent_time = timestamp - timedelta(minutes=5)
        query = select(Target).filter(
            Target.status == "active",
            Target.last_updated >= recent_time
        )
        result = await self.db.execute(query)
        targets = result.scalars().all()
        
        # Find closest target
        closest_target = None
        min_distance = float('inf')
        
        for target in targets:
            # Skip targets of different type
            if target.type != detection_type:
                continue
            
            # Calculate distance
            target_location = GeoLocation(**target.location)
            distance = self._calculate_distance(location, target_location)
            
            # Check if within association threshold
            association_threshold = 100.0  # meters
            if distance < association_threshold and distance < min_distance:
                closest_target = target
                min_distance = distance
        
        if closest_target:
            # Associate with existing target
            target_id = str(closest_target.id)
            
            # Get active track or create new one
            track = await self._get_or_create_active_track(target_id)
            
            # Add track point
            await self.add_track_point(
                track_id=str(track.id),
                timestamp=timestamp,
                location=location,
                sensor_id=sensor_id,
                velocity=velocity,
                confidence=confidence
            )
            
            # Update target
            target_update = TargetUpdate(
                location=location,
                velocity=velocity,
                confidence=max(closest_target.confidence, confidence * 0.8),  # Blend confidences
                last_updated=timestamp
            )
            
            # Update classification if provided and confidence is high
            if classification and confidence > 0.7:
                target_update.classification = classification
            
            # Update dimensions if provided
            if dimensions:
                from api.schemas import Dimensions as DimensionsSchema
                target_update.dimensions = DimensionsSchema(**dimensions)
            
            # Update target
            updated_target = await self.update_target(target_id, target_update)
            
            return updated_target, False
        else:
            # Create new target
            target_create = TargetCreate(
                name=f"Target-{uuid.uuid4().hex[:8]}",
                type=detection_type,
                classification=classification,
                confidence=confidence,
                location=location,
                velocity=velocity,
                metadata=metadata or {}
            )
            
            # Add dimensions if provided
            if dimensions:
                from api.schemas import Dimensions as DimensionsSchema
                target_create.dimensions = DimensionsSchema(**dimensions)
            
            # Create target
            new_target = await self.create_target(target_create)
            
            # Create track
            track_create = TrackCreate(
                target_id=new_target.id,
                start_time=timestamp,
                quality=confidence,
                metadata={}
            )
            
            # Add initial track point
            track_point = TrackPointBase(
                timestamp=timestamp,
                location=location,
                velocity=velocity,
                confidence=confidence,
                sensor_id=sensor_id
            )
            track_create.points.append(track_point)
            
            # Create track
            await self.create_track(track_create)
            
            return new_target, True
    
    async def _get_or_create_active_track(self, target_id: str) -> Track:
        """
        Get an active track for a target or create a new one.
        
        Args:
            target_id: Target ID
            
        Returns:
            Active track
        """
        # Get active track
        query = select(Track).filter(
            Track.target_id == target_id,
            Track.status == "active"
        )
        result = await self.db.execute(query)
        track = result.scalars().first()
        
        if track:
            return track
        
        # Create new track
        track = Track(
            target_id=target_id,
            start_time=datetime.utcnow(),
            quality=0.8,
            status="active",
            metadata={}
        )
        
        # Add to database
        self.db.add(track)
        await self.db.commit()
        await self.db.refresh(track)
        
        return track
    
    def _calculate_distance(self, loc1: GeoLocation, loc2: GeoLocation) -> float:
        """
        Calculate distance between two locations in meters.
        
        Args:
            loc1: First location
            loc2: Second location
            
        Returns:
            Distance in meters
        """
        # Haversine formula
        from math import radians, sin, cos, sqrt, atan2
        
        R = 6371000  # Earth radius in meters
        
        lat1 = radians(loc1.latitude)
        lon1 = radians(loc1.longitude)
        lat2 = radians(loc2.latitude)
        lon2 = radians(loc2.longitude)
        
        dlon = lon2 - lon1
        dlat = lat2 - lat1
        
        a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
        c = 2 * atan2(sqrt(a), sqrt(1-a))
        
        distance = R * c
        
        return distance
    
    def _convert_to_target_schema(self, target: Target) -> TargetSchema:
        """
        Convert a target database model to a schema.
        
        Args:
            target: Target database model
            
        Returns:
            Target schema
        """
        from api.schemas import (
            TargetType, TargetStatus,
            GeoLocation, Velocity, Dimensions
        )
        
        # Convert location
        location = GeoLocation(**target.location)
        
        # Convert velocity if exists
        velocity = Velocity(**target.velocity) if target.velocity else None
        
        # Convert dimensions if exists
        dimensions = Dimensions(**target.dimensions) if target.dimensions else None
        
        # Convert to schema
        return TargetSchema(
            id=target.id,
            name=target.name,
            type=TargetType(target.type),
            classification=target.classification,
            confidence=target.confidence,
            location=location,
            velocity=velocity,
            dimensions=dimensions,
            first_detected=target.first_detected,
            last_updated=target.last_updated,
            source_sensors=[sensor.id for sensor in target.sensors],
            metadata=target.metadata,
            priority=target.priority,
            status=TargetStatus(target.status),
            created_at=target.created_at,
            updated_at=target.updated_at
        )
    
    def _convert_to_track_schema(self, track: Track) -> TrackSchema:
        """
        Convert a track database model to a schema.
        
        Args:
            track: Track database model
            
        Returns:
            Track schema
        """
        from api.schemas import TrackStatus
        
        # Convert track points
        points = [self._convert_to_track_point_schema(point) for point in track.points]
        
        # Convert to schema
        return TrackSchema(
            id=track.id,
            target_id=track.target_id,
            start_time=track.start_time,
            end_time=track.end_time,
            points=points,
            quality=track.quality,
            status=TrackStatus(track.status),
            metadata=track.metadata,
            created_at=track.created_at,
            updated_at=track.updated_at
        )
    
    def _convert_to_track_point_schema(self, track_point: TrackPoint) -> TrackPointSchema:
        """
        Convert a track point database model to a schema.
        
        Args:
            track_point: Track point database model
            
        Returns:
            Track point schema
        """
        from api.schemas import GeoLocation, Velocity, Acceleration
        
        # Convert location
        location = GeoLocation(**track_point.location)
        
        # Convert velocity if exists
        velocity = Velocity(**track_point.velocity) if track_point.velocity else None
        
        # Convert acceleration if exists
        acceleration = Acceleration(**track_point.acceleration) if track_point.acceleration else None
        
        # Convert to schema
        return TrackPointSchema(
            timestamp=track_point.timestamp,
            location=location,
            altitude=track_point.altitude,
            velocity=velocity,
            acceleration=acceleration,
            heading=track_point.heading,
            confidence=track_point.confidence,
            sensor_id=track_point.sensor_id
        )
