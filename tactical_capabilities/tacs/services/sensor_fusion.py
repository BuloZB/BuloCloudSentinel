"""
Sensor fusion service for the TACS module.

This service is responsible for fusing data from multiple sensors, including:
- Combining detections from multiple sensors
- Improving target location accuracy
- Enhancing target classification
- Reducing false positives
"""

import logging
import uuid
import numpy as np
from datetime import datetime
from typing import Dict, List, Optional, Any, Tuple
from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy.future import select

from db.models import Target, Sensor, SensorData, FusionJob, FusionResult
from api.schemas import (
    GeoLocation, Velocity, Dimensions,
    FusionJobCreate, FusionJob as FusionJobSchema
)

logger = logging.getLogger(__name__)

class SensorFusion:
    """Sensor fusion service."""
    
    def __init__(self, db: AsyncSession):
        """
        Initialize the sensor fusion service.
        
        Args:
            db: Database session
        """
        self.db = db
    
    async def create_fusion_job(self, job_data: FusionJobCreate) -> FusionJobSchema:
        """
        Create a new fusion job.
        
        Args:
            job_data: Fusion job data
            
        Returns:
            Created fusion job
        """
        # Create fusion job
        fusion_job = FusionJob(
            target_id=job_data.target_id,
            status="pending",
            parameters=job_data.parameters,
            priority=job_data.priority,
            metadata=job_data.metadata
        )
        
        # Add to database
        self.db.add(fusion_job)
        await self.db.commit()
        await self.db.refresh(fusion_job)
        
        # Associate sensor data
        for sensor_data_id in job_data.sensor_data_ids:
            # Get sensor data
            result = await self.db.execute(select(SensorData).filter(SensorData.id == sensor_data_id))
            sensor_data = result.scalars().first()
            
            if sensor_data:
                # Add association
                fusion_job.sensor_data.append(sensor_data)
        
        # Commit changes
        await self.db.commit()
        await self.db.refresh(fusion_job)
        
        # Convert to schema
        return self._convert_to_fusion_job_schema(fusion_job)
    
    async def get_fusion_job(self, job_id: str) -> Optional[FusionJobSchema]:
        """
        Get a fusion job by ID.
        
        Args:
            job_id: Fusion job ID
            
        Returns:
            Fusion job or None if not found
        """
        # Get fusion job
        result = await self.db.execute(select(FusionJob).filter(FusionJob.id == job_id))
        fusion_job = result.scalars().first()
        
        if not fusion_job:
            return None
        
        # Convert to schema
        return self._convert_to_fusion_job_schema(fusion_job)
    
    async def process_fusion_job(self, job_id: str) -> Optional[FusionJobSchema]:
        """
        Process a fusion job.
        
        Args:
            job_id: Fusion job ID
            
        Returns:
            Updated fusion job or None if not found
        """
        # Get fusion job
        result = await self.db.execute(select(FusionJob).filter(FusionJob.id == job_id))
        fusion_job = result.scalars().first()
        
        if not fusion_job:
            return None
        
        try:
            # Update status
            fusion_job.status = "processing"
            await self.db.commit()
            
            # Get sensor data
            sensor_data_list = []
            for sensor_data in fusion_job.sensor_data:
                sensor_data_list.append(sensor_data)
            
            # Check if we have enough data
            if len(sensor_data_list) < 2:
                fusion_job.status = "failed"
                fusion_job.error_message = "Not enough sensor data for fusion"
                fusion_job.completed_at = datetime.utcnow()
                await self.db.commit()
                return self._convert_to_fusion_job_schema(fusion_job)
            
            # Process fusion based on data type
            data_types = set(sd.data_type for sd in sensor_data_list)
            
            if "detection" in data_types:
                # Fuse detections
                result_data = await self._fuse_detections(sensor_data_list, fusion_job.parameters)
            elif "image" in data_types:
                # Fuse images
                result_data = await self._fuse_images(sensor_data_list, fusion_job.parameters)
            elif "track" in data_types:
                # Fuse tracks
                result_data = await self._fuse_tracks(sensor_data_list, fusion_job.parameters)
            else:
                # Unknown data type
                fusion_job.status = "failed"
                fusion_job.error_message = f"Unsupported data types: {data_types}"
                fusion_job.completed_at = datetime.utcnow()
                await self.db.commit()
                return self._convert_to_fusion_job_schema(fusion_job)
            
            # Create fusion result
            fusion_result = FusionResult(
                fusion_job_id=fusion_job.id,
                result_type="detection" if "detection" in data_types else list(data_types)[0],
                result_data=result_data,
                confidence=result_data.get("confidence", 0.0),
                metadata={
                    "sensor_data_ids": [str(sd.id) for sd in sensor_data_list],
                    "fusion_parameters": fusion_job.parameters
                }
            )
            
            # Add to database
            self.db.add(fusion_result)
            await self.db.commit()
            await self.db.refresh(fusion_result)
            
            # Update fusion job
            fusion_job.status = "completed"
            fusion_job.result_id = fusion_result.id
            fusion_job.completed_at = datetime.utcnow()
            await self.db.commit()
            
            # Update target if specified
            if fusion_job.target_id and "location" in result_data:
                # Get target
                result = await self.db.execute(select(Target).filter(Target.id == fusion_job.target_id))
                target = result.scalars().first()
                
                if target:
                    # Update target location
                    target.location = result_data["location"]
                    
                    # Update velocity if available
                    if "velocity" in result_data:
                        target.velocity = result_data["velocity"]
                    
                    # Update confidence
                    if "confidence" in result_data:
                        target.confidence = max(target.confidence, result_data["confidence"])
                    
                    # Update last_updated timestamp
                    target.last_updated = datetime.utcnow()
                    
                    # Commit changes
                    await self.db.commit()
            
            # Return updated fusion job
            return self._convert_to_fusion_job_schema(fusion_job)
        
        except Exception as e:
            logger.error(f"Error processing fusion job {job_id}: {e}")
            
            # Update fusion job status
            fusion_job.status = "failed"
            fusion_job.error_message = str(e)
            fusion_job.completed_at = datetime.utcnow()
            await self.db.commit()
            
            return self._convert_to_fusion_job_schema(fusion_job)
    
    async def _fuse_detections(
        self,
        sensor_data_list: List[SensorData],
        parameters: Dict[str, Any]
    ) -> Dict[str, Any]:
        """
        Fuse detections from multiple sensors.
        
        Args:
            sensor_data_list: List of sensor data
            parameters: Fusion parameters
            
        Returns:
            Fusion result data
        """
        # Extract detections
        detections = []
        for sensor_data in sensor_data_list:
            # Get sensor
            result = await self.db.execute(select(Sensor).filter(Sensor.id == sensor_data.sensor_id))
            sensor = result.scalars().first()
            
            if not sensor:
                continue
            
            # Extract detection data
            detection = {
                "sensor_id": str(sensor.id),
                "sensor_type": sensor.type,
                "sensor_accuracy": sensor.accuracy,
                "timestamp": sensor_data.timestamp,
                "data": sensor_data.metadata.get("detection", {})
            }
            
            detections.append(detection)
        
        # Check if we have enough detections
        if len(detections) < 2:
            return {
                "error": "Not enough detections for fusion",
                "confidence": 0.0
            }
        
        # Fuse detections based on fusion method
        fusion_method = parameters.get("fusion_method", "weighted_average")
        
        if fusion_method == "weighted_average":
            return self._fuse_detections_weighted_average(detections, parameters)
        elif fusion_method == "kalman":
            return self._fuse_detections_kalman(detections, parameters)
        elif fusion_method == "particle":
            return self._fuse_detections_particle(detections, parameters)
        else:
            return {
                "error": f"Unsupported fusion method: {fusion_method}",
                "confidence": 0.0
            }
    
    def _fuse_detections_weighted_average(
        self,
        detections: List[Dict[str, Any]],
        parameters: Dict[str, Any]
    ) -> Dict[str, Any]:
        """
        Fuse detections using weighted average.
        
        Args:
            detections: List of detections
            parameters: Fusion parameters
            
        Returns:
            Fusion result data
        """
        # Extract locations and weights
        locations = []
        weights = []
        
        for detection in detections:
            # Get location
            data = detection["data"]
            if "location" not in data:
                continue
            
            location = data["location"]
            if "latitude" not in location or "longitude" not in location:
                continue
            
            # Get weight (based on sensor accuracy and detection confidence)
            sensor_accuracy = detection.get("sensor_accuracy", 0.5)
            detection_confidence = data.get("confidence", 0.5)
            
            # Calculate weight
            weight = sensor_accuracy * detection_confidence
            
            # Add to lists
            locations.append((location["latitude"], location["longitude"]))
            weights.append(weight)
        
        # Check if we have enough locations
        if len(locations) < 2:
            return {
                "error": "Not enough valid locations for fusion",
                "confidence": 0.0
            }
        
        # Normalize weights
        total_weight = sum(weights)
        if total_weight > 0:
            weights = [w / total_weight for w in weights]
        else:
            # Equal weights if all weights are zero
            weights = [1.0 / len(weights)] * len(weights)
        
        # Calculate weighted average
        lat_sum = sum(loc[0] * w for loc, w in zip(locations, weights))
        lon_sum = sum(loc[1] * w for loc, w in zip(locations, weights))
        
        # Calculate confidence
        confidence = min(1.0, sum(w**2 for w in weights) * 2)
        
        # Create result
        result = {
            "location": {
                "latitude": lat_sum,
                "longitude": lon_sum
            },
            "confidence": confidence,
            "fusion_method": "weighted_average",
            "num_detections": len(detections),
            "timestamp": datetime.utcnow().isoformat()
        }
        
        # Add velocity if available in detections
        velocities = []
        velocity_weights = []
        
        for detection, weight in zip(detections, weights):
            data = detection["data"]
            if "velocity" in data:
                velocity = data["velocity"]
                if "x" in velocity and "y" in velocity:
                    velocities.append((velocity["x"], velocity["y"]))
                    velocity_weights.append(weight)
        
        if velocities:
            # Calculate weighted average velocity
            vx_sum = sum(vel[0] * w for vel, w in zip(velocities, velocity_weights))
            vy_sum = sum(vel[1] * w for vel, w in zip(velocities, velocity_weights))
            
            # Add to result
            result["velocity"] = {
                "x": vx_sum,
                "y": vy_sum
            }
        
        return result
    
    def _fuse_detections_kalman(
        self,
        detections: List[Dict[str, Any]],
        parameters: Dict[str, Any]
    ) -> Dict[str, Any]:
        """
        Fuse detections using Kalman filter.
        
        Args:
            detections: List of detections
            parameters: Fusion parameters
            
        Returns:
            Fusion result data
        """
        # This is a simplified implementation
        # In a real system, this would use a proper Kalman filter
        
        # Sort detections by timestamp
        detections = sorted(detections, key=lambda d: d.get("timestamp", datetime.min))
        
        # Initialize state with first detection
        first_detection = detections[0]["data"]
        if "location" not in first_detection:
            return {
                "error": "First detection has no location",
                "confidence": 0.0
            }
        
        location = first_detection["location"]
        if "latitude" not in location or "longitude" not in location:
            return {
                "error": "First detection has invalid location",
                "confidence": 0.0
            }
        
        # Initialize state
        state = np.array([
            location["latitude"],
            location["longitude"],
            first_detection.get("velocity", {}).get("x", 0.0),
            first_detection.get("velocity", {}).get("y", 0.0)
        ])
        
        # Initialize covariance
        covariance = np.eye(4) * 0.01
        
        # Process each detection
        for detection in detections[1:]:
            data = detection["data"]
            if "location" not in data:
                continue
            
            location = data["location"]
            if "latitude" not in location or "longitude" not in location:
                continue
            
            # Get measurement
            measurement = np.array([
                location["latitude"],
                location["longitude"]
            ])
            
            # Get measurement noise
            sensor_accuracy = detection.get("sensor_accuracy", 0.5)
            detection_confidence = data.get("confidence", 0.5)
            measurement_noise = 0.01 / (sensor_accuracy * detection_confidence + 0.01)
            
            # Kalman filter update
            # This is a simplified update step
            # In a real system, this would use a proper Kalman filter
            
            # Measurement matrix
            H = np.array([
                [1, 0, 0, 0],
                [0, 1, 0, 0]
            ])
            
            # Measurement noise
            R = np.eye(2) * measurement_noise
            
            # Kalman gain
            K = covariance @ H.T @ np.linalg.inv(H @ covariance @ H.T + R)
            
            # Update state
            state = state + K @ (measurement - H @ state)
            
            # Update covariance
            covariance = (np.eye(4) - K @ H) @ covariance
        
        # Calculate confidence
        confidence = min(1.0, 1.0 / (np.trace(covariance[:2, :2]) + 0.01))
        
        # Create result
        result = {
            "location": {
                "latitude": float(state[0]),
                "longitude": float(state[1])
            },
            "velocity": {
                "x": float(state[2]),
                "y": float(state[3])
            },
            "confidence": float(confidence),
            "fusion_method": "kalman",
            "num_detections": len(detections),
            "timestamp": datetime.utcnow().isoformat()
        }
        
        return result
    
    def _fuse_detections_particle(
        self,
        detections: List[Dict[str, Any]],
        parameters: Dict[str, Any]
    ) -> Dict[str, Any]:
        """
        Fuse detections using particle filter.
        
        Args:
            detections: List of detections
            parameters: Fusion parameters
            
        Returns:
            Fusion result data
        """
        # This is a placeholder for a particle filter implementation
        # In a real system, this would use a proper particle filter
        
        # For now, fall back to weighted average
        return self._fuse_detections_weighted_average(detections, parameters)
    
    async def _fuse_images(
        self,
        sensor_data_list: List[SensorData],
        parameters: Dict[str, Any]
    ) -> Dict[str, Any]:
        """
        Fuse images from multiple sensors.
        
        Args:
            sensor_data_list: List of sensor data
            parameters: Fusion parameters
            
        Returns:
            Fusion result data
        """
        # This is a placeholder for image fusion
        # In a real system, this would implement various image fusion techniques
        
        return {
            "message": "Image fusion not implemented yet",
            "confidence": 0.0
        }
    
    async def _fuse_tracks(
        self,
        sensor_data_list: List[SensorData],
        parameters: Dict[str, Any]
    ) -> Dict[str, Any]:
        """
        Fuse tracks from multiple sensors.
        
        Args:
            sensor_data_list: List of sensor data
            parameters: Fusion parameters
            
        Returns:
            Fusion result data
        """
        # This is a placeholder for track fusion
        # In a real system, this would implement various track fusion techniques
        
        return {
            "message": "Track fusion not implemented yet",
            "confidence": 0.0
        }
    
    def _convert_to_fusion_job_schema(self, fusion_job: FusionJob) -> FusionJobSchema:
        """
        Convert a fusion job database model to a schema.
        
        Args:
            fusion_job: Fusion job database model
            
        Returns:
            Fusion job schema
        """
        from api.schemas import FusionJobStatus
        
        # Convert to schema
        return FusionJobSchema(
            id=fusion_job.id,
            sensor_data_ids=[sensor_data.id for sensor_data in fusion_job.sensor_data],
            target_id=fusion_job.target_id,
            status=FusionJobStatus(fusion_job.status),
            parameters=fusion_job.parameters,
            result_id=fusion_job.result_id,
            error_message=fusion_job.error_message,
            priority=fusion_job.priority,
            metadata=fusion_job.metadata,
            created_at=fusion_job.created_at,
            updated_at=fusion_job.updated_at,
            completed_at=fusion_job.completed_at
        )
