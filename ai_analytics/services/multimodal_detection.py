"""
Multimodal detection service for the AI Analytics module.

This service handles multimodal object detection using visual, thermal,
and other sensor data.
"""

import logging
import time
import uuid
import asyncio
from typing import List, Dict, Any, Optional, Tuple
import numpy as np
import cv2
from datetime import datetime

# Import local modules
from utils.config import Config
from services.video_stream_manager import VideoStreamManager
from services.event_publisher import EventPublisher
from models.multimodal.detector import MultimodalDetector
from api.schemas.multimodal import (
    MultimodalDetectionConfig,
    MultimodalDetectionResult,
    MultimodalDetectionZone,
    MultimodalDetectedObject,
    ObjectClass,
    BoundingBox,
    ModalityInfo
)

# Setup logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class MultimodalDetectionService:
    """
    Service for multimodal object detection.
    """
    
    def __init__(
        self,
        config: Config,
        video_stream_manager: VideoStreamManager,
        event_publisher: EventPublisher
    ):
        """
        Initialize the multimodal detection service.
        
        Args:
            config: Configuration object
            video_stream_manager: Video stream manager
            event_publisher: Event publisher
        """
        self.config = config
        self.video_stream_manager = video_stream_manager
        self.event_publisher = event_publisher
        
        # Initialize detector
        self.detector = None
        self._initialize_detector()
        
        # Detection zones
        self.detection_zones = {}
        
        # Active detection tasks
        self.active_tasks = {}
        
        logger.info("Multimodal detection service initialized")
    
    def _initialize_detector(self):
        """Initialize the multimodal detector."""
        try:
            logger.info("Initializing multimodal detector")
            
            # Get detector configuration
            detector_config = self.config.get("multimodal_detection", {})
            
            # Create detector
            self.detector = MultimodalDetector(detector_config)
            
            logger.info("Multimodal detector initialized")
        
        except Exception as e:
            logger.error(f"Error initializing multimodal detector: {str(e)}")
            raise
    
    async def start_detection(
        self,
        stream_ids: Dict[str, str],
        config: MultimodalDetectionConfig
    ) -> str:
        """
        Start multimodal detection on specified streams.
        
        Args:
            stream_ids: Dictionary mapping modality names to stream IDs
            config: Detection configuration
            
        Returns:
            Task ID for the detection task
        """
        try:
            # Generate task ID
            task_id = str(uuid.uuid4())
            
            # Create detection zones if provided
            if config.detection_zones:
                for zone in config.detection_zones:
                    self.detection_zones[zone.id] = zone
            
            # Start detection task
            task = asyncio.create_task(
                self._detection_task(task_id, stream_ids, config)
            )
            
            # Store task
            self.active_tasks[task_id] = {
                "task": task,
                "stream_ids": stream_ids,
                "config": config,
                "start_time": datetime.now()
            }
            
            logger.info(f"Started multimodal detection task {task_id}")
            
            return task_id
        
        except Exception as e:
            logger.error(f"Error starting multimodal detection: {str(e)}")
            raise
    
    async def stop_detection(self, task_id: str) -> bool:
        """
        Stop a multimodal detection task.
        
        Args:
            task_id: ID of the task to stop
            
        Returns:
            True if the task was stopped, False otherwise
        """
        try:
            if task_id in self.active_tasks:
                # Cancel task
                self.active_tasks[task_id]["task"].cancel()
                
                # Remove task
                del self.active_tasks[task_id]
                
                logger.info(f"Stopped multimodal detection task {task_id}")
                
                return True
            else:
                logger.warning(f"Detection task {task_id} not found")
                return False
        
        except Exception as e:
            logger.error(f"Error stopping multimodal detection: {str(e)}")
            raise
    
    async def get_detection_status(self, task_id: str) -> Dict[str, Any]:
        """
        Get the status of a multimodal detection task.
        
        Args:
            task_id: ID of the task
            
        Returns:
            Dictionary containing task status
        """
        try:
            if task_id in self.active_tasks:
                task_info = self.active_tasks[task_id]
                
                return {
                    "task_id": task_id,
                    "status": "running",
                    "stream_ids": task_info["stream_ids"],
                    "config": task_info["config"],
                    "start_time": task_info["start_time"],
                    "elapsed_time": (datetime.now() - task_info["start_time"]).total_seconds()
                }
            else:
                return {
                    "task_id": task_id,
                    "status": "not_found"
                }
        
        except Exception as e:
            logger.error(f"Error getting detection status: {str(e)}")
            raise
    
    async def get_active_tasks(self) -> List[Dict[str, Any]]:
        """
        Get all active multimodal detection tasks.
        
        Returns:
            List of dictionaries containing task information
        """
        try:
            tasks = []
            
            for task_id, task_info in self.active_tasks.items():
                tasks.append({
                    "task_id": task_id,
                    "status": "running",
                    "stream_ids": task_info["stream_ids"],
                    "config": task_info["config"],
                    "start_time": task_info["start_time"],
                    "elapsed_time": (datetime.now() - task_info["start_time"]).total_seconds()
                })
            
            return tasks
        
        except Exception as e:
            logger.error(f"Error getting active tasks: {str(e)}")
            raise
    
    async def _detection_task(
        self,
        task_id: str,
        stream_ids: Dict[str, str],
        config: MultimodalDetectionConfig
    ):
        """
        Background task for multimodal detection.
        
        Args:
            task_id: Task ID
            stream_ids: Dictionary mapping modality names to stream IDs
            config: Detection configuration
        """
        try:
            logger.info(f"Starting detection task {task_id}")
            
            # Get detection parameters
            interval = config.interval or 1.0  # seconds
            max_duration = config.max_duration or 0  # seconds (0 = no limit)
            
            # Start time
            start_time = time.time()
            
            # Main detection loop
            while True:
                # Check if max duration is reached
                if max_duration > 0 and (time.time() - start_time) > max_duration:
                    logger.info(f"Detection task {task_id} reached max duration")
                    break
                
                # Get frames from streams
                frames = {}
                for modality, stream_id in stream_ids.items():
                    frame = await self.video_stream_manager.get_frame(stream_id)
                    if frame is not None:
                        frames[modality] = frame
                
                # Skip if no frames are available
                if not frames:
                    logger.warning(f"No frames available for detection task {task_id}")
                    await asyncio.sleep(interval)
                    continue
                
                # Perform detection
                detection_result = await self.detector.detect(
                    frames=frames,
                    region_of_interest=config.region_of_interest.dict() if config.region_of_interest else None,
                    parameters=config.parameters.dict() if config.parameters else None
                )
                
                # Process detection result
                result = self._process_detection_result(detection_result, frames, config)
                
                # Publish result
                await self.event_publisher.publish_event(
                    event_type="multimodal_detection",
                    payload={
                        "task_id": task_id,
                        "result": result
                    }
                )
                
                # Wait for next interval
                await asyncio.sleep(interval)
        
        except asyncio.CancelledError:
            logger.info(f"Detection task {task_id} cancelled")
        
        except Exception as e:
            logger.error(f"Error in detection task {task_id}: {str(e)}")
            
            # Publish error event
            await self.event_publisher.publish_event(
                event_type="multimodal_detection_error",
                payload={
                    "task_id": task_id,
                    "error": str(e)
                }
            )
        
        finally:
            # Remove task from active tasks
            if task_id in self.active_tasks:
                del self.active_tasks[task_id]
            
            logger.info(f"Detection task {task_id} finished")
    
    def _process_detection_result(
        self,
        detection_result: Dict[str, Any],
        frames: Dict[str, np.ndarray],
        config: MultimodalDetectionConfig
    ) -> MultimodalDetectionResult:
        """
        Process detection result.
        
        Args:
            detection_result: Detection result from detector
            frames: Dictionary mapping modality names to frames
            config: Detection configuration
            
        Returns:
            Processed detection result
        """
        # Create result object
        result = MultimodalDetectionResult(
            timestamp=detection_result["timestamp"],
            processing_time=detection_result["processing_time"],
            objects=[],
            modality_info={}
        )
        
        # Add modality information
        for modality, frame in frames.items():
            result.modality_info[modality] = ModalityInfo(
                width=frame.shape[1],
                height=frame.shape[0],
                channels=frame.shape[2] if len(frame.shape) > 2 else 1
            )
        
        # Process detections
        for detection in detection_result["detections"]:
            # Create bounding box
            bbox = BoundingBox(
                x=detection["bbox"]["x1"],
                y=detection["bbox"]["y1"],
                width=detection["bbox"]["width"],
                height=detection["bbox"]["height"]
            )
            
            # Create object class
            object_class = ObjectClass(
                id=detection["class_id"],
                name=detection["class_name"],
                confidence=detection["confidence"]
            )
            
            # Create detected object
            detected_object = MultimodalDetectedObject(
                id=detection["id"],
                bbox=bbox,
                class_info=object_class,
                modalities=detection.get("modalities", [detection.get("modality", "unknown")]),
                attributes={}
            )
            
            # Check if object is in any detection zone
            if config.detection_zones:
                for zone_id, zone in self.detection_zones.items():
                    if self._is_in_zone(bbox, zone):
                        detected_object.zones.append(zone_id)
            
            # Add object to result
            result.objects.append(detected_object)
        
        return result
    
    def _is_in_zone(
        self,
        bbox: BoundingBox,
        zone: MultimodalDetectionZone
    ) -> bool:
        """
        Check if a bounding box is in a detection zone.
        
        Args:
            bbox: Bounding box
            zone: Detection zone
            
        Returns:
            True if the bounding box is in the zone, False otherwise
        """
        # Get zone type
        zone_type = zone.type
        
        if zone_type == "rectangle":
            # Check if bounding box center is in rectangle
            center_x = bbox.x + bbox.width / 2
            center_y = bbox.y + bbox.height / 2
            
            return (
                center_x >= zone.x and
                center_y >= zone.y and
                center_x <= zone.x + zone.width and
                center_y <= zone.y + zone.height
            )
        
        elif zone_type == "polygon":
            # Check if bounding box center is in polygon
            center_x = bbox.x + bbox.width / 2
            center_y = bbox.y + bbox.height / 2
            
            # Convert polygon points to numpy array
            points = np.array(zone.points, dtype=np.int32)
            
            # Check if point is in polygon
            return cv2.pointPolygonTest(points, (center_x, center_y), False) >= 0
        
        else:
            logger.warning(f"Unknown zone type: {zone_type}")
            return False
