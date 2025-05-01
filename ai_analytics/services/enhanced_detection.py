"""
Enhanced Detection service for the AI Analytics module.

This service handles object detection using the unified inference engine,
supporting multiple backends (TinyGrad, PyTorch, TFLite).
"""

import logging
import time
import uuid
import os
from typing import List, Dict, Any, Optional, Tuple
import numpy as np
import cv2
from datetime import datetime
import asyncio

# Import local modules
from utils.config import Config
from services.video_stream_manager import VideoStreamManager
from services.event_publisher import EventPublisher
from api.schemas.detection import (
    DetectionConfig,
    DetectionResult,
    DetectionZone,
    DetectedObject,
    ObjectClass,
    BoundingBox
)

# Import inference engine
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))
from ai.inference import InferenceEngine

# Setup logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class EnhancedDetectionService:
    """Enhanced service for object detection with multiple backends."""
    
    def __init__(
        self,
        config: Dict[str, Any],
        video_manager: VideoStreamManager,
        event_publisher: EventPublisher
    ):
        """Initialize the detection service."""
        self.config = config
        self.video_manager = video_manager
        self.event_publisher = event_publisher
        self.models = {}
        self.engines = {}
        self.trackers = {}
        self.zones = {}
        self.results_cache = {}
        self.active_tasks = {}
        
        # Get ML backend from environment or config
        self.ml_backend = os.environ.get("ML_BACKEND", config.get("ml_backend", "torch"))
        self.device = os.environ.get("DEVICE", config.get("device", "AUTO"))
        
        # Initialize models
        self._initialize_models()
    
    def _initialize_models(self):
        """Initialize detection models."""
        try:
            logger.info(f"Initializing detection models with {self.ml_backend} backend on {self.device}")
            
            # Get model configurations from config
            model_configs = self.config.get("models", [])
            
            for model_config in model_configs:
                model_name = model_config.get("name")
                model_path = model_config.get("path")
                
                if model_name and model_path and os.path.exists(model_path):
                    # Initialize inference engine for the model
                    engine = InferenceEngine(
                        backend=self.ml_backend,
                        model_path=model_path,
                        device=self.device
                    )
                    
                    # Store engine
                    self.engines[model_name] = engine
                    
                    # Store model metadata
                    self.models[model_name] = {
                        "name": model_name,
                        "path": model_path,
                        "description": model_config.get("description", ""),
                        "class_names": model_config.get("class_names", []),
                        "initialized": True
                    }
                    
                    logger.info(f"Initialized model {model_name} with {self.ml_backend} backend")
                else:
                    logger.warning(f"Skipping model {model_name}: path {model_path} not found")
            
            logger.info(f"Initialized {len(self.models)} detection models")
        except Exception as e:
            logger.error(f"Error initializing detection models: {str(e)}")
            raise
    
    async def get_config(self, camera_id: Optional[str] = None) -> DetectionConfig:
        """Get the current detection configuration."""
        # This is a placeholder for actual configuration retrieval
        # In a real implementation, you would retrieve the configuration from a database
        
        # Example configuration
        config = DetectionConfig(
            model_name=self.config.get("default_model", "yolov8n"),
            confidence_threshold=self.config.get("confidence_threshold", 0.5),
            enabled_classes=self.config.get("enabled_classes", ["person", "car", "truck", "bicycle", "motorcycle"]),
            tracking_enabled=self.config.get("tracking_enabled", True),
            tracking_max_age=self.config.get("tracking_max_age", 30),
            tracking_min_hits=self.config.get("tracking_min_hits", 3),
            tracking_iou_threshold=self.config.get("tracking_iou_threshold", 0.3)
        )
        
        return config
    
    async def update_config(self, config: DetectionConfig, camera_id: Optional[str] = None) -> DetectionConfig:
        """Update the detection configuration."""
        # This is a placeholder for actual configuration update
        # In a real implementation, you would update the configuration in a database
        
        # Update local config
        self.config["default_model"] = config.model_name
        self.config["confidence_threshold"] = config.confidence_threshold
        self.config["enabled_classes"] = config.enabled_classes
        self.config["tracking_enabled"] = config.tracking_enabled
        self.config["tracking_max_age"] = config.tracking_max_age
        self.config["tracking_min_hits"] = config.tracking_min_hits
        self.config["tracking_iou_threshold"] = config.tracking_iou_threshold
        
        return config
    
    async def get_models(self) -> List[Dict[str, Any]]:
        """Get available detection models."""
        return [
            {
                "name": name,
                "description": model.get("description", ""),
                "class_names": model.get("class_names", [])
            }
            for name, model in self.models.items()
        ]
    
    async def get_zones(self, camera_id: Optional[str] = None) -> List[DetectionZone]:
        """Get detection zones for a camera."""
        # This is a placeholder for actual zone retrieval
        # In a real implementation, you would retrieve zones from a database
        
        # Example zones
        zones = [
            DetectionZone(
                id="zone1",
                name="Entrance",
                points=[
                    {"x": 0.1, "y": 0.1},
                    {"x": 0.3, "y": 0.1},
                    {"x": 0.3, "y": 0.3},
                    {"x": 0.1, "y": 0.3}
                ],
                enabled_classes=["person"],
                min_confidence=0.6,
                min_dwell_time=5.0
            ),
            DetectionZone(
                id="zone2",
                name="Parking",
                points=[
                    {"x": 0.5, "y": 0.5},
                    {"x": 0.9, "y": 0.5},
                    {"x": 0.9, "y": 0.9},
                    {"x": 0.5, "y": 0.9}
                ],
                enabled_classes=["car", "truck"],
                min_confidence=0.5,
                min_dwell_time=3.0
            )
        ]
        
        return zones
    
    async def create_zone(self, zone: DetectionZone, camera_id: Optional[str] = None) -> DetectionZone:
        """Create a new detection zone."""
        # This is a placeholder for actual zone creation
        # In a real implementation, you would create the zone in a database
        
        # Store zone in memory
        self.zones[zone.id] = zone
        
        return zone
    
    async def update_zone(self, zone_id: str, zone: DetectionZone, camera_id: Optional[str] = None) -> Optional[DetectionZone]:
        """Update a detection zone."""
        # This is a placeholder for actual zone update
        # In a real implementation, you would update the zone in a database
        
        # Check if zone exists
        if zone_id not in self.zones:
            return None
        
        # Update zone
        self.zones[zone_id] = zone
        
        return zone
    
    async def delete_zone(self, zone_id: str, camera_id: Optional[str] = None) -> bool:
        """Delete a detection zone."""
        # This is a placeholder for actual zone deletion
        # In a real implementation, you would delete the zone from a database
        
        # Check if zone exists
        if zone_id not in self.zones:
            return False
        
        # Delete zone
        del self.zones[zone_id]
        
        return True
    
    async def run_test_detection(self, camera_id: str) -> Optional[DetectionResult]:
        """Run a test detection on a single frame from the camera."""
        try:
            # Get frame from camera
            frame = await self.video_manager.get_frame(camera_id)
            
            if frame is None:
                logger.error(f"Failed to get frame from camera {camera_id}")
                return None
            
            # Get detection configuration
            config = await self.get_config(camera_id)
            
            # Run detection
            result = await self._detect_objects(frame, config)
            
            # Publish result
            await self.event_publisher.publish_event(
                event_type="detection",
                payload={
                    "camera_id": camera_id,
                    "result": result.dict()
                }
            )
            
            return result
        except Exception as e:
            logger.error(f"Error running test detection: {str(e)}")
            return None
    
    async def start_detection(self, camera_id: str, interval: float = 1.0) -> str:
        """
        Start continuous detection on a camera.
        
        Args:
            camera_id: Camera ID
            interval: Detection interval in seconds
            
        Returns:
            Task ID
        """
        try:
            # Generate task ID
            task_id = str(uuid.uuid4())
            
            # Get detection configuration
            config = await self.get_config(camera_id)
            
            # Start detection task
            task = asyncio.create_task(
                self._detection_task(task_id, camera_id, interval, config)
            )
            
            # Store task
            self.active_tasks[task_id] = {
                "task": task,
                "camera_id": camera_id,
                "interval": interval,
                "config": config,
                "start_time": datetime.now()
            }
            
            logger.info(f"Started detection task {task_id} for camera {camera_id}")
            
            return task_id
        except Exception as e:
            logger.error(f"Error starting detection: {str(e)}")
            raise
    
    async def stop_detection(self, task_id: str) -> bool:
        """
        Stop a detection task.
        
        Args:
            task_id: Task ID
            
        Returns:
            True if task was stopped, False otherwise
        """
        try:
            # Check if task exists
            if task_id not in self.active_tasks:
                logger.warning(f"Task {task_id} not found")
                return False
            
            # Get task
            task_info = self.active_tasks[task_id]
            task = task_info["task"]
            
            # Cancel task
            task.cancel()
            
            # Remove task
            del self.active_tasks[task_id]
            
            logger.info(f"Stopped detection task {task_id}")
            
            return True
        except Exception as e:
            logger.error(f"Error stopping detection: {str(e)}")
            return False
    
    async def get_active_tasks(self) -> List[Dict[str, Any]]:
        """
        Get active detection tasks.
        
        Returns:
            List of active tasks
        """
        return [
            {
                "task_id": task_id,
                "camera_id": task_info["camera_id"],
                "interval": task_info["interval"],
                "start_time": task_info["start_time"].isoformat(),
                "config": task_info["config"].dict() if hasattr(task_info["config"], "dict") else task_info["config"]
            }
            for task_id, task_info in self.active_tasks.items()
        ]
    
    async def _detection_task(self, task_id: str, camera_id: str, interval: float, config: DetectionConfig):
        """
        Detection task.
        
        Args:
            task_id: Task ID
            camera_id: Camera ID
            interval: Detection interval in seconds
            config: Detection configuration
        """
        try:
            logger.info(f"Starting detection task {task_id} for camera {camera_id}")
            
            while True:
                # Get frame from camera
                frame = await self.video_manager.get_frame(camera_id)
                
                if frame is None:
                    logger.warning(f"Failed to get frame from camera {camera_id}")
                    await asyncio.sleep(interval)
                    continue
                
                # Run detection
                result = await self._detect_objects(frame, config)
                
                # Publish result
                await self.event_publisher.publish_event(
                    event_type="detection",
                    payload={
                        "task_id": task_id,
                        "camera_id": camera_id,
                        "result": result.dict()
                    }
                )
                
                # Wait for next interval
                await asyncio.sleep(interval)
        except asyncio.CancelledError:
            logger.info(f"Detection task {task_id} cancelled")
        except Exception as e:
            logger.error(f"Error in detection task {task_id}: {str(e)}")
    
    async def _detect_objects(self, frame: np.ndarray, config: DetectionConfig) -> DetectionResult:
        """
        Detect objects in a frame.
        
        Args:
            frame: Frame to detect objects in
            config: Detection configuration
            
        Returns:
            Detection result
        """
        try:
            # Get model name from config
            model_name = config.model_name
            
            # Check if model exists
            if model_name not in self.engines:
                logger.error(f"Model {model_name} not found")
                return DetectionResult(
                    timestamp=datetime.now().isoformat(),
                    objects=[],
                    processing_time=0.0
                )
            
            # Get engine
            engine = self.engines[model_name]
            
            # Preprocess frame
            preprocessed_frame = self._preprocess_frame(frame)
            
            # Run inference
            start_time = time.time()
            outputs = engine.predict({"input": preprocessed_frame})
            processing_time = (time.time() - start_time) * 1000  # Convert to ms
            
            # Postprocess outputs
            detected_objects = self._postprocess_outputs(outputs, frame.shape, config)
            
            # Create result
            result = DetectionResult(
                timestamp=datetime.now().isoformat(),
                objects=detected_objects,
                processing_time=processing_time
            )
            
            return result
        except Exception as e:
            logger.error(f"Error detecting objects: {str(e)}")
            return DetectionResult(
                timestamp=datetime.now().isoformat(),
                objects=[],
                processing_time=0.0
            )
    
    def _preprocess_frame(self, frame: np.ndarray) -> np.ndarray:
        """
        Preprocess a frame for inference.
        
        Args:
            frame: Frame to preprocess
            
        Returns:
            Preprocessed frame
        """
        # Resize frame to model input size
        resized_frame = cv2.resize(frame, (640, 640))
        
        # Convert to RGB
        rgb_frame = cv2.cvtColor(resized_frame, cv2.COLOR_BGR2RGB)
        
        # Normalize to [0, 1]
        normalized_frame = rgb_frame.astype(np.float32) / 255.0
        
        # Add batch dimension
        batched_frame = np.expand_dims(normalized_frame, axis=0)
        
        # Transpose to [batch, channels, height, width] for PyTorch models
        transposed_frame = np.transpose(batched_frame, (0, 3, 1, 2))
        
        return transposed_frame
    
    def _postprocess_outputs(self, outputs: Dict[str, np.ndarray], frame_shape: Tuple[int, int, int], config: DetectionConfig) -> List[DetectedObject]:
        """
        Postprocess model outputs.
        
        Args:
            outputs: Model outputs
            frame_shape: Original frame shape (height, width, channels)
            config: Detection configuration
            
        Returns:
            List of detected objects
        """
        # This is a simplified implementation for demonstration purposes
        # In a real implementation, you would need to handle different model output formats
        
        # Get output tensor
        if "output" in outputs:
            output = outputs["output"]
        elif "output_0" in outputs:
            output = outputs["output_0"]
        else:
            logger.warning(f"Unknown output format: {list(outputs.keys())}")
            return []
        
        # Get frame dimensions
        height, width = frame_shape[:2]
        
        # Parse detections
        detected_objects = []
        
        # Simplified parsing for demonstration
        # In a real implementation, you would need to handle different model output formats
        if len(output.shape) == 3 and output.shape[1] > 5:  # [batch, num_detections, 5+num_classes]
            # YOLO-style output
            for detection in output[0]:
                confidence = detection[4]
                
                # Skip low-confidence detections
                if confidence < config.confidence_threshold:
                    continue
                
                # Get class ID and score
                class_scores = detection[5:]
                class_id = np.argmax(class_scores)
                class_score = class_scores[class_id]
                
                # Skip if class is not enabled
                class_name = self._get_class_name(class_id)
                if class_name not in config.enabled_classes:
                    continue
                
                # Get bounding box
                x_center, y_center, box_width, box_height = detection[:4]
                
                # Convert to pixel coordinates
                x1 = int((x_center - box_width / 2) * width)
                y1 = int((y_center - box_height / 2) * height)
                x2 = int((x_center + box_width / 2) * width)
                y2 = int((y_center + box_height / 2) * height)
                
                # Create detected object
                detected_object = DetectedObject(
                    id=str(uuid.uuid4()),
                    class_name=class_name,
                    confidence=float(class_score),
                    bounding_box=BoundingBox(
                        x1=max(0, x1),
                        y1=max(0, y1),
                        x2=min(width, x2),
                        y2=min(height, y2)
                    )
                )
                
                detected_objects.append(detected_object)
        
        return detected_objects
    
    def _get_class_name(self, class_id: int) -> str:
        """
        Get class name from class ID.
        
        Args:
            class_id: Class ID
            
        Returns:
            Class name
        """
        # This is a simplified implementation for demonstration purposes
        # In a real implementation, you would get class names from the model metadata
        
        # COCO class names (simplified)
        coco_names = [
            "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat",
            "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat",
            "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack",
            "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball",
            "kite", "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle"
        ]
        
        if 0 <= class_id < len(coco_names):
            return coco_names[class_id]
        else:
            return f"class_{class_id}"
