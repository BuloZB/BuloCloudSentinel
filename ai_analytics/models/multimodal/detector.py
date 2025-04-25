"""
Multimodal detector for Bulo.Cloud Sentinel.

This module provides a detector that combines multiple modalities
(visual, thermal, etc.) for improved object detection.
"""

import logging
import torch
import torch.nn as nn
import torch.nn.functional as F
import numpy as np
import cv2
from typing import Dict, List, Tuple, Any, Optional, Union
from pathlib import Path
import time
import uuid
from datetime import datetime

from ultralytics import YOLO
from .fusion import FeatureFusion, DecisionFusion, HybridFusion

# Setup logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class MultimodalDetector:
    """
    Multimodal detector for object detection.
    
    This detector combines multiple modalities (visual, thermal, etc.)
    for improved object detection.
    """
    
    def __init__(
        self,
        config: Dict[str, Any],
        device: Optional[torch.device] = None
    ):
        """
        Initialize the multimodal detector.
        
        Args:
            config: Configuration dictionary
            device: Device to run the detector on (CPU or CUDA)
        """
        self.config = config
        self.device = device or torch.device("cuda" if torch.cuda.is_available() and 
                                            config.get("enable_gpu", True) else "cpu")
        
        # Initialize models
        self.models = {}
        self.fusion_model = None
        self._initialize_models()
        
        # Class names
        self.class_names = config.get("class_names", [])
        
        # Confidence threshold
        self.confidence_threshold = config.get("confidence_threshold", 0.5)
        
        # NMS threshold
        self.nms_threshold = config.get("nms_threshold", 0.45)
        
        # Modality weights
        self.modality_weights = config.get("modality_weights", {
            "visual": 1.0,
            "thermal": 0.8,
            "depth": 0.6
        })
        
        logger.info(f"Multimodal detector initialized on {self.device}")
    
    def _initialize_models(self):
        """Initialize detection models for each modality."""
        try:
            logger.info("Initializing multimodal detection models")
            
            # Load models for each modality
            modalities = self.config.get("modalities", ["visual"])
            
            for modality in modalities:
                model_config = self.config.get(f"{modality}_model", {})
                model_path = model_config.get("path")
                
                if model_path and Path(model_path).exists():
                    # Load YOLO model
                    self.models[modality] = YOLO(model_path)
                    logger.info(f"Loaded {modality} model from {model_path}")
                else:
                    # Use default YOLO model
                    model_size = model_config.get("size", "n")  # n, s, m, l, x
                    self.models[modality] = YOLO(f"yolov8{model_size}.pt")
                    logger.info(f"Loaded default YOLOv8{model_size} model for {modality}")
            
            # Initialize fusion model if multiple modalities are available
            if len(self.models) > 1:
                fusion_config = self.config.get("fusion", {})
                fusion_type = fusion_config.get("type", "decision")
                
                if fusion_type == "feature":
                    # Feature-level fusion
                    input_dims = {m: 256 for m in self.models.keys()}  # Assuming 256-dim features
                    self.fusion_model = FeatureFusion(
                        input_dims=input_dims,
                        fusion_dim=fusion_config.get("fusion_dim", 256),
                        dropout=fusion_config.get("dropout", 0.3)
                    ).to(self.device)
                
                elif fusion_type == "decision":
                    # Decision-level fusion
                    num_classes = len(self.class_names) if self.class_names else 80  # COCO classes
                    self.fusion_model = DecisionFusion(
                        num_classes=num_classes,
                        fusion_method=fusion_config.get("method", "weighted_average"),
                        weights=self.modality_weights
                    ).to(self.device)
                
                elif fusion_type == "hybrid":
                    # Hybrid fusion
                    input_dims = {m: 256 for m in self.models.keys()}  # Assuming 256-dim features
                    num_classes = len(self.class_names) if self.class_names else 80  # COCO classes
                    self.fusion_model = HybridFusion(
                        input_dims=input_dims,
                        num_classes=num_classes,
                        fusion_dim=fusion_config.get("fusion_dim", 256),
                        dropout=fusion_config.get("dropout", 0.3),
                        decision_fusion_method=fusion_config.get("method", "weighted_average")
                    ).to(self.device)
                
                logger.info(f"Initialized {fusion_type} fusion model")
            
            logger.info(f"Multimodal detection models initialized on {self.device}")
        
        except Exception as e:
            logger.error(f"Error initializing multimodal detection models: {str(e)}")
            raise
    
    async def detect(
        self,
        frames: Dict[str, np.ndarray],
        region_of_interest: Optional[Dict[str, Any]] = None,
        parameters: Optional[Dict[str, Any]] = None
    ) -> Dict[str, Any]:
        """
        Detect objects in multimodal frames.
        
        Args:
            frames: Dictionary mapping modality names to frames
            region_of_interest: Optional region of interest to analyze
            parameters: Optional parameters for detection
            
        Returns:
            Dictionary containing detection results
        """
        try:
            start_time = time.time()
            
            # Get parameters
            params = parameters or {}
            confidence_threshold = params.get("confidence_threshold", self.confidence_threshold)
            nms_threshold = params.get("nms_threshold", self.nms_threshold)
            
            # Apply region of interest if provided
            if region_of_interest:
                for modality, frame in frames.items():
                    frames[modality] = self._apply_roi(frame, region_of_interest)
            
            # Initialize results
            results = {
                "timestamp": datetime.now(),
                "detections": [],
                "modality_detections": {},
                "processing_time": 0.0
            }
            
            # Detect objects in each modality
            modality_results = {}
            for modality, frame in frames.items():
                if modality in self.models:
                    # Run detection
                    model_results = self.models[modality](
                        frame,
                        conf=confidence_threshold,
                        iou=nms_threshold,
                        verbose=False
                    )
                    
                    # Process results
                    detections = self._process_detections(model_results, modality)
                    modality_results[modality] = detections
                    results["modality_detections"][modality] = detections
            
            # Fuse detections if multiple modalities are available
            if len(modality_results) > 1 and self.fusion_model:
                fused_detections = self._fuse_detections(modality_results)
                results["detections"] = fused_detections
            elif len(modality_results) == 1:
                # If only one modality is available, use its detections
                results["detections"] = next(iter(modality_results.values()))
            
            # Calculate processing time
            processing_time = (time.time() - start_time) * 1000  # Convert to milliseconds
            results["processing_time"] = processing_time
            
            return results
        
        except Exception as e:
            logger.error(f"Error detecting objects in multimodal frames: {str(e)}")
            raise
    
    def _process_detections(
        self,
        model_results: List[Any],
        modality: str
    ) -> List[Dict[str, Any]]:
        """
        Process detection results from a model.
        
        Args:
            model_results: Results from the model
            modality: Modality name
            
        Returns:
            List of detection dictionaries
        """
        detections = []
        
        for result in model_results:
            boxes = result.boxes.cpu().numpy()
            
            for i in range(len(boxes)):
                # Get box coordinates
                box = boxes.xyxy[i] if hasattr(boxes, 'xyxy') else boxes.xyxy[i]
                x1, y1, x2, y2 = box
                
                # Get confidence and class
                confidence = float(boxes.conf[i])
                class_id = int(boxes.cls[i])
                class_name = result.names[class_id]
                
                # Create detection object
                detection = {
                    "id": str(uuid.uuid4()),
                    "modality": modality,
                    "bbox": {
                        "x1": float(x1),
                        "y1": float(y1),
                        "x2": float(x2),
                        "y2": float(y2),
                        "width": float(x2 - x1),
                        "height": float(y2 - y1)
                    },
                    "confidence": confidence,
                    "class_id": class_id,
                    "class_name": class_name
                }
                
                detections.append(detection)
        
        return detections
    
    def _fuse_detections(
        self,
        modality_results: Dict[str, List[Dict[str, Any]]]
    ) -> List[Dict[str, Any]]:
        """
        Fuse detections from multiple modalities.
        
        Args:
            modality_results: Dictionary mapping modality names to detection lists
            
        Returns:
            List of fused detection dictionaries
        """
        # This is a simplified fusion approach
        # In a real implementation, you would use the fusion model
        
        # Collect all detections
        all_detections = []
        for modality, detections in modality_results.items():
            for detection in detections:
                # Add modality weight to confidence
                weight = self.modality_weights.get(modality, 1.0)
                detection["weighted_confidence"] = detection["confidence"] * weight
                all_detections.append(detection)
        
        # Group detections by overlapping bounding boxes
        fused_detections = []
        used_indices = set()
        
        for i, detection in enumerate(all_detections):
            if i in used_indices:
                continue
            
            # Find overlapping detections
            overlaps = []
            for j, other in enumerate(all_detections):
                if j == i or j in used_indices:
                    continue
                
                # Check if bounding boxes overlap
                if self._boxes_overlap(detection["bbox"], other["bbox"]):
                    overlaps.append(j)
            
            # If there are overlapping detections, fuse them
            if overlaps:
                # Add the current detection
                to_fuse = [detection]
                
                # Add overlapping detections
                for j in overlaps:
                    to_fuse.append(all_detections[j])
                    used_indices.add(j)
                
                # Fuse detections
                fused = self._fuse_overlapping_detections(to_fuse)
                fused_detections.append(fused)
            else:
                # No overlaps, add the detection as is
                fused_detections.append(detection)
            
            used_indices.add(i)
        
        # Sort by confidence
        fused_detections.sort(key=lambda x: x.get("confidence", 0), reverse=True)
        
        return fused_detections
    
    def _boxes_overlap(
        self,
        box1: Dict[str, float],
        box2: Dict[str, float],
        iou_threshold: float = 0.5
    ) -> bool:
        """
        Check if two bounding boxes overlap.
        
        Args:
            box1: First bounding box
            box2: Second bounding box
            iou_threshold: IoU threshold for overlap
            
        Returns:
            True if boxes overlap, False otherwise
        """
        # Calculate intersection coordinates
        x1 = max(box1["x1"], box2["x1"])
        y1 = max(box1["y1"], box2["y1"])
        x2 = min(box1["x2"], box2["x2"])
        y2 = min(box1["y2"], box2["y2"])
        
        # Check if there is an intersection
        if x2 < x1 or y2 < y1:
            return False
        
        # Calculate areas
        intersection_area = (x2 - x1) * (y2 - y1)
        box1_area = box1["width"] * box1["height"]
        box2_area = box2["width"] * box2["height"]
        union_area = box1_area + box2_area - intersection_area
        
        # Calculate IoU
        iou = intersection_area / union_area if union_area > 0 else 0
        
        return iou >= iou_threshold
    
    def _fuse_overlapping_detections(
        self,
        detections: List[Dict[str, Any]]
    ) -> Dict[str, Any]:
        """
        Fuse overlapping detections.
        
        Args:
            detections: List of overlapping detections
            
        Returns:
            Fused detection dictionary
        """
        # Get detection with highest confidence
        best_detection = max(detections, key=lambda x: x.get("weighted_confidence", 0))
        
        # Calculate weighted average bounding box
        total_weight = sum(d.get("weighted_confidence", 0) for d in detections)
        
        x1 = sum(d["bbox"]["x1"] * d.get("weighted_confidence", 0) for d in detections) / total_weight
        y1 = sum(d["bbox"]["y1"] * d.get("weighted_confidence", 0) for d in detections) / total_weight
        x2 = sum(d["bbox"]["x2"] * d.get("weighted_confidence", 0) for d in detections) / total_weight
        y2 = sum(d["bbox"]["y2"] * d.get("weighted_confidence", 0) for d in detections) / total_weight
        
        # Create fused detection
        fused = {
            "id": str(uuid.uuid4()),
            "modalities": [d["modality"] for d in detections],
            "bbox": {
                "x1": float(x1),
                "y1": float(y1),
                "x2": float(x2),
                "y2": float(y2),
                "width": float(x2 - x1),
                "height": float(y2 - y1)
            },
            "confidence": sum(d["confidence"] for d in detections) / len(detections),
            "class_id": best_detection["class_id"],
            "class_name": best_detection["class_name"],
            "source_detections": detections
        }
        
        return fused
    
    def _apply_roi(
        self,
        frame: np.ndarray,
        roi: Dict[str, Any]
    ) -> np.ndarray:
        """
        Apply region of interest to a frame.
        
        Args:
            frame: Frame to apply ROI to
            roi: Region of interest dictionary
            
        Returns:
            Frame with ROI applied
        """
        # Get ROI coordinates
        roi_type = roi.get("type", "rectangle")
        
        if roi_type == "rectangle":
            x1 = int(roi.get("x1", 0))
            y1 = int(roi.get("y1", 0))
            x2 = int(roi.get("x2", frame.shape[1]))
            y2 = int(roi.get("y2", frame.shape[0]))
            
            # Ensure coordinates are within frame bounds
            x1 = max(0, min(x1, frame.shape[1] - 1))
            y1 = max(0, min(y1, frame.shape[0] - 1))
            x2 = max(0, min(x2, frame.shape[1]))
            y2 = max(0, min(y2, frame.shape[0]))
            
            # Crop frame to ROI
            return frame[y1:y2, x1:x2].copy()
        
        elif roi_type == "polygon":
            # Create mask from polygon points
            points = roi.get("points", [])
            if not points:
                return frame
            
            # Convert points to numpy array
            points = np.array(points, dtype=np.int32)
            
            # Create mask
            mask = np.zeros(frame.shape[:2], dtype=np.uint8)
            cv2.fillPoly(mask, [points], 255)
            
            # Apply mask
            result = cv2.bitwise_and(frame, frame, mask=mask)
            return result
        
        else:
            logger.warning(f"Unknown ROI type: {roi_type}")
            return frame
