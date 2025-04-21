"""
Crowd Analyzer for the Vision System.

This module provides crowd density estimation and people counting functionality.
"""

import logging
import time
import numpy as np
import cv2
import torch
from typing import Dict, List, Any, Optional, Tuple
import uuid
from datetime import datetime

# Setup logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class CrowdAnalyzer:
    """Analyzer for crowd density estimation and people counting."""
    
    def __init__(self, config: Dict[str, Any]):
        """Initialize the crowd analyzer."""
        self.config = config
        self.models = {}
        self.device = torch.device("cuda" if torch.cuda.is_available() and config.get("enable_gpu", True) else "cpu")
        
        # Initialize models
        self._initialize_models()
    
    def _initialize_models(self):
        """Initialize crowd analysis models."""
        try:
            logger.info("Initializing crowd analysis models")
            
            # Load CSRNet model for density estimation
            if self.config.get("use_csrnet", True):
                self._load_csrnet_model()
            
            # Load YOLO model for people detection
            if self.config.get("use_yolo", True):
                self._load_yolo_model()
            
            logger.info(f"Crowd analysis models initialized on {self.device}")
        except Exception as e:
            logger.error(f"Error initializing crowd analysis models: {str(e)}")
            raise
    
    def _load_csrnet_model(self):
        """Load CSRNet model for crowd density estimation."""
        try:
            # In a real implementation, you would load the actual CSRNet model
            # For example:
            # model_path = self.config.get("csrnet_model_path", "models/crowd/csrnet.pth")
            # self.models["csrnet"] = torch.load(model_path, map_location=self.device)
            # self.models["csrnet"].to(self.device)
            # self.models["csrnet"].eval()
            
            # For now, we'll just simulate model loading
            self.models["csrnet"] = {
                "name": "CSRNet",
                "type": "density_estimation",
                "loaded": True,
                "device": self.device
            }
            
            logger.info("CSRNet model loaded successfully")
        except Exception as e:
            logger.error(f"Error loading CSRNet model: {str(e)}")
            raise
    
    def _load_yolo_model(self):
        """Load YOLO model for people detection."""
        try:
            # In a real implementation, you would load the actual YOLO model
            # For example:
            # from ultralytics import YOLO
            # model_path = self.config.get("yolo_model_path", "models/crowd/yolov8n.pt")
            # self.models["yolo"] = YOLO(model_path)
            
            # For now, we'll just simulate model loading
            self.models["yolo"] = {
                "name": "YOLOv8",
                "type": "object_detection",
                "loaded": True,
                "device": self.device
            }
            
            logger.info("YOLO model loaded successfully")
        except Exception as e:
            logger.error(f"Error loading YOLO model: {str(e)}")
            raise
    
    async def analyze_frame(
        self,
        frame: np.ndarray,
        region_of_interest: Optional[Dict[str, Any]] = None,
        parameters: Optional[Dict[str, Any]] = None
    ) -> Dict[str, Any]:
        """
        Analyze a frame for crowd density and people count.
        
        Args:
            frame: The frame to analyze
            region_of_interest: Optional region of interest to analyze
            parameters: Optional parameters for analysis
            
        Returns:
            Dictionary containing analysis results
        """
        try:
            start_time = time.time()
            
            # Apply region of interest if provided
            if region_of_interest:
                frame = self._apply_roi(frame, region_of_interest)
            
            # Get parameters
            params = parameters or {}
            use_density_map = params.get("use_density_map", True)
            detect_people = params.get("detect_people", True)
            confidence_threshold = params.get("confidence_threshold", 0.5)
            
            # Initialize results
            results = {
                "timestamp": datetime.now(),
                "people_count": 0,
                "average_density": 0.0,
                "max_density": 0.0,
                "density_map": None,
                "detected_people": []
            }
            
            # Perform density estimation if enabled
            if use_density_map and "csrnet" in self.models:
                density_results = await self._estimate_density(frame)
                results.update(density_results)
            
            # Perform people detection if enabled
            if detect_people and "yolo" in self.models:
                detection_results = await self._detect_people(frame, confidence_threshold)
                
                # If density estimation wasn't used, use detection count
                if not use_density_map:
                    results["people_count"] = len(detection_results["detected_people"])
                
                results["detected_people"] = detection_results["detected_people"]
            
            # Calculate processing time
            processing_time = (time.time() - start_time) * 1000  # Convert to milliseconds
            results["processing_time"] = processing_time
            
            return results
        except Exception as e:
            logger.error(f"Error analyzing frame for crowd: {str(e)}")
            raise
    
    async def _estimate_density(self, frame: np.ndarray) -> Dict[str, Any]:
        """
        Estimate crowd density in a frame.
        
        Args:
            frame: The frame to analyze
            
        Returns:
            Dictionary containing density estimation results
        """
        try:
            # In a real implementation, you would run the CSRNet model on the frame
            # For example:
            # # Preprocess frame
            # frame_tensor = self._preprocess_for_csrnet(frame)
            # frame_tensor = frame_tensor.to(self.device)
            # 
            # # Run inference
            # with torch.no_grad():
            #     density_map = self.models["csrnet"](frame_tensor)
            # 
            # # Convert to numpy
            # density_map = density_map.cpu().numpy().squeeze()
            
            # For now, we'll just simulate density estimation
            height, width = frame.shape[:2]
            
            # Create a simulated density map
            # Higher density in the center, lower at the edges
            y, x = np.mgrid[0:height, 0:width]
            center_y, center_x = height // 2, width // 2
            dist_from_center = np.sqrt((x - center_x)**2 + (y - center_y)**2)
            max_dist = np.sqrt(center_x**2 + center_y**2)
            
            # Normalize distance and invert (center has highest density)
            normalized_dist = dist_from_center / max_dist
            density_factor = 1 - normalized_dist
            
            # Add some random variation
            np.random.seed(int(time.time()))
            noise = np.random.normal(0, 0.1, (height, width))
            
            # Create density map (higher in center, with noise)
            density_map = density_factor + noise
            density_map = np.clip(density_map, 0, None)
            
            # Scale density map to simulate realistic values
            # Assuming average person takes about 0.5 square meters
            # and we want density in people per square meter
            scale_factor = 2.0  # people per square meter at maximum density
            density_map = density_map * scale_factor
            
            # Calculate total count by summing density map
            total_count = np.sum(density_map)
            
            # Calculate average and max density
            average_density = np.mean(density_map)
            max_density = np.max(density_map)
            
            # Prepare density map for return
            # Resize to a smaller resolution for efficiency
            target_height, target_width = 120, 160
            resized_map = cv2.resize(density_map, (target_width, target_height))
            
            # Flatten for JSON serialization
            flattened_map = resized_map.flatten().tolist()
            
            return {
                "people_count": int(total_count),
                "average_density": float(average_density),
                "max_density": float(max_density),
                "density_map": {
                    "width": target_width,
                    "height": target_height,
                    "data": flattened_map,
                    "min_value": float(np.min(resized_map)),
                    "max_value": float(np.max(resized_map)),
                    "total_count": float(total_count)
                }
            }
        except Exception as e:
            logger.error(f"Error estimating density: {str(e)}")
            raise
    
    async def _detect_people(
        self,
        frame: np.ndarray,
        confidence_threshold: float = 0.5
    ) -> Dict[str, Any]:
        """
        Detect people in a frame.
        
        Args:
            frame: The frame to analyze
            confidence_threshold: Confidence threshold for detections
            
        Returns:
            Dictionary containing detection results
        """
        try:
            # In a real implementation, you would run the YOLO model on the frame
            # For example:
            # # Run inference
            # results = self.models["yolo"](frame, conf=confidence_threshold, classes=0)  # class 0 is person
            # 
            # # Process results
            # detections = []
            # for result in results:
            #     boxes = result.boxes.cpu().numpy()
            #     for i, box in enumerate(boxes):
            #         x1, y1, x2, y2 = box.xyxy[0]
            #         confidence = box.conf[0]
            #         
            #         # Convert to normalized coordinates
            #         height, width = frame.shape[:2]
            #         x = x1 / width
            #         y = y1 / height
            #         w = (x2 - x1) / width
            #         h = (y2 - y1) / height
            #         
            #         detections.append({
            #             "id": str(uuid.uuid4()),
            #             "type": "person",
            #             "confidence": float(confidence),
            #             "bounding_box": {"x": float(x), "y": float(y), "width": float(w), "height": float(h)}
            #         })
            
            # For now, we'll just simulate people detection
            height, width = frame.shape[:2]
            
            # Simulate 10-20 people detections
            num_people = np.random.randint(10, 21)
            detections = []
            
            for i in range(num_people):
                # Generate random bounding box
                box_width = np.random.uniform(0.05, 0.15)  # 5-15% of image width
                box_height = np.random.uniform(0.1, 0.3)   # 10-30% of image height
                
                # Ensure box is within image bounds
                x = np.random.uniform(0, 1 - box_width)
                y = np.random.uniform(0, 1 - box_height)
                
                # Generate random confidence score
                confidence = np.random.uniform(confidence_threshold, 1.0)
                
                detections.append({
                    "id": str(uuid.uuid4()),
                    "type": "person",
                    "confidence": float(confidence),
                    "bounding_box": {
                        "x": float(x),
                        "y": float(y),
                        "width": float(box_width),
                        "height": float(box_height)
                    }
                })
            
            return {
                "detected_people": detections
            }
        except Exception as e:
            logger.error(f"Error detecting people: {str(e)}")
            raise
    
    def _apply_roi(
        self,
        frame: np.ndarray,
        region_of_interest: Dict[str, Any]
    ) -> np.ndarray:
        """
        Apply region of interest to a frame.
        
        Args:
            frame: The frame to apply ROI to
            region_of_interest: ROI definition
            
        Returns:
            Frame with ROI applied (masked outside ROI)
        """
        try:
            height, width = frame.shape[:2]
            mask = np.zeros((height, width), dtype=np.uint8)
            
            roi_type = region_of_interest.get("type", "polygon")
            
            if roi_type == "polygon":
                points = region_of_interest.get("points", [])
                if points:
                    # Convert normalized points to pixel coordinates
                    points_px = []
                    for point in points:
                        x = int(point["x"] * width)
                        y = int(point["y"] * height)
                        points_px.append([x, y])
                    
                    # Create polygon mask
                    points_array = np.array(points_px, dtype=np.int32)
                    cv2.fillPoly(mask, [points_array], 255)
            
            elif roi_type == "rectangle":
                x = region_of_interest.get("x", 0)
                y = region_of_interest.get("y", 0)
                w = region_of_interest.get("width", 1)
                h = region_of_interest.get("height", 1)
                
                # Convert normalized coordinates to pixel coordinates
                x_px = int(x * width)
                y_px = int(y * height)
                w_px = int(w * width)
                h_px = int(h * height)
                
                # Create rectangle mask
                cv2.rectangle(mask, (x_px, y_px), (x_px + w_px, y_px + h_px), 255, -1)
            
            elif roi_type == "circle":
                center_x = region_of_interest.get("center_x", 0.5)
                center_y = region_of_interest.get("center_y", 0.5)
                radius = region_of_interest.get("radius", 0.5)
                
                # Convert normalized coordinates to pixel coordinates
                center_x_px = int(center_x * width)
                center_y_px = int(center_y * height)
                radius_px = int(radius * min(width, height))
                
                # Create circle mask
                cv2.circle(mask, (center_x_px, center_y_px), radius_px, 255, -1)
            
            # Apply mask to frame
            masked_frame = cv2.bitwise_and(frame, frame, mask=mask)
            
            return masked_frame
        except Exception as e:
            logger.error(f"Error applying ROI: {str(e)}")
            # Return original frame if error occurs
            return frame
    
    def _preprocess_for_csrnet(self, frame: np.ndarray) -> torch.Tensor:
        """
        Preprocess a frame for CSRNet model.
        
        Args:
            frame: The frame to preprocess
            
        Returns:
            Preprocessed frame as a PyTorch tensor
        """
        # Convert to RGB if needed
        if frame.shape[2] == 4:  # RGBA
            frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2RGB)
        elif frame.shape[2] == 1:  # Grayscale
            frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2RGB)
        
        # Resize to model input size
        input_size = self.config.get("csrnet_input_size", (640, 480))
        frame = cv2.resize(frame, input_size)
        
        # Normalize
        frame = frame.astype(np.float32) / 255.0
        
        # Convert to tensor and add batch dimension
        frame_tensor = torch.from_numpy(frame).permute(2, 0, 1).unsqueeze(0)
        
        return frame_tensor
