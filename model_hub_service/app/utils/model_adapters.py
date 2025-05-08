"""
Model adapters for different model types.

This module provides adapters for different model types to standardize their interface.
"""

import os
import logging
import tempfile
import json
import subprocess
from typing import Dict, List, Any, Optional, Tuple, Union, Callable
from pathlib import Path
import importlib.util

import numpy as np

# Setup logging
logger = logging.getLogger(__name__)

class ModelAdapter:
    """Base class for model adapters."""
    
    def __init__(self, model_path: str, config: Optional[Dict[str, Any]] = None):
        """
        Initialize the model adapter.
        
        Args:
            model_path: Path to the model file
            config: Configuration for the model
        """
        self.model_path = model_path
        self.config = config or {}
        self.model = None
    
    def load(self) -> Any:
        """
        Load the model.
        
        Returns:
            Loaded model
        """
        raise NotImplementedError("Subclasses must implement load()")
    
    def predict(self, inputs: Any) -> Any:
        """
        Run inference on the model.
        
        Args:
            inputs: Input data
            
        Returns:
            Model outputs
        """
        raise NotImplementedError("Subclasses must implement predict()")
    
    def get_metadata(self) -> Dict[str, Any]:
        """
        Get metadata about the model.
        
        Returns:
            Model metadata
        """
        raise NotImplementedError("Subclasses must implement get_metadata()")
    
    def export(self, output_path: str, format: str) -> str:
        """
        Export the model to a different format.
        
        Args:
            output_path: Path to save the exported model
            format: Format to export to (e.g., "onnx", "tflite")
            
        Returns:
            Path to the exported model
        """
        raise NotImplementedError("Subclasses must implement export()")

class YOLOAdapter(ModelAdapter):
    """Adapter for YOLO models."""
    
    def load(self) -> Any:
        """
        Load the YOLO model.
        
        Returns:
            Loaded model
        """
        try:
            # Check if ultralytics is available
            if importlib.util.find_spec("ultralytics") is not None:
                from ultralytics import YOLO
                self.model = YOLO(self.model_path)
                logger.info(f"Loaded YOLO model from {self.model_path} using ultralytics")
                return self.model
            else:
                logger.warning("ultralytics not available, trying PyTorch")
                import torch
                self.model = torch.load(self.model_path, map_location="cpu")
                logger.info(f"Loaded YOLO model from {self.model_path} using PyTorch")
                return self.model
        except Exception as e:
            logger.error(f"Error loading YOLO model: {e}")
            raise
    
    def predict(self, inputs: Any) -> Any:
        """
        Run inference on the YOLO model.
        
        Args:
            inputs: Input data (image or path to image)
            
        Returns:
            Model outputs (detections)
        """
        if self.model is None:
            self.load()
        
        try:
            if hasattr(self.model, "predict"):
                # Ultralytics YOLO
                results = self.model.predict(
                    source=inputs,
                    conf=self.config.get("conf", 0.25),
                    iou=self.config.get("iou", 0.45),
                    max_det=self.config.get("max_det", 300),
                    verbose=False,
                )
                return results
            else:
                # PyTorch model
                # This is a placeholder for actual inference
                # In a real implementation, you would need to preprocess the inputs
                # and run inference on the model
                return {"error": "PyTorch inference not implemented"}
        except Exception as e:
            logger.error(f"Error running inference on YOLO model: {e}")
            raise
    
    def get_metadata(self) -> Dict[str, Any]:
        """
        Get metadata about the YOLO model.
        
        Returns:
            Model metadata
        """
        if self.model is None:
            self.load()
        
        try:
            metadata = {
                "model_type": "yolo",
                "framework": "pytorch",
                "file_size": os.path.getsize(self.model_path),
            }
            
            if hasattr(self.model, "names"):
                # Ultralytics YOLO
                metadata["classes"] = self.model.names
                metadata["task"] = getattr(self.model, "task", "detect")
                metadata["version"] = getattr(self.model, "version", "unknown")
            
            return metadata
        except Exception as e:
            logger.error(f"Error getting YOLO model metadata: {e}")
            raise
    
    def export(self, output_path: str, format: str) -> str:
        """
        Export the YOLO model to a different format.
        
        Args:
            output_path: Path to save the exported model
            format: Format to export to (e.g., "onnx", "tflite")
            
        Returns:
            Path to the exported model
        """
        if self.model is None:
            self.load()
        
        try:
            if hasattr(self.model, "export"):
                # Ultralytics YOLO
                exported_path = self.model.export(format=format, output=output_path)
                logger.info(f"Exported YOLO model to {exported_path}")
                return exported_path
            else:
                # PyTorch model
                # This is a placeholder for actual export
                # In a real implementation, you would need to export the model
                # to the requested format
                import shutil
                shutil.copy(self.model_path, output_path)
                logger.warning(f"PyTorch export not implemented, copied model to {output_path}")
                return output_path
        except Exception as e:
            logger.error(f"Error exporting YOLO model: {e}")
            raise

class SAMAdapter(ModelAdapter):
    """Adapter for Segment Anything Model (SAM)."""
    
    def load(self) -> Any:
        """
        Load the SAM model.
        
        Returns:
            Loaded model
        """
        try:
            # Check if segment_anything is available
            if importlib.util.find_spec("segment_anything") is not None:
                from segment_anything import sam_model_registry, SamPredictor
                
                # Determine model type from path
                model_type = "vit_h"  # Default to ViT-H
                if "vit_b" in self.model_path:
                    model_type = "vit_b"
                elif "vit_l" in self.model_path:
                    model_type = "vit_l"
                
                # Load model
                sam = sam_model_registry[model_type](checkpoint=self.model_path)
                self.model = SamPredictor(sam)
                logger.info(f"Loaded SAM model from {self.model_path}")
                return self.model
            else:
                logger.warning("segment_anything not available")
                import torch
                self.model = torch.load(self.model_path, map_location="cpu")
                logger.info(f"Loaded SAM model from {self.model_path} using PyTorch")
                return self.model
        except Exception as e:
            logger.error(f"Error loading SAM model: {e}")
            raise
    
    def predict(self, inputs: Any) -> Any:
        """
        Run inference on the SAM model.
        
        Args:
            inputs: Input data (image and prompts)
            
        Returns:
            Model outputs (segmentation masks)
        """
        if self.model is None:
            self.load()
        
        try:
            if hasattr(self.model, "predict"):
                # SAM Predictor
                image = inputs.get("image")
                points = inputs.get("points")
                labels = inputs.get("labels")
                box = inputs.get("box")
                
                # Set image
                self.model.set_image(image)
                
                # Generate masks
                if points is not None and labels is not None:
                    masks, scores, logits = self.model.predict(
                        point_coords=points,
                        point_labels=labels,
                        box=box,
                        multimask_output=self.config.get("multimask_output", True),
                    )
                elif box is not None:
                    masks, scores, logits = self.model.predict(
                        box=box,
                        multimask_output=self.config.get("multimask_output", True),
                    )
                else:
                    raise ValueError("Either points or box must be provided")
                
                return {
                    "masks": masks,
                    "scores": scores,
                    "logits": logits,
                }
            else:
                # PyTorch model
                # This is a placeholder for actual inference
                # In a real implementation, you would need to preprocess the inputs
                # and run inference on the model
                return {"error": "PyTorch inference not implemented"}
        except Exception as e:
            logger.error(f"Error running inference on SAM model: {e}")
            raise
    
    def get_metadata(self) -> Dict[str, Any]:
        """
        Get metadata about the SAM model.
        
        Returns:
            Model metadata
        """
        if self.model is None:
            self.load()
        
        try:
            metadata = {
                "model_type": "sam",
                "framework": "pytorch",
                "file_size": os.path.getsize(self.model_path),
            }
            
            # Determine model type from path
            if "vit_b" in self.model_path:
                metadata["variant"] = "vit_b"
            elif "vit_l" in self.model_path:
                metadata["variant"] = "vit_l"
            elif "vit_h" in self.model_path:
                metadata["variant"] = "vit_h"
            else:
                metadata["variant"] = "unknown"
            
            return metadata
        except Exception as e:
            logger.error(f"Error getting SAM model metadata: {e}")
            raise
    
    def export(self, output_path: str, format: str) -> str:
        """
        Export the SAM model to a different format.
        
        Args:
            output_path: Path to save the exported model
            format: Format to export to (e.g., "onnx", "tflite")
            
        Returns:
            Path to the exported model
        """
        if self.model is None:
            self.load()
        
        try:
            # This is a placeholder for actual export
            # In a real implementation, you would need to export the model
            # to the requested format
            import shutil
            shutil.copy(self.model_path, output_path)
            logger.warning(f"SAM export not implemented, copied model to {output_path}")
            return output_path
        except Exception as e:
            logger.error(f"Error exporting SAM model: {e}")
            raise

class SuperGradientsAdapter(ModelAdapter):
    """Adapter for Super-Gradients models."""
    
    def load(self) -> Any:
        """
        Load the Super-Gradients model.
        
        Returns:
            Loaded model
        """
        try:
            # Check if super_gradients is available
            if importlib.util.find_spec("super_gradients") is not None:
                from super_gradients.training import models
                
                # Determine model type from config
                model_name = self.config.get("model_name", "yolo_nas_l")
                pretrained_weights = self.config.get("pretrained_weights", "coco")
                
                # Load model
                self.model = models.get(model_name, pretrained_weights=pretrained_weights)
                logger.info(f"Loaded Super-Gradients model {model_name}")
                
                # Load weights if provided
                if os.path.isfile(self.model_path):
                    import torch
                    weights = torch.load(self.model_path, map_location="cpu")
                    self.model.load_state_dict(weights)
                    logger.info(f"Loaded weights from {self.model_path}")
                
                return self.model
            else:
                logger.warning("super_gradients not available")
                import torch
                self.model = torch.load(self.model_path, map_location="cpu")
                logger.info(f"Loaded Super-Gradients model from {self.model_path} using PyTorch")
                return self.model
        except Exception as e:
            logger.error(f"Error loading Super-Gradients model: {e}")
            raise
    
    def predict(self, inputs: Any) -> Any:
        """
        Run inference on the Super-Gradients model.
        
        Args:
            inputs: Input data (image or path to image)
            
        Returns:
            Model outputs (detections)
        """
        if self.model is None:
            self.load()
        
        try:
            if hasattr(self.model, "predict"):
                # Super-Gradients model
                results = self.model.predict(
                    inputs,
                    conf=self.config.get("conf", 0.25),
                    iou=self.config.get("iou", 0.45),
                )
                return results
            else:
                # PyTorch model
                # This is a placeholder for actual inference
                # In a real implementation, you would need to preprocess the inputs
                # and run inference on the model
                return {"error": "PyTorch inference not implemented"}
        except Exception as e:
            logger.error(f"Error running inference on Super-Gradients model: {e}")
            raise
    
    def get_metadata(self) -> Dict[str, Any]:
        """
        Get metadata about the Super-Gradients model.
        
        Returns:
            Model metadata
        """
        if self.model is None:
            self.load()
        
        try:
            metadata = {
                "model_type": "super-gradients",
                "framework": "pytorch",
                "file_size": os.path.getsize(self.model_path) if os.path.isfile(self.model_path) else 0,
                "model_name": self.config.get("model_name", "yolo_nas_l"),
                "pretrained_weights": self.config.get("pretrained_weights", "coco"),
            }
            
            return metadata
        except Exception as e:
            logger.error(f"Error getting Super-Gradients model metadata: {e}")
            raise
    
    def export(self, output_path: str, format: str) -> str:
        """
        Export the Super-Gradients model to a different format.
        
        Args:
            output_path: Path to save the exported model
            format: Format to export to (e.g., "onnx", "tflite")
            
        Returns:
            Path to the exported model
        """
        if self.model is None:
            self.load()
        
        try:
            # This is a placeholder for actual export
            # In a real implementation, you would need to export the model
            # to the requested format
            if hasattr(self.model, "export"):
                # Super-Gradients model
                self.model.export(output_path, format=format)
                logger.info(f"Exported Super-Gradients model to {output_path}")
                return output_path
            else:
                # PyTorch model
                import shutil
                shutil.copy(self.model_path, output_path)
                logger.warning(f"PyTorch export not implemented, copied model to {output_path}")
                return output_path
        except Exception as e:
            logger.error(f"Error exporting Super-Gradients model: {e}")
            raise

def get_model_adapter(model_path: str, model_type: str, config: Optional[Dict[str, Any]] = None) -> ModelAdapter:
    """
    Get a model adapter for the specified model type.
    
    Args:
        model_path: Path to the model file
        model_type: Type of the model (e.g., "yolo", "sam", "super-gradients")
        config: Configuration for the model
        
    Returns:
        Model adapter
    """
    if model_type.lower() in ["yolo", "yolov5", "yolov8", "yolov10"]:
        return YOLOAdapter(model_path, config)
    elif model_type.lower() in ["sam", "segment-anything"]:
        return SAMAdapter(model_path, config)
    elif model_type.lower() in ["super-gradients", "yolo-nas"]:
        return SuperGradientsAdapter(model_path, config)
    else:
        raise ValueError(f"Unsupported model type: {model_type}")
