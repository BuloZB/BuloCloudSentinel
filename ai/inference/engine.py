"""
Inference engine for Bulo.Cloud Sentinel.

This module provides a central engine for loading and managing inference backends.
"""

import logging
import os
from typing import Dict, Any, Optional, Union, List, Tuple
from pathlib import Path
import importlib

from .base import InferenceBackend
from .tinygrad_backend import TinygradInference
from .torch_backend import TorchInference
from .tflite_backend import TFLiteInference

# Setup logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class InferenceEngine:
    """
    Inference engine for Bulo.Cloud Sentinel.
    
    This class provides a central engine for loading and managing inference backends.
    It supports multiple backends (TinyGrad, PyTorch, TFLite) and provides a unified
    interface for running inference.
    """
    
    # Available backends
    BACKENDS = {
        "tinygrad": TinygradInference,
        "torch": TorchInference,
        "tflite": TFLiteInference,
    }
    
    def __init__(self, backend: str = "torch", model_path: str = None, device: str = "AUTO"):
        """
        Initialize the inference engine.
        
        Args:
            backend: Backend to use ("tinygrad", "torch", "tflite")
            model_path: Path to the model file
            device: Device to run inference on ("CPU", "CUDA", "OCL", "AUTO")
        """
        self.backend_name = backend.lower()
        self.model_path = model_path
        self.device = device.upper()
        self.backend = None
        
        # Check if backend is valid
        if self.backend_name not in self.BACKENDS:
            raise ValueError(f"Unknown backend: {backend}. Available backends: {list(self.BACKENDS.keys())}")
        
        # Load model if path is provided
        if self.model_path:
            self.load_model(self.model_path, self.device)
    
    def load_model(self, model_path: str, device: str = "AUTO") -> None:
        """
        Load a model using the selected backend.
        
        Args:
            model_path: Path to the model file
            device: Device to run inference on ("CPU", "CUDA", "OCL", "AUTO")
        """
        try:
            # Update model path and device
            self.model_path = model_path
            self.device = device.upper()
            
            # Check if backend class is available
            if self.backend_name not in self.BACKENDS:
                raise ValueError(f"Unknown backend: {self.backend_name}. Available backends: {list(self.BACKENDS.keys())}")
            
            # Get backend class
            backend_class = self.BACKENDS[self.backend_name]
            
            # Initialize backend
            logger.info(f"Initializing {self.backend_name} backend with model {model_path} on device {device}")
            self.backend = backend_class(model_path, device)
            
            # Log success
            logger.info(f"Model loaded successfully with {self.backend_name} backend")
            
        except Exception as e:
            logger.error(f"Error loading model: {str(e)}")
            raise
    
    def warmup(self) -> None:
        """
        Perform a warmup inference to initialize the model.
        """
        if not self.backend:
            logger.warning("No backend initialized, cannot perform warmup")
            return
        
        self.backend.warmup()
    
    def predict(self, inputs: Dict[str, Any]) -> Dict[str, Any]:
        """
        Run inference on the model.
        
        Args:
            inputs: Dictionary mapping input names to numpy arrays
            
        Returns:
            Dictionary mapping output names to numpy arrays
        """
        if not self.backend:
            logger.warning("No backend initialized, cannot perform inference")
            return {}
        
        return self.backend.predict(inputs)
    
    def get_input_shapes(self) -> Dict[str, tuple]:
        """
        Get the shapes of the model inputs.
        
        Returns:
            Dictionary mapping input names to shapes
        """
        if not self.backend:
            logger.warning("No backend initialized, cannot get input shapes")
            return {}
        
        return self.backend.get_input_shapes()
    
    def get_output_shapes(self) -> Dict[str, tuple]:
        """
        Get the shapes of the model outputs.
        
        Returns:
            Dictionary mapping output names to shapes
        """
        if not self.backend:
            logger.warning("No backend initialized, cannot get output shapes")
            return {}
        
        return self.backend.get_output_shapes()
    
    def get_last_inference_time(self) -> float:
        """
        Get the time taken for the last inference.
        
        Returns:
            Time in milliseconds
        """
        if not self.backend:
            logger.warning("No backend initialized, cannot get inference time")
            return 0.0
        
        return self.backend.get_last_inference_time()
    
    @staticmethod
    def get_available_backends() -> List[str]:
        """
        Get a list of available backends.
        
        Returns:
            List of backend names
        """
        return list(InferenceEngine.BACKENDS.keys())
    
    @staticmethod
    def get_backend_from_file(model_path: str) -> str:
        """
        Determine the appropriate backend for a model file based on its extension.
        
        Args:
            model_path: Path to the model file
            
        Returns:
            Backend name
        """
        path = Path(model_path)
        extension = path.suffix.lower()
        
        if extension in [".npz", ".safetensors"]:
            return "tinygrad"
        elif extension in [".pt", ".pth", ".onnx"]:
            return "torch"
        elif extension == ".tflite":
            return "tflite"
        else:
            logger.warning(f"Unknown model format: {extension}, defaulting to PyTorch")
            return "torch"
