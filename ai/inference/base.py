"""
Base class for inference backends.

This module provides a base class for all inference backends to implement.
"""

from abc import ABC, abstractmethod
from typing import Dict, Any, Optional, Union
import numpy as np
import logging
import time
from pathlib import Path

# Setup logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class InferenceBackend(ABC):
    """
    Base class for all inference backends.
    
    This abstract class defines the interface that all inference backends must implement.
    """
    
    def __init__(self, model_path: str, device: str = "AUTO"):
        """
        Initialize the inference backend.
        
        Args:
            model_path: Path to the model file
            device: Device to run inference on ("CPU", "CUDA", "OCL", or "AUTO")
        """
        self.model_path = Path(model_path)
        self.device = device.upper()
        self.model = None
        self.input_shapes = {}
        self.output_shapes = {}
        self.initialized = False
        self.last_inference_time = 0.0
        
        # Validate model path
        if not self.model_path.exists():
            raise FileNotFoundError(f"Model file not found: {model_path}")
    
    @abstractmethod
    def load_model(self) -> None:
        """
        Load the model into memory.
        
        This method should be implemented by each backend to load the model
        using the appropriate framework.
        """
        pass
    
    @abstractmethod
    def warmup(self) -> None:
        """
        Perform a warmup inference to initialize the model.
        
        This method should be implemented by each backend to perform a warmup
        inference with dummy data.
        """
        pass
    
    @abstractmethod
    def predict(self, inputs: Dict[str, np.ndarray]) -> Dict[str, np.ndarray]:
        """
        Run inference on the model.
        
        Args:
            inputs: Dictionary mapping input names to numpy arrays
            
        Returns:
            Dictionary mapping output names to numpy arrays
        """
        pass
    
    def get_input_shapes(self) -> Dict[str, tuple]:
        """
        Get the shapes of the model inputs.
        
        Returns:
            Dictionary mapping input names to shapes
        """
        return self.input_shapes
    
    def get_output_shapes(self) -> Dict[str, tuple]:
        """
        Get the shapes of the model outputs.
        
        Returns:
            Dictionary mapping output names to shapes
        """
        return self.output_shapes
    
    def get_last_inference_time(self) -> float:
        """
        Get the time taken for the last inference.
        
        Returns:
            Time in milliseconds
        """
        return self.last_inference_time
    
    def _time_inference(self, func):
        """
        Decorator to time inference.
        
        Args:
            func: Function to time
            
        Returns:
            Wrapped function
        """
        def wrapper(*args, **kwargs):
            start_time = time.time()
            result = func(*args, **kwargs)
            self.last_inference_time = (time.time() - start_time) * 1000  # Convert to ms
            return result
        return wrapper
