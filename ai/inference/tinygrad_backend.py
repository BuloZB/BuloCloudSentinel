"""
TinyGrad inference backend for Bulo.Cloud Sentinel.

This module provides a TinyGrad-based inference backend for running ML models.
"""

import logging
import numpy as np
import os
import time
from typing import Dict, Any, Optional, Union, List, Tuple
from pathlib import Path

from .base import InferenceBackend

# Setup logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class TinygradInference(InferenceBackend):
    """
    TinyGrad inference backend.
    
    This class provides a TinyGrad-based inference backend for running ML models.
    It supports loading models in .npz or .safetensors format and running inference
    on CPU, CUDA, or OpenCL devices.
    """
    
    def __init__(self, model_path: str, device: str = "AUTO"):
        """
        Initialize the TinyGrad inference backend.
        
        Args:
            model_path: Path to the model file (.npz or .safetensors)
            device: Device to run inference on ("CPU", "CUDA", "OCL", or "AUTO")
        """
        super().__init__(model_path, device)
        
        # Set device
        self._set_device()
        
        # Load model
        self.load_model()
    
    def _set_device(self) -> None:
        """
        Set the device for TinyGrad.
        
        This method sets the appropriate environment variables for TinyGrad
        based on the requested device.
        """
        if self.device == "AUTO":
            # Try to detect the best available device
            try:
                # First try CUDA
                import torch
                if torch.cuda.is_available():
                    logger.info("CUDA detected, using CUDA backend")
                    os.environ["TINYGRAD_BACKEND"] = "CUDA"
                    self.device = "CUDA"
                    return
            except ImportError:
                pass
            
            try:
                # Then try OpenCL
                import pyopencl
                platforms = pyopencl.get_platforms()
                if platforms:
                    logger.info("OpenCL detected, using OpenCL backend")
                    os.environ["TINYGRAD_BACKEND"] = "OPENCL"
                    self.device = "OCL"
                    return
            except ImportError:
                pass
            
            # Fall back to CPU
            logger.info("No GPU detected, falling back to CPU backend")
            os.environ["TINYGRAD_BACKEND"] = "CPU"
            self.device = "CPU"
        elif self.device == "CUDA":
            os.environ["TINYGRAD_BACKEND"] = "CUDA"
        elif self.device == "OCL":
            os.environ["TINYGRAD_BACKEND"] = "OPENCL"
        elif self.device == "CPU":
            os.environ["TINYGRAD_BACKEND"] = "CPU"
        else:
            logger.warning(f"Unknown device {self.device}, falling back to CPU")
            os.environ["TINYGRAD_BACKEND"] = "CPU"
            self.device = "CPU"
    
    def load_model(self) -> None:
        """
        Load the model using TinyGrad.
        
        This method loads the model from the specified path using TinyGrad.
        It supports .npz and .safetensors formats.
        """
        try:
            # Import tinygrad here to avoid import errors if not installed
            from tinygrad.tensor import Tensor
            from tinygrad.nn import state_dict
            
            logger.info(f"Loading model from {self.model_path} using TinyGrad")
            
            # Check file extension
            if self.model_path.suffix == ".npz":
                # Load NPZ file
                self.weights = np.load(self.model_path, allow_pickle=True)
                self.model = state_dict.load_state_dict(self.weights)
            elif self.model_path.suffix == ".safetensors":
                # Load SafeTensors file
                try:
                    from safetensors.numpy import load_file
                    self.weights = load_file(self.model_path)
                    self.model = state_dict.load_state_dict(self.weights)
                except ImportError:
                    logger.error("safetensors package not found. Install with: pip install safetensors")
                    raise
            else:
                raise ValueError(f"Unsupported model format: {self.model_path.suffix}. Supported formats: .npz, .safetensors")
            
            # Set initialized flag
            self.initialized = True
            
            # Log success
            logger.info(f"Model loaded successfully with TinyGrad on {self.device}")
            
        except ImportError:
            logger.error("tinygrad package not found. Install with: pip install tinygrad")
            raise
        except Exception as e:
            logger.error(f"Error loading model: {str(e)}")
            raise
    
    def warmup(self) -> None:
        """
        Perform a warmup inference to initialize the model.
        
        This method creates dummy input data based on the model's input shapes
        and runs a single inference to warm up the model.
        """
        if not self.initialized:
            logger.warning("Model not initialized, cannot perform warmup")
            return
        
        try:
            # Import tinygrad here to avoid import errors if not installed
            from tinygrad.tensor import Tensor
            
            logger.info("Performing warmup inference")
            
            # Create dummy input data
            dummy_inputs = {}
            for name, shape in self.input_shapes.items():
                dummy_inputs[name] = np.random.random(shape).astype(np.float32)
            
            # Run inference
            _ = self.predict(dummy_inputs)
            
            logger.info("Warmup complete")
            
        except Exception as e:
            logger.error(f"Error during warmup: {str(e)}")
            raise
    
    def predict(self, inputs: Dict[str, np.ndarray]) -> Dict[str, np.ndarray]:
        """
        Run inference on the model.
        
        Args:
            inputs: Dictionary mapping input names to numpy arrays
            
        Returns:
            Dictionary mapping output names to numpy arrays
        """
        if not self.initialized:
            logger.warning("Model not initialized, cannot perform inference")
            return {}
        
        try:
            # Import tinygrad here to avoid import errors if not installed
            from tinygrad.tensor import Tensor
            
            # Time the inference
            start_time = time.time()
            
            # Convert inputs to TinyGrad tensors
            tinygrad_inputs = {}
            for name, array in inputs.items():
                tinygrad_inputs[name] = Tensor(array)
            
            # Run inference
            # Note: This is a simplified implementation. The actual implementation
            # will depend on the model architecture and how it's defined in TinyGrad.
            # For a real model, you would need to define the forward pass.
            tinygrad_outputs = self._forward(tinygrad_inputs)
            
            # Convert outputs back to numpy arrays
            outputs = {}
            for name, tensor in tinygrad_outputs.items():
                outputs[name] = tensor.numpy()
            
            # Record inference time
            self.last_inference_time = (time.time() - start_time) * 1000  # Convert to ms
            
            return outputs
            
        except Exception as e:
            logger.error(f"Error during inference: {str(e)}")
            raise
    
    def _forward(self, inputs: Dict[str, Any]) -> Dict[str, Any]:
        """
        Run the forward pass of the model.
        
        Args:
            inputs: Dictionary mapping input names to TinyGrad tensors
            
        Returns:
            Dictionary mapping output names to TinyGrad tensors
        """
        # This is a placeholder implementation. The actual implementation
        # will depend on the model architecture and how it's defined in TinyGrad.
        # For a real model, you would need to define the forward pass based on
        # the model architecture.
        
        # For example, for a simple MobileNet-like model:
        # x = inputs["input"]
        # for layer in self.model.layers:
        #     x = layer(x)
        # return {"output": x}
        
        # For now, we'll just return the inputs as outputs
        return inputs
