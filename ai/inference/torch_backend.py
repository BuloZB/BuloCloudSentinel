"""
PyTorch inference backend for Bulo.Cloud Sentinel.

This module provides a PyTorch-based inference backend for running ML models.
"""

import logging
import numpy as np
import time
from typing import Dict, Any, Optional, Union, List, Tuple
from pathlib import Path

from .base import InferenceBackend

# Setup logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class TorchInference(InferenceBackend):
    """
    PyTorch inference backend.
    
    This class provides a PyTorch-based inference backend for running ML models.
    It supports loading models in .pt, .pth, and .onnx formats and running inference
    on CPU or CUDA devices.
    """
    
    def __init__(self, model_path: str, device: str = "AUTO"):
        """
        Initialize the PyTorch inference backend.
        
        Args:
            model_path: Path to the model file (.pt, .pth, or .onnx)
            device: Device to run inference on ("CPU", "CUDA", or "AUTO")
        """
        super().__init__(model_path, device)
        
        # Set device
        self._set_device()
        
        # Load model
        self.load_model()
    
    def _set_device(self) -> None:
        """
        Set the device for PyTorch.
        
        This method determines the appropriate device for PyTorch
        based on the requested device and available hardware.
        """
        try:
            import torch
            
            if self.device == "AUTO":
                # Use CUDA if available, otherwise CPU
                self.torch_device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
                self.device = "CUDA" if torch.cuda.is_available() else "CPU"
            elif self.device == "CUDA":
                # Check if CUDA is available
                if torch.cuda.is_available():
                    self.torch_device = torch.device("cuda")
                else:
                    logger.warning("CUDA requested but not available, falling back to CPU")
                    self.torch_device = torch.device("cpu")
                    self.device = "CPU"
            elif self.device == "CPU":
                self.torch_device = torch.device("cpu")
            else:
                logger.warning(f"Unknown device {self.device}, falling back to CPU")
                self.torch_device = torch.device("cpu")
                self.device = "CPU"
            
            logger.info(f"Using PyTorch device: {self.torch_device}")
            
        except ImportError:
            logger.error("PyTorch not found. Install with: pip install torch")
            raise
    
    def load_model(self) -> None:
        """
        Load the model using PyTorch.
        
        This method loads the model from the specified path using PyTorch.
        It supports .pt, .pth, and .onnx formats.
        """
        try:
            import torch
            
            logger.info(f"Loading model from {self.model_path} using PyTorch")
            
            # Check file extension
            if self.model_path.suffix in [".pt", ".pth"]:
                # Load PyTorch model
                self.model = torch.load(self.model_path, map_location=self.torch_device)
                
                # Handle different model formats
                if isinstance(self.model, dict):
                    # If the model is a state dict, we need a model definition
                    if "model_definition" in self.model:
                        # If the model definition is included in the state dict
                        model_class = self.model["model_definition"]
                        model_instance = model_class()
                        model_instance.load_state_dict(self.model["state_dict"])
                        self.model = model_instance
                    else:
                        # If only the state dict is saved, we can't load the model
                        # without knowing its architecture
                        logger.warning("Model is a state dict without model definition")
                
                # Set model to evaluation mode
                if hasattr(self.model, "eval"):
                    self.model.eval()
                
            elif self.model_path.suffix == ".onnx":
                # Load ONNX model
                import onnxruntime as ort
                
                # Create ONNX Runtime session
                providers = []
                if self.device == "CUDA":
                    providers.append("CUDAExecutionProvider")
                providers.append("CPUExecutionProvider")
                
                self.model = ort.InferenceSession(
                    str(self.model_path),
                    providers=providers
                )
                
                # Get input and output shapes
                for input_info in self.model.get_inputs():
                    self.input_shapes[input_info.name] = input_info.shape
                
                for output_info in self.model.get_outputs():
                    self.output_shapes[output_info.name] = output_info.shape
                
            else:
                raise ValueError(f"Unsupported model format: {self.model_path.suffix}. Supported formats: .pt, .pth, .onnx")
            
            # Set initialized flag
            self.initialized = True
            
            # Log success
            logger.info(f"Model loaded successfully with PyTorch on {self.device}")
            
        except ImportError:
            logger.error("Required packages not found. Install with: pip install torch onnxruntime")
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
            import torch
            
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
            import torch
            
            # Time the inference
            start_time = time.time()
            
            # Check if model is ONNX
            if hasattr(self.model, "run"):
                # ONNX model
                
                # Prepare input names and data
                input_names = [input_info.name for input_info in self.model.get_inputs()]
                input_data = [inputs[name] for name in input_names]
                
                # Run inference
                outputs = self.model.run(None, {name: inputs[name] for name in input_names})
                
                # Get output names
                output_names = [output_info.name for output_info in self.model.get_outputs()]
                
                # Create output dictionary
                result = {name: output for name, output in zip(output_names, outputs)}
                
            else:
                # PyTorch model
                
                # Convert inputs to PyTorch tensors
                torch_inputs = {}
                for name, array in inputs.items():
                    torch_inputs[name] = torch.tensor(array, device=self.torch_device)
                
                # Run inference
                with torch.no_grad():
                    if isinstance(torch_inputs, dict) and len(torch_inputs) == 1:
                        # If there's only one input, pass it directly
                        input_tensor = next(iter(torch_inputs.values()))
                        output = self.model(input_tensor)
                    else:
                        # Otherwise, pass the dictionary
                        output = self.model(**torch_inputs)
                
                # Convert output to numpy
                if isinstance(output, torch.Tensor):
                    # Single output tensor
                    result = {"output": output.cpu().numpy()}
                elif isinstance(output, tuple) or isinstance(output, list):
                    # Multiple output tensors
                    result = {f"output_{i}": tensor.cpu().numpy() for i, tensor in enumerate(output)}
                elif isinstance(output, dict):
                    # Dictionary of output tensors
                    result = {name: tensor.cpu().numpy() for name, tensor in output.items()}
                else:
                    raise ValueError(f"Unexpected output type: {type(output)}")
            
            # Record inference time
            self.last_inference_time = (time.time() - start_time) * 1000  # Convert to ms
            
            return result
            
        except Exception as e:
            logger.error(f"Error during inference: {str(e)}")
            raise
