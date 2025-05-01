"""
TensorFlow Lite inference backend for Bulo.Cloud Sentinel.

This module provides a TensorFlow Lite-based inference backend for running ML models.
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


class TFLiteInference(InferenceBackend):
    """
    TensorFlow Lite inference backend.
    
    This class provides a TensorFlow Lite-based inference backend for running ML models.
    It supports loading models in .tflite format and running inference on CPU, GPU,
    or delegate devices.
    """
    
    def __init__(self, model_path: str, device: str = "AUTO"):
        """
        Initialize the TensorFlow Lite inference backend.
        
        Args:
            model_path: Path to the model file (.tflite)
            device: Device to run inference on ("CPU", "GPU", "NNAPI", or "AUTO")
        """
        super().__init__(model_path, device)
        
        # Load model
        self.load_model()
    
    def load_model(self) -> None:
        """
        Load the model using TensorFlow Lite.
        
        This method loads the model from the specified path using TensorFlow Lite.
        It supports .tflite format.
        """
        try:
            import tensorflow as tf
            
            logger.info(f"Loading model from {self.model_path} using TensorFlow Lite")
            
            # Check file extension
            if self.model_path.suffix != ".tflite":
                raise ValueError(f"Unsupported model format: {self.model_path.suffix}. Supported format: .tflite")
            
            # Load TFLite model
            self.interpreter = tf.lite.Interpreter(model_path=str(self.model_path))
            
            # Set up delegates based on device
            if self.device == "AUTO":
                # Try GPU delegate first
                try:
                    self.interpreter = tf.lite.Interpreter(
                        model_path=str(self.model_path),
                        experimental_delegates=[tf.lite.experimental.load_delegate('libdelegate.so')]
                    )
                    self.device = "GPU"
                    logger.info("Using GPU delegate for TFLite")
                except Exception as e:
                    logger.info(f"GPU delegate not available: {str(e)}")
                    self.interpreter = tf.lite.Interpreter(model_path=str(self.model_path))
                    self.device = "CPU"
                    logger.info("Falling back to CPU for TFLite")
            elif self.device == "GPU":
                try:
                    self.interpreter = tf.lite.Interpreter(
                        model_path=str(self.model_path),
                        experimental_delegates=[tf.lite.experimental.load_delegate('libdelegate.so')]
                    )
                    logger.info("Using GPU delegate for TFLite")
                except Exception as e:
                    logger.warning(f"GPU delegate not available: {str(e)}")
                    self.interpreter = tf.lite.Interpreter(model_path=str(self.model_path))
                    self.device = "CPU"
                    logger.info("Falling back to CPU for TFLite")
            elif self.device == "NNAPI":
                try:
                    self.interpreter = tf.lite.Interpreter(
                        model_path=str(self.model_path),
                        experimental_delegates=[tf.lite.experimental.load_delegate('libnnapi.so')]
                    )
                    logger.info("Using NNAPI delegate for TFLite")
                except Exception as e:
                    logger.warning(f"NNAPI delegate not available: {str(e)}")
                    self.interpreter = tf.lite.Interpreter(model_path=str(self.model_path))
                    self.device = "CPU"
                    logger.info("Falling back to CPU for TFLite")
            
            # Allocate tensors
            self.interpreter.allocate_tensors()
            
            # Get input and output details
            self.input_details = self.interpreter.get_input_details()
            self.output_details = self.interpreter.get_output_details()
            
            # Store input and output shapes
            for detail in self.input_details:
                self.input_shapes[detail['name']] = detail['shape']
            
            for detail in self.output_details:
                self.output_shapes[detail['name']] = detail['shape']
            
            # Set initialized flag
            self.initialized = True
            
            # Log success
            logger.info(f"Model loaded successfully with TensorFlow Lite on {self.device}")
            
        except ImportError:
            logger.error("TensorFlow not found. Install with: pip install tensorflow")
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
            logger.info("Performing warmup inference")
            
            # Create dummy input data
            dummy_inputs = {}
            for detail in self.input_details:
                shape = detail['shape']
                dtype = detail['dtype']
                if dtype == np.float32:
                    dummy_inputs[detail['name']] = np.random.random(shape).astype(np.float32)
                elif dtype == np.int32 or dtype == np.int64:
                    dummy_inputs[detail['name']] = np.random.randint(0, 10, shape).astype(dtype)
                else:
                    dummy_inputs[detail['name']] = np.zeros(shape, dtype=dtype)
            
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
            # Time the inference
            start_time = time.time()
            
            # Set input tensor values
            for detail in self.input_details:
                input_name = detail['name']
                if input_name in inputs:
                    self.interpreter.set_tensor(detail['index'], inputs[input_name])
                else:
                    # Try to find the input by index if name doesn't match
                    for name, array in inputs.items():
                        if len(inputs) == 1 and len(self.input_details) == 1:
                            # If there's only one input, use it regardless of name
                            self.interpreter.set_tensor(detail['index'], array)
                            break
                    else:
                        logger.warning(f"Input {input_name} not found in inputs")
            
            # Run inference
            self.interpreter.invoke()
            
            # Get output tensor values
            outputs = {}
            for detail in self.output_details:
                output_name = detail['name']
                outputs[output_name] = self.interpreter.get_tensor(detail['index'])
            
            # Record inference time
            self.last_inference_time = (time.time() - start_time) * 1000  # Convert to ms
            
            return outputs
            
        except Exception as e:
            logger.error(f"Error during inference: {str(e)}")
            raise
