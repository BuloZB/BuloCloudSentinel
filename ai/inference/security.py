"""
Security utilities for the inference module.

This module provides security utilities for the inference module, including
path validation, input validation, and safe model loading.
"""

import os
import re
import logging
from pathlib import Path
from typing import Optional, List, Dict, Any, Union

# Setup logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


def is_safe_path(base_dir: str, path: str) -> bool:
    """
    Check if a path is safe (doesn't escape the base directory).
    
    Args:
        base_dir: Base directory
        path: Path to check
        
    Returns:
        True if the path is safe, False otherwise
    """
    try:
        # Normalize paths
        abs_base = os.path.abspath(base_dir)
        abs_path = os.path.abspath(os.path.join(base_dir, path))
        
        # Check if path is within base directory
        return abs_path.startswith(abs_base)
    except Exception as e:
        logger.error(f"Error checking path safety: {str(e)}")
        return False


def is_safe_file(file_path: str, allowed_extensions: List[str] = None) -> bool:
    """
    Check if a file is safe to access.
    
    Args:
        file_path: Path to the file
        allowed_extensions: List of allowed file extensions
        
    Returns:
        True if the file is safe, False otherwise
    """
    try:
        # Check if file exists
        if not os.path.exists(file_path):
            logger.warning(f"File not found: {file_path}")
            return False
        
        # Check if file is a regular file (not a symlink, device file, etc.)
        if not os.path.isfile(file_path):
            logger.warning(f"Not a regular file: {file_path}")
            return False
        
        # Check file extension if allowed_extensions is provided
        if allowed_extensions:
            ext = os.path.splitext(file_path)[1].lower()
            if ext not in allowed_extensions:
                logger.warning(f"File extension not allowed: {ext}")
                return False
        
        return True
    except Exception as e:
        logger.error(f"Error checking file safety: {str(e)}")
        return False


def validate_input_shape(input_shape_str: str) -> Optional[List[int]]:
    """
    Validate input shape string.
    
    Args:
        input_shape_str: Input shape as a comma-separated string
        
    Returns:
        List of integers or None if invalid
    """
    if not input_shape_str:
        return None
    
    try:
        # Parse input shape
        shape = [int(dim.strip()) for dim in input_shape_str.split(",")]
        
        # Validate dimensions
        if any(dim <= 0 for dim in shape):
            logger.warning("All dimensions must be positive")
            return None
        
        # Validate number of dimensions
        if len(shape) < 1 or len(shape) > 5:
            logger.warning("Input shape must have between 1 and 5 dimensions")
            return None
        
        return shape
    except ValueError as e:
        logger.warning(f"Invalid input shape: {str(e)}")
        return None


def sanitize_filename(filename: str) -> str:
    """
    Sanitize a filename to prevent path traversal and other attacks.
    
    Args:
        filename: Filename to sanitize
        
    Returns:
        Sanitized filename
    """
    # Remove path components
    filename = os.path.basename(filename)
    
    # Remove special characters
    filename = re.sub(r'[^\w\.-]', '_', filename)
    
    # Ensure filename is not empty
    if not filename:
        filename = "unnamed_file"
    
    return filename


def get_safe_temp_file(prefix: str = "temp", suffix: str = "") -> str:
    """
    Get a safe temporary file path.
    
    Args:
        prefix: Prefix for the temporary file
        suffix: Suffix for the temporary file
        
    Returns:
        Path to the temporary file
    """
    import tempfile
    import uuid
    
    # Create temporary directory if it doesn't exist
    temp_dir = os.path.join(tempfile.gettempdir(), "inference_temp")
    os.makedirs(temp_dir, exist_ok=True)
    
    # Generate unique filename
    filename = f"{prefix}_{uuid.uuid4().hex}{suffix}"
    
    # Return path to temporary file
    return os.path.join(temp_dir, filename)


def clean_temp_files(temp_dir: str = None, max_age: int = 3600) -> None:
    """
    Clean temporary files older than max_age seconds.
    
    Args:
        temp_dir: Directory containing temporary files
        max_age: Maximum age of temporary files in seconds
    """
    import tempfile
    import time
    
    # Get temporary directory
    if temp_dir is None:
        temp_dir = os.path.join(tempfile.gettempdir(), "inference_temp")
    
    # Check if directory exists
    if not os.path.exists(temp_dir):
        return
    
    # Get current time
    current_time = time.time()
    
    # Clean old files
    for file_path in Path(temp_dir).glob("*"):
        try:
            # Check if file is a regular file
            if not file_path.is_file():
                continue
            
            # Check file age
            file_age = current_time - os.path.getmtime(file_path)
            if file_age > max_age:
                os.remove(file_path)
                logger.debug(f"Removed old temporary file: {file_path}")
        except Exception as e:
            logger.error(f"Error cleaning temporary file {file_path}: {str(e)}")


def validate_model_format(file_path: str) -> bool:
    """
    Validate that a file is a supported model format.
    
    Args:
        file_path: Path to the model file
        
    Returns:
        True if the file is a supported model format, False otherwise
    """
    # Get file extension
    ext = os.path.splitext(file_path)[1].lower()
    
    # Check if extension is supported
    supported_extensions = [
        ".pt", ".pth",  # PyTorch
        ".onnx",        # ONNX
        ".tflite",      # TensorFlow Lite
        ".npz",         # TinyGrad
        ".safetensors", # SafeTensors
        ".pb", ".h5"    # TensorFlow
    ]
    
    return ext in supported_extensions


def safe_load_model(file_path: str, model_type: str = None) -> Any:
    """
    Safely load a model from a file.
    
    Args:
        file_path: Path to the model file
        model_type: Type of model to load
        
    Returns:
        Loaded model or None if loading failed
    """
    try:
        # Check if file is safe
        if not is_safe_file(file_path):
            logger.error(f"Unsafe file: {file_path}")
            return None
        
        # Determine model type if not provided
        if model_type is None:
            ext = os.path.splitext(file_path)[1].lower()
            if ext in [".pt", ".pth"]:
                model_type = "pytorch"
            elif ext == ".onnx":
                model_type = "onnx"
            elif ext == ".tflite":
                model_type = "tflite"
            elif ext == ".npz":
                model_type = "tinygrad"
            elif ext == ".safetensors":
                model_type = "safetensors"
            elif ext in [".pb", ".h5"]:
                model_type = "tensorflow"
            else:
                logger.error(f"Unsupported model format: {ext}")
                return None
        
        # Load model based on type
        if model_type == "pytorch":
            import torch
            
            # Load model with safety measures
            model = torch.load(file_path, map_location="cpu")
            return model
        
        elif model_type == "onnx":
            import onnx
            
            # Load ONNX model
            model = onnx.load(file_path)
            return model
        
        elif model_type == "tflite":
            import tensorflow as tf
            
            # Load TFLite model
            interpreter = tf.lite.Interpreter(model_path=file_path)
            interpreter.allocate_tensors()
            return interpreter
        
        elif model_type == "tinygrad":
            import numpy as np
            
            # Load TinyGrad model
            model = np.load(file_path)
            return model
        
        elif model_type == "safetensors":
            from safetensors import safe_open
            
            # Load SafeTensors model
            model = safe_open(file_path, framework="numpy")
            return model
        
        elif model_type == "tensorflow":
            import tensorflow as tf
            
            # Load TensorFlow model
            ext = os.path.splitext(file_path)[1].lower()
            if ext == ".h5":
                model = tf.keras.models.load_model(file_path)
            else:
                model = tf.saved_model.load(file_path)
            return model
        
        else:
            logger.error(f"Unsupported model type: {model_type}")
            return None
    
    except Exception as e:
        logger.error(f"Error loading model: {str(e)}")
        return None
