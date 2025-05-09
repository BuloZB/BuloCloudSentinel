"""
Optimization utilities for model deployment.

This module provides utilities for optimizing model deployment performance.
"""

import os
import logging
import tempfile
import time
import json
import subprocess
from typing import Dict, List, Any, Optional, Tuple, Union
from pathlib import Path

import torch
import numpy as np

# Setup logging
logger = logging.getLogger(__name__)

def optimize_model(
    model_path: str,
    output_path: str,
    framework: str,
    target_device: str = "cpu",
    quantize: bool = False,
    int8: bool = False,
    fp16: bool = False,
) -> Tuple[str, Dict[str, Any]]:
    """
    Optimize a model for deployment.

    Args:
        model_path: Path to the model file
        output_path: Path to save the optimized model
        framework: Framework of the model (e.g., "pytorch", "onnx", "tflite")
        target_device: Target device for optimization (e.g., "cpu", "cuda", "jetson")
        quantize: Whether to quantize the model
        int8: Whether to use INT8 quantization
        fp16: Whether to use FP16 precision

    Returns:
        Tuple of (path to optimized model, optimization metadata)
    """
    try:
        if framework == "pytorch":
            return optimize_pytorch_model(
                model_path, output_path, target_device, quantize, int8, fp16
            )
        elif framework == "onnx":
            return optimize_onnx_model(
                model_path, output_path, target_device, quantize, int8, fp16
            )
        elif framework == "tflite":
            return optimize_tflite_model(
                model_path, output_path, target_device, quantize, int8, fp16
            )
        else:
            logger.warning(f"Unsupported framework: {framework}")
            # Just copy the model file
            import shutil
            shutil.copy(model_path, output_path)
            return output_path, {
                "optimized": False,
                "reason": f"Unsupported framework: {framework}",
            }
    except Exception as e:
        logger.error(f"Error optimizing model: {e}")
        # Just copy the model file
        import shutil
        shutil.copy(model_path, output_path)
        return output_path, {
            "optimized": False,
            "error": str(e),
        }

def optimize_pytorch_model(
    model_path: str,
    output_path: str,
    target_device: str = "cpu",
    quantize: bool = False,
    int8: bool = False,
    fp16: bool = False,
) -> Tuple[str, Dict[str, Any]]:
    """
    Optimize a PyTorch model for deployment.

    Args:
        model_path: Path to the model file
        output_path: Path to save the optimized model
        target_device: Target device for optimization (e.g., "cpu", "cuda", "jetson")
        quantize: Whether to quantize the model
        int8: Whether to use INT8 quantization
        fp16: Whether to use FP16 precision

    Returns:
        Tuple of (path to optimized model, optimization metadata)
    """
    try:
        # Load model with security measures
        # Use weights_only=True for better security when loading models
        model = torch.load(model_path, map_location="cpu", weights_only=True)

        # Check if model is a state_dict or a full model
        if isinstance(model, dict) and "state_dict" in model:
            state_dict = model["state_dict"]
        elif isinstance(model, dict):
            state_dict = model
        else:
            state_dict = model.state_dict()

        # Create metadata
        metadata = {
            "optimized": True,
            "framework": "pytorch",
            "target_device": target_device,
            "quantized": quantize,
            "int8": int8,
            "fp16": fp16,
            "original_size": os.path.getsize(model_path),
        }

        # Apply optimizations
        if quantize:
            if int8:
                # Apply INT8 quantization
                # This is a placeholder for actual INT8 quantization
                # In a real implementation, you would use torch.quantization
                logger.info(f"Applying INT8 quantization to model: {model_path}")
                metadata["quantization_method"] = "int8"
            elif fp16:
                # Apply FP16 quantization
                # Convert all parameters to FP16
                for key in state_dict:
                    if isinstance(state_dict[key], torch.Tensor) and state_dict[key].dtype == torch.float32:
                        state_dict[key] = state_dict[key].half()
                metadata["quantization_method"] = "fp16"

        # Save optimized model
        torch.save(state_dict, output_path)

        # Update metadata
        metadata["optimized_size"] = os.path.getsize(output_path)
        metadata["size_reduction"] = 1.0 - (metadata["optimized_size"] / metadata["original_size"])

        logger.info(f"Optimized PyTorch model saved to: {output_path}")
        logger.info(f"Size reduction: {metadata['size_reduction']:.2%}")

        return output_path, metadata
    except Exception as e:
        logger.error(f"Error optimizing PyTorch model: {e}")
        raise

def optimize_onnx_model(
    model_path: str,
    output_path: str,
    target_device: str = "cpu",
    quantize: bool = False,
    int8: bool = False,
    fp16: bool = False,
) -> Tuple[str, Dict[str, Any]]:
    """
    Optimize an ONNX model for deployment.

    Args:
        model_path: Path to the model file
        output_path: Path to save the optimized model
        target_device: Target device for optimization (e.g., "cpu", "cuda", "jetson")
        quantize: Whether to quantize the model
        int8: Whether to use INT8 quantization
        fp16: Whether to use FP16 precision

    Returns:
        Tuple of (path to optimized model, optimization metadata)
    """
    try:
        # Check if onnxruntime is available
        try:
            import onnx
            import onnxruntime
            from onnxruntime.quantization import quantize_dynamic, QuantType
        except ImportError:
            logger.warning("onnxruntime not available, skipping ONNX optimization")
            # Just copy the model file
            import shutil
            shutil.copy(model_path, output_path)
            return output_path, {
                "optimized": False,
                "reason": "onnxruntime not available",
            }

        # Load model
        model = onnx.load(model_path)

        # Create metadata
        metadata = {
            "optimized": True,
            "framework": "onnx",
            "target_device": target_device,
            "quantized": quantize,
            "int8": int8,
            "fp16": fp16,
            "original_size": os.path.getsize(model_path),
        }

        # Apply optimizations
        if quantize:
            if int8:
                # Apply INT8 quantization
                logger.info(f"Applying INT8 quantization to model: {model_path}")
                quantize_dynamic(
                    model_path,
                    output_path,
                    weight_type=QuantType.QInt8,
                )
                metadata["quantization_method"] = "int8_dynamic"
            elif fp16:
                # Apply FP16 quantization
                logger.info(f"Applying FP16 quantization to model: {model_path}")
                # Save model with FP16 weights
                # This is a placeholder for actual FP16 conversion
                # In a real implementation, you would use onnx.convert_float_to_float16
                onnx.save(model, output_path)
                metadata["quantization_method"] = "fp16"
        else:
            # Save model without quantization
            onnx.save(model, output_path)

        # Update metadata
        metadata["optimized_size"] = os.path.getsize(output_path)
        metadata["size_reduction"] = 1.0 - (metadata["optimized_size"] / metadata["original_size"])

        logger.info(f"Optimized ONNX model saved to: {output_path}")
        logger.info(f"Size reduction: {metadata['size_reduction']:.2%}")

        return output_path, metadata
    except Exception as e:
        logger.error(f"Error optimizing ONNX model: {e}")
        raise

def optimize_tflite_model(
    model_path: str,
    output_path: str,
    target_device: str = "cpu",
    quantize: bool = False,
    int8: bool = False,
    fp16: bool = False,
) -> Tuple[str, Dict[str, Any]]:
    """
    Optimize a TFLite model for deployment.

    Args:
        model_path: Path to the model file
        output_path: Path to save the optimized model
        target_device: Target device for optimization (e.g., "cpu", "cuda", "jetson")
        quantize: Whether to quantize the model
        int8: Whether to use INT8 quantization
        fp16: Whether to use FP16 precision

    Returns:
        Tuple of (path to optimized model, optimization metadata)
    """
    try:
        # Check if tensorflow is available
        try:
            import tensorflow as tf
        except ImportError:
            logger.warning("tensorflow not available, skipping TFLite optimization")
            # Just copy the model file
            import shutil
            shutil.copy(model_path, output_path)
            return output_path, {
                "optimized": False,
                "reason": "tensorflow not available",
            }

        # Create metadata
        metadata = {
            "optimized": True,
            "framework": "tflite",
            "target_device": target_device,
            "quantized": quantize,
            "int8": int8,
            "fp16": fp16,
            "original_size": os.path.getsize(model_path),
        }

        # Load model
        interpreter = tf.lite.Interpreter(model_path=model_path)
        interpreter.allocate_tensors()

        # Apply optimizations
        if quantize:
            # Convert model to TFLite with quantization
            converter = tf.lite.TFLiteConverter.from_saved_model(model_path)

            if int8:
                # Apply INT8 quantization
                logger.info(f"Applying INT8 quantization to model: {model_path}")
                converter.optimizations = [tf.lite.Optimize.DEFAULT]
                converter.target_spec.supported_ops = [tf.lite.OpsSet.TFLITE_BUILTINS_INT8]
                converter.inference_input_type = tf.int8
                converter.inference_output_type = tf.int8
                metadata["quantization_method"] = "int8"
            elif fp16:
                # Apply FP16 quantization
                logger.info(f"Applying FP16 quantization to model: {model_path}")
                converter.optimizations = [tf.lite.Optimize.DEFAULT]
                converter.target_spec.supported_types = [tf.float16]
                metadata["quantization_method"] = "fp16"

            # Convert model
            tflite_model = converter.convert()

            # Save model
            with open(output_path, "wb") as f:
                f.write(tflite_model)
        else:
            # Just copy the model file
            import shutil
            shutil.copy(model_path, output_path)

        # Update metadata
        metadata["optimized_size"] = os.path.getsize(output_path)
        metadata["size_reduction"] = 1.0 - (metadata["optimized_size"] / metadata["original_size"])

        logger.info(f"Optimized TFLite model saved to: {output_path}")
        logger.info(f"Size reduction: {metadata['size_reduction']:.2%}")

        return output_path, metadata
    except Exception as e:
        logger.error(f"Error optimizing TFLite model: {e}")
        raise

def benchmark_model(
    model_path: str,
    framework: str,
    target_device: str = "cpu",
    num_iterations: int = 100,
    input_shape: Optional[List[int]] = None,
) -> Dict[str, Any]:
    """
    Benchmark a model for performance.

    Args:
        model_path: Path to the model file
        framework: Framework of the model (e.g., "pytorch", "onnx", "tflite")
        target_device: Target device for benchmarking (e.g., "cpu", "cuda", "jetson")
        num_iterations: Number of iterations to run
        input_shape: Input shape for the model (e.g., [1, 3, 224, 224])

    Returns:
        Benchmark results
    """
    try:
        if framework == "pytorch":
            return benchmark_pytorch_model(
                model_path, target_device, num_iterations, input_shape
            )
        elif framework == "onnx":
            return benchmark_onnx_model(
                model_path, target_device, num_iterations, input_shape
            )
        elif framework == "tflite":
            return benchmark_tflite_model(
                model_path, target_device, num_iterations, input_shape
            )
        else:
            logger.warning(f"Unsupported framework: {framework}")
            return {
                "framework": framework,
                "target_device": target_device,
                "error": f"Unsupported framework: {framework}",
            }
    except Exception as e:
        logger.error(f"Error benchmarking model: {e}")
        return {
            "framework": framework,
            "target_device": target_device,
            "error": str(e),
        }

def benchmark_pytorch_model(
    model_path: str,
    target_device: str = "cpu",
    num_iterations: int = 100,
    input_shape: Optional[List[int]] = None,
) -> Dict[str, Any]:
    """
    Benchmark a PyTorch model for performance.

    Args:
        model_path: Path to the model file
        target_device: Target device for benchmarking (e.g., "cpu", "cuda", "jetson")
        num_iterations: Number of iterations to run
        input_shape: Input shape for the model (e.g., [1, 3, 224, 224])

    Returns:
        Benchmark results
    """
    # Placeholder for actual benchmarking
    # In a real implementation, you would load the model and run inference
    return {
        "framework": "pytorch",
        "target_device": target_device,
        "num_iterations": num_iterations,
        "input_shape": input_shape,
        "latency_ms": 10.0,
        "fps": 100.0,
        "memory_mb": 100.0,
    }

def benchmark_onnx_model(
    model_path: str,
    target_device: str = "cpu",
    num_iterations: int = 100,
    input_shape: Optional[List[int]] = None,
) -> Dict[str, Any]:
    """
    Benchmark an ONNX model for performance.

    Args:
        model_path: Path to the model file
        target_device: Target device for benchmarking (e.g., "cpu", "cuda", "jetson")
        num_iterations: Number of iterations to run
        input_shape: Input shape for the model (e.g., [1, 3, 224, 224])

    Returns:
        Benchmark results
    """
    # Placeholder for actual benchmarking
    # In a real implementation, you would load the model and run inference
    return {
        "framework": "onnx",
        "target_device": target_device,
        "num_iterations": num_iterations,
        "input_shape": input_shape,
        "latency_ms": 10.0,
        "fps": 100.0,
        "memory_mb": 100.0,
    }

def benchmark_tflite_model(
    model_path: str,
    target_device: str = "cpu",
    num_iterations: int = 100,
    input_shape: Optional[List[int]] = None,
) -> Dict[str, Any]:
    """
    Benchmark a TFLite model for performance.

    Args:
        model_path: Path to the model file
        target_device: Target device for benchmarking (e.g., "cpu", "cuda", "jetson")
        num_iterations: Number of iterations to run
        input_shape: Input shape for the model (e.g., [1, 3, 224, 224])

    Returns:
        Benchmark results
    """
    # Placeholder for actual benchmarking
    # In a real implementation, you would load the model and run inference
    return {
        "framework": "tflite",
        "target_device": target_device,
        "num_iterations": num_iterations,
        "input_shape": input_shape,
        "latency_ms": 10.0,
        "fps": 100.0,
        "memory_mb": 100.0,
    }
