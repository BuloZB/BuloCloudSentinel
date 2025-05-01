"""
Inference module for Bulo.Cloud Sentinel.

This module provides a unified interface for running inference with different ML backends.
It supports model conversion, batch inference, distributed inference, and visualization.
"""

from .base import InferenceBackend
from .engine import InferenceEngine
from .tinygrad_backend import TinygradInference
from .torch_backend import TorchInference
from .tflite_backend import TFLiteInference
from .batch_engine import BatchInferenceEngine, AsyncBatchInferenceEngine, StreamingInferenceEngine
from .distributed_engine import DistributedInferenceEngine, WorkerServer

__all__ = [
    # Base classes
    "InferenceBackend",
    "InferenceEngine",

    # Backend implementations
    "TinygradInference",
    "TorchInference",
    "TFLiteInference",

    # Batch inference
    "BatchInferenceEngine",
    "AsyncBatchInferenceEngine",
    "StreamingInferenceEngine",

    # Distributed inference
    "DistributedInferenceEngine",
    "WorkerServer",
]

# Utility functions
from .convert import (
    convert_torch_to_tinygrad,
    convert_onnx_to_tinygrad,
    convert_tflite_to_tinygrad,
    convert_tensorflow_to_tinygrad,
    convert_to_safetensors,
    convert_torch_to_onnx,
    convert_onnx_to_tflite,
    optimize_model,
    quantize_model,
    print_model_info
)

# Visualization functions
from .visualize import (
    visualize_model_architecture,
    visualize_model_weights,
    visualize_model_activations,
    visualize_model_gradients
)
