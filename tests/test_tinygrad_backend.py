#!/usr/bin/env python3
"""
Tests for the TinyGrad inference backend.

This module contains tests for the TinyGrad inference backend.
"""

import os
import sys
import pytest
import numpy as np
from pathlib import Path
import tempfile
import urllib.request

# Add project root to path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

# Import inference engine
from ai.inference import InferenceEngine, TinygradInference


class TestTinygradBackend:
    """Tests for the TinyGrad inference backend."""
    
    @pytest.fixture
    def model_path(self):
        """Fixture to download a test model."""
        # Create a temporary directory
        with tempfile.TemporaryDirectory() as temp_dir:
            # Path to the model file
            model_path = os.path.join(temp_dir, "test_model.npz")
            
            # Create a dummy model file
            # In a real test, you would download a real model
            # For now, we'll just create a dummy NPZ file
            np.savez(model_path, 
                     weight1=np.random.random((10, 10)).astype(np.float32),
                     weight2=np.random.random((10, 10)).astype(np.float32))
            
            yield model_path
    
    def test_engine_initialization(self, model_path):
        """Test that the inference engine can be initialized."""
        # Initialize engine with CPU device to avoid GPU requirements in CI
        engine = InferenceEngine(backend="tinygrad", model_path=model_path, device="CPU")
        assert engine is not None
        assert engine.backend_name == "tinygrad"
        assert engine.model_path == model_path
        assert engine.device == "CPU"
    
    def test_backend_initialization(self, model_path):
        """Test that the TinyGrad backend can be initialized."""
        # Initialize backend with CPU device to avoid GPU requirements in CI
        try:
            backend = TinygradInference(model_path=model_path, device="CPU")
            assert backend is not None
            assert backend.device == "CPU"
            assert backend.model_path == Path(model_path)
        except ImportError:
            pytest.skip("TinyGrad not installed")
    
    def test_get_available_backends(self):
        """Test that the available backends can be retrieved."""
        backends = InferenceEngine.get_available_backends()
        assert isinstance(backends, list)
        assert "tinygrad" in backends
        assert "torch" in backends
        assert "tflite" in backends
    
    def test_get_backend_from_file(self):
        """Test that the appropriate backend can be determined from a file extension."""
        assert InferenceEngine.get_backend_from_file("model.npz") == "tinygrad"
        assert InferenceEngine.get_backend_from_file("model.safetensors") == "tinygrad"
        assert InferenceEngine.get_backend_from_file("model.pt") == "torch"
        assert InferenceEngine.get_backend_from_file("model.pth") == "torch"
        assert InferenceEngine.get_backend_from_file("model.onnx") == "torch"
        assert InferenceEngine.get_backend_from_file("model.tflite") == "tflite"
        assert InferenceEngine.get_backend_from_file("model.unknown") == "torch"  # Default


if __name__ == "__main__":
    pytest.main(["-xvs", __file__])
