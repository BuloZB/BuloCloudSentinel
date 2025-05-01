#!/usr/bin/env python3
"""
TinyGrad Demo for Bulo.Cloud Sentinel

This script demonstrates the use of TinyGrad as an inference backend for
Bulo.Cloud Sentinel. It downloads a small detection model, converts it to
TinyGrad format, and runs inference on a sample image.
"""

import os
import sys
import time
import argparse
import numpy as np
import cv2
from pathlib import Path
import urllib.request
import logging
from typing import Dict, Any, List, Tuple

# Add project root to path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

# Import inference engine
from ai.inference import InferenceEngine, TinygradInference

# Setup logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


def download_model(url: str, output_path: str) -> str:
    """
    Download a model from a URL.
    
    Args:
        url: URL to download from
        output_path: Path to save the model to
        
    Returns:
        Path to the downloaded model
    """
    # Create output directory if it doesn't exist
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    
    # Download the model if it doesn't exist
    if not os.path.exists(output_path):
        logger.info(f"Downloading model from {url} to {output_path}")
        urllib.request.urlretrieve(url, output_path)
        logger.info("Download complete")
    else:
        logger.info(f"Model already exists at {output_path}")
    
    return output_path


def load_image(image_path: str, target_size: Tuple[int, int] = (224, 224)) -> np.ndarray:
    """
    Load and preprocess an image for inference.
    
    Args:
        image_path: Path to the image file
        target_size: Target size for resizing
        
    Returns:
        Preprocessed image as a numpy array
    """
    # Load image
    img = cv2.imread(image_path)
    if img is None:
        raise ValueError(f"Failed to load image: {image_path}")
    
    # Resize image
    img = cv2.resize(img, target_size)
    
    # Convert to RGB
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    
    # Normalize to [0, 1]
    img = img.astype(np.float32) / 255.0
    
    # Add batch dimension
    img = np.expand_dims(img, axis=0)
    
    return img


def run_inference(engine: InferenceEngine, image: np.ndarray) -> Dict[str, Any]:
    """
    Run inference on an image.
    
    Args:
        engine: Inference engine
        image: Preprocessed image
        
    Returns:
        Inference results
    """
    # Prepare input
    inputs = {"input": image}
    
    # Run inference
    outputs = engine.predict(inputs)
    
    return outputs


def benchmark_inference(engine: InferenceEngine, image: np.ndarray, num_runs: int = 10) -> float:
    """
    Benchmark inference performance.
    
    Args:
        engine: Inference engine
        image: Preprocessed image
        num_runs: Number of inference runs
        
    Returns:
        Average inference time in milliseconds
    """
    # Prepare input
    inputs = {"input": image}
    
    # Warmup
    for _ in range(3):
        _ = engine.predict(inputs)
    
    # Benchmark
    total_time = 0.0
    for _ in range(num_runs):
        start_time = time.time()
        _ = engine.predict(inputs)
        total_time += (time.time() - start_time) * 1000  # Convert to ms
    
    return total_time / num_runs


def main():
    """Main function."""
    # Parse arguments
    parser = argparse.ArgumentParser(description="TinyGrad Demo for Bulo.Cloud Sentinel")
    parser.add_argument("--model", type=str, default="models/mobilenet_v2.npz",
                        help="Path to the model file")
    parser.add_argument("--image", type=str, default="examples/data/drone_image.jpg",
                        help="Path to the image file")
    parser.add_argument("--backend", type=str, default="tinygrad",
                        choices=["tinygrad", "torch", "tflite"],
                        help="Inference backend to use")
    parser.add_argument("--device", type=str, default="AUTO",
                        choices=["AUTO", "CPU", "CUDA", "OCL"],
                        help="Device to run inference on")
    parser.add_argument("--benchmark", action="store_true",
                        help="Run benchmark")
    parser.add_argument("--num-runs", type=int, default=10,
                        help="Number of inference runs for benchmarking")
    args = parser.parse_args()
    
    # Download model if it's a URL
    if args.model.startswith("http"):
        model_path = download_model(args.model, "models/downloaded_model.npz")
    else:
        model_path = args.model
    
    # Create output directory for the model if it doesn't exist
    os.makedirs(os.path.dirname(model_path), exist_ok=True)
    
    # Download a sample model if it doesn't exist
    if not os.path.exists(model_path):
        # URL for a sample MobileNetV2 model converted to NPZ format
        model_url = "https://github.com/tinygrad/tinygrad/raw/master/weights/mobilenet_v2.npz"
        model_path = download_model(model_url, model_path)
    
    # Create examples/data directory if it doesn't exist
    os.makedirs(os.path.dirname(args.image), exist_ok=True)
    
    # Download a sample image if it doesn't exist
    if not os.path.exists(args.image):
        # URL for a sample drone image
        image_url = "https://raw.githubusercontent.com/ultralytics/yolov5/master/data/images/zidane.jpg"
        args.image = download_model(image_url, args.image)
    
    # Initialize inference engine
    logger.info(f"Initializing {args.backend} backend on {args.device}")
    engine = InferenceEngine(backend=args.backend, model_path=model_path, device=args.device)
    
    # Load and preprocess image
    logger.info(f"Loading image from {args.image}")
    image = load_image(args.image)
    
    # Run inference
    logger.info("Running inference")
    outputs = run_inference(engine, image)
    
    # Print results
    logger.info(f"Inference time: {engine.get_last_inference_time():.2f} ms")
    
    # Print top-1 class (for classification models)
    if "output" in outputs:
        output = outputs["output"]
        if len(output.shape) == 2 and output.shape[1] > 1:
            # Classification output
            top_class = np.argmax(output[0])
            confidence = output[0][top_class]
            logger.info(f"Top-1 class: {top_class} (confidence: {confidence:.4f})")
    
    # Run benchmark if requested
    if args.benchmark:
        logger.info(f"Running benchmark with {args.num_runs} inference runs")
        avg_time = benchmark_inference(engine, image, args.num_runs)
        logger.info(f"Average inference time: {avg_time:.2f} ms")
        logger.info(f"FPS: {1000 / avg_time:.2f}")


if __name__ == "__main__":
    main()
