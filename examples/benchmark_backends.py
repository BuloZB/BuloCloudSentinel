#!/usr/bin/env python3
"""
Benchmark script for comparing inference backends in Bulo.Cloud Sentinel.

This script benchmarks the performance of different inference backends
(TinyGrad, PyTorch, TFLite) on various hardware platforms.
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
import json
import platform
import subprocess
from tabulate import tabulate

# Add project root to path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

# Import inference engine
from ai.inference import InferenceEngine

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


def download_image(url: str, output_path: str) -> str:
    """
    Download an image from a URL.
    
    Args:
        url: URL to download from
        output_path: Path to save the image to
        
    Returns:
        Path to the downloaded image
    """
    # Create output directory if it doesn't exist
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    
    # Download the image if it doesn't exist
    if not os.path.exists(output_path):
        logger.info(f"Downloading image from {url} to {output_path}")
        urllib.request.urlretrieve(url, output_path)
        logger.info("Download complete")
    else:
        logger.info(f"Image already exists at {output_path}")
    
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
    
    # Transpose to [batch, channels, height, width] for PyTorch models
    img = np.transpose(img, (0, 3, 1, 2))
    
    return img


def get_system_info() -> Dict[str, str]:
    """
    Get system information.
    
    Returns:
        Dictionary with system information
    """
    info = {
        "platform": platform.platform(),
        "processor": platform.processor(),
        "python_version": platform.python_version(),
        "system": platform.system(),
        "release": platform.release(),
        "machine": platform.machine(),
    }
    
    # Get GPU information if available
    try:
        if platform.system() == "Linux":
            # Try to get NVIDIA GPU info
            try:
                nvidia_smi = subprocess.check_output(["nvidia-smi", "--query-gpu=name,memory.total", "--format=csv,noheader"]).decode("utf-8").strip()
                info["gpu"] = nvidia_smi
            except (subprocess.SubprocessError, FileNotFoundError):
                # Try to get OpenCL info
                try:
                    clinfo = subprocess.check_output(["clinfo", "-l"]).decode("utf-8").strip()
                    info["gpu"] = clinfo
                except (subprocess.SubprocessError, FileNotFoundError):
                    info["gpu"] = "Unknown"
        elif platform.system() == "Windows":
            # Try to get NVIDIA GPU info on Windows
            try:
                nvidia_smi = subprocess.check_output(["nvidia-smi", "--query-gpu=name,memory.total", "--format=csv,noheader"]).decode("utf-8").strip()
                info["gpu"] = nvidia_smi
            except (subprocess.SubprocessError, FileNotFoundError):
                info["gpu"] = "Unknown"
        else:
            info["gpu"] = "Unknown"
    except Exception as e:
        logger.warning(f"Error getting GPU info: {str(e)}")
        info["gpu"] = "Unknown"
    
    return info


def benchmark_backend(backend: str, model_path: str, image: np.ndarray, device: str = "AUTO", num_runs: int = 10) -> Dict[str, Any]:
    """
    Benchmark a backend.
    
    Args:
        backend: Backend to benchmark
        model_path: Path to the model file
        image: Preprocessed image
        device: Device to run inference on
        num_runs: Number of inference runs
        
    Returns:
        Dictionary with benchmark results
    """
    try:
        # Initialize engine
        logger.info(f"Initializing {backend} backend on {device}")
        engine = InferenceEngine(backend=backend, model_path=model_path, device=device)
        
        # Prepare input
        inputs = {"input": image}
        
        # Warmup
        logger.info("Warming up...")
        for _ in range(3):
            _ = engine.predict(inputs)
        
        # Benchmark
        logger.info(f"Running benchmark with {num_runs} inference runs")
        inference_times = []
        for _ in range(num_runs):
            start_time = time.time()
            _ = engine.predict(inputs)
            inference_time = (time.time() - start_time) * 1000  # Convert to ms
            inference_times.append(inference_time)
        
        # Calculate statistics
        avg_time = sum(inference_times) / len(inference_times)
        min_time = min(inference_times)
        max_time = max(inference_times)
        fps = 1000 / avg_time
        
        # Return results
        return {
            "backend": backend,
            "device": engine.device,
            "avg_time": avg_time,
            "min_time": min_time,
            "max_time": max_time,
            "fps": fps,
            "num_runs": num_runs
        }
    except Exception as e:
        logger.error(f"Error benchmarking {backend} backend: {str(e)}")
        return {
            "backend": backend,
            "device": device,
            "error": str(e)
        }


def main():
    """Main function."""
    parser = argparse.ArgumentParser(description="Benchmark inference backends for Bulo.Cloud Sentinel")
    parser.add_argument("--backends", type=str, default="tinygrad,torch,tflite",
                        help="Comma-separated list of backends to benchmark")
    parser.add_argument("--device", type=str, default="AUTO",
                        choices=["AUTO", "CPU", "CUDA", "OCL"],
                        help="Device to run inference on")
    parser.add_argument("--num-runs", type=int, default=10,
                        help="Number of inference runs for benchmarking")
    parser.add_argument("--output", type=str, default="benchmark_results.json",
                        help="Path to save benchmark results")
    args = parser.parse_args()
    
    # Get backends to benchmark
    backends = args.backends.split(",")
    
    # Download models
    models = {
        "tinygrad": "models/mobilenet_v2.npz",
        "torch": "models/mobilenet_v2.pt",
        "tflite": "models/mobilenet_v2.tflite"
    }
    
    for backend, model_path in models.items():
        if backend in backends:
            # Create output directory if it doesn't exist
            os.makedirs(os.path.dirname(model_path), exist_ok=True)
            
            # Download model if it doesn't exist
            if not os.path.exists(model_path):
                if backend == "tinygrad":
                    # URL for a sample MobileNetV2 model converted to NPZ format
                    model_url = "https://github.com/tinygrad/tinygrad/raw/master/weights/mobilenet_v2.npz"
                    download_model(model_url, model_path)
                elif backend == "torch":
                    # URL for a sample MobileNetV2 model in PyTorch format
                    model_url = "https://download.pytorch.org/models/mobilenet_v2-b0353104.pth"
                    download_model(model_url, model_path)
                elif backend == "tflite":
                    # URL for a sample MobileNetV2 model in TFLite format
                    model_url = "https://storage.googleapis.com/download.tensorflow.org/models/tflite_11_05_08/mobilenet_v2_1.0_224.tgz"
                    tgz_path = "models/mobilenet_v2.tgz"
                    download_model(model_url, tgz_path)
                    
                    # Extract TFLite model from TGZ
                    import tarfile
                    with tarfile.open(tgz_path, "r:gz") as tar:
                        for member in tar.getmembers():
                            if member.name.endswith(".tflite"):
                                member.name = os.path.basename(member.name)
                                tar.extract(member, os.path.dirname(model_path))
                                os.rename(
                                    os.path.join(os.path.dirname(model_path), member.name),
                                    model_path
                                )
                                break
    
    # Download a sample image
    image_path = "examples/data/sample_image.jpg"
    image_url = "https://raw.githubusercontent.com/ultralytics/yolov5/master/data/images/bus.jpg"
    download_image(image_url, image_path)
    
    # Load and preprocess image
    image = load_image(image_path)
    
    # Get system information
    system_info = get_system_info()
    logger.info(f"System information: {json.dumps(system_info, indent=2)}")
    
    # Run benchmarks
    results = []
    for backend in backends:
        if backend in models:
            model_path = models[backend]
            if os.path.exists(model_path):
                result = benchmark_backend(
                    backend=backend,
                    model_path=model_path,
                    image=image,
                    device=args.device,
                    num_runs=args.num_runs
                )
                results.append(result)
            else:
                logger.warning(f"Model for {backend} backend not found at {model_path}")
        else:
            logger.warning(f"Unknown backend: {backend}")
    
    # Print results
    table_data = []
    for result in results:
        if "error" in result:
            table_data.append([
                result["backend"],
                result["device"],
                "Error",
                "Error",
                "Error",
                result["error"]
            ])
        else:
            table_data.append([
                result["backend"],
                result["device"],
                f"{result['avg_time']:.2f} ms",
                f"{result['min_time']:.2f} ms",
                f"{result['max_time']:.2f} ms",
                f"{result['fps']:.2f}"
            ])
    
    print("\nBenchmark Results:")
    print(tabulate(
        table_data,
        headers=["Backend", "Device", "Avg Time", "Min Time", "Max Time", "FPS"],
        tablefmt="grid"
    ))
    
    # Save results
    output_data = {
        "system_info": system_info,
        "results": results
    }
    
    with open(args.output, "w") as f:
        json.dump(output_data, f, indent=2)
    
    logger.info(f"Benchmark results saved to {args.output}")


if __name__ == "__main__":
    main()
