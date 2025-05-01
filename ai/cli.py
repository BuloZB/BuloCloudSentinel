#!/usr/bin/env python3
"""
Command-line interface for Bulo.Cloud Sentinel AI.

This module provides a command-line interface for running inference with
different ML backends.
"""

import os
import sys
import argparse
import logging
from typing import Dict, Any, Optional, Union, List, Tuple
import numpy as np
import cv2
from pathlib import Path

# Add project root to path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

# Import inference engine
from ai.inference import InferenceEngine

# Setup logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


def main():
    """Main function."""
    parser = argparse.ArgumentParser(description="Bulo.Cloud Sentinel AI CLI")
    
    # Create subparsers
    subparsers = parser.add_subparsers(dest="command", help="Command to run")
    
    # List backends command
    list_backends_parser = subparsers.add_parser("list-backends", help="List available backends")
    
    # Run inference command
    run_parser = subparsers.add_parser("run", help="Run inference")
    run_parser.add_argument("--model", type=str, required=True,
                           help="Path to the model file")
    run_parser.add_argument("--image", type=str, required=True,
                           help="Path to the image file")
    run_parser.add_argument("--backend", type=str, default=os.environ.get("ML_BACKEND", "torch"),
                           choices=["tinygrad", "torch", "tflite"],
                           help="Inference backend to use")
    run_parser.add_argument("--device", type=str, default=os.environ.get("DEVICE", "AUTO"),
                           choices=["AUTO", "CPU", "CUDA", "OCL"],
                           help="Device to run inference on")
    run_parser.add_argument("--output", type=str, default=None,
                           help="Path to save output image")
    
    # Convert model command
    convert_parser = subparsers.add_parser("convert", help="Convert model to another format")
    convert_parser.add_argument("--input", type=str, required=True,
                              help="Path to the input model file")
    convert_parser.add_argument("--output", type=str, required=True,
                              help="Path to save the converted model file")
    convert_parser.add_argument("--format", type=str, default="tinygrad",
                              choices=["tinygrad", "safetensors"],
                              help="Output format")
    
    # Parse arguments
    args = parser.parse_args()
    
    # Handle commands
    if args.command == "list-backends":
        # List available backends
        backends = InferenceEngine.get_available_backends()
        print("Available backends:")
        for backend in backends:
            print(f"  - {backend}")
    
    elif args.command == "run":
        # Run inference
        try:
            # Check if model file exists
            if not os.path.exists(args.model):
                logger.error(f"Model file not found: {args.model}")
                sys.exit(1)
            
            # Check if image file exists
            if not os.path.exists(args.image):
                logger.error(f"Image file not found: {args.image}")
                sys.exit(1)
            
            # Initialize engine
            logger.info(f"Initializing {args.backend} backend on {args.device}")
            engine = InferenceEngine(backend=args.backend, model_path=args.model, device=args.device)
            
            # Load and preprocess image
            logger.info(f"Loading image from {args.image}")
            img = cv2.imread(args.image)
            if img is None:
                logger.error(f"Failed to load image: {args.image}")
                sys.exit(1)
            
            # Resize image
            img = cv2.resize(img, (224, 224))
            
            # Convert to RGB
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            
            # Normalize to [0, 1]
            img = img.astype(np.float32) / 255.0
            
            # Add batch dimension
            img = np.expand_dims(img, axis=0)
            
            # Transpose to [batch, channels, height, width] for PyTorch models
            img = np.transpose(img, (0, 3, 1, 2))
            
            # Run inference
            logger.info("Running inference")
            outputs = engine.predict({"input": img})
            
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
            
            # Save output image if requested
            if args.output:
                # This is a simplified implementation for demonstration purposes
                # In a real implementation, you would visualize the model outputs
                logger.info(f"Saving output image to {args.output}")
                cv2.imwrite(args.output, cv2.imread(args.image))
        
        except Exception as e:
            logger.error(f"Error running inference: {str(e)}")
            sys.exit(1)
    
    elif args.command == "convert":
        # Convert model
        try:
            # Check if input file exists
            if not os.path.exists(args.input):
                logger.error(f"Input file not found: {args.input}")
                sys.exit(1)
            
            # Create output directory if it doesn't exist
            os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
            
            # Import convert module
            from ai.inference.convert import convert_torch_to_tinygrad, convert_onnx_to_tinygrad, convert_tflite_to_tinygrad, convert_to_safetensors
            
            # Determine input format
            input_ext = Path(args.input).suffix.lower()
            
            # Convert model based on input format and desired output format
            if args.format == "tinygrad":
                if input_ext in [".pt", ".pth"]:
                    convert_torch_to_tinygrad(args.input, args.output)
                elif input_ext == ".onnx":
                    convert_onnx_to_tinygrad(args.input, args.output)
                elif input_ext == ".tflite":
                    convert_tflite_to_tinygrad(args.input, args.output)
                else:
                    logger.error(f"Unsupported input format for tinygrad conversion: {input_ext}")
                    sys.exit(1)
            elif args.format == "safetensors":
                convert_to_safetensors(args.input, args.output)
            else:
                logger.error(f"Unsupported output format: {args.format}")
                sys.exit(1)
            
            logger.info(f"Model converted successfully to {args.output}")
        
        except Exception as e:
            logger.error(f"Error converting model: {str(e)}")
            sys.exit(1)
    
    else:
        # No command specified
        parser.print_help()


if __name__ == "__main__":
    main()
