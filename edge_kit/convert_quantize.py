#!/usr/bin/env python3
"""
Bulo.CloudSentinel Edge Kit - Model Conversion and Quantization Utility

This script converts and quantizes AI models for optimal performance on edge devices.
Supports conversion to ONNX, TensorRT, and TFLite formats with INT8 quantization.
"""

import os
import sys
import argparse
import logging
from pathlib import Path
from typing import Optional, Dict, Any, List, Tuple
import shutil
import tempfile
import json
import numpy as np

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
)
logger = logging.getLogger("convert_quantize")

def parse_args() -> argparse.Namespace:
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(
        description="Convert and quantize models for edge deployment"
    )
    
    parser.add_argument(
        "--input", "-i", 
        required=True, 
        help="Input model path"
    )
    
    parser.add_argument(
        "--output", "-o", 
        required=True, 
        help="Output model path"
    )
    
    parser.add_argument(
        "--format", "-f", 
        choices=["onnx", "tensorrt", "tflite"], 
        default="onnx",
        help="Output format (default: onnx)"
    )
    
    parser.add_argument(
        "--quantize", "-q", 
        action="store_true", 
        help="Quantize the model to INT8"
    )
    
    parser.add_argument(
        "--calibration-data", 
        help="Path to calibration data for INT8 quantization"
    )
    
    parser.add_argument(
        "--device", 
        choices=["cpu", "cuda", "tensorrt"], 
        default="cpu",
        help="Device to use for conversion (default: cpu)"
    )
    
    parser.add_argument(
        "--input-shape", 
        help="Input shape in format 'n,c,h,w' (e.g., '1,3,640,640')"
    )
    
    parser.add_argument(
        "--model-type", 
        choices=["yolo", "classification", "segmentation", "detection", "custom"],
        default="yolo",
        help="Model type (default: yolo)"
    )
    
    parser.add_argument(
        "--config", 
        help="Path to model configuration file"
    )
    
    parser.add_argument(
        "--verbose", "-v", 
        action="store_true", 
        help="Enable verbose logging"
    )
    
    return parser.parse_args()

def check_dependencies() -> Dict[str, bool]:
    """Check if required dependencies are installed."""
    dependencies = {
        "onnx": False,
        "onnxruntime": False,
        "tensorflow": False,
        "torch": False,
        "tensorrt": False,
    }
    
    try:
        import onnx
        dependencies["onnx"] = True
    except ImportError:
        pass
    
    try:
        import onnxruntime
        dependencies["onnxruntime"] = True
    except ImportError:
        pass
    
    try:
        import tensorflow
        dependencies["tensorflow"] = True
    except ImportError:
        pass
    
    try:
        import torch
        dependencies["torch"] = True
    except ImportError:
        pass
    
    try:
        import tensorrt
        dependencies["tensorrt"] = True
    except ImportError:
        pass
    
    return dependencies

def convert_to_onnx(
    input_path: str, 
    output_path: str, 
    input_shape: Optional[List[int]] = None,
    model_type: str = "yolo",
    device: str = "cpu",
    config: Optional[str] = None,
) -> bool:
    """
    Convert a model to ONNX format.
    
    Args:
        input_path: Path to input model
        output_path: Path to save ONNX model
        input_shape: Input shape [batch_size, channels, height, width]
        model_type: Type of model (yolo, classification, etc.)
        device: Device to use for conversion (cpu, cuda)
        config: Path to model configuration file
        
    Returns:
        True if conversion was successful, False otherwise
    """
    try:
        # Determine input model format
        input_ext = Path(input_path).suffix.lower()
        
        if input_ext in [".pt", ".pth"]:
            # PyTorch model
            import torch
            
            logger.info(f"Converting PyTorch model to ONNX: {input_path} -> {output_path}")
            
            # Set device
            device_torch = torch.device(device if device == "cuda" and torch.cuda.is_available() else "cpu")
            
            # Load model
            model = torch.load(input_path, map_location=device_torch)
            
            # Ensure model is in eval mode
            model.eval()
            
            # Create dummy input
            if input_shape is None:
                if model_type == "yolo":
                    input_shape = [1, 3, 640, 640]
                else:
                    input_shape = [1, 3, 224, 224]
            
            dummy_input = torch.randn(*input_shape, device=device_torch)
            
            # Export to ONNX
            torch.onnx.export(
                model,
                dummy_input,
                output_path,
                export_params=True,
                opset_version=13,
                do_constant_folding=True,
                input_names=["input"],
                output_names=["output"],
                dynamic_axes={"input": {0: "batch_size"}, "output": {0: "batch_size"}},
            )
            
            logger.info(f"PyTorch model converted to ONNX successfully")
            return True
            
        elif input_ext == ".onnx":
            # Already ONNX, just copy
            logger.info(f"Model is already in ONNX format, copying: {input_path} -> {output_path}")
            shutil.copy(input_path, output_path)
            return True
            
        elif input_ext in [".tflite", ".pb", ".h5", ".keras"]:
            # TensorFlow model
            import tensorflow as tf
            
            logger.info(f"Converting TensorFlow model to ONNX: {input_path} -> {output_path}")
            
            # Load model
            if input_ext == ".tflite":
                # Convert TFLite to SavedModel first
                interpreter = tf.lite.Interpreter(model_path=input_path)
                interpreter.allocate_tensors()
                
                # Create a temporary directory for the SavedModel
                with tempfile.TemporaryDirectory() as tmpdir:
                    saved_model_path = os.path.join(tmpdir, "saved_model")
                    
                    # Convert TFLite to SavedModel
                    converter = tf.lite.TFLiteConverter.from_interpreter(interpreter)
                    saved_model = converter.convert()
                    
                    # Save the model
                    with open(os.path.join(tmpdir, "model.tflite"), "wb") as f:
                        f.write(saved_model)
                    
                    # Use tf2onnx to convert
                    import tf2onnx
                    
                    # Convert to ONNX
                    tf2onnx.convert.from_tflite(
                        input_path,
                        output_path=output_path,
                        opset=13,
                    )
            else:
                # Load SavedModel, H5, or Keras model
                model = tf.keras.models.load_model(input_path)
                
                # Use tf2onnx to convert
                import tf2onnx
                
                # Convert to ONNX
                tf2onnx.convert.from_keras(
                    model,
                    output_path=output_path,
                    opset=13,
                )
            
            logger.info(f"TensorFlow model converted to ONNX successfully")
            return True
            
        else:
            logger.error(f"Unsupported input format: {input_ext}")
            return False
            
    except Exception as e:
        logger.error(f"Error converting model to ONNX: {str(e)}")
        return False

def quantize_onnx(
    input_path: str, 
    output_path: str, 
    calibration_data: Optional[str] = None,
) -> bool:
    """
    Quantize an ONNX model to INT8.
    
    Args:
        input_path: Path to input ONNX model
        output_path: Path to save quantized ONNX model
        calibration_data: Path to calibration data
        
    Returns:
        True if quantization was successful, False otherwise
    """
    try:
        import onnx
        from onnxruntime.quantization import quantize_dynamic, QuantType
        
        logger.info(f"Quantizing ONNX model to INT8: {input_path} -> {output_path}")
        
        # Quantize model
        quantize_dynamic(
            input_path,
            output_path,
            weight_type=QuantType.QInt8,
        )
        
        logger.info(f"ONNX model quantized successfully")
        return True
        
    except Exception as e:
        logger.error(f"Error quantizing ONNX model: {str(e)}")
        return False

def convert_to_tensorrt(
    input_path: str, 
    output_path: str, 
    input_shape: Optional[List[int]] = None,
    quantize: bool = False,
    calibration_data: Optional[str] = None,
) -> bool:
    """
    Convert an ONNX model to TensorRT.
    
    Args:
        input_path: Path to input ONNX model
        output_path: Path to save TensorRT engine
        input_shape: Input shape [batch_size, channels, height, width]
        quantize: Whether to quantize the model to INT8
        calibration_data: Path to calibration data
        
    Returns:
        True if conversion was successful, False otherwise
    """
    try:
        import tensorrt as trt
        import pycuda.driver as cuda
        import pycuda.autoinit
        
        logger.info(f"Converting ONNX model to TensorRT: {input_path} -> {output_path}")
        
        # Create builder and network
        TRT_LOGGER = trt.Logger(trt.Logger.INFO)
        builder = trt.Builder(TRT_LOGGER)
        network = builder.create_network(1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH))
        parser = trt.OnnxParser(network, TRT_LOGGER)
        
        # Parse ONNX model
        with open(input_path, "rb") as f:
            if not parser.parse(f.read()):
                for error in range(parser.num_errors):
                    logger.error(f"ONNX parse error: {parser.get_error(error)}")
                return False
        
        # Create config
        config = builder.create_builder_config()
        config.max_workspace_size = 1 << 30  # 1GB
        
        # Set precision
        if quantize:
            config.set_flag(trt.BuilderFlag.INT8)
            
            # Set up calibrator if calibration data is provided
            if calibration_data:
                # Implement calibration here
                pass
        else:
            config.set_flag(trt.BuilderFlag.FP16)
        
        # Build engine
        engine = builder.build_engine(network, config)
        
        # Save engine
        with open(output_path, "wb") as f:
            f.write(engine.serialize())
        
        logger.info(f"ONNX model converted to TensorRT successfully")
        return True
        
    except Exception as e:
        logger.error(f"Error converting ONNX to TensorRT: {str(e)}")
        return False

def convert_to_tflite(
    input_path: str, 
    output_path: str, 
    quantize: bool = False,
    calibration_data: Optional[str] = None,
) -> bool:
    """
    Convert an ONNX model to TFLite.
    
    Args:
        input_path: Path to input ONNX model
        output_path: Path to save TFLite model
        quantize: Whether to quantize the model to INT8
        calibration_data: Path to calibration data
        
    Returns:
        True if conversion was successful, False otherwise
    """
    try:
        import tensorflow as tf
        import onnx
        from onnx_tf.backend import prepare
        
        logger.info(f"Converting ONNX model to TFLite: {input_path} -> {output_path}")
        
        # Load ONNX model
        onnx_model = onnx.load(input_path)
        
        # Convert ONNX to TensorFlow
        tf_rep = prepare(onnx_model)
        
        # Create a temporary directory for the SavedModel
        with tempfile.TemporaryDirectory() as tmpdir:
            saved_model_path = os.path.join(tmpdir, "saved_model")
            
            # Export the TensorFlow model
            tf_rep.export_graph(saved_model_path)
            
            # Load the SavedModel
            model = tf.saved_model.load(saved_model_path)
            
            # Convert to TFLite
            converter = tf.lite.TFLiteConverter.from_saved_model(saved_model_path)
            
            # Set optimization options
            if quantize:
                converter.optimizations = [tf.lite.Optimize.DEFAULT]
                converter.target_spec.supported_types = [tf.int8]
                
                # Set up representative dataset if calibration data is provided
                if calibration_data:
                    # Implement calibration here
                    pass
            
            # Convert the model
            tflite_model = converter.convert()
            
            # Save the model
            with open(output_path, "wb") as f:
                f.write(tflite_model)
        
        logger.info(f"ONNX model converted to TFLite successfully")
        return True
        
    except Exception as e:
        logger.error(f"Error converting ONNX to TFLite: {str(e)}")
        return False

def main():
    """Main function."""
    args = parse_args()
    
    # Set logging level
    if args.verbose:
        logger.setLevel(logging.DEBUG)
    
    # Check dependencies
    dependencies = check_dependencies()
    
    # Parse input shape if provided
    input_shape = None
    if args.input_shape:
        try:
            input_shape = [int(dim) for dim in args.input_shape.split(",")]
        except ValueError:
            logger.error(f"Invalid input shape: {args.input_shape}. Expected format: 'n,c,h,w'")
            sys.exit(1)
    
    # Create output directory if it doesn't exist
    os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
    
    # Convert to ONNX first if needed
    if args.format == "onnx" or args.format in ["tensorrt", "tflite"]:
        if not dependencies["onnx"] or not dependencies["onnxruntime"]:
            logger.error("ONNX and ONNX Runtime are required for ONNX conversion")
            sys.exit(1)
        
        # If output format is not ONNX, use a temporary file
        if args.format != "onnx":
            with tempfile.NamedTemporaryFile(suffix=".onnx", delete=False) as tmp:
                onnx_path = tmp.name
        else:
            onnx_path = args.output
        
        # Convert to ONNX
        success = convert_to_onnx(
            args.input,
            onnx_path,
            input_shape=input_shape,
            model_type=args.model_type,
            device=args.device,
            config=args.config,
        )
        
        if not success:
            logger.error("Failed to convert model to ONNX")
            if args.format != "onnx":
                os.unlink(onnx_path)
            sys.exit(1)
        
        # Quantize ONNX if requested
        if args.quantize and args.format == "onnx":
            if not quantize_onnx(onnx_path, args.output, args.calibration_data):
                logger.error("Failed to quantize ONNX model")
                if args.format != "onnx":
                    os.unlink(onnx_path)
                sys.exit(1)
        
        # Convert to final format if not ONNX
        if args.format == "tensorrt":
            if not dependencies["tensorrt"]:
                logger.error("TensorRT is required for TensorRT conversion")
                os.unlink(onnx_path)
                sys.exit(1)
            
            success = convert_to_tensorrt(
                onnx_path,
                args.output,
                input_shape=input_shape,
                quantize=args.quantize,
                calibration_data=args.calibration_data,
            )
            
            # Clean up temporary ONNX file
            os.unlink(onnx_path)
            
            if not success:
                logger.error("Failed to convert ONNX model to TensorRT")
                sys.exit(1)
                
        elif args.format == "tflite":
            if not dependencies["tensorflow"]:
                logger.error("TensorFlow is required for TFLite conversion")
                os.unlink(onnx_path)
                sys.exit(1)
            
            success = convert_to_tflite(
                onnx_path,
                args.output,
                quantize=args.quantize,
                calibration_data=args.calibration_data,
            )
            
            # Clean up temporary ONNX file
            os.unlink(onnx_path)
            
            if not success:
                logger.error("Failed to convert ONNX model to TFLite")
                sys.exit(1)
    
    logger.info(f"Model conversion completed successfully: {args.output}")

if __name__ == "__main__":
    main()
