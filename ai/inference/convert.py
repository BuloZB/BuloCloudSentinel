#!/usr/bin/env python3
"""
Model conversion utility for Bulo.Cloud Sentinel.

This script provides utilities for converting models to formats compatible with
different inference backends. It supports conversion between various formats:
- PyTorch (.pt, .pth) to TinyGrad (.npz)
- ONNX (.onnx) to TinyGrad (.npz)
- TensorFlow Lite (.tflite) to TinyGrad (.npz)
- TensorFlow (.pb, .h5) to TinyGrad (.npz)
- Any supported format to SafeTensors (.safetensors)
- PyTorch to ONNX
- ONNX to TensorFlow Lite
"""

import os
import sys
import argparse
import logging
import json
import shutil
import tempfile
from pathlib import Path
from typing import Dict, Any, Optional, Union, List, Tuple
import numpy as np

# Setup logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


def convert_torch_to_tinygrad(input_path: str, output_path: str) -> None:
    """
    Convert a PyTorch model to TinyGrad format.

    Args:
        input_path: Path to the PyTorch model file (.pt or .pth)
        output_path: Path to save the TinyGrad model file (.npz)
    """
    try:
        import torch

        logger.info(f"Converting PyTorch model from {input_path} to TinyGrad format at {output_path}")

        # Load PyTorch model with security measures
        # Use weights_only=True for better security when loading models
        model = torch.load(input_path, map_location="cpu", weights_only=True)

        # Extract state dict if needed
        if hasattr(model, "state_dict"):
            state_dict = model.state_dict()
        elif isinstance(model, dict) and "state_dict" in model:
            state_dict = model["state_dict"]
        else:
            state_dict = model

        # Convert state dict to numpy arrays
        numpy_dict = {}
        for key, tensor in state_dict.items():
            numpy_dict[key] = tensor.cpu().numpy()

        # Save as NPZ file
        np.savez(output_path, **numpy_dict)

        logger.info(f"Model converted successfully to {output_path}")

    except ImportError:
        logger.error("PyTorch not found. Install with: pip install torch")
        raise
    except Exception as e:
        logger.error(f"Error converting model: {str(e)}")
        raise


def convert_onnx_to_tinygrad(input_path: str, output_path: str) -> None:
    """
    Convert an ONNX model to TinyGrad format.

    Args:
        input_path: Path to the ONNX model file (.onnx)
        output_path: Path to save the TinyGrad model file (.npz)
    """
    try:
        import onnx
        import torch

        logger.info(f"Converting ONNX model from {input_path} to TinyGrad format at {output_path}")

        # Load ONNX model
        onnx_model = onnx.load(input_path)

        # Extract weights from ONNX model
        numpy_dict = {}
        for initializer in onnx_model.graph.initializer:
            # Convert ONNX tensor to numpy array
            tensor = onnx.numpy_helper.to_array(initializer)
            numpy_dict[initializer.name] = tensor

        # Save as NPZ file
        np.savez(output_path, **numpy_dict)

        logger.info(f"Model converted successfully to {output_path}")

    except ImportError:
        logger.error("ONNX not found. Install with: pip install onnx")
        raise
    except Exception as e:
        logger.error(f"Error converting model: {str(e)}")
        raise


def convert_tflite_to_tinygrad(input_path: str, output_path: str) -> None:
    """
    Convert a TensorFlow Lite model to TinyGrad format.

    Args:
        input_path: Path to the TFLite model file (.tflite)
        output_path: Path to save the TinyGrad model file (.npz)
    """
    try:
        import tensorflow as tf

        logger.info(f"Converting TFLite model from {input_path} to TinyGrad format at {output_path}")

        # Load TFLite model
        interpreter = tf.lite.Interpreter(model_path=input_path)
        interpreter.allocate_tensors()

        # Get tensor details
        tensor_details = interpreter.get_tensor_details()

        # Extract weights from TFLite model
        numpy_dict = {}
        for tensor in tensor_details:
            if tensor["name"] != "":
                # Skip input and output tensors
                if tensor["index"] not in interpreter._inputs and tensor["index"] not in interpreter._outputs:
                    numpy_dict[tensor["name"]] = interpreter.tensor(tensor["index"])()

        # Save as NPZ file
        np.savez(output_path, **numpy_dict)

        logger.info(f"Model converted successfully to {output_path}")

    except ImportError:
        logger.error("TensorFlow not found. Install with: pip install tensorflow")
        raise
    except Exception as e:
        logger.error(f"Error converting model: {str(e)}")
        raise


def convert_to_safetensors(input_path: str, output_path: str) -> None:
    """
    Convert a model to SafeTensors format.

    Args:
        input_path: Path to the input model file (.npz, .pt, .pth, .onnx, or .tflite)
        output_path: Path to save the SafeTensors model file (.safetensors)
    """
    try:
        from safetensors.numpy import save_file

        logger.info(f"Converting model from {input_path} to SafeTensors format at {output_path}")

        # Determine input format
        input_ext = Path(input_path).suffix.lower()

        # Load model based on format
        if input_ext == ".npz":
            # Load NPZ file
            numpy_dict = dict(np.load(input_path))
        elif input_ext in [".pt", ".pth"]:
            # Convert PyTorch model to numpy dict first
            temp_npz = f"{output_path}.temp.npz"
            convert_torch_to_tinygrad(input_path, temp_npz)
            numpy_dict = dict(np.load(temp_npz))
            os.remove(temp_npz)
        elif input_ext == ".onnx":
            # Convert ONNX model to numpy dict first
            temp_npz = f"{output_path}.temp.npz"
            convert_onnx_to_tinygrad(input_path, temp_npz)
            numpy_dict = dict(np.load(temp_npz))
            os.remove(temp_npz)
        elif input_ext == ".tflite":
            # Convert TFLite model to numpy dict first
            temp_npz = f"{output_path}.temp.npz"
            convert_tflite_to_tinygrad(input_path, temp_npz)
            numpy_dict = dict(np.load(temp_npz))
            os.remove(temp_npz)
        else:
            raise ValueError(f"Unsupported input format: {input_ext}")

        # Convert numpy arrays to contiguous arrays if needed
        for key, array in numpy_dict.items():
            if not array.flags.c_contiguous:
                numpy_dict[key] = np.ascontiguousarray(array)

        # Save as SafeTensors file
        save_file(numpy_dict, output_path)

        logger.info(f"Model converted successfully to {output_path}")

    except ImportError:
        logger.error("safetensors not found. Install with: pip install safetensors")
        raise
    except Exception as e:
        logger.error(f"Error converting model: {str(e)}")
        raise


def convert_tensorflow_to_tinygrad(input_path: str, output_path: str) -> None:
    """
    Convert a TensorFlow model to TinyGrad format.

    Args:
        input_path: Path to the TensorFlow model file (.pb or .h5)
        output_path: Path to save the TinyGrad model file (.npz)
    """
    try:
        import tensorflow as tf

        logger.info(f"Converting TensorFlow model from {input_path} to TinyGrad format at {output_path}")

        # Load TensorFlow model
        input_ext = Path(input_path).suffix.lower()
        if input_ext == ".h5":
            model = tf.keras.models.load_model(input_path)
        elif input_ext == ".pb":
            # Load SavedModel
            model = tf.saved_model.load(input_path)
        else:
            raise ValueError(f"Unsupported TensorFlow model format: {input_ext}")

        # Extract weights
        numpy_dict = {}

        if input_ext == ".h5":
            # For Keras models
            for layer in model.layers:
                for weight in layer.weights:
                    name = weight.name.replace(':', '_')
                    numpy_dict[name] = weight.numpy()
        else:
            # For SavedModel
            for var in model.variables:
                name = var.name.replace(':', '_')
                numpy_dict[name] = var.numpy()

        # Save as NPZ file
        np.savez(output_path, **numpy_dict)

        logger.info(f"Model converted successfully to {output_path}")

    except ImportError:
        logger.error("TensorFlow not found. Install with: pip install tensorflow")
        raise
    except Exception as e:
        logger.error(f"Error converting model: {str(e)}")
        raise


def convert_torch_to_onnx(input_path: str, output_path: str, input_shape: Optional[List[int]] = None) -> None:
    """
    Convert a PyTorch model to ONNX format.

    Args:
        input_path: Path to the PyTorch model file (.pt or .pth)
        output_path: Path to save the ONNX model file (.onnx)
        input_shape: Input shape for the model (default: [1, 3, 224, 224])
    """
    try:
        import torch
        import torch.onnx

        logger.info(f"Converting PyTorch model from {input_path} to ONNX format at {output_path}")

        # Set default input shape if not provided
        if input_shape is None:
            input_shape = [1, 3, 224, 224]

        # Load PyTorch model with security measures
        # Use weights_only=True for better security when loading models
        model = torch.load(input_path, map_location="cpu", weights_only=True)

        # Handle different model formats
        if isinstance(model, dict):
            # If the model is a state dict, we need a model definition
            if "model_definition" in model:
                # If the model definition is included in the state dict
                model_class = model["model_definition"]
                model_instance = model_class()
                model_instance.load_state_dict(model["state_dict"])
                model = model_instance
            else:
                # If only the state dict is saved, we can't load the model
                # without knowing its architecture
                raise ValueError("Model is a state dict without model definition")

        # Set model to evaluation mode
        if hasattr(model, "eval"):
            model.eval()

        # Create dummy input
        dummy_input = torch.randn(*input_shape)

        # Export to ONNX
        torch.onnx.export(
            model,
            dummy_input,
            output_path,
            export_params=True,
            opset_version=12,
            do_constant_folding=True,
            input_names=["input"],
            output_names=["output"],
            dynamic_axes={"input": {0: "batch_size"}, "output": {0: "batch_size"}}
        )

        logger.info(f"Model converted successfully to {output_path}")

    except ImportError:
        logger.error("PyTorch not found. Install with: pip install torch")
        raise
    except Exception as e:
        logger.error(f"Error converting model: {str(e)}")
        raise


def convert_onnx_to_tflite(input_path: str, output_path: str) -> None:
    """
    Convert an ONNX model to TensorFlow Lite format.

    Args:
        input_path: Path to the ONNX model file (.onnx)
        output_path: Path to save the TFLite model file (.tflite)
    """
    try:
        import onnx
        import tensorflow as tf
        from onnx_tf.backend import prepare

        logger.info(f"Converting ONNX model from {input_path} to TFLite format at {output_path}")

        # Load ONNX model
        onnx_model = onnx.load(input_path)

        # Check ONNX model
        onnx.checker.check_model(onnx_model)

        # Convert ONNX to TensorFlow
        tf_rep = prepare(onnx_model)

        # Create a temporary directory for the SavedModel
        with tempfile.TemporaryDirectory() as temp_dir:
            # Save the TensorFlow model
            tf_rep.export_graph(temp_dir)

            # Load the SavedModel
            converter = tf.lite.TFLiteConverter.from_saved_model(temp_dir)

            # Set optimization flags
            converter.optimizations = [tf.lite.Optimize.DEFAULT]

            # Convert to TFLite
            tflite_model = converter.convert()

            # Save the TFLite model
            with open(output_path, "wb") as f:
                f.write(tflite_model)

        logger.info(f"Model converted successfully to {output_path}")

    except ImportError:
        logger.error("Required packages not found. Install with: pip install onnx tensorflow onnx-tf")
        raise
    except Exception as e:
        logger.error(f"Error converting model: {str(e)}")
        raise


def optimize_model(input_path: str, output_path: str, optimization_level: int = 1) -> None:
    """
    Optimize a model for inference.

    Args:
        input_path: Path to the input model file
        output_path: Path to save the optimized model file
        optimization_level: Optimization level (1-3, higher is more aggressive)
    """
    try:
        input_ext = Path(input_path).suffix.lower()

        if input_ext == ".onnx":
            # Optimize ONNX model
            import onnx
            from onnxoptimizer import optimize

            logger.info(f"Optimizing ONNX model from {input_path} to {output_path}")

            # Load ONNX model
            model = onnx.load(input_path)

            # Define optimization passes based on level
            if optimization_level == 1:
                passes = ["eliminate_identity", "eliminate_nop_transpose", "fuse_bn_into_conv"]
            elif optimization_level == 2:
                passes = ["eliminate_identity", "eliminate_nop_transpose", "fuse_bn_into_conv",
                          "fuse_consecutive_squeezes", "fuse_consecutive_transposes"]
            else:  # level 3
                passes = ["eliminate_identity", "eliminate_nop_transpose", "fuse_bn_into_conv",
                          "fuse_consecutive_squeezes", "fuse_consecutive_transposes",
                          "fuse_add_bias_into_conv", "fuse_transpose_into_gemm"]

            # Optimize model
            optimized_model = optimize(model, passes)

            # Save optimized model
            onnx.save(optimized_model, output_path)

        elif input_ext == ".tflite":
            # Optimize TFLite model
            import tensorflow as tf

            logger.info(f"Optimizing TFLite model from {input_path} to {output_path}")

            # Load TFLite model
            with open(input_path, "rb") as f:
                model_content = f.read()

            # Create converter
            converter = tf.lite.TFLiteConverter.from_buffer(model_content)

            # Set optimization flags based on level
            converter.optimizations = [tf.lite.Optimize.DEFAULT]

            if optimization_level >= 2:
                converter.target_spec.supported_ops = [tf.lite.OpsSet.TFLITE_BUILTINS]

            if optimization_level >= 3:
                converter.target_spec.supported_ops = [
                    tf.lite.OpsSet.TFLITE_BUILTINS_INT8,
                    tf.lite.OpsSet.SELECT_TF_OPS
                ]
                converter.inference_input_type = tf.int8
                converter.inference_output_type = tf.int8

            # Convert model
            optimized_model = converter.convert()

            # Save optimized model
            with open(output_path, "wb") as f:
                f.write(optimized_model)

        else:
            logger.error(f"Unsupported model format for optimization: {input_ext}")
            raise ValueError(f"Unsupported model format for optimization: {input_ext}")

        logger.info(f"Model optimized successfully to {output_path}")

    except ImportError:
        logger.error("Required packages not found for optimization")
        raise
    except Exception as e:
        logger.error(f"Error optimizing model: {str(e)}")
        raise


def quantize_model(input_path: str, output_path: str, quantization_type: str = "int8") -> None:
    """
    Quantize a model for reduced size and faster inference.

    Args:
        input_path: Path to the input model file
        output_path: Path to save the quantized model file
        quantization_type: Type of quantization (int8, float16)
    """
    try:
        input_ext = Path(input_path).suffix.lower()

        if input_ext == ".onnx":
            # Quantize ONNX model
            import onnx
            from onnxruntime.quantization import quantize_dynamic, QuantType

            logger.info(f"Quantizing ONNX model from {input_path} to {output_path}")

            # Determine quantization type
            if quantization_type == "int8":
                quant_type = QuantType.QInt8
            elif quantization_type == "float16":
                quant_type = QuantType.QFloat16
            else:
                raise ValueError(f"Unsupported quantization type: {quantization_type}")

            # Quantize model
            quantize_dynamic(input_path, output_path, weight_type=quant_type)

        elif input_ext == ".tflite":
            # Quantize TFLite model
            import tensorflow as tf

            logger.info(f"Quantizing TFLite model from {input_path} to {output_path}")

            # Load TFLite model
            with open(input_path, "rb") as f:
                model_content = f.read()

            # Create converter
            converter = tf.lite.TFLiteConverter.from_buffer(model_content)

            # Set quantization flags
            converter.optimizations = [tf.lite.Optimize.DEFAULT]

            if quantization_type == "int8":
                converter.target_spec.supported_ops = [tf.lite.OpsSet.TFLITE_BUILTINS_INT8]
                converter.inference_input_type = tf.int8
                converter.inference_output_type = tf.int8
            elif quantization_type == "float16":
                converter.target_spec.supported_types = [tf.float16]
            else:
                raise ValueError(f"Unsupported quantization type: {quantization_type}")

            # Convert model
            quantized_model = converter.convert()

            # Save quantized model
            with open(output_path, "wb") as f:
                f.write(quantized_model)

        else:
            logger.error(f"Unsupported model format for quantization: {input_ext}")
            raise ValueError(f"Unsupported model format for quantization: {input_ext}")

        logger.info(f"Model quantized successfully to {output_path}")

    except ImportError:
        logger.error("Required packages not found for quantization")
        raise
    except Exception as e:
        logger.error(f"Error quantizing model: {str(e)}")
        raise


def main():
    """Main function."""
    parser = argparse.ArgumentParser(description="Model conversion utility for Bulo.Cloud Sentinel")
    parser.add_argument("input", help="Path to the input model file")
    parser.add_argument("output", help="Path to save the converted model file")
    parser.add_argument("--format", choices=["tinygrad", "safetensors", "onnx", "tflite"], default="tinygrad",
                        help="Output format (default: tinygrad)")
    parser.add_argument("--input-shape", type=int, nargs="+", default=None,
                        help="Input shape for the model (e.g., 1 3 224 224)")
    parser.add_argument("--optimize", action="store_true", help="Optimize the model for inference")
    parser.add_argument("--optimization-level", type=int, choices=[1, 2, 3], default=1,
                        help="Optimization level (1-3, higher is more aggressive)")
    parser.add_argument("--quantize", action="store_true", help="Quantize the model for reduced size and faster inference")
    parser.add_argument("--quantization-type", choices=["int8", "float16"], default="int8",
                        help="Type of quantization (default: int8)")
    parser.add_argument("--info", action="store_true", help="Print model information")
    args = parser.parse_args()

    # Create output directory if it doesn't exist
    os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)

    # Determine input format
    input_ext = Path(args.input).suffix.lower()

    # Print model information if requested
    if args.info:
        print_model_info(args.input)
        return

    # Convert model based on input format and desired output format
    if args.format == "tinygrad":
        if input_ext in [".pt", ".pth"]:
            convert_torch_to_tinygrad(args.input, args.output)
        elif input_ext == ".onnx":
            convert_onnx_to_tinygrad(args.input, args.output)
        elif input_ext == ".tflite":
            convert_tflite_to_tinygrad(args.input, args.output)
        elif input_ext in [".pb", ".h5"]:
            convert_tensorflow_to_tinygrad(args.input, args.output)
        else:
            logger.error(f"Unsupported input format for tinygrad conversion: {input_ext}")
            sys.exit(1)
    elif args.format == "safetensors":
        convert_to_safetensors(args.input, args.output)
    elif args.format == "onnx":
        if input_ext in [".pt", ".pth"]:
            convert_torch_to_onnx(args.input, args.output, args.input_shape)
        else:
            logger.error(f"Unsupported input format for ONNX conversion: {input_ext}")
            sys.exit(1)
    elif args.format == "tflite":
        if input_ext == ".onnx":
            convert_onnx_to_tflite(args.input, args.output)
        else:
            logger.error(f"Unsupported input format for TFLite conversion: {input_ext}")
            sys.exit(1)
    else:
        logger.error(f"Unsupported output format: {args.format}")
        sys.exit(1)

    # Optimize model if requested
    if args.optimize:
        # Create a temporary file for the optimized model
        temp_output = f"{args.output}.temp"
        shutil.move(args.output, temp_output)

        try:
            optimize_model(temp_output, args.output, args.optimization_level)
            os.remove(temp_output)
        except Exception as e:
            # Restore original file if optimization fails
            shutil.move(temp_output, args.output)
            logger.error(f"Error optimizing model: {str(e)}")

    # Quantize model if requested
    if args.quantize:
        # Create a temporary file for the quantized model
        temp_output = f"{args.output}.temp"
        shutil.move(args.output, temp_output)

        try:
            quantize_model(temp_output, args.output, args.quantization_type)
            os.remove(temp_output)
        except Exception as e:
            # Restore original file if quantization fails
            shutil.move(temp_output, args.output)
            logger.error(f"Error quantizing model: {str(e)}")


def print_model_info(model_path: str) -> None:
    """
    Print information about a model.

    Args:
        model_path: Path to the model file
    """
    try:
        input_ext = Path(model_path).suffix.lower()

        if input_ext in [".pt", ".pth"]:
            # Print PyTorch model info
            import torch

            logger.info(f"Loading PyTorch model from {model_path}")

            # Load PyTorch model safely
            # Use torch.jit.load for TorchScript models or load only the state_dict
            model = torch.load(model_path, map_location="cpu")
            if not isinstance(model, dict) or "state_dict" not in model:
                raise ValueError("Invalid PyTorch model format. Expected a state_dict.")
            state_dict = model["state_dict"]

            # Print model info
            print("\nPyTorch Model Information:")
            print(f"Type: {type(model)}")

            if hasattr(model, "state_dict"):
                state_dict = model.state_dict()
                print(f"Number of parameters: {len(state_dict)}")
                total_params = sum(p.numel() for p in model.parameters())
                print(f"Total parameters: {total_params:,}")

                # Print model architecture if available
                if hasattr(model, "__repr__"):
                    print("\nModel Architecture:")
                    print(model)
            elif isinstance(model, dict) and "state_dict" in model:
                state_dict = model["state_dict"]
                print(f"Number of parameters: {len(state_dict)}")

                # Print keys
                print("\nModel Keys:")
                for key in model.keys():
                    print(f"  - {key}")
            else:
                print("Model is a state dict")
                print(f"Number of parameters: {len(model)}")

        elif input_ext == ".onnx":
            # Print ONNX model info
            import onnx

            logger.info(f"Loading ONNX model from {model_path}")

            # Load ONNX model
            model = onnx.load(model_path)

            # Print model info
            print("\nONNX Model Information:")
            print(f"IR version: {model.ir_version}")
            print(f"Producer name: {model.producer_name}")
            print(f"Producer version: {model.producer_version}")
            print(f"Domain: {model.domain}")
            print(f"Model version: {model.model_version}")

            # Print inputs
            print("\nInputs:")
            for input in model.graph.input:
                print(f"  - {input.name}")

                # Print shape if available
                if input.type.tensor_type.shape.dim:
                    shape = []
                    for dim in input.type.tensor_type.shape.dim:
                        if dim.dim_param:
                            shape.append(dim.dim_param)
                        else:
                            shape.append(dim.dim_value)
                    print(f"    Shape: {shape}")

            # Print outputs
            print("\nOutputs:")
            for output in model.graph.output:
                print(f"  - {output.name}")

                # Print shape if available
                if output.type.tensor_type.shape.dim:
                    shape = []
                    for dim in output.type.tensor_type.shape.dim:
                        if dim.dim_param:
                            shape.append(dim.dim_param)
                        else:
                            shape.append(dim.dim_value)
                    print(f"    Shape: {shape}")

            # Print nodes
            print(f"\nNumber of nodes: {len(model.graph.node)}")

            # Print operators
            ops = {}
            for node in model.graph.node:
                if node.op_type in ops:
                    ops[node.op_type] += 1
                else:
                    ops[node.op_type] = 1

            print("\nOperators:")
            for op, count in ops.items():
                print(f"  - {op}: {count}")

        elif input_ext == ".tflite":
            # Print TFLite model info
            import tensorflow as tf

            logger.info(f"Loading TFLite model from {model_path}")

            # Load TFLite model
            interpreter = tf.lite.Interpreter(model_path=model_path)
            interpreter.allocate_tensors()

            # Print model info
            print("\nTensorFlow Lite Model Information:")

            # Print inputs
            input_details = interpreter.get_input_details()
            print("\nInputs:")
            for input in input_details:
                print(f"  - {input['name']}")
                print(f"    Shape: {input['shape']}")
                print(f"    Type: {input['dtype']}")

            # Print outputs
            output_details = interpreter.get_output_details()
            print("\nOutputs:")
            for output in output_details:
                print(f"  - {output['name']}")
                print(f"    Shape: {output['shape']}")
                print(f"    Type: {output['dtype']}")

            # Print tensors
            tensor_details = interpreter.get_tensor_details()
            print(f"\nNumber of tensors: {len(tensor_details)}")

            # Print operators
            ops = {}
            for i, op in enumerate(interpreter._get_ops_details()):
                op_name = op["op_name"]
                if op_name in ops:
                    ops[op_name] += 1
                else:
                    ops[op_name] = 1

            print("\nOperators:")
            for op, count in ops.items():
                print(f"  - {op}: {count}")

        elif input_ext in [".pb", ".h5"]:
            # Print TensorFlow model info
            import tensorflow as tf

            logger.info(f"Loading TensorFlow model from {model_path}")

            # Load TensorFlow model
            if input_ext == ".h5":
                model = tf.keras.models.load_model(model_path)

                # Print model info
                print("\nTensorFlow Keras Model Information:")
                model.summary()
            else:
                # Load SavedModel
                model = tf.saved_model.load(model_path)

                # Print model info
                print("\nTensorFlow SavedModel Information:")
                print(f"Type: {type(model)}")

                # Print signatures
                print("\nSignatures:")
                for signature_name, signature in model.signatures.items():
                    print(f"  - {signature_name}")

                    # Print inputs
                    print("    Inputs:")
                    for name, tensor_spec in signature.structured_input_signature[1].items():
                        print(f"      - {name}: {tensor_spec.shape}, {tensor_spec.dtype}")

                    # Print outputs
                    print("    Outputs:")
                    for name, tensor_spec in signature.structured_outputs.items():
                        print(f"      - {name}: {tensor_spec.shape}, {tensor_spec.dtype}")

        elif input_ext == ".npz":
            # Print NPZ model info
            logger.info(f"Loading NPZ model from {model_path}")

            # Load NPZ file
            model = np.load(model_path)

            # Print model info
            print("\nNPZ Model Information:")
            print(f"Number of arrays: {len(model.files)}")

            # Print arrays
            print("\nArrays:")
            for name in model.files:
                array = model[name]
                print(f"  - {name}: {array.shape}, {array.dtype}")

        elif input_ext == ".safetensors":
            # Print SafeTensors model info
            from safetensors import safe_open

            logger.info(f"Loading SafeTensors model from {model_path}")

            # Load SafeTensors file
            with safe_open(model_path, framework="numpy") as model:
                # Print model info
                print("\nSafeTensors Model Information:")

                # Get tensor names
                tensor_names = model.keys()
                print(f"Number of tensors: {len(tensor_names)}")

                # Print tensors
                print("\nTensors:")
                for name in tensor_names:
                    tensor = model.get_tensor(name)
                    print(f"  - {name}: {tensor.shape}, {tensor.dtype}")

        else:
            logger.error(f"Unsupported model format: {input_ext}")
            sys.exit(1)

    except ImportError as e:
        logger.error(f"Required packages not found: {str(e)}")
        sys.exit(1)
    except Exception as e:
        logger.error(f"Error printing model info: {str(e)}")
        sys.exit(1)


if __name__ == "__main__":
    main()
