#!/usr/bin/env python3
"""
Web interface for model conversion.

This module provides a web interface for converting models between different formats.
"""

import os
import sys
import logging
import tempfile
import uuid
from pathlib import Path
from typing import Dict, Any, Optional, List
import shutil

import gradio as gr

# Add project root to path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))

# Import conversion functions
from ai.inference.convert import (
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

# Setup logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Create temporary directory for uploaded models
TEMP_DIR = os.path.join(tempfile.gettempdir(), "model_converter")
os.makedirs(TEMP_DIR, exist_ok=True)

# Create output directory for converted models
OUTPUT_DIR = os.path.join(os.path.dirname(__file__), "../../models/converted")
os.makedirs(OUTPUT_DIR, exist_ok=True)


def convert_model(
    model_file: str,
    output_format: str,
    input_shape: Optional[str] = None,
    optimize: bool = False,
    optimization_level: int = 1,
    quantize: bool = False,
    quantization_type: str = "int8"
) -> str:
    """
    Convert a model to another format.
    
    Args:
        model_file: Path to the model file
        output_format: Output format
        input_shape: Input shape for the model (comma-separated integers)
        optimize: Whether to optimize the model
        optimization_level: Optimization level
        quantize: Whether to quantize the model
        quantization_type: Type of quantization
        
    Returns:
        Path to the converted model file
    """
    try:
        # Get input file extension
        input_ext = Path(model_file).suffix.lower()
        
        # Determine output file extension
        if output_format == "tinygrad":
            output_ext = ".npz"
        elif output_format == "safetensors":
            output_ext = ".safetensors"
        elif output_format == "onnx":
            output_ext = ".onnx"
        elif output_format == "tflite":
            output_ext = ".tflite"
        else:
            return f"Unsupported output format: {output_format}"
        
        # Generate output file path
        output_file = os.path.join(
            OUTPUT_DIR,
            f"{Path(model_file).stem}_{output_format}_{uuid.uuid4().hex[:8]}{output_ext}"
        )
        
        # Parse input shape if provided
        parsed_input_shape = None
        if input_shape:
            try:
                parsed_input_shape = [int(dim.strip()) for dim in input_shape.split(",")]
            except ValueError:
                return f"Invalid input shape: {input_shape}. Expected comma-separated integers."
        
        # Convert model based on input format and desired output format
        if output_format == "tinygrad":
            if input_ext in [".pt", ".pth"]:
                convert_torch_to_tinygrad(model_file, output_file)
            elif input_ext == ".onnx":
                convert_onnx_to_tinygrad(model_file, output_file)
            elif input_ext == ".tflite":
                convert_tflite_to_tinygrad(model_file, output_file)
            elif input_ext in [".pb", ".h5"]:
                convert_tensorflow_to_tinygrad(model_file, output_file)
            else:
                return f"Unsupported input format for tinygrad conversion: {input_ext}"
        elif output_format == "safetensors":
            convert_to_safetensors(model_file, output_file)
        elif output_format == "onnx":
            if input_ext in [".pt", ".pth"]:
                convert_torch_to_onnx(model_file, output_file, parsed_input_shape)
            else:
                return f"Unsupported input format for ONNX conversion: {input_ext}"
        elif output_format == "tflite":
            if input_ext == ".onnx":
                convert_onnx_to_tflite(model_file, output_file)
            else:
                return f"Unsupported input format for TFLite conversion: {input_ext}"
        else:
            return f"Unsupported output format: {output_format}"
        
        # Optimize model if requested
        if optimize:
            # Create a temporary file for the optimized model
            temp_output = f"{output_file}.temp"
            shutil.move(output_file, temp_output)
            
            try:
                optimize_model(temp_output, output_file, optimization_level)
                os.remove(temp_output)
            except Exception as e:
                # Restore original file if optimization fails
                shutil.move(temp_output, output_file)
                logger.error(f"Error optimizing model: {str(e)}")
                return f"Error optimizing model: {str(e)}"
        
        # Quantize model if requested
        if quantize:
            # Create a temporary file for the quantized model
            temp_output = f"{output_file}.temp"
            shutil.move(output_file, temp_output)
            
            try:
                quantize_model(temp_output, output_file, quantization_type)
                os.remove(temp_output)
            except Exception as e:
                # Restore original file if quantization fails
                shutil.move(temp_output, output_file)
                logger.error(f"Error quantizing model: {str(e)}")
                return f"Error quantizing model: {str(e)}"
        
        return output_file
    
    except Exception as e:
        logger.error(f"Error converting model: {str(e)}")
        return f"Error converting model: {str(e)}"


def get_model_info(model_file: str) -> str:
    """
    Get information about a model.
    
    Args:
        model_file: Path to the model file
        
    Returns:
        Model information as a string
    """
    try:
        # Redirect stdout to capture print output
        import io
        from contextlib import redirect_stdout
        
        f = io.StringIO()
        with redirect_stdout(f):
            print_model_info(model_file)
        
        return f.getvalue()
    
    except Exception as e:
        logger.error(f"Error getting model info: {str(e)}")
        return f"Error getting model info: {str(e)}"


def upload_model(file):
    """
    Upload a model file.
    
    Args:
        file: Uploaded file
        
    Returns:
        Path to the uploaded file
    """
    try:
        # Generate a unique filename
        filename = f"{uuid.uuid4().hex}{Path(file.name).suffix}"
        
        # Save the file to the temporary directory
        file_path = os.path.join(TEMP_DIR, filename)
        with open(file_path, "wb") as f:
            f.write(file)
        
        return file_path
    
    except Exception as e:
        logger.error(f"Error uploading model: {str(e)}")
        return None


def create_web_interface():
    """Create the web interface."""
    with gr.Blocks(title="Bulo.Cloud Sentinel Model Converter") as app:
        gr.Markdown("# Bulo.Cloud Sentinel Model Converter")
        gr.Markdown("Convert models between different formats for use with Bulo.Cloud Sentinel.")
        
        with gr.Tab("Convert Model"):
            with gr.Row():
                with gr.Column():
                    model_file = gr.File(label="Upload Model File")
                    output_format = gr.Dropdown(
                        choices=["tinygrad", "safetensors", "onnx", "tflite"],
                        value="tinygrad",
                        label="Output Format"
                    )
                    input_shape = gr.Textbox(
                        label="Input Shape (comma-separated integers, e.g., 1,3,224,224)",
                        placeholder="1,3,224,224"
                    )
                    
                    with gr.Accordion("Advanced Options", open=False):
                        optimize = gr.Checkbox(label="Optimize Model", value=False)
                        optimization_level = gr.Slider(
                            minimum=1,
                            maximum=3,
                            value=1,
                            step=1,
                            label="Optimization Level"
                        )
                        quantize = gr.Checkbox(label="Quantize Model", value=False)
                        quantization_type = gr.Dropdown(
                            choices=["int8", "float16"],
                            value="int8",
                            label="Quantization Type"
                        )
                    
                    convert_button = gr.Button("Convert Model")
                
                with gr.Column():
                    output_file = gr.Textbox(label="Output File Path")
                    download_button = gr.Button("Download Converted Model")
                    download_file = gr.File(label="Download")
            
            convert_button.click(
                fn=lambda file, format, shape, opt, level, quant, type: convert_model(
                    upload_model(file),
                    format,
                    shape,
                    opt,
                    level,
                    quant,
                    type
                ),
                inputs=[
                    model_file,
                    output_format,
                    input_shape,
                    optimize,
                    optimization_level,
                    quantize,
                    quantization_type
                ],
                outputs=output_file
            )
            
            download_button.click(
                fn=lambda path: path if os.path.exists(path) else None,
                inputs=output_file,
                outputs=download_file
            )
        
        with gr.Tab("Model Info"):
            with gr.Row():
                with gr.Column():
                    info_model_file = gr.File(label="Upload Model File")
                    info_button = gr.Button("Get Model Info")
                
                with gr.Column():
                    model_info = gr.Textbox(label="Model Information", lines=20)
            
            info_button.click(
                fn=lambda file: get_model_info(upload_model(file)),
                inputs=info_model_file,
                outputs=model_info
            )
        
        gr.Markdown("## Supported Conversions")
        gr.Markdown("""
        - PyTorch (.pt, .pth) → TinyGrad (.npz)
        - ONNX (.onnx) → TinyGrad (.npz)
        - TensorFlow Lite (.tflite) → TinyGrad (.npz)
        - TensorFlow (.pb, .h5) → TinyGrad (.npz)
        - Any supported format → SafeTensors (.safetensors)
        - PyTorch (.pt, .pth) → ONNX (.onnx)
        - ONNX (.onnx) → TensorFlow Lite (.tflite)
        """)
    
    return app


def main():
    """Main function."""
    app = create_web_interface()
    app.launch(server_name="0.0.0.0", server_port=7860)


if __name__ == "__main__":
    main()
