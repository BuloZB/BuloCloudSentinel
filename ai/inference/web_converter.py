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

# Import security utilities
from ai.inference.security import (
    is_safe_path,
    is_safe_file,
    validate_input_shape,
    sanitize_filename,
    get_safe_temp_file,
    clean_temp_files,
    validate_model_format
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

# Set maximum file size (100MB)
MAX_UPLOAD_SIZE = 100 * 1024 * 1024

# Set maximum execution time (5 minutes)
MAX_EXECUTION_TIME = 300


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
        # Check if model file is valid
        if not model_file or not os.path.exists(model_file):
            return "Model file not found"

        # Check if model file is safe
        if not is_safe_file(model_file):
            return "Invalid model file"

        # Get input file extension
        input_ext = Path(model_file).suffix.lower()

        # Check if input format is supported
        if not validate_model_format(model_file):
            return f"Unsupported input format: {input_ext}"

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

        # Generate safe output file path
        safe_filename = sanitize_filename(Path(model_file).name)
        output_file = os.path.join(
            OUTPUT_DIR,
            f"{Path(safe_filename).stem}_{output_format}_{uuid.uuid4().hex[:8]}{output_ext}"
        )

        # Validate output directory
        os.makedirs(os.path.dirname(output_file), exist_ok=True)

        # Parse and validate input shape if provided
        parsed_input_shape = None
        if input_shape:
            parsed_input_shape = validate_input_shape(input_shape)
            if parsed_input_shape is None:
                return f"Invalid input shape: {input_shape}. Expected comma-separated positive integers."

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
            temp_output = get_safe_temp_file(prefix="optimize", suffix=output_ext)

            try:
                # Copy the file to the temporary location
                shutil.copy2(output_file, temp_output)

                # Optimize the model
                optimize_model(temp_output, output_file, optimization_level)

                # Clean up
                if os.path.exists(temp_output):
                    os.remove(temp_output)
            except Exception as e:
                # Clean up on error
                if os.path.exists(temp_output):
                    os.remove(temp_output)
                logger.error(f"Error optimizing model: {str(e)}")
                return f"Error optimizing model: {str(e)}"

        # Quantize model if requested
        if quantize:
            # Create a temporary file for the quantized model
            temp_output = get_safe_temp_file(prefix="quantize", suffix=output_ext)

            try:
                # Copy the file to the temporary location
                shutil.copy2(output_file, temp_output)

                # Quantize the model
                quantize_model(temp_output, output_file, quantization_type)

                # Clean up
                if os.path.exists(temp_output):
                    os.remove(temp_output)
            except Exception as e:
                # Clean up on error
                if os.path.exists(temp_output):
                    os.remove(temp_output)
                logger.error(f"Error quantizing model: {str(e)}")
                return f"Error quantizing model: {str(e)}"

        # Clean up temporary files
        clean_temp_files()

        return output_file

    except Exception as e:
        # Log detailed error for administrators
        logger.error(f"Error converting model: {str(e)}", exc_info=True)
        # Return generic error for users
        return "An error occurred while converting the model. Please check the logs for details."


def get_model_info(model_file: str) -> str:
    """
    Get information about a model.

    Args:
        model_file: Path to the model file

    Returns:
        Model information as a string
    """
    try:
        # Check if model path is valid
        if not model_file or not os.path.exists(model_file):
            return "Model file not found"

        # Check if model file is safe
        if not is_safe_file(model_file):
            return "Invalid model file"

        # Check if model format is supported
        if not validate_model_format(model_file):
            return f"Unsupported model format: {Path(model_file).suffix.lower()}"

        # Redirect stdout to capture print output
        import io
        from contextlib import redirect_stdout

        f = io.StringIO()
        with redirect_stdout(f):
            print_model_info(model_file)

        return f.getvalue()

    except Exception as e:
        # Log detailed error for administrators
        logger.error(f"Error getting model info: {str(e)}", exc_info=True)
        # Return generic error for users
        return "An error occurred while getting model information. Please check the logs for details."


def upload_model(file):
    """
    Upload a model file.

    Args:
        file: Uploaded file

    Returns:
        Path to the uploaded file
    """
    try:
        # Check if file is valid
        if file is None:
            logger.warning("No file uploaded")
            return None

        # Check file size
        if len(file) > MAX_UPLOAD_SIZE:
            logger.warning(f"File too large: {len(file)} bytes (max: {MAX_UPLOAD_SIZE} bytes)")
            return "File too large. Maximum size is 100MB."

        # Get file extension
        file_ext = Path(file.name).suffix.lower()

        # Check if file extension is supported
        if not validate_model_format(file.name):
            logger.warning(f"Unsupported file extension: {file_ext}")
            return None

        # Sanitize filename
        safe_filename = sanitize_filename(file.name)

        # Generate a unique filename
        filename = f"{uuid.uuid4().hex}_{safe_filename}"

        # Save the file to the temporary directory
        file_path = os.path.join(TEMP_DIR, filename)
        with open(file_path, "wb") as f:
            f.write(file)

        # Clean old temporary files
        clean_temp_files(TEMP_DIR)

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

    # Launch the web interface with security settings
    app.launch(
        server_name="0.0.0.0",
        server_port=7860,
        max_threads=10,  # Limit number of concurrent requests
        share=False,     # Disable public sharing
        auth=None,       # No authentication (add if needed)
        ssl_verify=True  # Verify SSL certificates
    )


if __name__ == "__main__":
    main()
