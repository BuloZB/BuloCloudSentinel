#!/usr/bin/env python3
"""
Model management interface for Bulo.Cloud Sentinel.

This module provides a web interface for managing ML models, including uploading,
downloading, visualization, and analysis.
"""

import os
import sys
import logging
import tempfile
import uuid
import json
import shutil
import traceback
from pathlib import Path
from typing import Dict, Any, Optional, List, Tuple
import numpy as np
import matplotlib.pyplot as plt
import gradio as gr

# Add project root to path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))

# Import local modules
from ai.inference.convert import print_model_info
from ai.inference import InferenceEngine
from ai.inference.security import (
    is_safe_path,
    is_safe_file,
    validate_input_shape,
    sanitize_filename,
    get_safe_temp_file,
    clean_temp_files,
    validate_model_format,
    safe_load_model
)
from ai.inference.config import ConfigManager
from ai.inference.monitoring import structured_logger, metrics_collector, alert_manager
from ai.inference.auth import AuthManager

# Setup logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Create model directory
MODEL_DIR = os.path.join(os.path.dirname(__file__), "../../models")
os.makedirs(MODEL_DIR, exist_ok=True)

# Create temporary directory for uploaded models
TEMP_DIR = os.path.join(tempfile.gettempdir(), "model_manager")
os.makedirs(TEMP_DIR, exist_ok=True)


def get_model_list() -> List[Dict[str, Any]]:
    """
    Get a list of available models.

    Returns:
        List of model information dictionaries
    """
    models = []

    # Scan model directory
    for file_path in Path(MODEL_DIR).glob("**/*"):
        if file_path.is_file() and file_path.suffix.lower() in [
            ".pt", ".pth", ".onnx", ".tflite", ".npz", ".safetensors", ".pb", ".h5"
        ]:
            # Get relative path
            rel_path = file_path.relative_to(MODEL_DIR)

            # Get file size
            size_bytes = file_path.stat().st_size
            size_mb = size_bytes / (1024 * 1024)

            # Get file modification time
            mod_time = file_path.stat().st_mtime

            # Determine model type
            model_type = "Unknown"
            if file_path.suffix.lower() in [".pt", ".pth"]:
                model_type = "PyTorch"
            elif file_path.suffix.lower() == ".onnx":
                model_type = "ONNX"
            elif file_path.suffix.lower() == ".tflite":
                model_type = "TensorFlow Lite"
            elif file_path.suffix.lower() == ".npz":
                model_type = "TinyGrad"
            elif file_path.suffix.lower() == ".safetensors":
                model_type = "SafeTensors"
            elif file_path.suffix.lower() in [".pb", ".h5"]:
                model_type = "TensorFlow"

            # Add model to list
            models.append({
                "name": file_path.name,
                "path": str(rel_path),
                "full_path": str(file_path),
                "type": model_type,
                "size": size_mb,
                "modified": mod_time
            })

    # Sort models by name
    models.sort(key=lambda x: x["name"])

    return models


def upload_model(file) -> str:
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
            structured_logger.warning("No file uploaded", logger_name="model_manager")
            return "No file uploaded"

        # Check file size (limit to 500MB)
        MAX_UPLOAD_SIZE = 500 * 1024 * 1024
        if len(file) > MAX_UPLOAD_SIZE:
            structured_logger.warning(
                "File too large",
                logger_name="model_manager",
                file_size=len(file),
                max_size=MAX_UPLOAD_SIZE
            )
            return f"File too large. Maximum size is 500MB."

        # Get file extension
        file_ext = Path(file.name).suffix.lower()

        # Check if file extension is supported
        if not validate_model_format(file.name):
            structured_logger.warning(
                "Unsupported file extension",
                logger_name="model_manager",
                file_extension=file_ext
            )
            return f"Unsupported file extension: {file_ext}"

        # Sanitize filename
        safe_filename = sanitize_filename(file.name)

        # Save file to model directory
        file_path = os.path.join(MODEL_DIR, safe_filename)

        # Check if file already exists
        if os.path.exists(file_path):
            # Generate a unique filename
            base_name = Path(safe_filename).stem
            file_ext = Path(safe_filename).suffix
            file_path = os.path.join(MODEL_DIR, f"{base_name}_{uuid.uuid4().hex[:8]}{file_ext}")

        # Create a temporary file first
        temp_path = get_safe_temp_file(prefix="upload", suffix=file_ext)

        try:
            # Save to temporary file
            with open(temp_path, "wb") as f:
                f.write(file)

            # Validate the model file
            if not is_safe_file(temp_path):
                structured_logger.warning(
                    "Invalid model file",
                    logger_name="model_manager",
                    file_path=temp_path
                )
                os.remove(temp_path)
                return "Invalid model file"

            # Move to final location
            shutil.move(temp_path, file_path)

            # Log successful upload
            structured_logger.info(
                "Model uploaded successfully",
                logger_name="model_manager",
                file_path=file_path,
                file_size=len(file),
                file_type=file_ext
            )

            # Increment metrics
            metrics_collector.increment_metric("models_uploaded")
            metrics_collector.increment_metric("upload_bytes", len(file))

            # Clean up temporary files
            clean_temp_files()

            return f"Model uploaded successfully: {os.path.basename(file_path)}"

        except Exception as e:
            # Clean up on error
            if os.path.exists(temp_path):
                os.remove(temp_path)
            raise e

    except Exception as e:
        # Log detailed error
        structured_logger.error(
            "Error uploading model",
            logger_name="model_manager",
            error=str(e),
            traceback=traceback.format_exc()
        )

        # Create alert
        alert_manager.create_alert(
            level="error",
            message=f"Error uploading model: {str(e)}",
            source="model_manager"
        )

        # Return generic error message
        return "An error occurred while uploading the model. Please check the logs for details."


def delete_model(model_path: str) -> str:
    """
    Delete a model file.

    Args:
        model_path: Path to the model file

    Returns:
        Status message
    """
    try:
        # Check if model path is valid
        if not model_path:
            structured_logger.warning("No model selected for deletion", logger_name="model_manager")
            return "No model selected"

        # Validate path
        if not is_safe_path(MODEL_DIR, model_path):
            structured_logger.warning(
                "Invalid model path",
                logger_name="model_manager",
                model_path=model_path
            )
            return "Invalid model path"

        # Get full path
        full_path = os.path.join(MODEL_DIR, model_path)

        # Check if file exists
        if not os.path.exists(full_path):
            structured_logger.warning(
                "Model not found",
                logger_name="model_manager",
                model_path=model_path
            )
            return f"Model not found: {model_path}"

        # Check if file is a regular file
        if not os.path.isfile(full_path):
            structured_logger.warning(
                "Not a regular file",
                logger_name="model_manager",
                model_path=model_path
            )
            return f"Not a regular file: {model_path}"

        # Delete file
        os.remove(full_path)

        # Log successful deletion
        structured_logger.info(
            "Model deleted successfully",
            logger_name="model_manager",
            model_path=model_path
        )

        # Increment metrics
        metrics_collector.increment_metric("models_deleted")

        return f"Model deleted successfully: {model_path}"

    except Exception as e:
        # Log detailed error
        structured_logger.error(
            "Error deleting model",
            logger_name="model_manager",
            error=str(e),
            traceback=traceback.format_exc(),
            model_path=model_path
        )

        # Create alert
        alert_manager.create_alert(
            level="error",
            message=f"Error deleting model: {str(e)}",
            source="model_manager",
            model_path=model_path
        )

        # Return generic error message
        return "An error occurred while deleting the model. Please check the logs for details."


def get_model_info(model_path: str) -> str:
    """
    Get information about a model.

    Args:
        model_path: Path to the model file

    Returns:
        Model information as a string
    """
    try:
        # Check if model path is valid
        if not model_path:
            structured_logger.warning("No model selected for info", logger_name="model_manager")
            return "No model selected"

        # Validate path
        if not is_safe_path(MODEL_DIR, model_path):
            structured_logger.warning(
                "Invalid model path",
                logger_name="model_manager",
                model_path=model_path
            )
            return "Invalid model path"

        # Get full path
        full_path = os.path.join(MODEL_DIR, model_path)

        # Check if file exists
        if not os.path.exists(full_path):
            structured_logger.warning(
                "Model not found",
                logger_name="model_manager",
                model_path=model_path
            )
            return f"Model not found: {model_path}"

        # Check if file is safe
        if not is_safe_file(full_path):
            structured_logger.warning(
                "Invalid model file",
                logger_name="model_manager",
                model_path=model_path
            )
            return "Invalid model file"

        # Check if model format is supported
        if not validate_model_format(full_path):
            structured_logger.warning(
                "Unsupported model format",
                logger_name="model_manager",
                model_path=model_path,
                file_extension=Path(full_path).suffix.lower()
            )
            return f"Unsupported model format: {Path(full_path).suffix.lower()}"

        # Redirect stdout to capture print output
        import io
        from contextlib import redirect_stdout

        f = io.StringIO()
        with redirect_stdout(f):
            print_model_info(full_path)

        # Log successful info retrieval
        structured_logger.info(
            "Model info retrieved successfully",
            logger_name="model_manager",
            model_path=model_path
        )

        # Increment metrics
        metrics_collector.increment_metric("model_info_requests")

        return f.getvalue()

    except Exception as e:
        # Log detailed error
        structured_logger.error(
            "Error getting model info",
            logger_name="model_manager",
            error=str(e),
            traceback=traceback.format_exc(),
            model_path=model_path
        )

        # Create alert
        alert_manager.create_alert(
            level="error",
            message=f"Error getting model info: {str(e)}",
            source="model_manager",
            model_path=model_path
        )

        # Return generic error message
        return "An error occurred while getting model information. Please check the logs for details."


def visualize_model(model_path: str) -> Optional[str]:
    """
    Visualize a model.

    Args:
        model_path: Path to the model file

    Returns:
        Path to the visualization image
    """
    try:
        # Check if model path is valid
        if not model_path:
            return None

        # Get full path
        full_path = os.path.join(MODEL_DIR, model_path)

        # Check if file exists
        if not os.path.exists(full_path):
            return None

        # Get file extension
        file_ext = Path(full_path).suffix.lower()

        # Create visualization based on model type
        if file_ext in [".pt", ".pth"]:
            # Visualize PyTorch model
            try:
                import torch
                import torchviz

                # Load model safely
                model = safe_load_model(full_path, model_type="pytorch")

                # Check if model is a state dict
                if isinstance(model, dict) and "state_dict" in model:
                    # If model definition is available
                    if "model_definition" in model:
                        model_class = model["model_definition"]
                        model_instance = model_class()
                        model_instance.load_state_dict(model["state_dict"])
                        model = model_instance
                    else:
                        return "Model is a state dict without model definition"

                # Create dummy input
                dummy_input = torch.randn(1, 3, 224, 224)

                # Create visualization
                vis_path = os.path.join(TEMP_DIR, f"{uuid.uuid4().hex}.png")

                # Create graph
                graph = torchviz.make_dot(model(dummy_input), params=dict(model.named_parameters()))

                # Save graph
                graph.render(vis_path, format="png")

                return f"{vis_path}.png"

            except Exception as e:
                logger.error(f"Error visualizing PyTorch model: {str(e)}")
                return None

        elif file_ext == ".onnx":
            # Visualize ONNX model
            try:
                import onnx
                from onnx.tools.net_drawer import GetPydotGraph, GetOpNodeProducer

                # Load model safely
                model = safe_load_model(full_path, model_type="onnx")

                # Create visualization
                vis_path = os.path.join(TEMP_DIR, f"{uuid.uuid4().hex}.png")

                # Create graph
                pydot_graph = GetPydotGraph(
                    model.graph,
                    name=model.graph.name,
                    rankdir="TB",
                    node_producer=GetOpNodeProducer("docstring")
                )

                # Save graph
                pydot_graph.write_png(vis_path)

                return vis_path

            except Exception as e:
                logger.error(f"Error visualizing ONNX model: {str(e)}")
                return None

        elif file_ext in [".pb", ".h5"]:
            # Visualize TensorFlow model
            try:
                import tensorflow as tf

                # Load model safely
                if file_ext == ".h5":
                    model = safe_load_model(full_path, model_type="tensorflow")

                    # Create visualization
                    vis_path = os.path.join(TEMP_DIR, f"{uuid.uuid4().hex}.png")

                    # Plot model
                    tf.keras.utils.plot_model(
                        model,
                        to_file=vis_path,
                        show_shapes=True,
                        show_layer_names=True
                    )

                    return vis_path
                else:
                    return "TensorFlow SavedModel visualization not supported"

            except Exception as e:
                logger.error(f"Error visualizing TensorFlow model: {str(e)}")
                return None

        else:
            # Unsupported model type
            return None

    except Exception as e:
        logger.error(f"Error visualizing model: {str(e)}")
        return None


def analyze_model_weights(model_path: str) -> Optional[str]:
    """
    Analyze model weights.

    Args:
        model_path: Path to the model file

    Returns:
        Path to the analysis image
    """
    try:
        # Check if model path is valid
        if not model_path:
            return None

        # Get full path
        full_path = os.path.join(MODEL_DIR, model_path)

        # Check if file exists
        if not os.path.exists(full_path):
            return None

        # Get file extension
        file_ext = Path(full_path).suffix.lower()

        # Create analysis based on model type
        if file_ext == ".npz":
            # Analyze TinyGrad model
            try:
                # Load model safely
                weights = safe_load_model(full_path, model_type="tinygrad")

                # Create visualization
                vis_path = os.path.join(TEMP_DIR, f"{uuid.uuid4().hex}.png")

                # Create figure
                fig, axs = plt.subplots(2, 2, figsize=(12, 10))

                # Plot weight distributions
                weight_names = list(weights.keys())

                # Plot histograms for up to 4 weight tensors
                for i, name in enumerate(weight_names[:4]):
                    row, col = i // 2, i % 2
                    weight = weights[name].flatten()
                    axs[row, col].hist(weight, bins=50)
                    axs[row, col].set_title(f"{name} (shape: {weights[name].shape})")
                    axs[row, col].set_xlabel("Value")
                    axs[row, col].set_ylabel("Frequency")

                # Add overall title
                fig.suptitle(f"Weight Distribution Analysis: {os.path.basename(model_path)}")

                # Save figure
                plt.tight_layout()
                plt.savefig(vis_path)
                plt.close(fig)

                return vis_path

            except Exception as e:
                logger.error(f"Error analyzing TinyGrad model: {str(e)}")
                return None

        elif file_ext == ".safetensors":
            # Analyze SafeTensors model
            try:
                from safetensors import safe_open

                # Load model safely
                weights = safe_load_model(full_path, model_type="safetensors")

                # Create visualization
                vis_path = os.path.join(TEMP_DIR, f"{uuid.uuid4().hex}.png")

                # Create figure
                fig, axs = plt.subplots(2, 2, figsize=(12, 10))

                # Get weight names
                weight_names = list(weights.keys())

                # Plot histograms for up to 4 weight tensors
                for i, name in enumerate(weight_names[:4]):
                    row, col = i // 2, i % 2
                    weight = weights.get_tensor(name).flatten()
                    axs[row, col].hist(weight, bins=50)
                    axs[row, col].set_title(f"{name} (shape: {weights.get_tensor(name).shape})")
                    axs[row, col].set_xlabel("Value")
                    axs[row, col].set_ylabel("Frequency")

                # Add overall title
                fig.suptitle(f"Weight Distribution Analysis: {os.path.basename(model_path)}")

                # Save figure
                plt.tight_layout()
                plt.savefig(vis_path)
                plt.close(fig)

                return vis_path

            except Exception as e:
                logger.error(f"Error analyzing SafeTensors model: {str(e)}")
                return None

        elif file_ext in [".pt", ".pth"]:
            # Analyze PyTorch model
            try:
                import torch

                # Load model safely
                state_dict = safe_load_model(full_path, model_type="pytorch")

                # Check if model is a state dict
                if isinstance(state_dict, dict) and "state_dict" in state_dict:
                    state_dict = state_dict["state_dict"]
                elif not isinstance(state_dict, dict) and hasattr(state_dict, "state_dict"):
                    state_dict = state_dict.state_dict()

                # Create visualization
                vis_path = os.path.join(TEMP_DIR, f"{uuid.uuid4().hex}.png")

                # Create figure
                fig, axs = plt.subplots(2, 2, figsize=(12, 10))

                # Get weight names
                weight_names = list(state_dict.keys())

                # Plot histograms for up to 4 weight tensors
                for i, name in enumerate(weight_names[:4]):
                    row, col = i // 2, i % 2
                    weight = state_dict[name].cpu().numpy().flatten()
                    axs[row, col].hist(weight, bins=50)
                    axs[row, col].set_title(f"{name} (shape: {state_dict[name].shape})")
                    axs[row, col].set_xlabel("Value")
                    axs[row, col].set_ylabel("Frequency")

                # Add overall title
                fig.suptitle(f"Weight Distribution Analysis: {os.path.basename(model_path)}")

                # Save figure
                plt.tight_layout()
                plt.savefig(vis_path)
                plt.close(fig)

                return vis_path

            except Exception as e:
                logger.error(f"Error analyzing PyTorch model: {str(e)}")
                return None

        else:
            # Unsupported model type
            return None

    except Exception as e:
        logger.error(f"Error analyzing model: {str(e)}")
        return None


def run_inference(model_path: str, image_path: str) -> Dict[str, Any]:
    """
    Run inference on an image.

    Args:
        model_path: Path to the model file
        image_path: Path to the image file

    Returns:
        Inference results
    """
    try:
        # Check if model path is valid
        if not model_path:
            structured_logger.warning("No model selected for inference", logger_name="model_manager")
            return {"error": "No model selected"}

        # Check if image path is valid
        if not image_path:
            structured_logger.warning("No image selected for inference", logger_name="model_manager")
            return {"error": "No image selected"}

        # Validate model path
        if not is_safe_path(MODEL_DIR, model_path):
            structured_logger.warning(
                "Invalid model path",
                logger_name="model_manager",
                model_path=model_path
            )
            return {"error": "Invalid model path"}

        # Get full paths
        model_full_path = os.path.join(MODEL_DIR, model_path)

        # Check if files exist
        if not os.path.exists(model_full_path):
            structured_logger.warning(
                "Model not found",
                logger_name="model_manager",
                model_path=model_path
            )
            return {"error": f"Model not found: {model_path}"}

        if not os.path.exists(image_path):
            structured_logger.warning(
                "Image not found",
                logger_name="model_manager",
                image_path=image_path
            )
            return {"error": f"Image not found: {image_path}"}

        # Check if model file is safe
        if not is_safe_file(model_full_path):
            structured_logger.warning(
                "Invalid model file",
                logger_name="model_manager",
                model_path=model_path
            )
            return {"error": "Invalid model file"}

        # Check if model format is supported
        if not validate_model_format(model_full_path):
            structured_logger.warning(
                "Unsupported model format",
                logger_name="model_manager",
                model_path=model_path,
                file_extension=Path(model_full_path).suffix.lower()
            )
            return {"error": f"Unsupported model format: {Path(model_full_path).suffix.lower()}"}

        # Determine backend
        backend = InferenceEngine.get_backend_from_file(model_full_path)

        # Initialize inference engine
        engine = InferenceEngine(backend=backend, model_path=model_full_path, device="AUTO")

        # Load and preprocess image
        import cv2

        # Load image
        img = cv2.imread(image_path)
        if img is None:
            return {"error": f"Failed to load image: {image_path}"}

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
        outputs = engine.predict({"input": img})

        # Process outputs
        results = {}

        # Check if output is classification
        if "output" in outputs:
            output = outputs["output"]
            if len(output.shape) == 2 and output.shape[1] > 1:
                # Classification output
                top_indices = np.argsort(output[0])[-5:][::-1]
                top_scores = output[0][top_indices]

                # Add top classes to results
                results["top_classes"] = [
                    {"class_id": int(idx), "score": float(score)}
                    for idx, score in zip(top_indices, top_scores)
                ]

                # Add inference time
                results["inference_time"] = float(engine.get_last_inference_time())

                # Add backend info
                results["backend"] = backend
                results["device"] = engine.device

        # Return results
        return results

    except Exception as e:
        # Log detailed error
        structured_logger.error(
            "Error running inference",
            logger_name="model_manager",
            error=str(e),
            traceback=traceback.format_exc(),
            model_path=model_path,
            image_path=image_path
        )

        # Create alert
        alert_manager.create_alert(
            level="error",
            message=f"Error running inference: {str(e)}",
            source="model_manager",
            model_path=model_path,
            image_path=image_path
        )

        # Return generic error message
        return {"error": "An error occurred while running inference. Please check the logs for details."}


def create_web_interface():
    """Create the web interface."""
    with gr.Blocks(title="Bulo.Cloud Sentinel Model Manager") as app:
        gr.Markdown("# Bulo.Cloud Sentinel Model Manager")
        gr.Markdown("Manage ML models for use with Bulo.Cloud Sentinel.")

        with gr.Tab("Model List"):
            with gr.Row():
                with gr.Column():
                    refresh_button = gr.Button("Refresh Model List")
                    model_table = gr.Dataframe(
                        headers=["Name", "Type", "Size (MB)", "Path"],
                        datatype=["str", "str", "number", "str"],
                        label="Available Models"
                    )

                    def refresh_model_list():
                        models = get_model_list()
                        return [[
                            model["name"],
                            model["type"],
                            round(model["size"], 2),
                            model["path"]
                        ] for model in models]

                    refresh_button.click(refresh_model_list, outputs=model_table)

        with gr.Tab("Upload Model"):
            with gr.Row():
                with gr.Column():
                    upload_file = gr.File(label="Upload Model File")
                    upload_button = gr.Button("Upload Model")
                    upload_status = gr.Textbox(label="Upload Status")

                    upload_button.click(
                        fn=lambda file: upload_model(file),
                        inputs=upload_file,
                        outputs=upload_status
                    )

        with gr.Tab("Model Info"):
            with gr.Row():
                with gr.Column():
                    info_model_dropdown = gr.Dropdown(label="Select Model")
                    info_button = gr.Button("Get Model Info")
                    model_info = gr.Textbox(label="Model Information", lines=20)

                    def update_model_dropdown():
                        models = get_model_list()
                        return gr.Dropdown.update(
                            choices=[model["path"] for model in models],
                            value=models[0]["path"] if models else None
                        )

                    info_button.click(
                        fn=lambda model: get_model_info(model),
                        inputs=info_model_dropdown,
                        outputs=model_info
                    )

        with gr.Tab("Visualize Model"):
            with gr.Row():
                with gr.Column():
                    vis_model_dropdown = gr.Dropdown(label="Select Model")
                    vis_button = gr.Button("Visualize Model")
                    vis_image = gr.Image(label="Model Visualization")

                    vis_button.click(
                        fn=lambda model: visualize_model(model),
                        inputs=vis_model_dropdown,
                        outputs=vis_image
                    )

        with gr.Tab("Analyze Weights"):
            with gr.Row():
                with gr.Column():
                    weight_model_dropdown = gr.Dropdown(label="Select Model")
                    weight_button = gr.Button("Analyze Weights")
                    weight_image = gr.Image(label="Weight Analysis")

                    weight_button.click(
                        fn=lambda model: analyze_model_weights(model),
                        inputs=weight_model_dropdown,
                        outputs=weight_image
                    )

        with gr.Tab("Run Inference"):
            with gr.Row():
                with gr.Column():
                    inf_model_dropdown = gr.Dropdown(label="Select Model")
                    inf_image = gr.Image(label="Upload Image", type="filepath")
                    inf_button = gr.Button("Run Inference")
                    inf_results = gr.JSON(label="Inference Results")

                    inf_button.click(
                        fn=lambda model, image: run_inference(model, image),
                        inputs=[inf_model_dropdown, inf_image],
                        outputs=inf_results
                    )

        with gr.Tab("Delete Model"):
            with gr.Row():
                with gr.Column():
                    del_model_dropdown = gr.Dropdown(label="Select Model")
                    del_button = gr.Button("Delete Model")
                    del_status = gr.Textbox(label="Delete Status")

                    del_button.click(
                        fn=lambda model: delete_model(model),
                        inputs=del_model_dropdown,
                        outputs=del_status
                    )

        # Update dropdowns when the interface is loaded
        app.load(
            fn=update_model_dropdown,
            outputs=[
                info_model_dropdown,
                vis_model_dropdown,
                weight_model_dropdown,
                inf_model_dropdown,
                del_model_dropdown
            ]
        )

        # Update dropdowns when the refresh button is clicked
        refresh_button.click(
            fn=update_model_dropdown,
            outputs=[
                info_model_dropdown,
                vis_model_dropdown,
                weight_model_dropdown,
                inf_model_dropdown,
                del_model_dropdown
            ]
        )

    return app


def main():
    """Main function."""
    # Initialize metrics collector
    metrics_collector.start()

    # Create configuration manager
    config = ConfigManager()

    # Create authentication manager
    auth_manager = AuthManager(
        secret_key=config.get("auth.jwt_secret"),
        token_expiration=config.get("auth.token_expiration", 3600),
        refresh_token_expiration=config.get("auth.refresh_token_expiration", 86400 * 7)
    )

    # Create web interface
    app = create_web_interface()

    # Launch web interface with security settings
    app.launch(
        server_name="0.0.0.0",
        server_port=7861,
        max_threads=10,  # Limit number of concurrent requests
        share=False,     # Disable public sharing
        auth=None,       # No authentication (add if needed)
        ssl_verify=True  # Verify SSL certificates
    )

    # Log startup
    structured_logger.info(
        "Model manager started",
        logger_name="model_manager",
        port=7861
    )


if __name__ == "__main__":
    main()
