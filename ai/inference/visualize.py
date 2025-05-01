#!/usr/bin/env python3
"""
Model visualization tool for Bulo.Cloud Sentinel.

This script provides utilities for visualizing model architectures and weights.
"""

import os
import sys
import argparse
import logging
import tempfile
import uuid
from pathlib import Path
from typing import Dict, Any, Optional, List, Tuple, Union
import numpy as np
import matplotlib.pyplot as plt

# Setup logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


def visualize_model_architecture(model_path: str, output_path: Optional[str] = None) -> Optional[str]:
    """
    Visualize model architecture.
    
    Args:
        model_path: Path to the model file
        output_path: Path to save the visualization image
        
    Returns:
        Path to the visualization image or None if visualization failed
    """
    try:
        # Check if model path is valid
        if not os.path.exists(model_path):
            logger.error(f"Model not found: {model_path}")
            return None
        
        # Get file extension
        file_ext = Path(model_path).suffix.lower()
        
        # Create output path if not provided
        if output_path is None:
            output_path = os.path.join(
                tempfile.gettempdir(),
                f"model_architecture_{uuid.uuid4().hex}.png"
            )
        
        # Create visualization based on model type
        if file_ext in [".pt", ".pth"]:
            # Visualize PyTorch model
            try:
                import torch
                import torchviz
                
                logger.info(f"Visualizing PyTorch model: {model_path}")
                
                # Load model
                model = torch.load(model_path, map_location="cpu")
                
                # Check if model is a state dict
                if isinstance(model, dict) and "state_dict" in model:
                    # If model definition is available
                    if "model_definition" in model:
                        model_class = model["model_definition"]
                        model_instance = model_class()
                        model_instance.load_state_dict(model["state_dict"])
                        model = model_instance
                    else:
                        logger.error("Model is a state dict without model definition")
                        return None
                
                # Create dummy input
                dummy_input = torch.randn(1, 3, 224, 224)
                
                # Create graph
                graph = torchviz.make_dot(model(dummy_input), params=dict(model.named_parameters()))
                
                # Save graph
                graph.render(output_path, format="png")
                
                return f"{output_path}.png"
            
            except Exception as e:
                logger.error(f"Error visualizing PyTorch model: {str(e)}")
                return None
        
        elif file_ext == ".onnx":
            # Visualize ONNX model
            try:
                import onnx
                from onnx.tools.net_drawer import GetPydotGraph, GetOpNodeProducer
                
                logger.info(f"Visualizing ONNX model: {model_path}")
                
                # Load model
                model = onnx.load(model_path)
                
                # Create graph
                pydot_graph = GetPydotGraph(
                    model.graph,
                    name=model.graph.name,
                    rankdir="TB",
                    node_producer=GetOpNodeProducer("docstring")
                )
                
                # Save graph
                pydot_graph.write_png(output_path)
                
                return output_path
            
            except Exception as e:
                logger.error(f"Error visualizing ONNX model: {str(e)}")
                return None
        
        elif file_ext in [".pb", ".h5"]:
            # Visualize TensorFlow model
            try:
                import tensorflow as tf
                
                logger.info(f"Visualizing TensorFlow model: {model_path}")
                
                # Load model
                if file_ext == ".h5":
                    model = tf.keras.models.load_model(model_path)
                    
                    # Plot model
                    tf.keras.utils.plot_model(
                        model,
                        to_file=output_path,
                        show_shapes=True,
                        show_layer_names=True
                    )
                    
                    return output_path
                else:
                    logger.error("TensorFlow SavedModel visualization not supported")
                    return None
            
            except Exception as e:
                logger.error(f"Error visualizing TensorFlow model: {str(e)}")
                return None
        
        else:
            logger.error(f"Unsupported model format: {file_ext}")
            return None
    
    except Exception as e:
        logger.error(f"Error visualizing model architecture: {str(e)}")
        return None


def visualize_model_weights(model_path: str, output_path: Optional[str] = None) -> Optional[str]:
    """
    Visualize model weights.
    
    Args:
        model_path: Path to the model file
        output_path: Path to save the visualization image
        
    Returns:
        Path to the visualization image or None if visualization failed
    """
    try:
        # Check if model path is valid
        if not os.path.exists(model_path):
            logger.error(f"Model not found: {model_path}")
            return None
        
        # Get file extension
        file_ext = Path(model_path).suffix.lower()
        
        # Create output path if not provided
        if output_path is None:
            output_path = os.path.join(
                tempfile.gettempdir(),
                f"model_weights_{uuid.uuid4().hex}.png"
            )
        
        # Create visualization based on model type
        if file_ext == ".npz":
            # Visualize TinyGrad model
            try:
                logger.info(f"Visualizing TinyGrad model weights: {model_path}")
                
                # Load model
                weights = np.load(model_path)
                
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
                plt.savefig(output_path)
                plt.close(fig)
                
                return output_path
            
            except Exception as e:
                logger.error(f"Error visualizing TinyGrad model weights: {str(e)}")
                return None
        
        elif file_ext == ".safetensors":
            # Visualize SafeTensors model
            try:
                from safetensors import safe_open
                
                logger.info(f"Visualizing SafeTensors model weights: {model_path}")
                
                # Load model
                with safe_open(model_path, framework="numpy") as weights:
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
                    plt.savefig(output_path)
                    plt.close(fig)
                    
                    return output_path
            
            except Exception as e:
                logger.error(f"Error visualizing SafeTensors model weights: {str(e)}")
                return None
        
        elif file_ext in [".pt", ".pth"]:
            # Visualize PyTorch model
            try:
                import torch
                
                logger.info(f"Visualizing PyTorch model weights: {model_path}")
                
                # Load model
                state_dict = torch.load(model_path, map_location="cpu")
                
                # Check if model is a state dict
                if isinstance(state_dict, dict) and "state_dict" in state_dict:
                    state_dict = state_dict["state_dict"]
                elif not isinstance(state_dict, dict) and hasattr(state_dict, "state_dict"):
                    state_dict = state_dict.state_dict()
                
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
                plt.savefig(output_path)
                plt.close(fig)
                
                return output_path
            
            except Exception as e:
                logger.error(f"Error visualizing PyTorch model weights: {str(e)}")
                return None
        
        elif file_ext in [".pb", ".h5"]:
            # Visualize TensorFlow model
            try:
                import tensorflow as tf
                
                logger.info(f"Visualizing TensorFlow model weights: {model_path}")
                
                # Load model
                if file_ext == ".h5":
                    model = tf.keras.models.load_model(model_path)
                    
                    # Create figure
                    fig, axs = plt.subplots(2, 2, figsize=(12, 10))
                    
                    # Get weights
                    weights = model.get_weights()
                    
                    # Plot histograms for up to 4 weight tensors
                    for i, weight in enumerate(weights[:4]):
                        row, col = i // 2, i % 2
                        weight_flat = weight.flatten()
                        axs[row, col].hist(weight_flat, bins=50)
                        axs[row, col].set_title(f"Weight {i} (shape: {weight.shape})")
                        axs[row, col].set_xlabel("Value")
                        axs[row, col].set_ylabel("Frequency")
                    
                    # Add overall title
                    fig.suptitle(f"Weight Distribution Analysis: {os.path.basename(model_path)}")
                    
                    # Save figure
                    plt.tight_layout()
                    plt.savefig(output_path)
                    plt.close(fig)
                    
                    return output_path
                else:
                    logger.error("TensorFlow SavedModel weight visualization not supported")
                    return None
            
            except Exception as e:
                logger.error(f"Error visualizing TensorFlow model weights: {str(e)}")
                return None
        
        else:
            logger.error(f"Unsupported model format: {file_ext}")
            return None
    
    except Exception as e:
        logger.error(f"Error visualizing model weights: {str(e)}")
        return None


def visualize_model_activations(
    model_path: str,
    input_path: str,
    output_path: Optional[str] = None
) -> Optional[str]:
    """
    Visualize model activations.
    
    Args:
        model_path: Path to the model file
        input_path: Path to the input image
        output_path: Path to save the visualization image
        
    Returns:
        Path to the visualization image or None if visualization failed
    """
    try:
        # Check if model path is valid
        if not os.path.exists(model_path):
            logger.error(f"Model not found: {model_path}")
            return None
        
        # Check if input path is valid
        if not os.path.exists(input_path):
            logger.error(f"Input not found: {input_path}")
            return None
        
        # Get file extension
        file_ext = Path(model_path).suffix.lower()
        
        # Create output path if not provided
        if output_path is None:
            output_path = os.path.join(
                tempfile.gettempdir(),
                f"model_activations_{uuid.uuid4().hex}.png"
            )
        
        # Create visualization based on model type
        if file_ext in [".pt", ".pth"]:
            # Visualize PyTorch model
            try:
                import torch
                import cv2
                from torchvision import transforms
                
                logger.info(f"Visualizing PyTorch model activations: {model_path}")
                
                # Load model
                model = torch.load(model_path, map_location="cpu")
                
                # Check if model is a state dict
                if isinstance(model, dict) and "state_dict" in model:
                    # If model definition is available
                    if "model_definition" in model:
                        model_class = model["model_definition"]
                        model_instance = model_class()
                        model_instance.load_state_dict(model["state_dict"])
                        model = model_instance
                    else:
                        logger.error("Model is a state dict without model definition")
                        return None
                
                # Set model to evaluation mode
                if hasattr(model, "eval"):
                    model.eval()
                
                # Load and preprocess image
                img = cv2.imread(input_path)
                img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                img = cv2.resize(img, (224, 224))
                
                # Convert to tensor
                transform = transforms.Compose([
                    transforms.ToTensor(),
                    transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
                ])
                img_tensor = transform(img).unsqueeze(0)
                
                # Register hooks to get activations
                activations = {}
                
                def get_activation(name):
                    def hook(model, input, output):
                        activations[name] = output.detach()
                    return hook
                
                # Register hooks for convolutional layers
                for name, module in model.named_modules():
                    if isinstance(module, torch.nn.Conv2d):
                        module.register_forward_hook(get_activation(name))
                
                # Forward pass
                with torch.no_grad():
                    model(img_tensor)
                
                # Create figure
                num_layers = min(len(activations), 16)
                rows = int(np.ceil(np.sqrt(num_layers)))
                cols = int(np.ceil(num_layers / rows))
                
                fig, axs = plt.subplots(rows, cols, figsize=(cols * 3, rows * 3))
                
                # Plot activations
                for i, (name, activation) in enumerate(list(activations.items())[:num_layers]):
                    if rows == 1:
                        ax = axs[i]
                    else:
                        row, col = i // cols, i % cols
                        ax = axs[row, col]
                    
                    # Get first channel of first sample
                    act = activation[0, 0].cpu().numpy()
                    
                    # Plot activation
                    im = ax.imshow(act, cmap="viridis")
                    ax.set_title(f"{name}")
                    ax.axis("off")
                    
                    # Add colorbar
                    plt.colorbar(im, ax=ax)
                
                # Hide empty subplots
                for i in range(num_layers, rows * cols):
                    if rows == 1:
                        axs[i].axis("off")
                    else:
                        row, col = i // cols, i % cols
                        axs[row, col].axis("off")
                
                # Add overall title
                fig.suptitle(f"Activation Maps: {os.path.basename(model_path)}")
                
                # Save figure
                plt.tight_layout()
                plt.savefig(output_path)
                plt.close(fig)
                
                return output_path
            
            except Exception as e:
                logger.error(f"Error visualizing PyTorch model activations: {str(e)}")
                return None
        
        elif file_ext in [".pb", ".h5"]:
            # Visualize TensorFlow model
            try:
                import tensorflow as tf
                import cv2
                
                logger.info(f"Visualizing TensorFlow model activations: {model_path}")
                
                # Load model
                if file_ext == ".h5":
                    model = tf.keras.models.load_model(model_path)
                    
                    # Load and preprocess image
                    img = cv2.imread(input_path)
                    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                    img = cv2.resize(img, (224, 224))
                    img = img.astype(np.float32) / 255.0
                    img = np.expand_dims(img, axis=0)
                    
                    # Create a model that outputs layer activations
                    layer_outputs = [layer.output for layer in model.layers if isinstance(
                        layer, tf.keras.layers.Conv2D
                    )]
                    activation_model = tf.keras.models.Model(
                        inputs=model.input, outputs=layer_outputs
                    )
                    
                    # Get activations
                    activations = activation_model.predict(img)
                    
                    # Create figure
                    num_layers = min(len(activations), 16)
                    rows = int(np.ceil(np.sqrt(num_layers)))
                    cols = int(np.ceil(num_layers / rows))
                    
                    fig, axs = plt.subplots(rows, cols, figsize=(cols * 3, rows * 3))
                    
                    # Plot activations
                    for i, activation in enumerate(activations[:num_layers]):
                        if rows == 1:
                            ax = axs[i]
                        else:
                            row, col = i // cols, i % cols
                            ax = axs[row, col]
                        
                        # Get first channel of first sample
                        act = activation[0, :, :, 0]
                        
                        # Plot activation
                        im = ax.imshow(act, cmap="viridis")
                        ax.set_title(f"Layer {i+1}")
                        ax.axis("off")
                        
                        # Add colorbar
                        plt.colorbar(im, ax=ax)
                    
                    # Hide empty subplots
                    for i in range(num_layers, rows * cols):
                        if rows == 1:
                            axs[i].axis("off")
                        else:
                            row, col = i // cols, i % cols
                            axs[row, col].axis("off")
                    
                    # Add overall title
                    fig.suptitle(f"Activation Maps: {os.path.basename(model_path)}")
                    
                    # Save figure
                    plt.tight_layout()
                    plt.savefig(output_path)
                    plt.close(fig)
                    
                    return output_path
                else:
                    logger.error("TensorFlow SavedModel activation visualization not supported")
                    return None
            
            except Exception as e:
                logger.error(f"Error visualizing TensorFlow model activations: {str(e)}")
                return None
        
        else:
            logger.error(f"Unsupported model format: {file_ext}")
            return None
    
    except Exception as e:
        logger.error(f"Error visualizing model activations: {str(e)}")
        return None


def visualize_model_gradients(
    model_path: str,
    input_path: str,
    output_path: Optional[str] = None
) -> Optional[str]:
    """
    Visualize model gradients.
    
    Args:
        model_path: Path to the model file
        input_path: Path to the input image
        output_path: Path to save the visualization image
        
    Returns:
        Path to the visualization image or None if visualization failed
    """
    try:
        # Check if model path is valid
        if not os.path.exists(model_path):
            logger.error(f"Model not found: {model_path}")
            return None
        
        # Check if input path is valid
        if not os.path.exists(input_path):
            logger.error(f"Input not found: {input_path}")
            return None
        
        # Get file extension
        file_ext = Path(model_path).suffix.lower()
        
        # Create output path if not provided
        if output_path is None:
            output_path = os.path.join(
                tempfile.gettempdir(),
                f"model_gradients_{uuid.uuid4().hex}.png"
            )
        
        # Create visualization based on model type
        if file_ext in [".pt", ".pth"]:
            # Visualize PyTorch model
            try:
                import torch
                import cv2
                from torchvision import transforms
                
                logger.info(f"Visualizing PyTorch model gradients: {model_path}")
                
                # Load model
                model = torch.load(model_path, map_location="cpu")
                
                # Check if model is a state dict
                if isinstance(model, dict) and "state_dict" in model:
                    # If model definition is available
                    if "model_definition" in model:
                        model_class = model["model_definition"]
                        model_instance = model_class()
                        model_instance.load_state_dict(model["state_dict"])
                        model = model_instance
                    else:
                        logger.error("Model is a state dict without model definition")
                        return None
                
                # Set model to evaluation mode
                if hasattr(model, "eval"):
                    model.eval()
                
                # Load and preprocess image
                img = cv2.imread(input_path)
                img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                img = cv2.resize(img, (224, 224))
                
                # Convert to tensor
                transform = transforms.Compose([
                    transforms.ToTensor(),
                    transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
                ])
                img_tensor = transform(img).unsqueeze(0)
                img_tensor.requires_grad = True
                
                # Forward pass
                output = model(img_tensor)
                
                # Get predicted class
                if len(output.shape) == 2:
                    # Classification model
                    pred_class = output.argmax(dim=1)
                    
                    # Backward pass
                    model.zero_grad()
                    output[0, pred_class].backward()
                    
                    # Get gradients
                    gradients = img_tensor.grad[0].cpu().numpy()
                    
                    # Create figure
                    fig, axs = plt.subplots(1, 3, figsize=(15, 5))
                    
                    # Plot original image
                    axs[0].imshow(img)
                    axs[0].set_title("Original Image")
                    axs[0].axis("off")
                    
                    # Plot gradients
                    for i in range(3):
                        # Normalize gradients
                        grad = gradients[i]
                        grad = (grad - grad.min()) / (grad.max() - grad.min() + 1e-8)
                        
                        # Plot gradient
                        axs[i+1].imshow(grad, cmap="viridis")
                        axs[i+1].set_title(f"Gradient (Channel {i+1})")
                        axs[i+1].axis("off")
                    
                    # Add overall title
                    fig.suptitle(f"Gradient Visualization: {os.path.basename(model_path)}")
                    
                    # Save figure
                    plt.tight_layout()
                    plt.savefig(output_path)
                    plt.close(fig)
                    
                    return output_path
                else:
                    logger.error("Model output is not a classification output")
                    return None
            
            except Exception as e:
                logger.error(f"Error visualizing PyTorch model gradients: {str(e)}")
                return None
        
        elif file_ext in [".pb", ".h5"]:
            # Visualize TensorFlow model
            try:
                import tensorflow as tf
                import cv2
                
                logger.info(f"Visualizing TensorFlow model gradients: {model_path}")
                
                # Load model
                if file_ext == ".h5":
                    model = tf.keras.models.load_model(model_path)
                    
                    # Load and preprocess image
                    img = cv2.imread(input_path)
                    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                    img = cv2.resize(img, (224, 224))
                    img = img.astype(np.float32) / 255.0
                    img = np.expand_dims(img, axis=0)
                    
                    # Create gradient model
                    with tf.GradientTape() as tape:
                        # Convert image to tensor
                        img_tensor = tf.convert_to_tensor(img)
                        tape.watch(img_tensor)
                        
                        # Forward pass
                        output = model(img_tensor)
                        
                        # Get predicted class
                        if len(output.shape) == 2:
                            # Classification model
                            pred_class = tf.argmax(output[0])
                            
                            # Get gradients
                            gradients = tape.gradient(output[0, pred_class], img_tensor)[0]
                            
                            # Create figure
                            fig, axs = plt.subplots(1, 4, figsize=(20, 5))
                            
                            # Plot original image
                            axs[0].imshow(img[0])
                            axs[0].set_title("Original Image")
                            axs[0].axis("off")
                            
                            # Plot gradients
                            for i in range(3):
                                # Normalize gradients
                                grad = gradients[:, :, i].numpy()
                                grad = (grad - grad.min()) / (grad.max() - grad.min() + 1e-8)
                                
                                # Plot gradient
                                axs[i+1].imshow(grad, cmap="viridis")
                                axs[i+1].set_title(f"Gradient (Channel {i+1})")
                                axs[i+1].axis("off")
                            
                            # Add overall title
                            fig.suptitle(f"Gradient Visualization: {os.path.basename(model_path)}")
                            
                            # Save figure
                            plt.tight_layout()
                            plt.savefig(output_path)
                            plt.close(fig)
                            
                            return output_path
                        else:
                            logger.error("Model output is not a classification output")
                            return None
                else:
                    logger.error("TensorFlow SavedModel gradient visualization not supported")
                    return None
            
            except Exception as e:
                logger.error(f"Error visualizing TensorFlow model gradients: {str(e)}")
                return None
        
        else:
            logger.error(f"Unsupported model format: {file_ext}")
            return None
    
    except Exception as e:
        logger.error(f"Error visualizing model gradients: {str(e)}")
        return None


def main():
    """Main function."""
    parser = argparse.ArgumentParser(description="Model visualization tool for Bulo.Cloud Sentinel")
    parser.add_argument("model", help="Path to the model file")
    parser.add_argument("--output", help="Path to save the visualization image")
    parser.add_argument("--type", choices=["architecture", "weights", "activations", "gradients"],
                        default="architecture", help="Type of visualization")
    parser.add_argument("--input", help="Path to the input image (required for activations and gradients)")
    args = parser.parse_args()
    
    # Check if input is provided for activations and gradients
    if args.type in ["activations", "gradients"] and not args.input:
        logger.error(f"Input image is required for {args.type} visualization")
        sys.exit(1)
    
    # Visualize model
    if args.type == "architecture":
        output_path = visualize_model_architecture(args.model, args.output)
    elif args.type == "weights":
        output_path = visualize_model_weights(args.model, args.output)
    elif args.type == "activations":
        output_path = visualize_model_activations(args.model, args.input, args.output)
    elif args.type == "gradients":
        output_path = visualize_model_gradients(args.model, args.input, args.output)
    else:
        logger.error(f"Unsupported visualization type: {args.type}")
        sys.exit(1)
    
    # Check if visualization was successful
    if output_path:
        logger.info(f"Visualization saved to {output_path}")
    else:
        logger.error("Visualization failed")
        sys.exit(1)


if __name__ == "__main__":
    main()
