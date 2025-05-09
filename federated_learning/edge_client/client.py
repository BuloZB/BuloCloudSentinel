#!/usr/bin/env python3
"""
Federated Learning Client for Bulo.CloudSentinel

This module implements a Flower client for federated learning on edge devices.
It trains a lightweight model head on locally extracted embeddings and shares
only model updates (gradients) with the server.
"""

import os
import sys
import time
import uuid
import json
import logging
import argparse
from pathlib import Path
from typing import Dict, List, Tuple, Optional, Union, Any

import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import DataLoader, Dataset
import flwr as fl
from flwr.common import NDArrays, Scalar
from opacus import PrivacyEngine
from opacus.validators import ModuleValidator
import yaml
import paho.mqtt.client as mqtt
import ssl

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
)
logger = logging.getLogger("edge_flower_client")

# Default configuration
DEFAULT_CONFIG = {
    "client": {
        "local_epochs": 3,
        "batch_size": 32,
        "learning_rate": 0.01,
        "optimizer": "adam",
        "device": "cuda" if torch.cuda.is_available() else "cpu",
        "embedding_dim": 512,
        "num_classes": 80,  # COCO dataset classes
        "model_head_type": "mlp",
        "embeddings_path": "/data/embeddings",
        "labels_path": "/data/labels",
    },
    "privacy": {
        "differential_privacy": True,
        "epsilon": 8.0,
        "delta": 1e-5,
        "max_grad_norm": 1.0,
        "noise_multiplier": 1.0,
    },
    "server": {
        "address": "fl_server:8080",
        "mqtt_broker": "mqtt_broker",
        "mqtt_port": 8883,
        "mqtt_topic": "fl/model_updates",
        "tls_enabled": True,
        "ca_cert_path": "/certs/ca.crt",
        "client_cert_path": "/certs/client.crt",
        "client_key_path": "/certs/client.key",
    },
}

class EmbeddingDataset(Dataset):
    """Dataset for loading embeddings and labels."""
    
    def __init__(self, embeddings_path: str, labels_path: str):
        """
        Initialize the dataset.
        
        Args:
            embeddings_path: Path to the embeddings directory
            labels_path: Path to the labels directory
        """
        self.embeddings_path = Path(embeddings_path)
        self.labels_path = Path(labels_path)
        
        # Get list of embedding files
        self.embedding_files = list(self.embeddings_path.glob("*.npy"))
        logger.info(f"Found {len(self.embedding_files)} embedding files")
        
    def __len__(self) -> int:
        return len(self.embedding_files)
    
    def __getitem__(self, idx: int) -> Tuple[torch.Tensor, torch.Tensor]:
        # Load embedding
        embedding_file = self.embedding_files[idx]
        embedding = np.load(embedding_file)
        
        # Load corresponding label
        label_file = self.labels_path / embedding_file.name.replace(".npy", ".json")
        with open(label_file, "r") as f:
            label_data = json.load(f)
        
        # Convert label to tensor
        label = torch.tensor(label_data["class_id"], dtype=torch.long)
        
        return torch.tensor(embedding, dtype=torch.float32), label

class ModelHead(nn.Module):
    """Lightweight model head for classification."""
    
    def __init__(self, embedding_dim: int, num_classes: int, head_type: str = "mlp"):
        """
        Initialize the model head.
        
        Args:
            embedding_dim: Dimension of the input embeddings
            num_classes: Number of output classes
            head_type: Type of head architecture (mlp or linear)
        """
        super(ModelHead, self).__init__()
        
        if head_type == "mlp":
            self.head = nn.Sequential(
                nn.Linear(embedding_dim, 256),
                nn.ReLU(),
                nn.Dropout(0.5),
                nn.Linear(256, num_classes)
            )
        elif head_type == "linear":
            self.head = nn.Linear(embedding_dim, num_classes)
        else:
            raise ValueError(f"Unsupported head type: {head_type}")
    
    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return self.head(x)

class FlowerClient(fl.client.NumPyClient):
    """Flower client for federated learning."""
    
    def __init__(self, config: Dict[str, Any]):
        """
        Initialize the Flower client.
        
        Args:
            config: Client configuration
        """
        self.config = config
        self.client_id = str(uuid.uuid4())[:8]
        logger.info(f"Initializing Flower client with ID: {self.client_id}")
        
        # Set device
        self.device = torch.device(config["client"]["device"])
        logger.info(f"Using device: {self.device}")
        
        # Initialize dataset
        self.dataset = EmbeddingDataset(
            config["client"]["embeddings_path"],
            config["client"]["labels_path"]
        )
        
        # Create data loader
        self.train_loader = DataLoader(
            self.dataset,
            batch_size=config["client"]["batch_size"],
            shuffle=True,
            num_workers=4,
            pin_memory=True if self.device.type == "cuda" else False
        )
        
        # Initialize model
        self.model = ModelHead(
            embedding_dim=config["client"]["embedding_dim"],
            num_classes=config["client"]["num_classes"],
            head_type=config["client"]["model_head_type"]
        )
        
        # Make model compatible with Opacus
        if config["privacy"]["differential_privacy"]:
            self.model = ModuleValidator.fix(self.model)
            ModuleValidator.validate(self.model, strict=False)
        
        self.model.to(self.device)
        
        # Initialize loss function
        self.criterion = nn.CrossEntropyLoss()
        
        # Initialize optimizer
        if config["client"]["optimizer"].lower() == "adam":
            self.optimizer = optim.Adam(
                self.model.parameters(),
                lr=config["client"]["learning_rate"]
            )
        elif config["client"]["optimizer"].lower() == "sgd":
            self.optimizer = optim.SGD(
                self.model.parameters(),
                lr=config["client"]["learning_rate"],
                momentum=0.9
            )
        else:
            raise ValueError(f"Unsupported optimizer: {config['client']['optimizer']}")
        
        # Initialize privacy engine
        if config["privacy"]["differential_privacy"]:
            self.privacy_engine = PrivacyEngine()
            self.model, self.optimizer, self.train_loader = self.privacy_engine.make_private_with_epsilon(
                module=self.model,
                optimizer=self.optimizer,
                data_loader=self.train_loader,
                epochs=config["client"]["local_epochs"],
                target_epsilon=config["privacy"]["epsilon"],
                target_delta=config["privacy"]["delta"],
                max_grad_norm=config["privacy"]["max_grad_norm"],
            )
            logger.info(f"Differential privacy enabled with ε={config['privacy']['epsilon']}, δ={config['privacy']['delta']}")
    
    def get_parameters(self, config: Dict[str, Any]) -> NDArrays:
        """
        Get model parameters as a list of NumPy arrays.
        
        Args:
            config: Configuration from server
            
        Returns:
            List of model parameters as NumPy arrays
        """
        # Remove device identifiers
        return [val.cpu().numpy() for _, val in self.model.state_dict().items()]
    
    def set_parameters(self, parameters: NDArrays) -> None:
        """
        Set model parameters from a list of NumPy arrays.
        
        Args:
            parameters: List of model parameters as NumPy arrays
        """
        params_dict = zip(self.model.state_dict().keys(), parameters)
        state_dict = {k: torch.tensor(v) for k, v in params_dict}
        self.model.load_state_dict(state_dict, strict=True)
    
    def fit(self, parameters: NDArrays, config: Dict[str, Any]) -> Tuple[NDArrays, int, Dict[str, Scalar]]:
        """
        Train the model on the local dataset.
        
        Args:
            parameters: Initial model parameters
            config: Training configuration
            
        Returns:
            Tuple of (updated model parameters, number of training examples, training metrics)
        """
        # Set model parameters
        self.set_parameters(parameters)
        
        # Get training config
        local_epochs = config.get("local_epochs", self.config["client"]["local_epochs"])
        
        # Train the model
        self.model.train()
        train_loss = 0.0
        train_acc = 0.0
        n_samples = 0
        
        for epoch in range(local_epochs):
            epoch_loss = 0.0
            epoch_correct = 0
            epoch_samples = 0
            
            for embeddings, labels in self.train_loader:
                # Move data to device
                embeddings, labels = embeddings.to(self.device), labels.to(self.device)
                
                # Zero the parameter gradients
                self.optimizer.zero_grad()
                
                # Forward pass
                outputs = self.model(embeddings)
                loss = self.criterion(outputs, labels)
                
                # Backward pass and optimize
                loss.backward()
                self.optimizer.step()
                
                # Calculate metrics
                _, predicted = torch.max(outputs.data, 1)
                epoch_correct += (predicted == labels).sum().item()
                epoch_loss += loss.item() * embeddings.size(0)
                epoch_samples += embeddings.size(0)
            
            # Log epoch metrics
            epoch_loss /= epoch_samples
            epoch_acc = epoch_correct / epoch_samples
            logger.info(f"Epoch {epoch+1}/{local_epochs} - Loss: {epoch_loss:.4f}, Acc: {epoch_acc:.4f}")
            
            train_loss += epoch_loss
            train_acc += epoch_acc
            n_samples = epoch_samples
        
        # Calculate average metrics
        train_loss /= local_epochs
        train_acc /= local_epochs
        
        # Get updated parameters
        parameters_updated = self.get_parameters(config)
        
        # Return updated parameters and metrics
        return parameters_updated, n_samples, {
            "loss": float(train_loss),
            "accuracy": float(train_acc),
            "dp_epsilon": float(self.privacy_engine.get_epsilon()) if self.config["privacy"]["differential_privacy"] else 0.0,
        }
    
    def evaluate(self, parameters: NDArrays, config: Dict[str, Any]) -> Tuple[float, int, Dict[str, Scalar]]:
        """
        Evaluate the model on the local dataset.
        
        Args:
            parameters: Model parameters
            config: Evaluation configuration
            
        Returns:
            Tuple of (loss, number of evaluation examples, evaluation metrics)
        """
        # Set model parameters
        self.set_parameters(parameters)
        
        # Evaluate the model
        self.model.eval()
        val_loss = 0.0
        val_correct = 0
        n_samples = 0
        
        with torch.no_grad():
            for embeddings, labels in self.train_loader:  # Using train_loader as validation for simplicity
                # Move data to device
                embeddings, labels = embeddings.to(self.device), labels.to(self.device)
                
                # Forward pass
                outputs = self.model(embeddings)
                loss = self.criterion(outputs, labels)
                
                # Calculate metrics
                _, predicted = torch.max(outputs.data, 1)
                val_correct += (predicted == labels).sum().item()
                val_loss += loss.item() * embeddings.size(0)
                n_samples += embeddings.size(0)
        
        # Calculate average metrics
        val_loss /= n_samples
        val_acc = val_correct / n_samples
        
        # Return metrics
        return float(val_loss), n_samples, {
            "accuracy": float(val_acc),
        }

def setup_mqtt_client(config: Dict[str, Any]) -> mqtt.Client:
    """
    Set up MQTT client for secure communication.
    
    Args:
        config: MQTT configuration
        
    Returns:
        Configured MQTT client
    """
    # Create MQTT client
    client = mqtt.Client(client_id=f"fl_client_{uuid.uuid4().hex[:8]}", protocol=mqtt.MQTTv5)
    
    # Set up TLS if enabled
    if config["server"]["tls_enabled"]:
        client.tls_set(
            ca_certs=config["server"]["ca_cert_path"],
            certfile=config["server"]["client_cert_path"],
            keyfile=config["server"]["client_key_path"],
            cert_reqs=ssl.CERT_REQUIRED,
            tls_version=ssl.PROTOCOL_TLS,
        )
    
    # Connect to broker
    client.connect(
        host=config["server"]["mqtt_broker"],
        port=config["server"]["mqtt_port"],
    )
    
    # Start the loop
    client.loop_start()
    
    return client

def load_config(config_path: Optional[str] = None) -> Dict[str, Any]:
    """
    Load configuration from file or use default.
    
    Args:
        config_path: Path to configuration file
        
    Returns:
        Configuration dictionary
    """
    config = DEFAULT_CONFIG.copy()
    
    if config_path and os.path.exists(config_path):
        with open(config_path, "r") as f:
            file_config = yaml.safe_load(f)
            
        # Update config with file values
        for section, values in file_config.items():
            if section in config:
                config[section].update(values)
            else:
                config[section] = values
    
    # Override with environment variables
    for section in config:
        for key in config[section]:
            env_var = f"FL_{section.upper()}_{key.upper()}"
            if env_var in os.environ:
                # Convert environment variable to appropriate type
                orig_value = config[section][key]
                env_value = os.environ[env_var]
                
                if isinstance(orig_value, bool):
                    config[section][key] = env_value.lower() in ("true", "1", "yes")
                elif isinstance(orig_value, int):
                    config[section][key] = int(env_value)
                elif isinstance(orig_value, float):
                    config[section][key] = float(env_value)
                else:
                    config[section][key] = env_value
    
    return config

def main():
    """Main function to start the Flower client."""
    parser = argparse.ArgumentParser(description="Federated Learning Client")
    parser.add_argument(
        "--config", type=str, default=None,
        help="Path to configuration file"
    )
    args = parser.parse_args()
    
    # Load configuration
    config = load_config(args.config)
    
    # Initialize client
    client = FlowerClient(config)
    
    # Start Flower client
    fl.client.start_numpy_client(
        server_address=config["server"]["address"],
        client=client,
    )

if __name__ == "__main__":
    main()
