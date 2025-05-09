#!/usr/bin/env python3
"""
Integration Test for Federated Learning in Bulo.CloudSentinel

This script tests the federated learning system by simulating multiple clients
with synthetic COCO subset data.
"""

import os
import sys
import time
import json
import logging
import argparse
import threading
from pathlib import Path
from typing import Dict, List, Tuple, Optional, Union, Any
import multiprocessing as mp

import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import DataLoader, Dataset
import flwr as fl
from flwr.common import NDArrays, Scalar
import yaml
import requests
from tqdm import tqdm

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
)
logger = logging.getLogger("fl_integration_test")

# Default configuration
DEFAULT_CONFIG = {
    "test": {
        "client_count": 3,
        "rounds": 5,
        "dataset_size": 100,
        "embedding_dim": 512,
        "num_classes": 80,
        "timeout_seconds": 600,
    },
    "server": {
        "address": "fl_server:8080",
        "api_url": "http://fl_server:8000",
    },
}

class SyntheticDataset(Dataset):
    """Synthetic dataset for testing."""
    
    def __init__(self, size: int, embedding_dim: int, num_classes: int, client_id: int):
        """
        Initialize the dataset.
        
        Args:
            size: Number of samples
            embedding_dim: Dimension of embeddings
            num_classes: Number of classes
            client_id: Client ID for data partitioning
        """
        self.size = size
        self.embedding_dim = embedding_dim
        self.num_classes = num_classes
        self.client_id = client_id
        
        # Generate synthetic data
        np.random.seed(42 + client_id)  # Different seed for each client
        self.embeddings = np.random.randn(size, embedding_dim).astype(np.float32)
        self.labels = np.random.randint(0, num_classes, size=size).astype(np.int64)
        
        logger.info(f"Created synthetic dataset for client {client_id} with {size} samples")
    
    def __len__(self) -> int:
        return self.size
    
    def __getitem__(self, idx: int) -> Tuple[torch.Tensor, torch.Tensor]:
        return torch.tensor(self.embeddings[idx]), torch.tensor(self.labels[idx])

class ModelHead(nn.Module):
    """Lightweight model head for classification."""
    
    def __init__(self, embedding_dim: int, num_classes: int):
        """
        Initialize the model head.
        
        Args:
            embedding_dim: Dimension of the input embeddings
            num_classes: Number of output classes
        """
        super(ModelHead, self).__init__()
        
        self.head = nn.Sequential(
            nn.Linear(embedding_dim, 256),
            nn.ReLU(),
            nn.Dropout(0.5),
            nn.Linear(256, num_classes)
        )
    
    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return self.head(x)

class FlowerClient(fl.client.NumPyClient):
    """Flower client for testing."""
    
    def __init__(self, client_id: int, config: Dict[str, Any]):
        """
        Initialize the Flower client.
        
        Args:
            client_id: Client ID
            config: Client configuration
        """
        self.client_id = client_id
        self.config = config
        logger.info(f"Initializing Flower client {client_id}")
        
        # Set device
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        
        # Initialize dataset
        self.dataset = SyntheticDataset(
            size=config["test"]["dataset_size"],
            embedding_dim=config["test"]["embedding_dim"],
            num_classes=config["test"]["num_classes"],
            client_id=client_id,
        )
        
        # Create data loader
        self.train_loader = DataLoader(
            self.dataset,
            batch_size=32,
            shuffle=True,
            num_workers=0,
        )
        
        # Initialize model
        self.model = ModelHead(
            embedding_dim=config["test"]["embedding_dim"],
            num_classes=config["test"]["num_classes"],
        )
        
        self.model.to(self.device)
        
        # Initialize loss function
        self.criterion = nn.CrossEntropyLoss()
        
        # Initialize optimizer
        self.optimizer = optim.Adam(self.model.parameters(), lr=0.01)
    
    def get_parameters(self, config: Dict[str, Any]) -> NDArrays:
        """
        Get model parameters as a list of NumPy arrays.
        
        Args:
            config: Configuration from server
            
        Returns:
            List of model parameters as NumPy arrays
        """
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
        local_epochs = 1  # Use a single epoch for testing
        
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
            logger.info(f"Client {self.client_id} - Epoch {epoch+1}/{local_epochs} - Loss: {epoch_loss:.4f}, Acc: {epoch_acc:.4f}")
            
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
        
        logger.info(f"Client {self.client_id} - Evaluation - Loss: {val_loss:.4f}, Acc: {val_acc:.4f}")
        
        # Return metrics
        return float(val_loss), n_samples, {
            "accuracy": float(val_acc),
        }

def start_client(client_id: int, config: Dict[str, Any]) -> None:
    """
    Start a Flower client.
    
    Args:
        client_id: Client ID
        config: Client configuration
    """
    # Initialize client
    client = FlowerClient(client_id, config)
    
    # Start Flower client
    fl.client.start_numpy_client(
        server_address=config["server"]["address"],
        client=client,
    )

def start_federated_learning(config: Dict[str, Any]) -> None:
    """
    Start federated learning.
    
    Args:
        config: Test configuration
    """
    # Start server via API
    try:
        response = requests.post(f"{config['server']['api_url']}/start")
        if response.status_code != 200:
            logger.error(f"Failed to start server: {response.text}")
            return
        logger.info("Started federated learning server")
    except Exception as e:
        logger.error(f"Error starting server: {e}")
        return
    
    # Wait for server to start
    time.sleep(5)
    
    # Start clients
    processes = []
    for i in range(config["test"]["client_count"]):
        p = mp.Process(target=start_client, args=(i, config))
        p.start()
        processes.append(p)
        logger.info(f"Started client {i}")
    
    # Wait for clients to finish
    for p in processes:
        p.join(timeout=config["test"]["timeout_seconds"])
        if p.is_alive():
            logger.warning("Client process did not finish in time, terminating")
            p.terminate()
    
    logger.info("All clients finished")

def load_config() -> Dict[str, Any]:
    """
    Load configuration from environment variables or use default.
    
    Returns:
        Configuration dictionary
    """
    config = DEFAULT_CONFIG.copy()
    
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
    """Main function to run the integration test."""
    # Load configuration
    config = load_config()
    
    logger.info(f"Starting integration test with {config['test']['client_count']} clients")
    
    # Start federated learning
    start_federated_learning(config)
    
    # Check if models were saved
    try:
        response = requests.get(f"{config['server']['api_url']}/status")
        if response.status_code == 200:
            status = response.json()
            logger.info(f"Server status: {status}")
            
            if status.get("current_round", 0) >= config["test"]["rounds"]:
                logger.info("Integration test passed: All rounds completed")
            else:
                logger.warning(f"Integration test warning: Only {status.get('current_round', 0)}/{config['test']['rounds']} rounds completed")
        else:
            logger.error(f"Failed to get server status: {response.text}")
    except Exception as e:
        logger.error(f"Error checking server status: {e}")
    
    logger.info("Integration test completed")

if __name__ == "__main__":
    main()
