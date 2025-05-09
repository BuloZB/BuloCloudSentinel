#!/usr/bin/env python3
"""
Federated Learning Server for Bulo.CloudSentinel

This module implements a Flower server for federated learning.
It aggregates model updates from edge devices and saves the
aggregated model to the Model Hub.
"""

import os
import sys
import time
import json
import logging
import argparse
from pathlib import Path
from typing import Dict, List, Tuple, Optional, Union, Any, Callable
from collections import OrderedDict
import threading

import numpy as np
import torch
import torch.nn as nn
import flwr as fl
from flwr.common import Metrics, NDArrays, Parameters, Scalar, FitRes, EvaluateRes
from flwr.server.client_proxy import ClientProxy
from flwr.server.strategy import FedAvg
import yaml
import paho.mqtt.client as mqtt
import ssl
import requests
from fastapi import FastAPI, UploadFile, File, Form, HTTPException, BackgroundTasks
from fastapi.responses import JSONResponse
import uvicorn
from datetime import datetime

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
)
logger = logging.getLogger("fl_server")

# Default configuration
DEFAULT_CONFIG = {
    "server": {
        "address": "0.0.0.0:8080",
        "min_clients": 2,
        "max_clients": 100,
        "rounds": 50,
        "aggregation_method": "fedavg",
        "timeout_seconds": 600,
        "mqtt_broker": "mqtt_broker",
        "mqtt_port": 8883,
        "mqtt_topic": "fl/model_updates",
        "tls_enabled": True,
        "ca_cert_path": "/certs/ca.crt",
        "server_cert_path": "/certs/server.crt",
        "server_key_path": "/certs/server.key",
    },
    "model": {
        "embedding_dim": 512,
        "num_classes": 80,  # COCO dataset classes
        "model_head_type": "mlp",
        "save_path": "/models",
    },
    "model_hub": {
        "api_url": "http://model-hub-service:8000/api/v1",
        "api_key": "",
    },
}

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

class FedAvgWithModelSaving(FedAvg):
    """FedAvg strategy with model saving capability."""
    
    def __init__(
        self,
        *args,
        config: Dict[str, Any],
        **kwargs
    ):
        super().__init__(*args, **kwargs)
        self.config = config
        self.current_round = 0
        self.mqtt_client = setup_mqtt_client(config) if config["server"]["mqtt_broker"] else None
    
    def aggregate_fit(
        self,
        server_round: int,
        results: List[Tuple[ClientProxy, FitRes]],
        failures: List[Union[Tuple[ClientProxy, FitRes], BaseException]],
    ) -> Tuple[Optional[Parameters], Dict[str, Scalar]]:
        """
        Aggregate model updates from clients.
        
        Args:
            server_round: Current round number
            results: List of client results
            failures: List of client failures
            
        Returns:
            Tuple of (aggregated parameters, metrics)
        """
        # Call parent's aggregate_fit
        aggregated_parameters, metrics = super().aggregate_fit(server_round, results, failures)
        
        if aggregated_parameters is not None:
            # Convert parameters to NumPy arrays
            aggregated_ndarrays = fl.common.parameters_to_ndarrays(aggregated_parameters)
            
            # Save model
            self.save_model(server_round, aggregated_ndarrays, metrics)
            
            # Publish model update via MQTT if enabled
            if self.mqtt_client:
                self.publish_model_update(server_round, metrics)
        
        self.current_round = server_round
        return aggregated_parameters, metrics
    
    def save_model(
        self,
        server_round: int,
        parameters: NDArrays,
        metrics: Dict[str, Scalar],
    ) -> None:
        """
        Save the aggregated model.
        
        Args:
            server_round: Current round number
            parameters: Aggregated model parameters
            metrics: Aggregation metrics
        """
        # Initialize model
        model = ModelHead(
            embedding_dim=self.config["model"]["embedding_dim"],
            num_classes=self.config["model"]["num_classes"],
            head_type=self.config["model"]["model_head_type"]
        )
        
        # Load parameters into model
        params_dict = zip(model.state_dict().keys(), parameters)
        state_dict = OrderedDict({k: torch.tensor(v) for k, v in params_dict})
        model.load_state_dict(state_dict, strict=True)
        
        # Create save directory if it doesn't exist
        save_dir = Path(self.config["model"]["save_path"])
        save_dir.mkdir(parents=True, exist_ok=True)
        
        # Save model
        model_path = save_dir / f"model_round_{server_round}.pt"
        torch.save({
            "round": server_round,
            "state_dict": model.state_dict(),
            "metrics": metrics,
            "timestamp": datetime.now().isoformat(),
        }, model_path)
        
        logger.info(f"Saved model to {model_path}")
        
        # Register model with Model Hub
        self.register_model_with_hub(model_path, server_round, metrics)
    
    def register_model_with_hub(
        self,
        model_path: Path,
        server_round: int,
        metrics: Dict[str, Scalar],
    ) -> None:
        """
        Register the model with the Model Hub.
        
        Args:
            model_path: Path to the saved model
            server_round: Current round number
            metrics: Model metrics
        """
        if not self.config["model_hub"]["api_url"]:
            logger.warning("Model Hub API URL not configured, skipping registration")
            return
        
        try:
            # Prepare model metadata
            model_name = f"federated_model_round_{server_round}"
            model_version = f"1.0.{server_round}"
            model_type = "classification"
            
            # Create multipart form data
            files = {
                "file": open(model_path, "rb"),
            }
            data = {
                "name": model_name,
                "version": model_version,
                "model_type": model_type,
                "framework": "pytorch",
                "description": f"Federated learning model from round {server_round}",
                "metrics": json.dumps(metrics),
            }
            
            # Set headers
            headers = {}
            if self.config["model_hub"]["api_key"]:
                headers["Authorization"] = f"Bearer {self.config['model_hub']['api_key']}"
            
            # Register model with Model Hub
            response = requests.post(
                f"{self.config['model_hub']['api_url']}/models",
                files=files,
                data=data,
                headers=headers,
            )
            
            if response.status_code == 200 or response.status_code == 201:
                logger.info(f"Successfully registered model with Model Hub: {response.json()}")
            else:
                logger.error(f"Failed to register model with Model Hub: {response.text}")
        
        except Exception as e:
            logger.error(f"Error registering model with Model Hub: {e}")
    
    def publish_model_update(
        self,
        server_round: int,
        metrics: Dict[str, Scalar],
    ) -> None:
        """
        Publish model update notification via MQTT.
        
        Args:
            server_round: Current round number
            metrics: Model metrics
        """
        try:
            # Create message
            message = {
                "event": "model_updated",
                "round": server_round,
                "metrics": metrics,
                "timestamp": datetime.now().isoformat(),
            }
            
            # Publish message
            self.mqtt_client.publish(
                topic=self.config["server"]["mqtt_topic"],
                payload=json.dumps(message),
                qos=1,
            )
            
            logger.info(f"Published model update notification for round {server_round}")
        
        except Exception as e:
            logger.error(f"Error publishing model update notification: {e}")

def setup_mqtt_client(config: Dict[str, Any]) -> mqtt.Client:
    """
    Set up MQTT client for secure communication.
    
    Args:
        config: MQTT configuration
        
    Returns:
        Configured MQTT client
    """
    # Create MQTT client
    client = mqtt.Client(client_id="fl_server", protocol=mqtt.MQTTv5)
    
    # Set up TLS if enabled
    if config["server"]["tls_enabled"]:
        client.tls_set(
            ca_certs=config["server"]["ca_cert_path"],
            certfile=config["server"]["server_cert_path"],
            keyfile=config["server"]["server_key_path"],
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

def get_evaluate_fn(config: Dict[str, Any]) -> Callable[[int, NDArrays, Dict[str, Scalar]], Tuple[float, Dict[str, Scalar]]]:
    """
    Get evaluation function for the server.
    
    Args:
        config: Server configuration
        
    Returns:
        Evaluation function
    """
    def evaluate_fn(
        server_round: int,
        parameters: NDArrays,
        config: Dict[str, Scalar],
    ) -> Tuple[float, Dict[str, Scalar]]:
        """
        Evaluate the global model.
        
        Args:
            server_round: Current round number
            parameters: Model parameters
            config: Evaluation configuration
            
        Returns:
            Tuple of (loss, metrics)
        """
        # In a real implementation, this would evaluate the model on a validation dataset
        # For now, we'll just return placeholder metrics
        
        # Log evaluation
        logger.info(f"Evaluating global model in round {server_round}")
        
        # Return placeholder metrics
        return 0.0, {
            "accuracy": 0.0,
            "round": server_round,
        }
    
    return evaluate_fn

def start_server(config: Dict[str, Any]) -> None:
    """
    Start the Flower server.
    
    Args:
        config: Server configuration
    """
    # Initialize model
    model = ModelHead(
        embedding_dim=config["model"]["embedding_dim"],
        num_classes=config["model"]["num_classes"],
        head_type=config["model"]["model_head_type"]
    )
    
    # Get initial parameters
    initial_parameters = [val.cpu().numpy() for _, val in model.state_dict().items()]
    
    # Create strategy
    strategy = FedAvgWithModelSaving(
        fraction_fit=0.0,  # Sample all available clients for training
        fraction_evaluate=0.0,  # Sample all available clients for evaluation
        min_fit_clients=config["server"]["min_clients"],
        min_evaluate_clients=config["server"]["min_clients"],
        min_available_clients=config["server"]["min_clients"],
        evaluate_fn=get_evaluate_fn(config),
        initial_parameters=fl.common.ndarrays_to_parameters(initial_parameters),
        config=config,
    )
    
    # Parse server address
    server_address = config["server"]["address"]
    if ":" in server_address:
        host, port = server_address.split(":")
        port = int(port)
    else:
        host, port = server_address, 8080
    
    # Start Flower server
    fl.server.start_server(
        server_address=f"{host}:{port}",
        strategy=strategy,
        config=fl.server.ServerConfig(num_rounds=config["server"]["rounds"]),
    )

# Create FastAPI app
app = FastAPI(title="Federated Learning Server API")

@app.get("/")
async def root():
    """Root endpoint."""
    return {"message": "Federated Learning Server API"}

@app.get("/status")
async def status():
    """Get server status."""
    return {
        "status": "running",
        "current_round": getattr(app.state, "current_round", 0),
        "total_rounds": getattr(app.state, "total_rounds", 0),
        "connected_clients": getattr(app.state, "connected_clients", 0),
    }

@app.post("/start")
async def start_federated_learning(
    background_tasks: BackgroundTasks,
    config_path: Optional[str] = None,
):
    """
    Start federated learning.
    
    Args:
        background_tasks: FastAPI background tasks
        config_path: Path to configuration file
    """
    # Load configuration
    config = load_config(config_path)
    
    # Store config in app state
    app.state.config = config
    app.state.current_round = 0
    app.state.total_rounds = config["server"]["rounds"]
    app.state.connected_clients = 0
    
    # Start server in background
    background_tasks.add_task(start_server, config)
    
    return {"message": "Federated learning started", "config": config}

def main():
    """Main function to start the Flower server."""
    parser = argparse.ArgumentParser(description="Federated Learning Server")
    parser.add_argument(
        "--config", type=str, default=None,
        help="Path to configuration file"
    )
    parser.add_argument(
        "--api-only", action="store_true",
        help="Run only the API server without starting Flower server"
    )
    args = parser.parse_args()
    
    # Load configuration
    config = load_config(args.config)
    
    if args.api_only:
        # Run only the API server
        uvicorn.run(
            "server:app",
            host="0.0.0.0",
            port=8000,
            reload=False,
        )
    else:
        # Start Flower server
        start_server(config)

if __name__ == "__main__":
    main()
