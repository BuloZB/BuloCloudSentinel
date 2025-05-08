"""
Integration tests for the Model Hub service.

This module provides tests for the integration between the Model Hub, Edge Kit, and SentinelWeb.
"""

import os
import sys
import pytest
import tempfile
import random
import string
import time
import json
import asyncio
import websockets
from pathlib import Path
from typing import Dict, Any, Generator, Tuple

import httpx
from fastapi.testclient import TestClient

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from app.main import app

# Create test client
client = TestClient(app)

@pytest.fixture
def test_model() -> Generator[Dict[str, Any], None, None]:
    """
    Create a test model.
    
    Yields:
        Test model data
    """
    # Create temporary file
    with tempfile.NamedTemporaryFile(suffix=".pt", delete=False) as temp_file:
        # Write random data to file
        temp_file.write(os.urandom(1024))
        temp_file_path = temp_file.name
    
    try:
        # Generate random model name
        model_name = "test_model_" + "".join(random.choices(string.ascii_lowercase, k=8))
        
        # Upload model
        with open(temp_file_path, "rb") as f:
            files = {"file": (os.path.basename(temp_file_path), f)}
            data = {
                "name": model_name,
                "version": "1.0.0",
                "description": "Test model",
                "model_type": "test",
                "framework": "pytorch",
                "stage": "development",
            }
            response = client.post("/api/v1/models", files=files, data=data)
        
        # Check response
        assert response.status_code == 201
        
        # Get model data
        model_data = response.json()
        
        yield model_data
    finally:
        # Clean up
        if os.path.exists(temp_file_path):
            os.unlink(temp_file_path)

@pytest.mark.asyncio
async def test_edge_kit_integration(test_model: Dict[str, Any]):
    """
    Test integration with Edge Kit.
    
    This test simulates an Edge Kit client connecting to the Model Hub and receiving a model update.
    
    Args:
        test_model: Test model data
    """
    # Create deployment
    deployment_data = {
        "model_id": test_model["id"],
        "environment": "production",
        "deployment_type": "blue-green",
        "target": "edge",
        "auto_rollback_enabled": True,
    }
    response = client.post("/api/v1/deployments", json=deployment_data)
    
    # Check response
    assert response.status_code == 201
    
    # Get deployment ID
    deployment_id = response.json()["id"]
    
    # Simulate Edge Kit client
    client_id = "edge-kit-" + "".join(random.choices(string.ascii_lowercase, k=8))
    
    # Connect to WebSocket
    async with websockets.connect(f"ws://localhost:8000/api/v1/grpc/ws/{client_id}") as websocket:
        # Receive initial message
        response = await websocket.recv()
        data = json.loads(response)
        assert data["type"] == "connected"
        
        # Send status update
        await websocket.send(json.dumps({
            "type": "status",
            "status": "ready",
            "hardware": {
                "device": "jetson",
                "memory": 8192,
                "cpu_cores": 6,
                "gpu_memory": 4096,
            },
            "models": [],
        }))
        
        # Receive acknowledgement
        response = await websocket.recv()
        data = json.loads(response)
        assert data["type"] == "ack"
        
        # Deploy model to client
        response = client.post(f"/api/v1/grpc/deploy/{client_id}", json={"model_id": test_model["id"]})
        
        # Check response
        assert response.status_code == 200
        
        # Receive deployment message
        response = await websocket.recv()
        data = json.loads(response)
        assert data["type"] == "deploy"
        assert data["model_id"] == test_model["id"]
        
        # Send deployment status
        await websocket.send(json.dumps({
            "type": "deployment_status",
            "model_id": test_model["id"],
            "status": "downloading",
            "progress": 0.0,
        }))
        
        # Receive acknowledgement
        response = await websocket.recv()
        data = json.loads(response)
        assert data["type"] == "ack"
        
        # Send deployment status
        await websocket.send(json.dumps({
            "type": "deployment_status",
            "model_id": test_model["id"],
            "status": "downloading",
            "progress": 0.5,
        }))
        
        # Receive acknowledgement
        response = await websocket.recv()
        data = json.loads(response)
        assert data["type"] == "ack"
        
        # Send deployment status
        await websocket.send(json.dumps({
            "type": "deployment_status",
            "model_id": test_model["id"],
            "status": "downloading",
            "progress": 1.0,
        }))
        
        # Receive acknowledgement
        response = await websocket.recv()
        data = json.loads(response)
        assert data["type"] == "ack"
        
        # Send deployment status
        await websocket.send(json.dumps({
            "type": "deployment_status",
            "model_id": test_model["id"],
            "status": "verifying",
        }))
        
        # Receive acknowledgement
        response = await websocket.recv()
        data = json.loads(response)
        assert data["type"] == "ack"
        
        # Send deployment status
        await websocket.send(json.dumps({
            "type": "deployment_status",
            "model_id": test_model["id"],
            "status": "deploying",
        }))
        
        # Receive acknowledgement
        response = await websocket.recv()
        data = json.loads(response)
        assert data["type"] == "ack"
        
        # Send deployment status
        await websocket.send(json.dumps({
            "type": "deployment_status",
            "model_id": test_model["id"],
            "status": "deployed",
            "metrics": {
                "fps": 30.0,
                "map": 0.85,
                "latency_ms": 33.5,
            },
        }))
        
        # Receive acknowledgement
        response = await websocket.recv()
        data = json.loads(response)
        assert data["type"] == "ack"
        
        # Send ping
        await websocket.send(json.dumps({
            "type": "ping",
            "timestamp": time.time(),
        }))
        
        # Receive pong
        response = await websocket.recv()
        data = json.loads(response)
        assert data["type"] == "pong"

@pytest.mark.asyncio
async def test_sentinelweb_integration(test_model: Dict[str, Any]):
    """
    Test integration with SentinelWeb.
    
    This test simulates SentinelWeb accessing the Model Hub API.
    
    Args:
        test_model: Test model data
    """
    # Get model
    response = client.get(f"/api/v1/models/{test_model['id']}")
    
    # Check response
    assert response.status_code == 200
    
    # Check response data
    data = response.json()
    assert data["id"] == test_model["id"]
    assert data["name"] == test_model["name"]
    assert data["version"] == test_model["version"]
    
    # List models
    response = client.get("/api/v1/models")
    
    # Check response
    assert response.status_code == 200
    
    # Check response data
    data = response.json()
    assert isinstance(data, list)
    assert any(model["id"] == test_model["id"] for model in data)
    
    # Create deployment
    deployment_data = {
        "model_id": test_model["id"],
        "environment": "production",
        "deployment_type": "blue-green",
        "target": "all",
        "auto_rollback_enabled": True,
    }
    response = client.post("/api/v1/deployments", json=deployment_data)
    
    # Check response
    assert response.status_code == 201
    
    # Get deployment ID
    deployment_id = response.json()["id"]
    
    # Get deployment
    response = client.get(f"/api/v1/deployments/{deployment_id}")
    
    # Check response
    assert response.status_code == 200
    
    # Check response data
    data = response.json()
    assert data["id"] == deployment_id
    assert data["model_id"] == test_model["id"]
    
    # List deployments
    response = client.get("/api/v1/deployments")
    
    # Check response
    assert response.status_code == 200
    
    # Check response data
    data = response.json()
    assert isinstance(data, list)
    assert any(deployment["id"] == deployment_id for deployment in data)
    
    # Update deployment
    update_data = {
        "status": "running",
        "fps": 30.0,
        "map": 0.85,
        "latency_ms": 33.5,
    }
    response = client.put(f"/api/v1/deployments/{deployment_id}", json=update_data)
    
    # Check response
    assert response.status_code == 200
    
    # Check response data
    data = response.json()
    assert data["id"] == deployment_id
    assert data["status"] == update_data["status"]
    assert data["fps"] == update_data["fps"]
    assert data["map"] == update_data["map"]
    assert data["latency_ms"] == update_data["latency_ms"]
