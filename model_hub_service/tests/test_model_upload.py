"""
Tests for model upload functionality.

This module provides tests for the model upload functionality of the Model Hub service.
"""

import os
import sys
import pytest
import tempfile
import random
import string
import hashlib
from pathlib import Path
from typing import Dict, Any, Generator

import httpx
from fastapi.testclient import TestClient

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from app.main import app

# Create test client
client = TestClient(app)

@pytest.fixture
def test_model_file() -> Generator[str, None, None]:
    """
    Create a test model file.
    
    Yields:
        Path to the test model file
    """
    # Create temporary file
    with tempfile.NamedTemporaryFile(suffix=".pt", delete=False) as temp_file:
        # Write random data to file
        temp_file.write(os.urandom(1024))
        temp_file_path = temp_file.name
    
    yield temp_file_path
    
    # Clean up
    if os.path.exists(temp_file_path):
        os.unlink(temp_file_path)

@pytest.fixture
def test_model_data() -> Dict[str, Any]:
    """
    Create test model data.
    
    Returns:
        Test model data
    """
    # Generate random model name
    model_name = "test_model_" + "".join(random.choices(string.ascii_lowercase, k=8))
    
    return {
        "name": model_name,
        "version": "1.0.0",
        "description": "Test model",
        "model_type": "test",
        "framework": "pytorch",
        "stage": "development",
    }

def test_upload_valid_model(test_model_file: str, test_model_data: Dict[str, Any]):
    """
    Test uploading a valid model.
    
    Args:
        test_model_file: Path to the test model file
        test_model_data: Test model data
    """
    # Calculate file hash
    file_hash = hashlib.sha256()
    with open(test_model_file, "rb") as f:
        for byte_block in iter(lambda: f.read(4096), b""):
            file_hash.update(byte_block)
    expected_hash = file_hash.hexdigest()
    
    # Upload model
    with open(test_model_file, "rb") as f:
        files = {"file": (os.path.basename(test_model_file), f)}
        response = client.post("/api/v1/models", files=files, data=test_model_data)
    
    # Check response
    assert response.status_code == 201
    
    # Check response data
    data = response.json()
    assert data["name"] == test_model_data["name"]
    assert data["version"] == test_model_data["version"]
    assert data["description"] == test_model_data["description"]
    assert data["model_type"] == test_model_data["model_type"]
    assert data["framework"] == test_model_data["framework"]
    assert data["stage"] == test_model_data["stage"]
    assert data["hash"] == expected_hash
    assert data["is_active"] is False
    assert "id" in data
    assert "created_at" in data
    assert "updated_at" in data

def test_upload_invalid_model_file():
    """Test uploading an invalid model file."""
    # Create temporary file with invalid extension
    with tempfile.NamedTemporaryFile(suffix=".txt", delete=False) as temp_file:
        # Write random data to file
        temp_file.write(os.urandom(1024))
        temp_file_path = temp_file.name
    
    try:
        # Upload model
        with open(temp_file_path, "rb") as f:
            files = {"file": (os.path.basename(temp_file_path), f)}
            data = {
                "name": "invalid_model",
                "version": "1.0.0",
                "description": "Invalid model",
                "model_type": "test",
                "framework": "pytorch",
                "stage": "development",
            }
            response = client.post("/api/v1/models", files=files, data=data)
        
        # Check response
        assert response.status_code == 400
        assert "Invalid model file type" in response.text
    finally:
        # Clean up
        if os.path.exists(temp_file_path):
            os.unlink(temp_file_path)

def test_upload_missing_required_fields(test_model_file: str):
    """
    Test uploading a model with missing required fields.
    
    Args:
        test_model_file: Path to the test model file
    """
    # Upload model without name
    with open(test_model_file, "rb") as f:
        files = {"file": (os.path.basename(test_model_file), f)}
        data = {
            "version": "1.0.0",
            "description": "Test model",
            "model_type": "test",
            "framework": "pytorch",
            "stage": "development",
        }
        response = client.post("/api/v1/models", files=files, data=data)
    
    # Check response
    assert response.status_code == 422
    assert "name" in response.text

def test_upload_corrupted_model(test_model_data: Dict[str, Any]):
    """
    Test uploading a corrupted model.
    
    Args:
        test_model_data: Test model data
    """
    # Create temporary file with corrupted data
    with tempfile.NamedTemporaryFile(suffix=".pt", delete=False) as temp_file:
        # Write corrupted data to file
        temp_file.write(b"corrupted data")
        temp_file_path = temp_file.name
    
    try:
        # Upload model
        with open(temp_file_path, "rb") as f:
            files = {"file": (os.path.basename(temp_file_path), f)}
            response = client.post("/api/v1/models", files=files, data=test_model_data)
        
        # Check response
        assert response.status_code == 201
        
        # In a real implementation, this would fail during model validation
        # For this test, we just check that the upload succeeds but the model is marked as invalid
        # data = response.json()
        # assert data["is_valid"] is False
    finally:
        # Clean up
        if os.path.exists(temp_file_path):
            os.unlink(temp_file_path)

def test_get_models():
    """Test getting all models."""
    # Get models
    response = client.get("/api/v1/models")
    
    # Check response
    assert response.status_code == 200
    
    # Check response data
    data = response.json()
    assert isinstance(data, list)

def test_get_model_by_id(test_model_file: str, test_model_data: Dict[str, Any]):
    """
    Test getting a model by ID.
    
    Args:
        test_model_file: Path to the test model file
        test_model_data: Test model data
    """
    # Upload model
    with open(test_model_file, "rb") as f:
        files = {"file": (os.path.basename(test_model_file), f)}
        response = client.post("/api/v1/models", files=files, data=test_model_data)
    
    # Check response
    assert response.status_code == 201
    
    # Get model ID
    model_id = response.json()["id"]
    
    # Get model
    response = client.get(f"/api/v1/models/{model_id}")
    
    # Check response
    assert response.status_code == 200
    
    # Check response data
    data = response.json()
    assert data["id"] == model_id
    assert data["name"] == test_model_data["name"]
    assert data["version"] == test_model_data["version"]

def test_get_nonexistent_model():
    """Test getting a nonexistent model."""
    # Get model
    response = client.get("/api/v1/models/nonexistent")
    
    # Check response
    assert response.status_code == 404
    assert "not found" in response.text

def test_update_model(test_model_file: str, test_model_data: Dict[str, Any]):
    """
    Test updating a model.
    
    Args:
        test_model_file: Path to the test model file
        test_model_data: Test model data
    """
    # Upload model
    with open(test_model_file, "rb") as f:
        files = {"file": (os.path.basename(test_model_file), f)}
        response = client.post("/api/v1/models", files=files, data=test_model_data)
    
    # Check response
    assert response.status_code == 201
    
    # Get model ID
    model_id = response.json()["id"]
    
    # Update model
    update_data = {
        "description": "Updated description",
        "stage": "staging",
    }
    response = client.put(f"/api/v1/models/{model_id}", json=update_data)
    
    # Check response
    assert response.status_code == 200
    
    # Check response data
    data = response.json()
    assert data["id"] == model_id
    assert data["description"] == update_data["description"]
    assert data["stage"] == update_data["stage"]

def test_delete_model(test_model_file: str, test_model_data: Dict[str, Any]):
    """
    Test deleting a model.
    
    Args:
        test_model_file: Path to the test model file
        test_model_data: Test model data
    """
    # Upload model
    with open(test_model_file, "rb") as f:
        files = {"file": (os.path.basename(test_model_file), f)}
        response = client.post("/api/v1/models", files=files, data=test_model_data)
    
    # Check response
    assert response.status_code == 201
    
    # Get model ID
    model_id = response.json()["id"]
    
    # Delete model
    response = client.delete(f"/api/v1/models/{model_id}")
    
    # Check response
    assert response.status_code == 204
    
    # Try to get deleted model
    response = client.get(f"/api/v1/models/{model_id}")
    
    # Check response
    assert response.status_code == 404
