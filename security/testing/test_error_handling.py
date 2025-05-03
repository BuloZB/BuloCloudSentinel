"""
Test cases for error handling.

This module provides test cases for the secure error handling middleware and utilities.
"""

import pytest
import json
import re
from fastapi import FastAPI, Request, Response, Depends, HTTPException, status
from fastapi.testclient import TestClient
from fastapi.routing import APIRouter
from pydantic import BaseModel, Field

from security.error_handling.secure_error_handler import (
    configure_error_handlers,
    configure_custom_exception_handlers,
    ErrorType,
    ErrorResponse,
    create_error_response,
    get_request_id,
    handle_http_exception,
    handle_validation_exception,
    handle_internal_exception,
    SecurityException,
    AuthenticationException,
    AuthorizationException,
    RateLimitException,
    ValidationException
)

# Create a test app
def create_test_app():
    """Create a test FastAPI application with error handling."""
    app = FastAPI()
    
    # Configure error handlers
    configure_error_handlers(app)
    configure_custom_exception_handlers(app)
    
    # Add test routes
    router = APIRouter()
    
    @router.get("/http-error")
    def http_error():
        """Raise an HTTP exception."""
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="Resource not found"
        )
    
    @router.get("/http-error-with-headers")
    def http_error_with_headers():
        """Raise an HTTP exception with headers."""
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Bad request",
            headers={
                "X-Error-Details": json.dumps([
                    {"field": "username", "error": "Invalid username"}
                ])
            }
        )
    
    class Item(BaseModel):
        """Test model for validation error."""
        name: str = Field(..., min_length=3)
        price: float = Field(..., gt=0)
    
    @router.post("/validation-error")
    def validation_error(item: Item):
        """Endpoint that will trigger validation error."""
        return item
    
    @router.get("/internal-error")
    def internal_error():
        """Raise an internal server error."""
        # This will raise a ZeroDivisionError
        return 1 / 0
    
    @router.get("/custom-error")
    def custom_error():
        """Raise a custom error."""
        raise AuthorizationException("You are not authorized to access this resource")
    
    @router.get("/sensitive-error")
    def sensitive_error():
        """Raise an error with sensitive information."""
        try:
            # This will raise a FileNotFoundError
            with open("/path/to/sensitive/file.txt", "r") as f:
                return f.read()
        except FileNotFoundError as e:
            # This error message contains sensitive information
            raise Exception(
                "Error reading file: /path/to/sensitive/file.txt. "
                "Database connection failed: mongodb://user:password@localhost:27017/db"
            )
    
    app.include_router(router)
    
    return app

# Test cases
def test_http_error_handling():
    """Test HTTP error handling."""
    app = create_test_app()
    client = TestClient(app)
    
    # Make a request that will trigger an HTTP error
    response = client.get("/http-error")
    
    # Check the response
    assert response.status_code == 404
    assert response.headers["content-type"] == "application/json"
    
    # Check the response body
    data = response.json()
    assert "error" in data
    assert data["error"]["type"] == ErrorType.RESOURCE_NOT_FOUND
    assert data["error"]["message"] == "Resource not found"
    assert data["error"]["status_code"] == 404
    assert "timestamp" in data["error"]
    assert "request_id" in data["error"]

def test_http_error_with_headers():
    """Test HTTP error handling with headers."""
    app = create_test_app()
    client = TestClient(app)
    
    # Make a request that will trigger an HTTP error with headers
    response = client.get("/http-error-with-headers")
    
    # Check the response
    assert response.status_code == 400
    assert response.headers["content-type"] == "application/json"
    
    # Check the response body
    data = response.json()
    assert "error" in data
    assert data["error"]["type"] == ErrorType.BAD_REQUEST
    assert data["error"]["message"] == "Bad request"
    assert data["error"]["status_code"] == 400
    assert "details" in data["error"]
    assert len(data["error"]["details"]) == 1
    assert data["error"]["details"][0]["field"] == "username"
    assert data["error"]["details"][0]["error"] == "Invalid username"

def test_validation_error_handling():
    """Test validation error handling."""
    app = create_test_app()
    client = TestClient(app)
    
    # Make a request with invalid data
    response = client.post(
        "/validation-error",
        json={"name": "a", "price": -1}
    )
    
    # Check the response
    assert response.status_code == 422
    assert response.headers["content-type"] == "application/json"
    
    # Check the response body
    data = response.json()
    assert "error" in data
    assert data["error"]["type"] == ErrorType.VALIDATION
    assert data["error"]["message"] == "Request validation error"
    assert data["error"]["status_code"] == 422
    assert "details" in data["error"]
    assert len(data["error"]["details"]) == 2  # Two validation errors
    
    # Check that the details contain the validation errors
    errors = {error["loc"][-1]: error["msg"] for error in data["error"]["details"]}
    assert "name" in errors
    assert "price" in errors
    assert "shorter than" in errors["name"]
    assert "greater than" in errors["price"]

def test_internal_error_handling():
    """Test internal error handling."""
    app = create_test_app()
    client = TestClient(app)
    
    # Make a request that will trigger an internal error
    response = client.get("/internal-error")
    
    # Check the response
    assert response.status_code == 500
    assert response.headers["content-type"] == "application/json"
    
    # Check the response body
    data = response.json()
    assert "error" in data
    assert data["error"]["type"] == ErrorType.INTERNAL_SERVER
    assert data["error"]["message"] == "An internal server error occurred"
    assert data["error"]["status_code"] == 500
    assert "code" in data["error"]
    assert data["error"]["code"].startswith("INTERNAL_ERROR_")

def test_custom_error_handling():
    """Test custom error handling."""
    app = create_test_app()
    client = TestClient(app)
    
    # Make a request that will trigger a custom error
    response = client.get("/custom-error")
    
    # Check the response
    assert response.status_code == 403
    assert response.headers["content-type"] == "application/json"
    
    # Check the response body
    data = response.json()
    assert "error" in data
    assert data["error"]["type"] == ErrorType.AUTHORIZATION
    assert data["error"]["message"] == "You are not authorized to access this resource"
    assert data["error"]["status_code"] == 403

def test_sensitive_information_redaction():
    """Test that sensitive information is redacted from error messages."""
    app = create_test_app()
    client = TestClient(app)
    
    # Make a request that will trigger an error with sensitive information
    response = client.get("/sensitive-error")
    
    # Check the response
    assert response.status_code == 500
    assert response.headers["content-type"] == "application/json"
    
    # Check the response body
    data = response.json()
    assert "error" in data
    
    # Check that sensitive information is redacted
    assert "mongodb://" not in data["error"]["message"]
    assert "password" not in data["error"]["message"]
    assert "/path/to/sensitive/file.txt" not in data["error"]["message"]
    assert "[REDACTED]" in data["error"]["message"]

def test_error_response_model():
    """Test the ErrorResponse model."""
    # Create an error response
    error_response = ErrorResponse(
        error_type=ErrorType.AUTHENTICATION,
        message="Authentication failed",
        status_code=401,
        details=[{"field": "token", "error": "Invalid token"}],
        request_id="test-request-id",
        code="AUTH_ERROR",
        path="/api/auth",
        help_url="https://docs.example.com/errors/auth"
    )
    
    # Convert to dictionary
    data = error_response.to_dict()
    
    # Check the dictionary
    assert "error" in data
    assert data["error"]["type"] == ErrorType.AUTHENTICATION
    assert data["error"]["message"] == "Authentication failed"
    assert data["error"]["status_code"] == 401
    assert data["error"]["details"] == [{"field": "token", "error": "Invalid token"}]
    assert data["error"]["request_id"] == "test-request-id"
    assert data["error"]["code"] == "AUTH_ERROR"
    assert data["error"]["path"] == "/api/auth"
    assert data["error"]["help_url"] == "https://docs.example.com/errors/auth"

def test_error_type_from_status_code():
    """Test the ErrorType.from_status_code method."""
    assert ErrorType.from_status_code(400) == ErrorType.BAD_REQUEST
    assert ErrorType.from_status_code(401) == ErrorType.AUTHENTICATION
    assert ErrorType.from_status_code(403) == ErrorType.AUTHORIZATION
    assert ErrorType.from_status_code(404) == ErrorType.RESOURCE_NOT_FOUND
    assert ErrorType.from_status_code(409) == ErrorType.RESOURCE_CONFLICT
    assert ErrorType.from_status_code(422) == ErrorType.VALIDATION
    assert ErrorType.from_status_code(429) == ErrorType.RATE_LIMIT
    assert ErrorType.from_status_code(500) == ErrorType.INTERNAL_SERVER
    assert ErrorType.from_status_code(503) == ErrorType.SERVICE_UNAVAILABLE
