"""
Test cases for CSRF protection.

This module provides test cases for the CSRF protection middleware and utilities.
"""

import pytest
import hmac
import hashlib
import base64
import time
from fastapi import FastAPI, Request, Response, Depends, HTTPException
from fastapi.testclient import TestClient
from fastapi.routing import APIRouter

from security.api.csrf_protection import (
    CSRFMiddleware,
    add_csrf_protection,
    csrf_protect,
    get_csrf_token,
    set_csrf_token
)

# Create a test app
def create_test_app():
    """Create a test FastAPI application with CSRF protection."""
    app = FastAPI()
    
    # Add CSRF protection
    add_csrf_protection(
        app,
        secret_key="test_secret_key_for_csrf_protection_middleware",
        cookie_name="csrf_token",
        header_name="X-CSRF-Token",
        cookie_secure=False,  # For testing
        cookie_httponly=True,
        cookie_samesite="Lax",
        token_rotation_age=3600,
        double_submit=True
    )
    
    # Add test routes
    router = APIRouter()
    
    @router.get("/csrf-token")
    def get_token(request: Request, response: Response):
        """Get a CSRF token."""
        token = get_csrf_token(request)
        return {"token": token}
    
    @router.post("/protected")
    def protected_route(request: Request, _=Depends(csrf_protect)):
        """Protected route that requires CSRF token."""
        return {"message": "Access granted"}
    
    @router.post("/custom-protected")
    def custom_protected_route(
        request: Request,
        _=Depends(lambda req: csrf_protect(req, csrf_cookie_name="custom_csrf_token"))
    ):
        """Protected route with custom CSRF token name."""
        return {"message": "Access granted"}
    
    app.include_router(router)
    
    return app

# Test cases
def test_csrf_token_generation():
    """Test CSRF token generation."""
    app = create_test_app()
    client = TestClient(app)
    
    # Get a CSRF token
    response = client.get("/csrf-token")
    
    # Check that the response contains a CSRF token cookie
    assert "csrf_token" in response.cookies
    
    # Check that the response contains a CSRF token value cookie
    assert "csrf_token_value" in response.cookies
    
    # Check that the cookies have the correct attributes
    csrf_cookie = response.cookies["csrf_token"]
    assert csrf_cookie.get("httponly") is True
    assert csrf_cookie.get("samesite") == "Lax"
    
    csrf_value_cookie = response.cookies["csrf_token_value"]
    assert csrf_value_cookie.get("httponly") is None  # Not HttpOnly
    assert csrf_value_cookie.get("samesite") == "Lax"

def test_csrf_protection_success():
    """Test successful CSRF protection."""
    app = create_test_app()
    client = TestClient(app)
    
    # Get a CSRF token
    response = client.get("/csrf-token")
    csrf_token = response.cookies["csrf_token_value"]
    
    # Make a request with the CSRF token
    response = client.post(
        "/protected",
        headers={"X-CSRF-Token": csrf_token}
    )
    
    # Check that the request was successful
    assert response.status_code == 200
    assert response.json() == {"message": "Access granted"}

def test_csrf_protection_failure_missing_token():
    """Test CSRF protection failure due to missing token."""
    app = create_test_app()
    client = TestClient(app)
    
    # Make a request without a CSRF token
    response = client.post("/protected")
    
    # Check that the request was rejected
    assert response.status_code == 403
    assert "CSRF token missing or invalid" in response.text

def test_csrf_protection_failure_invalid_token():
    """Test CSRF protection failure due to invalid token."""
    app = create_test_app()
    client = TestClient(app)
    
    # Get a CSRF token
    response = client.get("/csrf-token")
    
    # Make a request with an invalid CSRF token
    response = client.post(
        "/protected",
        headers={"X-CSRF-Token": "invalid_token"}
    )
    
    # Check that the request was rejected
    assert response.status_code == 403
    assert "CSRF token missing or invalid" in response.text

def test_csrf_token_rotation():
    """Test CSRF token rotation."""
    # Create a middleware instance directly for testing
    middleware = CSRFMiddleware(
        app=None,
        secret_key="test_secret_key",
        token_rotation_age=1  # 1 second for testing
    )
    
    # Generate a token
    token, signed_token = middleware.generate_csrf_token()
    
    # Wait for the token to expire
    time.sleep(2)
    
    # Check that the token should be rotated
    assert middleware._should_rotate_token(token) is True

def test_csrf_token_verification():
    """Test CSRF token verification."""
    # Create a middleware instance directly for testing
    middleware = CSRFMiddleware(
        app=None,
        secret_key="test_secret_key"
    )
    
    # Generate a token
    token, signed_token = middleware.generate_csrf_token()
    
    # Verify the token
    assert middleware._verify_token(token, signed_token) is True
    
    # Verify with an invalid token
    assert middleware._verify_token("invalid_token", signed_token) is False
    
    # Verify with an invalid signature
    assert middleware._verify_token(token, "invalid_signature") is False

def test_csrf_double_submit():
    """Test CSRF double submit pattern."""
    # Create an app with double submit enabled
    app = FastAPI()
    add_csrf_protection(
        app,
        secret_key="test_secret_key",
        double_submit=True
    )
    
    # Add a test route
    @app.post("/protected")
    def protected_route(request: Request, _=Depends(csrf_protect)):
        return {"message": "Access granted"}
    
    client = TestClient(app)
    
    # Make a request to get the CSRF token
    response = client.get("/")
    
    # Check that both cookies are set
    assert "csrf_token" in response.cookies
    assert "csrf_token_value" in response.cookies
    
    # Get the token value
    csrf_token_value = response.cookies["csrf_token_value"]
    
    # Make a request with the token
    response = client.post(
        "/protected",
        headers={"X-CSRF-Token": csrf_token_value}
    )
    
    # Check that the request was successful
    assert response.status_code == 200
    assert response.json() == {"message": "Access granted"}

def test_csrf_without_double_submit():
    """Test CSRF without double submit pattern."""
    # Create an app without double submit
    app = FastAPI()
    add_csrf_protection(
        app,
        secret_key="test_secret_key",
        double_submit=False
    )
    
    # Add a test route
    @app.post("/protected")
    def protected_route(request: Request, _=Depends(csrf_protect)):
        return {"message": "Access granted"}
    
    client = TestClient(app)
    
    # Make a request to get the CSRF token
    response = client.get("/")
    
    # Check that only the main cookie is set
    assert "csrf_token" in response.cookies
    assert "csrf_token_value" not in response.cookies
    
    # Get the token value
    csrf_token = response.cookies["csrf_token"]
    
    # Make a request with the token
    response = client.post(
        "/protected",
        headers={"X-CSRF-Token": csrf_token}
    )
    
    # Check that the request was successful
    assert response.status_code == 200
    assert response.json() == {"message": "Access granted"}
