"""
CORS utilities for Bulo.Cloud Sentinel.

This module provides functions for configuring CORS (Cross-Origin Resource Sharing)
to control which domains can access the API.
"""

from typing import List, Optional, Union

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from starlette.middleware.cors import ALL_METHODS

from ..logging.secure_logging import get_secure_logger


def configure_cors(
    app: FastAPI,
    allow_origins: Union[List[str], str] = ["http://localhost:3000", "https://bulocloud-sentinel.com"],
    allow_methods: List[str] = ["GET", "POST", "PUT", "DELETE", "OPTIONS", "PATCH"],
    allow_headers: List[str] = ["Accept", "Authorization", "Content-Type", "X-Requested-With", "X-CSRF-Token"],
    allow_credentials: bool = False,
    expose_headers: List[str] = ["Content-Length", "Content-Type", "X-Request-ID"],
    max_age: int = 600
):
    """
    Configure CORS for a FastAPI application.

    Args:
        app: FastAPI application
        allow_origins: List of allowed origins or "*" for all
        allow_methods: List of allowed HTTP methods
        allow_headers: List of allowed HTTP headers
        allow_credentials: Whether to allow credentials
        expose_headers: List of headers to expose
        max_age: Maximum age of preflight requests
    """
    logger = get_secure_logger("cors")

    # Convert string to list if needed
    if isinstance(allow_origins, str):
        if allow_origins == "*":
            logger.warning("Wildcard origin '*' is insecure. Consider specifying explicit origins.")
            allow_origins = ["*"]
        else:
            allow_origins = [origin.strip() for origin in allow_origins.split(",")]

    # Check for wildcard headers
    if "*" in allow_headers:
        logger.warning("Wildcard headers '*' is insecure. Consider specifying explicit headers.")

    # Check for credentials with wildcard origin
    if allow_credentials and "*" in allow_origins:
        logger.error("Insecure CORS configuration: credentials with wildcard origin is not allowed")
        raise ValueError("Cannot use credentials with wildcard origin")

    # Log CORS configuration
    logger.info(
        "Configuring CORS",
        {
            "allow_origins": allow_origins,
            "allow_methods": allow_methods,
            "allow_headers": allow_headers,
            "allow_credentials": allow_credentials,
            "expose_headers": expose_headers,
            "max_age": max_age
        }
    )

    # Add CORS middleware
    app.add_middleware(
        CORSMiddleware,
        allow_origins=allow_origins,
        allow_methods=allow_methods,
        allow_headers=allow_headers,
        allow_credentials=allow_credentials,
        expose_headers=expose_headers,
        max_age=max_age
    )


def configure_secure_cors(
    app: FastAPI,
    allow_origins: List[str],
    allow_methods: Optional[List[str]] = None,
    allow_headers: Optional[List[str]] = None,
    expose_headers: Optional[List[str]] = None,
    max_age: int = 600
):
    """
    Configure secure CORS for a FastAPI application.

    This function sets up CORS with secure defaults:
    - No wildcard origins
    - No credentials
    - Limited methods and headers

    Args:
        app: FastAPI application
        allow_origins: List of allowed origins (no wildcards)
        allow_methods: List of allowed HTTP methods
        allow_headers: List of allowed HTTP headers
        expose_headers: List of headers to expose
        max_age: Maximum age of preflight requests
    """
    logger = get_secure_logger("cors")

    # Validate origins
    if not allow_origins or "*" in allow_origins:
        logger.warning(
            "Insecure CORS configuration: wildcard or empty origins",
            {"allow_origins": allow_origins}
        )
        raise ValueError("Secure CORS requires specific origins (no wildcards)")

    # Set default methods if not provided
    if allow_methods is None:
        allow_methods = ["GET", "POST", "PUT", "DELETE", "OPTIONS"]

    # Set default headers if not provided
    if allow_headers is None:
        allow_headers = [
            "Accept",
            "Authorization",
            "Content-Type",
            "X-Requested-With"
        ]

    # Set default exposed headers if not provided
    if expose_headers is None:
        expose_headers = [
            "Content-Length",
            "Content-Type",
            "X-Request-ID"
        ]

    # Log CORS configuration
    logger.info(
        "Configuring secure CORS",
        {
            "allow_origins": allow_origins,
            "allow_methods": allow_methods,
            "allow_headers": allow_headers,
            "expose_headers": expose_headers,
            "max_age": max_age
        }
    )

    # Add CORS middleware
    app.add_middleware(
        CORSMiddleware,
        allow_origins=allow_origins,
        allow_methods=allow_methods,
        allow_headers=allow_headers,
        allow_credentials=False,  # No credentials for security
        expose_headers=expose_headers,
        max_age=max_age
    )
