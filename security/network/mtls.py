"""
Mutual TLS (mTLS) configuration for Bulo.Cloud Sentinel.

This module provides utilities for configuring mutual TLS for secure service-to-service communication.
"""

import os
import logging
import ssl
from typing import Dict, Optional, Union, List, Tuple, Callable, Any
from pathlib import Path
import json
import time
import threading
from datetime import datetime, timedelta

from fastapi import FastAPI, Request, Response, HTTPException, status, Depends
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
from starlette.middleware.base import BaseHTTPMiddleware
from starlette.types import ASGIApp

from .tls import TLSConfig, check_cert_expiration, get_cert_fingerprint

# Configure logging
logger = logging.getLogger(__name__)


class MTLSConfig:
    """
    Mutual TLS configuration for service-to-service communication.
    
    This class provides configuration for mutual TLS authentication
    between services.
    """
    
    def __init__(
        self,
        cert_dir: str,
        ca_file: str,
        client_cert_file: Optional[str] = None,
        client_key_file: Optional[str] = None,
        verify_client: bool = True,
        allowed_fingerprints: Optional[List[str]] = None,
        allowed_subjects: Optional[List[str]] = None,
        refresh_interval: int = 3600
    ):
        """
        Initialize mTLS configuration.
        
        Args:
            cert_dir: Directory containing certificates
            ca_file: Path to CA certificate file
            client_cert_file: Path to client certificate file
            client_key_file: Path to client key file
            verify_client: Whether to verify client certificates
            allowed_fingerprints: List of allowed certificate fingerprints
            allowed_subjects: List of allowed certificate subjects
            refresh_interval: Interval in seconds to refresh certificate information
        """
        self.cert_dir = cert_dir
        self.ca_file = ca_file
        self.client_cert_file = client_cert_file
        self.client_key_file = client_key_file
        self.verify_client = verify_client
        self.allowed_fingerprints = allowed_fingerprints or []
        self.allowed_subjects = allowed_subjects or []
        self.refresh_interval = refresh_interval
        
        # Certificate information cache
        self.cert_info = {}
        self.cert_info_lock = threading.RLock()
        self.last_refresh = 0
        
        # Load certificate information
        self._refresh_cert_info()
        
        logger.info(f"Initialized mTLS configuration with CA: {ca_file}")
    
    def _refresh_cert_info(self):
        """Refresh certificate information."""
        with self.cert_info_lock:
            now = time.time()
            if now - self.last_refresh < self.refresh_interval:
                return
            
            # Refresh CA certificate information
            try:
                self.cert_info["ca"] = check_cert_expiration(self.ca_file)
                logger.debug(f"Refreshed CA certificate information: {self.cert_info['ca']}")
            except Exception as e:
                logger.error(f"Failed to refresh CA certificate information: {str(e)}")
            
            # Refresh client certificate information
            if self.client_cert_file:
                try:
                    self.cert_info["client"] = check_cert_expiration(self.client_cert_file)
                    logger.debug(f"Refreshed client certificate information: {self.cert_info['client']}")
                except Exception as e:
                    logger.error(f"Failed to refresh client certificate information: {str(e)}")
            
            # Update last refresh time
            self.last_refresh = now
    
    def get_server_ssl_context(self) -> ssl.SSLContext:
        """
        Get SSL context for server.
        
        Returns:
            SSL context
        """
        # Refresh certificate information
        self._refresh_cert_info()
        
        # Create TLS configuration
        tls_config = TLSConfig(
            cert_file=os.path.join(self.cert_dir, "server.crt"),
            key_file=os.path.join(self.cert_dir, "server.key"),
            ca_file=self.ca_file,
            verify_client=self.verify_client
        )
        
        # Get SSL context
        return tls_config.get_ssl_context()
    
    def get_client_ssl_context(self) -> ssl.SSLContext:
        """
        Get SSL context for client.
        
        Returns:
            SSL context
        """
        # Refresh certificate information
        self._refresh_cert_info()
        
        # Create TLS configuration
        tls_config = TLSConfig(
            cert_file=self.client_cert_file,
            key_file=self.client_key_file,
            ca_file=self.ca_file,
            verify_client=False
        )
        
        # Get SSL context
        context = tls_config.get_ssl_context()
        
        # Set verification mode
        context.verify_mode = ssl.CERT_REQUIRED
        context.check_hostname = True
        
        return context
    
    def is_cert_allowed(self, cert_file: str) -> bool:
        """
        Check if a certificate is allowed.
        
        Args:
            cert_file: Path to certificate file
            
        Returns:
            True if allowed, False otherwise
        """
        # Check fingerprint
        if self.allowed_fingerprints:
            fingerprint = get_cert_fingerprint(cert_file)
            if fingerprint not in self.allowed_fingerprints:
                logger.warning(f"Certificate fingerprint not allowed: {fingerprint}")
                return False
        
        # Check subject
        if self.allowed_subjects:
            cert_info = check_cert_expiration(cert_file)
            subject = cert_info["subject"]
            if subject not in self.allowed_subjects:
                logger.warning(f"Certificate subject not allowed: {subject}")
                return False
        
        return True
    
    def add_allowed_fingerprint(self, fingerprint: str):
        """
        Add an allowed certificate fingerprint.
        
        Args:
            fingerprint: Certificate fingerprint
        """
        with self.cert_info_lock:
            if fingerprint not in self.allowed_fingerprints:
                self.allowed_fingerprints.append(fingerprint)
                logger.info(f"Added allowed certificate fingerprint: {fingerprint}")
    
    def add_allowed_subject(self, subject: str):
        """
        Add an allowed certificate subject.
        
        Args:
            subject: Certificate subject
        """
        with self.cert_info_lock:
            if subject not in self.allowed_subjects:
                self.allowed_subjects.append(subject)
                logger.info(f"Added allowed certificate subject: {subject}")
    
    def remove_allowed_fingerprint(self, fingerprint: str):
        """
        Remove an allowed certificate fingerprint.
        
        Args:
            fingerprint: Certificate fingerprint
        """
        with self.cert_info_lock:
            if fingerprint in self.allowed_fingerprints:
                self.allowed_fingerprints.remove(fingerprint)
                logger.info(f"Removed allowed certificate fingerprint: {fingerprint}")
    
    def remove_allowed_subject(self, subject: str):
        """
        Remove an allowed certificate subject.
        
        Args:
            subject: Certificate subject
        """
        with self.cert_info_lock:
            if subject in self.allowed_subjects:
                self.allowed_subjects.remove(subject)
                logger.info(f"Removed allowed certificate subject: {subject}")
    
    def get_cert_info(self) -> Dict[str, Any]:
        """
        Get certificate information.
        
        Returns:
            Dictionary with certificate information
        """
        # Refresh certificate information
        self._refresh_cert_info()
        
        return self.cert_info


class MTLSMiddleware(BaseHTTPMiddleware):
    """
    Middleware for mutual TLS authentication.
    
    This middleware validates client certificates for mutual TLS authentication.
    """
    
    def __init__(
        self,
        app: ASGIApp,
        mtls_config: MTLSConfig,
        exclude_paths: Optional[List[str]] = None
    ):
        """
        Initialize mTLS middleware.
        
        Args:
            app: ASGI application
            mtls_config: mTLS configuration
            exclude_paths: Paths to exclude from mTLS authentication
        """
        super().__init__(app)
        self.mtls_config = mtls_config
        self.exclude_paths = exclude_paths or []
        
        logger.info(f"Initialized mTLS middleware with {len(self.exclude_paths)} excluded paths")
    
    async def dispatch(self, request: Request, call_next):
        """
        Process a request.
        
        Args:
            request: Request object
            call_next: Function to call next middleware
            
        Returns:
            Response
        """
        # Skip excluded paths
        if any(request.url.path.startswith(path) for path in self.exclude_paths):
            return await call_next(request)
        
        # Get client certificate
        client_cert = request.headers.get("X-Client-Cert")
        if not client_cert:
            logger.warning("No client certificate provided")
            return Response(
                content=json.dumps({"detail": "Client certificate required"}),
                status_code=status.HTTP_401_UNAUTHORIZED,
                media_type="application/json"
            )
        
        # Validate client certificate
        try:
            # In a real implementation, you would validate the certificate
            # against the CA and check if it's allowed
            
            # For now, just check if the certificate is in the allowed list
            if client_cert not in self.mtls_config.allowed_fingerprints:
                logger.warning(f"Client certificate not allowed: {client_cert}")
                return Response(
                    content=json.dumps({"detail": "Client certificate not allowed"}),
                    status_code=status.HTTP_403_FORBIDDEN,
                    media_type="application/json"
                )
            
            # Process request
            return await call_next(request)
        
        except Exception as e:
            logger.error(f"Error validating client certificate: {str(e)}")
            return Response(
                content=json.dumps({"detail": "Error validating client certificate"}),
                status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
                media_type="application/json"
            )


class MTLSBearer(HTTPBearer):
    """
    Security scheme for mutual TLS authentication.
    
    This class provides a security scheme for mutual TLS authentication
    using client certificates.
    """
    
    def __init__(
        self,
        mtls_config: MTLSConfig,
        auto_error: bool = True
    ):
        """
        Initialize mTLS bearer.
        
        Args:
            mtls_config: mTLS configuration
            auto_error: Whether to raise errors automatically
        """
        super().__init__(auto_error=auto_error)
        self.mtls_config = mtls_config
        
        logger.info("Initialized mTLS bearer")
    
    async def __call__(self, request: Request) -> HTTPAuthorizationCredentials:
        """
        Validate client certificate.
        
        Args:
            request: Request object
            
        Returns:
            Authorization credentials
        """
        # Get client certificate
        client_cert = request.headers.get("X-Client-Cert")
        if not client_cert:
            logger.warning("No client certificate provided")
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Client certificate required"
            )
        
        # Validate client certificate
        try:
            # In a real implementation, you would validate the certificate
            # against the CA and check if it's allowed
            
            # For now, just check if the certificate is in the allowed list
            if client_cert not in self.mtls_config.allowed_fingerprints:
                logger.warning(f"Client certificate not allowed: {client_cert}")
                raise HTTPException(
                    status_code=status.HTTP_403_FORBIDDEN,
                    detail="Client certificate not allowed"
                )
            
            # Return credentials
            return HTTPAuthorizationCredentials(
                scheme="mtls",
                credentials=client_cert
            )
        
        except HTTPException:
            raise
        
        except Exception as e:
            logger.error(f"Error validating client certificate: {str(e)}")
            raise HTTPException(
                status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
                detail="Error validating client certificate"
            )


def configure_mtls(
    app: FastAPI,
    cert_dir: str,
    ca_file: str,
    client_cert_file: Optional[str] = None,
    client_key_file: Optional[str] = None,
    verify_client: bool = True,
    allowed_fingerprints: Optional[List[str]] = None,
    allowed_subjects: Optional[List[str]] = None,
    exclude_paths: Optional[List[str]] = None
) -> MTLSConfig:
    """
    Configure mutual TLS for a FastAPI application.
    
    Args:
        app: FastAPI application
        cert_dir: Directory containing certificates
        ca_file: Path to CA certificate file
        client_cert_file: Path to client certificate file
        client_key_file: Path to client key file
        verify_client: Whether to verify client certificates
        allowed_fingerprints: List of allowed certificate fingerprints
        allowed_subjects: List of allowed certificate subjects
        exclude_paths: Paths to exclude from mTLS authentication
        
    Returns:
        mTLS configuration
    """
    # Create mTLS configuration
    mtls_config = MTLSConfig(
        cert_dir=cert_dir,
        ca_file=ca_file,
        client_cert_file=client_cert_file,
        client_key_file=client_key_file,
        verify_client=verify_client,
        allowed_fingerprints=allowed_fingerprints,
        allowed_subjects=allowed_subjects
    )
    
    # Add mTLS middleware
    app.add_middleware(
        MTLSMiddleware,
        mtls_config=mtls_config,
        exclude_paths=exclude_paths
    )
    
    # Store mTLS configuration in app state
    app.state.mtls_config = mtls_config
    
    logger.info("Configured mutual TLS for FastAPI application")
    
    return mtls_config


def get_mtls_config(app: FastAPI) -> MTLSConfig:
    """
    Get mTLS configuration from FastAPI application.
    
    Args:
        app: FastAPI application
        
    Returns:
        mTLS configuration
    """
    return app.state.mtls_config


def get_mtls_bearer(app: FastAPI) -> MTLSBearer:
    """
    Get mTLS bearer from FastAPI application.
    
    Args:
        app: FastAPI application
        
    Returns:
        mTLS bearer
    """
    mtls_config = get_mtls_config(app)
    return MTLSBearer(mtls_config=mtls_config)


def require_mtls(app: FastAPI):
    """
    Dependency for requiring mutual TLS authentication.
    
    Args:
        app: FastAPI application
        
    Returns:
        Dependency function
    """
    mtls_bearer = get_mtls_bearer(app)
    
    async def validate_mtls(credentials: HTTPAuthorizationCredentials = Depends(mtls_bearer)):
        return credentials
    
    return validate_mtls
