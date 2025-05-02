"""
HTTPS support for Bulo.Cloud Sentinel.

This module provides utilities for HTTPS support, including certificate generation
and validation.
"""

import os
import sys
import logging
import subprocess
import tempfile
import shutil
import datetime
from pathlib import Path
from typing import Dict, Any, Optional, List, Tuple

# Add project root to path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))

# Import local modules
from ai.inference.config import ConfigManager
from ai.inference.monitoring import structured_logger, alert_manager

# Setup logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Create configuration manager
config = ConfigManager()


class HTTPSManager:
    """
    HTTPS manager for Bulo.Cloud Sentinel.
    
    This class provides utilities for HTTPS support, including certificate generation
    and validation.
    """
    
    def __init__(
        self,
        cert_dir: Optional[str] = None,
        cert_validity: int = 365,  # 1 year
        key_size: int = 2048,
        common_name: str = "localhost"
    ):
        """
        Initialize the HTTPS manager.
        
        Args:
            cert_dir: Directory for certificates
            cert_validity: Certificate validity in days
            key_size: Key size in bits
            common_name: Common name for the certificate
        """
        # Set certificate directory
        self.cert_dir = cert_dir or os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            "../../config/certs"
        )
        
        # Create certificate directory if it doesn't exist
        os.makedirs(self.cert_dir, exist_ok=True)
        
        # Set certificate parameters
        self.cert_validity = cert_validity
        self.key_size = key_size
        self.common_name = common_name
        
        # Set certificate paths
        self.cert_path = os.path.join(self.cert_dir, "server.crt")
        self.key_path = os.path.join(self.cert_dir, "server.key")
        self.ca_cert_path = os.path.join(self.cert_dir, "ca.crt")
        self.ca_key_path = os.path.join(self.cert_dir, "ca.key")
        
        # Check if certificates exist
        self._check_certificates()
    
    def _check_certificates(self):
        """Check if certificates exist and generate them if needed."""
        try:
            # Check if certificates exist
            if (
                os.path.exists(self.cert_path) and
                os.path.exists(self.key_path) and
                os.path.exists(self.ca_cert_path) and
                os.path.exists(self.ca_key_path)
            ):
                # Check if certificates are valid
                if self._validate_certificates():
                    structured_logger.info(
                        "Certificates are valid",
                        logger_name="https"
                    )
                    return
                else:
                    structured_logger.warning(
                        "Certificates are invalid or expired",
                        logger_name="https"
                    )
            else:
                structured_logger.warning(
                    "Certificates not found",
                    logger_name="https"
                )
            
            # Generate certificates
            self._generate_certificates()
        
        except Exception as e:
            # Log error
            structured_logger.error(
                "Error checking certificates",
                logger_name="https",
                error=str(e)
            )
            
            # Create alert
            alert_manager.create_alert(
                level="error",
                message=f"Error checking certificates: {str(e)}",
                source="https"
            )
    
    def _validate_certificates(self) -> bool:
        """
        Validate certificates.
        
        Returns:
            True if certificates are valid, False otherwise
        """
        try:
            # Check if certificates exist
            if (
                not os.path.exists(self.cert_path) or
                not os.path.exists(self.key_path) or
                not os.path.exists(self.ca_cert_path) or
                not os.path.exists(self.ca_key_path)
            ):
                return False
            
            # Check certificate expiration
            import ssl
            import datetime
            
            # Load certificate
            cert = ssl.PEM_cert_to_DER_cert(open(self.cert_path, "r").read())
            
            # Get certificate expiration
            from cryptography import x509
            from cryptography.hazmat.backends import default_backend
            
            cert = x509.load_der_x509_certificate(cert, default_backend())
            expiration = cert.not_valid_after
            
            # Check if certificate is expired
            if expiration < datetime.datetime.now():
                structured_logger.warning(
                    "Certificate is expired",
                    logger_name="https",
                    expiration=expiration.isoformat()
                )
                return False
            
            # Check if certificate is about to expire
            if expiration < datetime.datetime.now() + datetime.timedelta(days=30):
                structured_logger.warning(
                    "Certificate is about to expire",
                    logger_name="https",
                    expiration=expiration.isoformat()
                )
                
                # Create alert
                alert_manager.create_alert(
                    level="warning",
                    message=f"Certificate is about to expire: {expiration.isoformat()}",
                    source="https"
                )
            
            return True
        
        except Exception as e:
            # Log error
            structured_logger.error(
                "Error validating certificates",
                logger_name="https",
                error=str(e)
            )
            
            return False
    
    def _generate_certificates(self):
        """Generate certificates."""
        try:
            # Create temporary directory
            with tempfile.TemporaryDirectory() as temp_dir:
                # Generate CA key
                ca_key_path = os.path.join(temp_dir, "ca.key")
                subprocess.run([
                    "openssl", "genrsa",
                    "-out", ca_key_path,
                    str(self.key_size)
                ], check=True)
                
                # Generate CA certificate
                ca_cert_path = os.path.join(temp_dir, "ca.crt")
                subprocess.run([
                    "openssl", "req",
                    "-new", "-x509",
                    "-key", ca_key_path,
                    "-out", ca_cert_path,
                    "-days", str(self.cert_validity),
                    "-subj", f"/CN=Bulo.Cloud Sentinel CA"
                ], check=True)
                
                # Generate server key
                server_key_path = os.path.join(temp_dir, "server.key")
                subprocess.run([
                    "openssl", "genrsa",
                    "-out", server_key_path,
                    str(self.key_size)
                ], check=True)
                
                # Generate server CSR
                server_csr_path = os.path.join(temp_dir, "server.csr")
                subprocess.run([
                    "openssl", "req",
                    "-new",
                    "-key", server_key_path,
                    "-out", server_csr_path,
                    "-subj", f"/CN={self.common_name}"
                ], check=True)
                
                # Create server certificate
                server_cert_path = os.path.join(temp_dir, "server.crt")
                subprocess.run([
                    "openssl", "x509",
                    "-req",
                    "-in", server_csr_path,
                    "-CA", ca_cert_path,
                    "-CAkey", ca_key_path,
                    "-CAcreateserial",
                    "-out", server_cert_path,
                    "-days", str(self.cert_validity)
                ], check=True)
                
                # Copy certificates to certificate directory
                shutil.copy2(ca_key_path, self.ca_key_path)
                shutil.copy2(ca_cert_path, self.ca_cert_path)
                shutil.copy2(server_key_path, self.key_path)
                shutil.copy2(server_cert_path, self.cert_path)
                
                # Set secure permissions for key files
                os.chmod(self.ca_key_path, 0o600)
                os.chmod(self.key_path, 0o600)
                
                structured_logger.info(
                    "Certificates generated successfully",
                    logger_name="https"
                )
        
        except Exception as e:
            # Log error
            structured_logger.error(
                "Error generating certificates",
                logger_name="https",
                error=str(e)
            )
            
            # Create alert
            alert_manager.create_alert(
                level="error",
                message=f"Error generating certificates: {str(e)}",
                source="https"
            )
    
    def get_ssl_context(self):
        """
        Get SSL context.
        
        Returns:
            SSL context
        """
        try:
            # Check if certificates exist
            if (
                not os.path.exists(self.cert_path) or
                not os.path.exists(self.key_path)
            ):
                structured_logger.warning(
                    "Certificates not found",
                    logger_name="https"
                )
                return None
            
            # Create SSL context
            import ssl
            
            ssl_context = ssl.create_default_context(ssl.Purpose.CLIENT_AUTH)
            ssl_context.load_cert_chain(self.cert_path, self.key_path)
            
            return ssl_context
        
        except Exception as e:
            # Log error
            structured_logger.error(
                "Error creating SSL context",
                logger_name="https",
                error=str(e)
            )
            
            return None


# Create global HTTPS manager
https_manager = HTTPSManager()
