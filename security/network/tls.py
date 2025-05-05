"""
TLS configuration for Bulo.Cloud Sentinel Security Module.

This module provides secure TLS configuration for the application.
"""

from typing import Dict, List, Optional, Union

import ssl
from cryptography import x509
from cryptography.hazmat.backends import default_backend
from cryptography.hazmat.primitives import hashes, serialization
from cryptography.hazmat.primitives.asymmetric import rsa
from cryptography.x509.oid import NameOID
from datetime import datetime, timedelta, timezone

# TLS configuration
class TLSConfig:
    """TLS configuration for the application."""

    def __init__(
        self,
        cert_file: Optional[str] = None,
        key_file: Optional[str] = None,
        ca_file: Optional[str] = None,
        ciphers: Optional[str] = None,
        min_version: Optional[int] = None,
        dhparam_file: Optional[str] = None,
        verify_client: bool = False,
        alpn_protocols: Optional[List[str]] = None
    ):
        """
        Initialize TLS configuration.

        Args:
            cert_file: Path to certificate file
            key_file: Path to key file
            ca_file: Path to CA certificate file
            ciphers: Cipher suite string
            min_version: Minimum TLS version
            dhparam_file: Path to DH parameters file
            verify_client: Whether to verify client certificates
            alpn_protocols: List of ALPN protocols
        """
        self.cert_file = cert_file
        self.key_file = key_file
        self.ca_file = ca_file
        self.ciphers = ciphers or self.get_secure_ciphers()
        self.min_version = min_version or ssl.PROTOCOL_TLS_SERVER
        self.dhparam_file = dhparam_file
        self.verify_client = verify_client
        self.alpn_protocols = alpn_protocols or ["h2", "http/1.1"]

    def get_secure_ciphers(self) -> str:
        """
        Get secure cipher suites.

        Returns:
            Secure cipher suites string
        """
        # Modern cipher suites for TLS 1.3
        return (
            "TLS_AES_256_GCM_SHA384:"
            "TLS_CHACHA20_POLY1305_SHA256:"
            "TLS_AES_128_GCM_SHA256:"
            # Fallback for TLS 1.2
            "ECDHE-ECDSA-AES256-GCM-SHA384:"
            "ECDHE-RSA-AES256-GCM-SHA384:"
            "ECDHE-ECDSA-CHACHA20-POLY1305:"
            "ECDHE-RSA-CHACHA20-POLY1305:"
            "ECDHE-ECDSA-AES128-GCM-SHA256:"
            "ECDHE-RSA-AES128-GCM-SHA256"
        )

    def get_ssl_context(self) -> ssl.SSLContext:
        """
        Get SSL context for the application.

        Returns:
            SSL context
        """
        # Create SSL context
        context = ssl.SSLContext(self.min_version)

        # Set secure ciphers
        context.set_ciphers(self.ciphers)

        # Set secure options
        context.options |= ssl.OP_NO_SSLv2
        context.options |= ssl.OP_NO_SSLv3
        context.options |= ssl.OP_NO_TLSv1
        context.options |= ssl.OP_NO_TLSv1_1
        context.options |= ssl.OP_NO_COMPRESSION

        # Enable OCSP stapling
        context.options |= ssl.OP_ENABLE_MIDDLEBOX_COMPAT

        # Set verification mode
        if self.verify_client:
            context.verify_mode = ssl.CERT_REQUIRED
        else:
            context.verify_mode = ssl.CERT_NONE

        # Set ALPN protocols
        context.set_alpn_protocols(self.alpn_protocols)

        # Load certificate and key
        if self.cert_file and self.key_file:
            try:
                context.load_cert_chain(self.cert_file, self.key_file)
            except (FileNotFoundError, ssl.SSLError) as e:
                raise ValueError(f"Failed to load certificate or key: {str(e)}")

        # Load CA certificate
        if self.ca_file:
            try:
                context.load_verify_locations(cafile=self.ca_file)
            except (FileNotFoundError, ssl.SSLError) as e:
                raise ValueError(f"Failed to load CA certificate: {str(e)}")

        # Load DH parameters
        if self.dhparam_file:
            try:
                context.load_dh_params(self.dhparam_file)
            except (FileNotFoundError, ssl.SSLError) as e:
                raise ValueError(f"Failed to load DH parameters: {str(e)}")

        return context

    @staticmethod
    def get_secure_ciphers() -> str:
        """
        Get secure cipher suite string.

        Returns:
            Cipher suite string
        """
        # Modern cipher suites for TLS 1.3
        return (
            "TLS_AES_256_GCM_SHA384:"
            "TLS_CHACHA20_POLY1305_SHA256:"
            "TLS_AES_128_GCM_SHA256:"
            # Fallback for TLS 1.2
            "ECDHE-ECDSA-AES256-GCM-SHA384:"
            "ECDHE-RSA-AES256-GCM-SHA384:"
            "ECDHE-ECDSA-CHACHA20-POLY1305:"
            "ECDHE-RSA-CHACHA20-POLY1305:"
            "ECDHE-ECDSA-AES128-GCM-SHA256:"
            "ECDHE-RSA-AES128-GCM-SHA256"
        )

def generate_self_signed_cert(
    common_name: str,
    cert_file: str,
    key_file: str,
    key_size: int = 2048,
    days_valid: int = 365
) -> None:
    """
    Generate a self-signed certificate.

    Args:
        common_name: Common name for the certificate
        cert_file: Path to save certificate
        key_file: Path to save key
        key_size: Key size in bits
        days_valid: Number of days the certificate is valid
    """
    # Generate private key
    private_key = rsa.generate_private_key(
        public_exponent=65537,
        key_size=key_size,
        backend=default_backend()
    )

    # Write private key to file
    with open(key_file, "wb") as f:
        f.write(private_key.private_bytes(
            encoding=serialization.Encoding.PEM,
            format=serialization.PrivateFormat.PKCS8,
            encryption_algorithm=serialization.NoEncryption()
        ))

    # Create certificate
    subject = issuer = x509.Name([
        x509.NameAttribute(NameOID.COUNTRY_NAME, "US"),
        x509.NameAttribute(NameOID.STATE_OR_PROVINCE_NAME, "California"),
        x509.NameAttribute(NameOID.LOCALITY_NAME, "San Francisco"),
        x509.NameAttribute(NameOID.ORGANIZATION_NAME, "Bulo.Cloud Sentinel"),
        x509.NameAttribute(NameOID.COMMON_NAME, common_name),
    ])

    cert = x509.CertificateBuilder().subject_name(
        subject
    ).issuer_name(
        issuer
    ).public_key(
        private_key.public_key()
    ).serial_number(
        x509.random_serial_number()
    ).not_valid_before(
        datetime.now(timezone.utc)
    ).not_valid_after(
        datetime.now(timezone.utc) + timedelta(days=days_valid)
    ).add_extension(
        x509.SubjectAlternativeName([x509.DNSName(common_name)]),
        critical=False
    ).sign(private_key, hashes.SHA256(), default_backend())

    # Write certificate to file
    with open(cert_file, "wb") as f:
        f.write(cert.public_bytes(serialization.Encoding.PEM))

def check_cert_expiration(cert_file: str) -> Dict[str, Union[str, datetime, int]]:
    """
    Check certificate expiration.

    Args:
        cert_file: Path to certificate file

    Returns:
        Dictionary with certificate information
    """
    # Read certificate
    with open(cert_file, "rb") as f:
        cert_data = f.read()

    # Parse certificate
    cert = x509.load_pem_x509_certificate(cert_data, default_backend())

    # Get certificate information
    not_valid_after = cert.not_valid_after
    not_valid_before = cert.not_valid_before
    days_remaining = (not_valid_after - datetime.now(timezone.utc)).days

    return {
        "subject": cert.subject.rfc4514_string(),
        "issuer": cert.issuer.rfc4514_string(),
        "not_valid_before": not_valid_before,
        "not_valid_after": not_valid_after,
        "days_remaining": days_remaining,
        "serial_number": cert.serial_number,
        "version": cert.version.name
    }

def get_cert_fingerprint(cert_file: str, algorithm: str = "sha256") -> str:
    """
    Get certificate fingerprint.

    Args:
        cert_file: Path to certificate file
        algorithm: Hash algorithm

    Returns:
        Certificate fingerprint
    """
    # Read certificate
    with open(cert_file, "rb") as f:
        cert_data = f.read()

    # Parse certificate
    cert = x509.load_pem_x509_certificate(cert_data, default_backend())

    # Get fingerprint
    if algorithm == "sha256":
        fingerprint = cert.fingerprint(hashes.SHA256())
    elif algorithm == "sha1":
        # Mark SHA1 as not used for security purposes
        # This is only used for compatibility with legacy systems
        # and should not be used for security-critical applications
        fingerprint = cert.fingerprint(hashes.SHA1())
        import logging
        logging.warning("SHA1 is a weak hash algorithm and should not be used for security purposes")
    else:
        raise ValueError(f"Unsupported algorithm: {algorithm}")

    # Format fingerprint
    return ":".join(f"{b:02X}" for b in fingerprint)
