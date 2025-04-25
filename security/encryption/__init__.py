"""
Encryption utilities for Bulo.Cloud Sentinel.

This package provides functions for securely encrypting and decrypting data,
as well as managing cryptographic keys.
"""

from .encryption import (
    generate_aes_key,
    aes_encrypt,
    aes_decrypt,
    generate_fernet_key,
    derive_key_from_password,
    fernet_encrypt,
    fernet_decrypt,
    generate_rsa_key_pair,
    rsa_encrypt,
    rsa_decrypt,
    hybrid_encrypt,
    hybrid_decrypt,
)

from .key_management import (
    KeyStore,
    KeyRotationManager,
    create_key_store,
    generate_key_pair,
)

__all__ = [
    # Encryption
    "generate_aes_key",
    "aes_encrypt",
    "aes_decrypt",
    "generate_fernet_key",
    "derive_key_from_password",
    "fernet_encrypt",
    "fernet_decrypt",
    "generate_rsa_key_pair",
    "rsa_encrypt",
    "rsa_decrypt",
    "hybrid_encrypt",
    "hybrid_decrypt",
    
    # Key management
    "KeyStore",
    "KeyRotationManager",
    "create_key_store",
    "generate_key_pair",
]
