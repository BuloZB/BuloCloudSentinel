"""
Encryption utilities for Bulo.Cloud Sentinel Security Module.

This module provides encryption and decryption utilities for sensitive data.
"""

import os
import base64
from typing import Tuple, Union, Optional

from cryptography.fernet import Fernet
from cryptography.hazmat.primitives import hashes
from cryptography.hazmat.primitives.kdf.pbkdf2 import PBKDF2HMAC
from cryptography.hazmat.primitives.asymmetric import rsa, padding
from cryptography.hazmat.primitives.ciphers import Cipher, algorithms, modes
from cryptography.hazmat.primitives import serialization
from cryptography.hazmat.backends import default_backend

# AES encryption

def generate_aes_key() -> bytes:
    """
    Generate a secure AES-256 key.
    
    Returns:
        32-byte key
    """
    return os.urandom(32)

def aes_encrypt(data: Union[str, bytes], key: bytes) -> bytes:
    """
    Encrypt data using AES-256-GCM.
    
    Args:
        data: Data to encrypt (string or bytes)
        key: 32-byte encryption key
        
    Returns:
        Encrypted data with IV and tag
    """
    if isinstance(data, str):
        data = data.encode('utf-8')
    
    # Generate a random 96-bit IV
    iv = os.urandom(12)
    
    # Create an encryptor object
    cipher = Cipher(algorithms.AES(key), modes.GCM(iv), backend=default_backend())
    encryptor = cipher.encryptor()
    
    # Encrypt the data
    ciphertext = encryptor.update(data) + encryptor.finalize()
    
    # Get the tag
    tag = encryptor.tag
    
    # Return IV + ciphertext + tag
    return iv + ciphertext + tag

def aes_decrypt(encrypted_data: bytes, key: bytes) -> bytes:
    """
    Decrypt data using AES-256-GCM.
    
    Args:
        encrypted_data: Encrypted data with IV and tag
        key: 32-byte encryption key
        
    Returns:
        Decrypted data
    """
    # Extract IV (first 12 bytes)
    iv = encrypted_data[:12]
    
    # Extract tag (last 16 bytes)
    tag = encrypted_data[-16:]
    
    # Extract ciphertext (everything in between)
    ciphertext = encrypted_data[12:-16]
    
    # Create a decryptor object
    cipher = Cipher(algorithms.AES(key), modes.GCM(iv, tag), backend=default_backend())
    decryptor = cipher.decryptor()
    
    # Decrypt the data
    return decryptor.update(ciphertext) + decryptor.finalize()

# Fernet encryption (simpler API for AES)

def generate_fernet_key() -> bytes:
    """
    Generate a secure Fernet key.
    
    Returns:
        Fernet key
    """
    return Fernet.generate_key()

def derive_key_from_password(password: str, salt: Optional[bytes] = None) -> Tuple[bytes, bytes]:
    """
    Derive a key from a password using PBKDF2.
    
    Args:
        password: Password to derive key from
        salt: Optional salt (generated if not provided)
        
    Returns:
        Tuple of (key, salt)
    """
    if salt is None:
        salt = os.urandom(16)
    
    kdf = PBKDF2HMAC(
        algorithm=hashes.SHA256(),
        length=32,
        salt=salt,
        iterations=100000,
        backend=default_backend()
    )
    
    key = base64.urlsafe_b64encode(kdf.derive(password.encode()))
    
    return key, salt

def fernet_encrypt(data: Union[str, bytes], key: bytes) -> bytes:
    """
    Encrypt data using Fernet (AES-128-CBC with HMAC).
    
    Args:
        data: Data to encrypt (string or bytes)
        key: Fernet key
        
    Returns:
        Encrypted data
    """
    if isinstance(data, str):
        data = data.encode('utf-8')
    
    f = Fernet(key)
    return f.encrypt(data)

def fernet_decrypt(encrypted_data: bytes, key: bytes) -> bytes:
    """
    Decrypt data using Fernet.
    
    Args:
        encrypted_data: Encrypted data
        key: Fernet key
        
    Returns:
        Decrypted data
    """
    f = Fernet(key)
    return f.decrypt(encrypted_data)

# RSA encryption

def generate_rsa_key_pair(key_size: int = 2048) -> Tuple[bytes, bytes]:
    """
    Generate an RSA key pair.
    
    Args:
        key_size: Key size in bits (2048 or 4096 recommended)
        
    Returns:
        Tuple of (private_key_pem, public_key_pem)
    """
    # Generate private key
    private_key = rsa.generate_private_key(
        public_exponent=65537,
        key_size=key_size,
        backend=default_backend()
    )
    
    # Get public key
    public_key = private_key.public_key()
    
    # Serialize private key to PEM format
    private_pem = private_key.private_bytes(
        encoding=serialization.Encoding.PEM,
        format=serialization.PrivateFormat.PKCS8,
        encryption_algorithm=serialization.NoEncryption()
    )
    
    # Serialize public key to PEM format
    public_pem = public_key.public_bytes(
        encoding=serialization.Encoding.PEM,
        format=serialization.PublicFormat.SubjectPublicKeyInfo
    )
    
    return private_pem, public_pem

def rsa_encrypt(data: Union[str, bytes], public_key_pem: bytes) -> bytes:
    """
    Encrypt data using RSA.
    
    Args:
        data: Data to encrypt (string or bytes)
        public_key_pem: Public key in PEM format
        
    Returns:
        Encrypted data
    """
    if isinstance(data, str):
        data = data.encode('utf-8')
    
    # Load public key
    public_key = serialization.load_pem_public_key(
        public_key_pem,
        backend=default_backend()
    )
    
    # Encrypt data
    ciphertext = public_key.encrypt(
        data,
        padding.OAEP(
            mgf=padding.MGF1(algorithm=hashes.SHA256()),
            algorithm=hashes.SHA256(),
            label=None
        )
    )
    
    return ciphertext

def rsa_decrypt(encrypted_data: bytes, private_key_pem: bytes) -> bytes:
    """
    Decrypt data using RSA.
    
    Args:
        encrypted_data: Encrypted data
        private_key_pem: Private key in PEM format
        
    Returns:
        Decrypted data
    """
    # Load private key
    private_key = serialization.load_pem_private_key(
        private_key_pem,
        password=None,
        backend=default_backend()
    )
    
    # Decrypt data
    plaintext = private_key.decrypt(
        encrypted_data,
        padding.OAEP(
            mgf=padding.MGF1(algorithm=hashes.SHA256()),
            algorithm=hashes.SHA256(),
            label=None
        )
    )
    
    return plaintext

# Hybrid encryption (RSA + AES)

def hybrid_encrypt(data: Union[str, bytes], public_key_pem: bytes) -> Tuple[bytes, bytes]:
    """
    Encrypt data using hybrid encryption (RSA + AES).
    
    This encrypts the data with AES, then encrypts the AES key with RSA.
    
    Args:
        data: Data to encrypt (string or bytes)
        public_key_pem: Public key in PEM format
        
    Returns:
        Tuple of (encrypted_data, encrypted_key)
    """
    if isinstance(data, str):
        data = data.encode('utf-8')
    
    # Generate a random AES key
    aes_key = generate_aes_key()
    
    # Encrypt the data with AES
    encrypted_data = aes_encrypt(data, aes_key)
    
    # Encrypt the AES key with RSA
    encrypted_key = rsa_encrypt(aes_key, public_key_pem)
    
    return encrypted_data, encrypted_key

def hybrid_decrypt(encrypted_data: bytes, encrypted_key: bytes, private_key_pem: bytes) -> bytes:
    """
    Decrypt data using hybrid encryption (RSA + AES).
    
    Args:
        encrypted_data: AES-encrypted data
        encrypted_key: RSA-encrypted AES key
        private_key_pem: Private key in PEM format
        
    Returns:
        Decrypted data
    """
    # Decrypt the AES key with RSA
    aes_key = rsa_decrypt(encrypted_key, private_key_pem)
    
    # Decrypt the data with AES
    decrypted_data = aes_decrypt(encrypted_data, aes_key)
    
    return decrypted_data
