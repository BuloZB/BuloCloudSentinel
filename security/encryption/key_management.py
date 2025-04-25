"""
Key management utilities for Bulo.Cloud Sentinel.

This module provides functions for securely managing cryptographic keys,
including key rotation, storage, and retrieval.
"""

import os
import json
import base64
import secrets
import hashlib
from typing import Dict, List, Optional, Tuple, Union
from datetime import datetime, timedelta, timezone
from pathlib import Path

from cryptography.fernet import Fernet
from cryptography.hazmat.primitives import hashes
from cryptography.hazmat.primitives.kdf.pbkdf2 import PBKDF2HMAC
from cryptography.hazmat.primitives.asymmetric import rsa, padding
from cryptography.hazmat.primitives.serialization import (
    load_pem_private_key,
    load_pem_public_key,
    Encoding,
    PrivateFormat,
    PublicFormat,
    NoEncryption,
    BestAvailableEncryption,
)

from .encryption import (
    generate_key,
    derive_key,
    encrypt_aes_gcm,
    decrypt_aes_gcm,
    encrypt_with_password,
    decrypt_with_password,
)


class KeyStore:
    """
    A secure key store for managing cryptographic keys.
    """
    
    def __init__(self, store_path: Union[str, Path], master_password: Optional[str] = None):
        """
        Initialize the key store.
        
        Args:
            store_path: Path to the key store file
            master_password: Master password for the key store
        """
        self.store_path = Path(store_path)
        self.master_password = master_password
        self.keys = {}
        self.metadata = {
            "created_at": datetime.now(timezone.utc).isoformat(),
            "last_rotated": None,
            "version": 1
        }
        
        # Load existing key store if it exists
        if self.store_path.exists():
            self.load()
    
    def load(self):
        """
        Load the key store from disk.
        
        Raises:
            ValueError: If the master password is incorrect
        """
        if not self.store_path.exists():
            return
        
        # Read encrypted data
        encrypted_data = self.store_path.read_bytes()
        
        # Decrypt data
        if self.master_password:
            try:
                decrypted_data = decrypt_with_password(encrypted_data, self.master_password)
                data = json.loads(decrypted_data.decode('utf-8'))
            except Exception as e:
                raise ValueError(f"Failed to decrypt key store: {str(e)}")
        else:
            try:
                data = json.loads(encrypted_data.decode('utf-8'))
            except Exception as e:
                raise ValueError(f"Failed to load key store: {str(e)}")
        
        # Load keys and metadata
        self.keys = {k: base64.b64decode(v) for k, v in data.get("keys", {}).items()}
        self.metadata = data.get("metadata", {})
    
    def save(self):
        """
        Save the key store to disk.
        """
        # Prepare data
        data = {
            "keys": {k: base64.b64encode(v).decode('utf-8') for k, v in self.keys.items()},
            "metadata": self.metadata
        }
        
        # Update metadata
        self.metadata["last_modified"] = datetime.now(timezone.utc).isoformat()
        
        # Convert to JSON
        json_data = json.dumps(data, indent=2).encode('utf-8')
        
        # Encrypt data if master password is set
        if self.master_password:
            encrypted_data = encrypt_with_password(json_data, self.master_password)
            self.store_path.write_bytes(encrypted_data)
        else:
            self.store_path.write_bytes(json_data)
    
    def add_key(self, key_id: str, key: bytes, metadata: Optional[Dict] = None):
        """
        Add a key to the key store.
        
        Args:
            key_id: Unique identifier for the key
            key: The key to store
            metadata: Optional metadata for the key
        """
        if key_id in self.keys:
            raise ValueError(f"Key with ID '{key_id}' already exists")
        
        self.keys[key_id] = key
        
        # Add metadata
        if "key_metadata" not in self.metadata:
            self.metadata["key_metadata"] = {}
        
        self.metadata["key_metadata"][key_id] = metadata or {
            "created_at": datetime.now(timezone.utc).isoformat(),
            "last_used": None,
            "purpose": "general"
        }
        
        self.save()
    
    def get_key(self, key_id: str) -> bytes:
        """
        Get a key from the key store.
        
        Args:
            key_id: Unique identifier for the key
            
        Returns:
            The key
            
        Raises:
            KeyError: If the key does not exist
        """
        if key_id not in self.keys:
            raise KeyError(f"Key with ID '{key_id}' not found")
        
        # Update last used timestamp
        if "key_metadata" in self.metadata and key_id in self.metadata["key_metadata"]:
            self.metadata["key_metadata"][key_id]["last_used"] = datetime.now(timezone.utc).isoformat()
            self.save()
        
        return self.keys[key_id]
    
    def remove_key(self, key_id: str):
        """
        Remove a key from the key store.
        
        Args:
            key_id: Unique identifier for the key
            
        Raises:
            KeyError: If the key does not exist
        """
        if key_id not in self.keys:
            raise KeyError(f"Key with ID '{key_id}' not found")
        
        del self.keys[key_id]
        
        # Remove metadata
        if "key_metadata" in self.metadata and key_id in self.metadata["key_metadata"]:
            del self.metadata["key_metadata"][key_id]
        
        self.save()
    
    def rotate_key(self, key_id: str, new_key: Optional[bytes] = None) -> bytes:
        """
        Rotate a key in the key store.
        
        Args:
            key_id: Unique identifier for the key
            new_key: The new key (generated if not provided)
            
        Returns:
            The new key
            
        Raises:
            KeyError: If the key does not exist
        """
        if key_id not in self.keys:
            raise KeyError(f"Key with ID '{key_id}' not found")
        
        # Generate a new key if not provided
        if new_key is None:
            new_key = generate_key()
        
        # Store the old key with a versioned ID
        old_key = self.keys[key_id]
        old_key_id = f"{key_id}_v{datetime.now(timezone.utc).strftime('%Y%m%d%H%M%S')}"
        self.keys[old_key_id] = old_key
        
        # Update metadata for the old key
        if "key_metadata" in self.metadata and key_id in self.metadata["key_metadata"]:
            old_metadata = self.metadata["key_metadata"][key_id].copy()
            old_metadata["rotated_at"] = datetime.now(timezone.utc).isoformat()
            old_metadata["active"] = False
            self.metadata["key_metadata"][old_key_id] = old_metadata
        
        # Update the current key
        self.keys[key_id] = new_key
        
        # Update metadata for the new key
        if "key_metadata" in self.metadata and key_id in self.metadata["key_metadata"]:
            self.metadata["key_metadata"][key_id]["rotated_at"] = datetime.now(timezone.utc).isoformat()
            self.metadata["key_metadata"][key_id]["active"] = True
        
        # Update global metadata
        self.metadata["last_rotated"] = datetime.now(timezone.utc).isoformat()
        
        self.save()
        
        return new_key
    
    def list_keys(self) -> List[Dict]:
        """
        List all keys in the key store.
        
        Returns:
            A list of key metadata
        """
        result = []
        
        for key_id in self.keys:
            metadata = {}
            
            if "key_metadata" in self.metadata and key_id in self.metadata["key_metadata"]:
                metadata = self.metadata["key_metadata"][key_id].copy()
            
            result.append({
                "id": key_id,
                "metadata": metadata
            })
        
        return result
    
    def change_master_password(self, new_password: str):
        """
        Change the master password for the key store.
        
        Args:
            new_password: The new master password
        """
        self.master_password = new_password
        self.save()


class KeyRotationManager:
    """
    Manager for automatic key rotation.
    """
    
    def __init__(self, key_store: KeyStore, rotation_interval: timedelta = timedelta(days=90)):
        """
        Initialize the key rotation manager.
        
        Args:
            key_store: The key store to manage
            rotation_interval: The interval at which to rotate keys
        """
        self.key_store = key_store
        self.rotation_interval = rotation_interval
    
    def check_rotation_needed(self) -> List[str]:
        """
        Check which keys need rotation.
        
        Returns:
            A list of key IDs that need rotation
        """
        keys_to_rotate = []
        now = datetime.now(timezone.utc)
        
        for key_info in self.key_store.list_keys():
            key_id = key_info["id"]
            metadata = key_info["metadata"]
            
            # Skip versioned keys
            if "_v" in key_id:
                continue
            
            # Check if rotation is needed
            if "rotated_at" in metadata:
                last_rotated = datetime.fromisoformat(metadata["rotated_at"])
                if now - last_rotated >= self.rotation_interval:
                    keys_to_rotate.append(key_id)
            elif "created_at" in metadata:
                created_at = datetime.fromisoformat(metadata["created_at"])
                if now - created_at >= self.rotation_interval:
                    keys_to_rotate.append(key_id)
        
        return keys_to_rotate
    
    def rotate_keys(self, key_ids: Optional[List[str]] = None) -> Dict[str, bytes]:
        """
        Rotate the specified keys or all keys that need rotation.
        
        Args:
            key_ids: List of key IDs to rotate (checks all if not provided)
            
        Returns:
            A dictionary of rotated key IDs and their new values
        """
        if key_ids is None:
            key_ids = self.check_rotation_needed()
        
        rotated_keys = {}
        
        for key_id in key_ids:
            try:
                new_key = self.key_store.rotate_key(key_id)
                rotated_keys[key_id] = new_key
            except KeyError:
                continue
        
        return rotated_keys


def create_key_store(
    store_path: Union[str, Path],
    master_password: Optional[str] = None,
    initial_keys: Optional[Dict[str, bytes]] = None
) -> KeyStore:
    """
    Create a new key store.
    
    Args:
        store_path: Path to the key store file
        master_password: Master password for the key store
        initial_keys: Initial keys to add to the store
        
    Returns:
        The created key store
    """
    key_store = KeyStore(store_path, master_password)
    
    # Add initial keys
    if initial_keys:
        for key_id, key in initial_keys.items():
            key_store.add_key(key_id, key)
    
    return key_store


def generate_key_pair(key_id: str, key_store: KeyStore) -> Tuple[bytes, bytes]:
    """
    Generate an RSA key pair and store the private key.
    
    Args:
        key_id: Unique identifier for the key
        key_store: The key store to use
        
    Returns:
        A tuple of (private_key_pem, public_key_pem)
    """
    # Generate key pair
    private_key = rsa.generate_private_key(
        public_exponent=65537,
        key_size=4096,
    )
    
    # Serialize private key
    private_pem = private_key.private_bytes(
        encoding=Encoding.PEM,
        format=PrivateFormat.PKCS8,
        encryption_algorithm=NoEncryption()
    )
    
    # Serialize public key
    public_key = private_key.public_key()
    public_pem = public_key.public_bytes(
        encoding=Encoding.PEM,
        format=PublicFormat.SubjectPublicKeyInfo
    )
    
    # Store private key
    key_store.add_key(key_id, private_pem, {
        "created_at": datetime.now(timezone.utc).isoformat(),
        "type": "rsa_private_key",
        "purpose": "encryption",
        "active": True
    })
    
    return private_pem, public_pem
