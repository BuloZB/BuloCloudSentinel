"""
Encryption utilities.

This module provides functions for secure encryption and decryption.
"""

import base64
import os
import secrets
import time
import uuid
from datetime import datetime, timedelta
from typing import Dict, Any, Optional, Tuple, Union

from cryptography.hazmat.primitives.ciphers import Cipher, algorithms, modes
from cryptography.hazmat.primitives import hashes, padding
from cryptography.hazmat.primitives.kdf.pbkdf2 import PBKDF2HMAC
from cryptography.hazmat.primitives.kdf.scrypt import Scrypt
from cryptography.hazmat.backends import default_backend
from cryptography.fernet import Fernet

class EncryptionService:
    """
    Service for encrypting and decrypting data.
    
    This service provides methods for securely encrypting and decrypting data
    using AES-GCM and Fernet.
    """
    
    def __init__(self, master_key: Optional[str] = None, key_rotation_days: int = 7):
        """
        Initialize the encryption service.
        
        Args:
            master_key: Optional master key for encryption/decryption
            key_rotation_days: Number of days before key rotation
        """
        self.master_key = master_key or secrets.token_hex(32)
        self._key_bytes = bytes.fromhex(self.master_key)
        self.key_rotation_days = key_rotation_days
        self.key_creation_time = datetime.utcnow()
        self.previous_keys = []  # List of (key, expiration_time) tuples
        self.key_id = str(uuid.uuid4())
        
    def _check_key_rotation(self) -> None:
        """
        Check if key rotation is needed and rotate if necessary.
        """
        current_time = datetime.utcnow()
        key_age = current_time - self.key_creation_time
        
        # Rotate key if it's older than the rotation period
        if key_age.days >= self.key_rotation_days:
            # Store the old key with its expiration time (keep for 1 more rotation period)
            expiration_time = current_time + timedelta(days=self.key_rotation_days)
            self.previous_keys.append((self._key_bytes, expiration_time))
            
            # Generate a new key
            self.master_key = secrets.token_hex(32)
            self._key_bytes = bytes.fromhex(self.master_key)
            self.key_creation_time = current_time
            self.key_id = str(uuid.uuid4())
            
            # Clean up expired keys
            self.previous_keys = [(k, exp) for k, exp in self.previous_keys if exp > current_time]
    
    def _derive_key(self, password: str, salt: bytes, iterations: int = 100000) -> bytes:
        """
        Derive a key from a password using PBKDF2.
        
        Args:
            password: Password to derive key from
            salt: Salt for key derivation
            iterations: Number of iterations for PBKDF2
            
        Returns:
            Derived key
        """
        kdf = PBKDF2HMAC(
            algorithm=hashes.SHA256(),
            length=32,
            salt=salt,
            iterations=iterations,
            backend=default_backend()
        )
        return kdf.derive(password.encode('utf-8'))
    
    def _derive_key_scrypt(self, password: str, salt: bytes) -> bytes:
        """
        Derive a key from a password using Scrypt.
        
        Args:
            password: Password to derive key from
            salt: Salt for key derivation
            
        Returns:
            Derived key
        """
        kdf = Scrypt(
            salt=salt,
            length=32,
            n=2**14,  # CPU/memory cost factor
            r=8,       # Block size
            p=1,       # Parallelization factor
            backend=default_backend()
        )
        return kdf.derive(password.encode('utf-8'))
    
    def encrypt_data(self, data: bytes) -> Dict[str, str]:
        """
        Encrypt binary data using AES-GCM.
        
        Args:
            data: Data to encrypt
            
        Returns:
            Dictionary with encrypted data and metadata
        """
        # Check if key rotation is needed
        self._check_key_rotation()
        
        # Generate a random IV
        iv = os.urandom(12)  # 96 bits for GCM
        
        # Encrypt the data
        cipher = Cipher(algorithms.AES(self._key_bytes), modes.GCM(iv), backend=default_backend())
        encryptor = cipher.encryptor()
        encrypted_data = encryptor.update(data) + encryptor.finalize()
        
        # Get the authentication tag
        tag = encryptor.tag
        
        # Encode binary data as base64 strings
        return {
            "encrypted_data": base64.b64encode(encrypted_data).decode('utf-8'),
            "iv": base64.b64encode(iv).decode('utf-8'),
            "tag": base64.b64encode(tag).decode('utf-8'),
            "key_id": self.key_id,
            "timestamp": int(time.time())
        }
    
    def decrypt_data(self, encrypted_data_dict: Dict[str, str]) -> Optional[bytes]:
        """
        Decrypt data that was encrypted with encrypt_data.
        
        Args:
            encrypted_data_dict: Dictionary with encrypted data and metadata
            
        Returns:
            Decrypted data or None if decryption fails
        """
        try:
            # Decode base64 strings to binary data
            encrypted_data = base64.b64decode(encrypted_data_dict["encrypted_data"])
            iv = base64.b64decode(encrypted_data_dict["iv"])
            tag = base64.b64decode(encrypted_data_dict["tag"])
            key_id = encrypted_data_dict.get("key_id")
            
            # Determine which key to use
            decryption_key = self._key_bytes
            
            # If key ID doesn't match current key, try previous keys
            if key_id and key_id != self.key_id:
                for prev_key, expiration_time in self.previous_keys:
                    if datetime.utcnow() < expiration_time:
                        # Try to decrypt with this key
                        try:
                            cipher = Cipher(algorithms.AES(prev_key), modes.GCM(iv, tag), backend=default_backend())
                            decryptor = cipher.decryptor()
                            decrypted_data = decryptor.update(encrypted_data) + decryptor.finalize()
                            
                            # If we get here, decryption succeeded
                            return decrypted_data
                        except Exception:
                            # Try the next key
                            continue
            
            # Decrypt with the current key
            cipher = Cipher(algorithms.AES(decryption_key), modes.GCM(iv, tag), backend=default_backend())
            decryptor = cipher.decryptor()
            decrypted_data = decryptor.update(encrypted_data) + decryptor.finalize()
            
            return decrypted_data
        except Exception as e:
            print(f"Error decrypting data: {str(e)}")
            return None
    
    def encrypt_string(self, text: str) -> Dict[str, str]:
        """
        Encrypt a string using AES-GCM.
        
        Args:
            text: String to encrypt
            
        Returns:
            Dictionary with encrypted data and metadata
        """
        return self.encrypt_data(text.encode('utf-8'))
    
    def decrypt_string(self, encrypted_data_dict: Dict[str, str]) -> Optional[str]:
        """
        Decrypt a string that was encrypted with encrypt_string.
        
        Args:
            encrypted_data_dict: Dictionary with encrypted data and metadata
            
        Returns:
            Decrypted string or None if decryption fails
        """
        decrypted_data = self.decrypt_data(encrypted_data_dict)
        if decrypted_data is None:
            return None
        return decrypted_data.decode('utf-8')
    
    def encrypt_field(self, value: str) -> str:
        """
        Encrypt a single field value for database storage.
        
        Args:
            value: Value to encrypt
            
        Returns:
            Base64-encoded encrypted value
        """
        # Generate a random key for this field
        key = Fernet.generate_key()
        f = Fernet(key)
        
        # Encrypt the value
        encrypted_value = f.encrypt(value.encode('utf-8'))
        
        # Encrypt the key with the master key
        encrypted_key_dict = self.encrypt_data(key)
        encrypted_key_json = base64.b64encode(str(encrypted_key_dict).encode('utf-8')).decode('utf-8')
        
        # Combine the encrypted key and value
        return f"{encrypted_key_json}:{base64.b64encode(encrypted_value).decode('utf-8')}"
    
    def decrypt_field(self, encrypted_value: str) -> Optional[str]:
        """
        Decrypt a field value that was encrypted with encrypt_field.
        
        Args:
            encrypted_value: Encrypted value
            
        Returns:
            Decrypted value or None if decryption fails
        """
        try:
            # Split the encrypted key and value
            encrypted_key_json, encrypted_value_b64 = encrypted_value.split(':', 1)
            
            # Decode and decrypt the key
            encrypted_key_dict = eval(base64.b64decode(encrypted_key_json).decode('utf-8'))
            key = self.decrypt_data(encrypted_key_dict)
            
            if key is None:
                return None
                
            # Decrypt the value
            f = Fernet(key)
            decrypted_value = f.decrypt(base64.b64decode(encrypted_value_b64))
            
            return decrypted_value.decode('utf-8')
        except Exception as e:
            print(f"Error decrypting field: {str(e)}")
            return None
