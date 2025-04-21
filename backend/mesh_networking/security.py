"""
Security module for mesh networking in Bulo.Cloud Sentinel Tactical Use Module.

This module provides security features for mesh networking communication.
"""

import base64
import hashlib
import hmac
import json
import os
import time
import uuid
from datetime import datetime, timedelta
from typing import Dict, Any, Optional, Tuple, List

from cryptography.hazmat.primitives.ciphers import Cipher, algorithms, modes
from cryptography.hazmat.primitives import padding, hashes
from cryptography.hazmat.primitives.kdf.pbkdf2 import PBKDF2HMAC
from cryptography.hazmat.backends import default_backend


class MeshSecurity:
    """
    Security provider for mesh network communication.
    """

    def __init__(self, shared_key: Optional[str] = None, key_rotation_days: int = 7):
        """
        Initialize the security provider.

        Args:
            shared_key: Optional shared key for encryption/authentication
            key_rotation_days: Number of days before key rotation
        """
        self.shared_key = shared_key or os.urandom(32).hex()
        self._key_bytes = bytes.fromhex(self.shared_key)
        self.key_rotation_days = key_rotation_days
        self.key_creation_time = datetime.utcnow()
        self.previous_keys: List[Tuple[bytes, datetime]] = []  # List of (key, expiration_time) tuples
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
            self.shared_key = os.urandom(32).hex()
            self._key_bytes = bytes.fromhex(self.shared_key)
            self.key_creation_time = current_time
            self.key_id = str(uuid.uuid4())

            # Clean up expired keys
            self.previous_keys = [(k, exp) for k, exp in self.previous_keys if exp > current_time]

    def _derive_key(self, password: str, salt: bytes) -> bytes:
        """
        Derive a key from a password using PBKDF2.

        Args:
            password: Password to derive key from
            salt: Salt for key derivation

        Returns:
            Derived key
        """
        kdf = PBKDF2HMAC(
            algorithm=hashes.SHA256(),
            length=32,
            salt=salt,
            iterations=100000,
            backend=default_backend()
        )
        return kdf.derive(password.encode('utf-8'))

    def encrypt_message(self, message_data: Dict[str, Any]) -> Dict[str, Any]:
        """
        Encrypt a message.

        Args:
            message_data: Message data to encrypt

        Returns:
            Encrypted message data
        """
        # Check if key rotation is needed
        self._check_key_rotation()

        # Convert message to JSON string
        message_json = json.dumps(message_data)
        message_bytes = message_json.encode('utf-8')

        # Generate a random IV
        iv = os.urandom(16)

        # Pad the message
        padder = padding.PKCS7(128).padder()
        padded_data = padder.update(message_bytes) + padder.finalize()

        # Encrypt the message
        cipher = Cipher(algorithms.AES(self._key_bytes), modes.GCM(iv), backend=default_backend())
        encryptor = cipher.encryptor()
        encrypted_data = encryptor.update(padded_data) + encryptor.finalize()

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

    def decrypt_message(self, encrypted_message: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        """
        Decrypt a message.

        Args:
            encrypted_message: Encrypted message data

        Returns:
            Decrypted message data or None if authentication fails
        """
        try:
            # Decode base64 strings to binary data
            encrypted_data = base64.b64decode(encrypted_message["encrypted_data"])
            iv = base64.b64decode(encrypted_message["iv"])
            tag = base64.b64decode(encrypted_message["tag"])
            key_id = encrypted_message.get("key_id")

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
                            padded_data = decryptor.update(encrypted_data) + decryptor.finalize()

                            # If we get here, decryption succeeded
                            decryption_key = prev_key
                            break
                        except Exception:
                            # Try the next key
                            continue

            # Decrypt the message
            cipher = Cipher(algorithms.AES(decryption_key), modes.GCM(iv, tag), backend=default_backend())
            decryptor = cipher.decryptor()
            padded_data = decryptor.update(encrypted_data) + decryptor.finalize()

            # Unpad the message
            unpadder = padding.PKCS7(128).unpadder()
            data = unpadder.update(padded_data) + unpadder.finalize()

            # Parse JSON
            return json.loads(data.decode('utf-8'))
        except Exception as e:
            print(f"Error decrypting message: {str(e)}")
            return None

    def generate_signature(self, data: Dict[str, Any]) -> str:
        """
        Generate a signature for data.

        Args:
            data: Data to sign

        Returns:
            Base64-encoded signature
        """
        # Convert data to JSON string
        data_json = json.dumps(data, sort_keys=True)
        data_bytes = data_json.encode('utf-8')

        # Generate HMAC
        signature = hmac.new(self._key_bytes, data_bytes, hashlib.sha256).digest()

        # Encode as base64 string
        return base64.b64encode(signature).decode('utf-8')

    def verify_signature(self, data: Dict[str, Any], signature: str) -> bool:
        """
        Verify a signature for data.

        Args:
            data: Data to verify
            signature: Base64-encoded signature

        Returns:
            True if signature is valid, False otherwise
        """
        try:
            # Convert data to JSON string
            data_json = json.dumps(data, sort_keys=True)
            data_bytes = data_json.encode('utf-8')

            # Decode signature
            signature_bytes = base64.b64decode(signature)

            # Generate expected HMAC
            expected_signature = hmac.new(self._key_bytes, data_bytes, hashlib.sha256).digest()

            # Compare signatures
            return hmac.compare_digest(signature_bytes, expected_signature)
        except Exception as e:
            print(f"Error verifying signature: {str(e)}")
            return False

    def rotate_key(self, force: bool = False) -> str:
        """
        Rotate the shared key.

        Args:
            force: Force key rotation regardless of age

        Returns:
            New shared key
        """
        current_time = datetime.utcnow()

        # Store the old key with its expiration time
        expiration_time = current_time + timedelta(days=self.key_rotation_days)
        self.previous_keys.append((self._key_bytes, expiration_time))

        # Generate a new key
        self.shared_key = os.urandom(32).hex()
        self._key_bytes = bytes.fromhex(self.shared_key)
        self.key_creation_time = current_time
        old_key_id = self.key_id
        self.key_id = str(uuid.uuid4())

        # Clean up expired keys
        self.previous_keys = [(k, exp) for k, exp in self.previous_keys if exp > current_time]

        return self.shared_key


class SecureMessageWrapper:
    """
    Wrapper for secure message handling.
    """

    def __init__(self, security_provider: MeshSecurity):
        """
        Initialize the secure message wrapper.

        Args:
            security_provider: Security provider for encryption/authentication
        """
        self.security = security_provider

    def wrap_message(self, message_data: Dict[str, Any]) -> Dict[str, Any]:
        """
        Wrap a message with security features.

        Args:
            message_data: Original message data

        Returns:
            Wrapped message data
        """
        # Add timestamp if not present
        if "timestamp" not in message_data:
            message_data["timestamp"] = int(time.time())

        # Generate signature
        signature = self.security.generate_signature(message_data)

        # Encrypt message
        encrypted_data = self.security.encrypt_message(message_data)

        # Create wrapped message
        return {
            "encrypted": True,
            "data": encrypted_data,
            "signature": signature,
            "version": "1.0"
        }

    def unwrap_message(self, wrapped_message: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        """
        Unwrap a message with security features.

        Args:
            wrapped_message: Wrapped message data

        Returns:
            Original message data or None if verification fails
        """
        # Check if message is encrypted
        if not wrapped_message.get("encrypted", False):
            return wrapped_message

        # Get encrypted data and signature
        encrypted_data = wrapped_message["data"]
        signature = wrapped_message["signature"]

        # Decrypt message
        decrypted_data = self.security.decrypt_message(encrypted_data)
        if decrypted_data is None:
            return None

        # Verify signature
        if not self.security.verify_signature(decrypted_data, signature):
            print("Signature verification failed")
            return None

        # Check timestamp to prevent replay attacks
        message_time = decrypted_data.get("timestamp", 0)
        current_time = int(time.time())
        if current_time - message_time > 300:  # 5 minutes
            print("Message too old, possible replay attack")
            return None

        return decrypted_data
