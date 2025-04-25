"""
Multi-factor authentication utilities for Bulo.Cloud Sentinel.

This module provides functions for implementing multi-factor authentication,
including TOTP (Time-based One-Time Password) and backup codes.
"""

import os
import time
import base64
import secrets
import hashlib
from typing import Dict, List, Optional, Tuple, Union
from datetime import datetime, timedelta, timezone

import pyotp
import qrcode
from io import BytesIO
from fastapi import HTTPException, status

from ..logging.secure_logging import get_secure_logger

logger = get_secure_logger("mfa")


class TOTPManager:
    """
    Manager for TOTP (Time-based One-Time Password) authentication.
    """
    
    def __init__(
        self,
        issuer_name: str = "Bulo.Cloud Sentinel",
        digits: int = 6,
        interval: int = 30,
        algorithm: str = "SHA1"
    ):
        """
        Initialize the TOTP manager.
        
        Args:
            issuer_name: Name of the issuer for the TOTP
            digits: Number of digits in the TOTP
            interval: Time interval in seconds
            algorithm: Hash algorithm to use
        """
        self.issuer_name = issuer_name
        self.digits = digits
        self.interval = interval
        self.algorithm = algorithm
        self.logger = logger
    
    def generate_secret(self) -> str:
        """
        Generate a new TOTP secret.
        
        Returns:
            Base32-encoded secret
        """
        # Generate a random secret
        secret = pyotp.random_base32()
        
        self.logger.info("Generated new TOTP secret")
        
        return secret
    
    def get_provisioning_uri(self, username: str, secret: str) -> str:
        """
        Get the provisioning URI for the TOTP.
        
        Args:
            username: Username for the TOTP
            secret: TOTP secret
            
        Returns:
            Provisioning URI
        """
        totp = pyotp.TOTP(
            secret,
            digits=self.digits,
            interval=self.interval,
            name=username,
            issuer=self.issuer_name
        )
        
        return totp.provisioning_uri()
    
    def generate_qr_code(self, username: str, secret: str) -> bytes:
        """
        Generate a QR code for the TOTP.
        
        Args:
            username: Username for the TOTP
            secret: TOTP secret
            
        Returns:
            QR code image as bytes
        """
        # Get the provisioning URI
        uri = self.get_provisioning_uri(username, secret)
        
        # Generate QR code
        qr = qrcode.QRCode(
            version=1,
            error_correction=qrcode.constants.ERROR_CORRECT_L,
            box_size=10,
            border=4,
        )
        qr.add_data(uri)
        qr.make(fit=True)
        
        # Create image
        img = qr.make_image(fill_color="black", back_color="white")
        
        # Save image to bytes
        buffer = BytesIO()
        img.save(buffer, format="PNG")
        
        self.logger.info(f"Generated QR code for user: {username}")
        
        return buffer.getvalue()
    
    def verify_code(self, secret: str, code: str, valid_window: int = 1) -> bool:
        """
        Verify a TOTP code.
        
        Args:
            secret: TOTP secret
            code: TOTP code to verify
            valid_window: Number of intervals to check before and after the current one
            
        Returns:
            True if the code is valid, False otherwise
        """
        if not secret or not code:
            return False
        
        # Create TOTP object
        totp = pyotp.TOTP(
            secret,
            digits=self.digits,
            interval=self.interval
        )
        
        # Verify code
        result = totp.verify(code, valid_window=valid_window)
        
        if result:
            self.logger.info("TOTP code verified successfully")
        else:
            self.logger.warning("Invalid TOTP code")
        
        return result


class BackupCodeManager:
    """
    Manager for backup codes.
    """
    
    def __init__(
        self,
        code_count: int = 10,
        code_length: int = 8,
        hash_algorithm: str = "sha256"
    ):
        """
        Initialize the backup code manager.
        
        Args:
            code_count: Number of backup codes to generate
            code_length: Length of each backup code
            hash_algorithm: Hash algorithm to use
        """
        self.code_count = code_count
        self.code_length = code_length
        self.hash_algorithm = hash_algorithm
        self.logger = logger
    
    def generate_codes(self) -> Tuple[List[str], List[str]]:
        """
        Generate backup codes.
        
        Returns:
            Tuple of (plain_codes, hashed_codes)
        """
        # Generate random codes
        plain_codes = []
        hashed_codes = []
        
        for _ in range(self.code_count):
            # Generate a random code
            code = "".join(secrets.choice("0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ") for _ in range(self.code_length))
            
            # Hash the code
            hashed_code = self._hash_code(code)
            
            plain_codes.append(code)
            hashed_codes.append(hashed_code)
        
        self.logger.info(f"Generated {self.code_count} backup codes")
        
        return plain_codes, hashed_codes
    
    def _hash_code(self, code: str) -> str:
        """
        Hash a backup code.
        
        Args:
            code: Backup code to hash
            
        Returns:
            Hashed code
        """
        # Create a hash object
        hash_obj = hashlib.new(self.hash_algorithm)
        
        # Update with the code
        hash_obj.update(code.encode())
        
        # Return the hexadecimal digest
        return hash_obj.hexdigest()
    
    def verify_code(self, code: str, hashed_codes: List[str]) -> bool:
        """
        Verify a backup code.
        
        Args:
            code: Backup code to verify
            hashed_codes: List of hashed backup codes
            
        Returns:
            True if the code is valid, False otherwise
        """
        if not code or not hashed_codes:
            return False
        
        # Hash the code
        hashed_code = self._hash_code(code)
        
        # Check if the hashed code is in the list
        result = hashed_code in hashed_codes
        
        if result:
            self.logger.info("Backup code verified successfully")
        else:
            self.logger.warning("Invalid backup code")
        
        return result


class MFAManager:
    """
    Manager for multi-factor authentication.
    """
    
    def __init__(
        self,
        totp_manager: Optional[TOTPManager] = None,
        backup_code_manager: Optional[BackupCodeManager] = None
    ):
        """
        Initialize the MFA manager.
        
        Args:
            totp_manager: TOTP manager
            backup_code_manager: Backup code manager
        """
        self.totp_manager = totp_manager or TOTPManager()
        self.backup_code_manager = backup_code_manager or BackupCodeManager()
        self.logger = logger
    
    def setup_mfa(self, username: str) -> Dict[str, Union[str, List[str], bytes]]:
        """
        Set up MFA for a user.
        
        Args:
            username: Username for the MFA
            
        Returns:
            Dictionary with MFA setup information
        """
        # Generate TOTP secret
        totp_secret = self.totp_manager.generate_secret()
        
        # Generate QR code
        qr_code = self.totp_manager.generate_qr_code(username, totp_secret)
        
        # Generate backup codes
        backup_codes, hashed_backup_codes = self.backup_code_manager.generate_codes()
        
        self.logger.info(f"MFA setup for user: {username}")
        
        return {
            "totp_secret": totp_secret,
            "qr_code": qr_code,
            "backup_codes": backup_codes,
            "hashed_backup_codes": hashed_backup_codes
        }
    
    def verify_mfa(
        self,
        code: str,
        totp_secret: Optional[str] = None,
        hashed_backup_codes: Optional[List[str]] = None
    ) -> bool:
        """
        Verify an MFA code.
        
        Args:
            code: MFA code to verify
            totp_secret: TOTP secret
            hashed_backup_codes: List of hashed backup codes
            
        Returns:
            True if the code is valid, False otherwise
            
        Raises:
            HTTPException: If no verification method is available
        """
        if not code:
            return False
        
        # Try TOTP verification
        if totp_secret:
            if self.totp_manager.verify_code(totp_secret, code):
                return True
        
        # Try backup code verification
        if hashed_backup_codes:
            if self.backup_code_manager.verify_code(code, hashed_backup_codes):
                return True
        
        # If we get here, no verification method succeeded
        if not totp_secret and not hashed_backup_codes:
            self.logger.error("No MFA verification method available")
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="No MFA verification method available"
            )
        
        self.logger.warning("MFA verification failed")
        return False


# Create default instances
totp_manager = TOTPManager()
backup_code_manager = BackupCodeManager()
mfa_manager = MFAManager(totp_manager, backup_code_manager)
