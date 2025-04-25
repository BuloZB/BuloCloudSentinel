"""
Authentication utilities for Bulo.Cloud Sentinel.

This package provides functions for authentication, authorization,
and user management.
"""

from .jwt_handler import (
    create_access_token,
    create_refresh_token,
    create_token_response,
    decode_token,
    get_current_token_data,
    get_current_user_id,
    has_role,
    has_permission,
)

from .password import (
    hash_password,
    verify_password,
    check_password_needs_rehash,
    validate_password,
    generate_password,
)

from .mfa import (
    TOTPManager,
    BackupCodeManager,
    MFAManager,
    totp_manager,
    backup_code_manager,
    mfa_manager,
)

__all__ = [
    # JWT handling
    "create_access_token",
    "create_refresh_token",
    "create_token_response",
    "decode_token",
    "get_current_token_data",
    "get_current_user_id",
    "has_role",
    "has_permission",
    
    # Password handling
    "hash_password",
    "verify_password",
    "check_password_needs_rehash",
    "validate_password",
    "generate_password",
    
    # MFA
    "TOTPManager",
    "BackupCodeManager",
    "MFAManager",
    "totp_manager",
    "backup_code_manager",
    "mfa_manager",
]
