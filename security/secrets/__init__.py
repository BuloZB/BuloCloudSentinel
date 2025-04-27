"""
Secrets management for Bulo.Cloud Sentinel.

This module provides functionality for securely managing secrets.
"""

from .secrets_manager import (
    get_secret,
    set_secret,
    delete_secret,
    list_secrets,
    configure_vault_backend,
    configure_aws_secrets_manager_backend
)

from .config import (
    configure_from_env,
    configure_from_file,
    initialize_secrets
)

from .rotation import (
    add_secret_for_rotation,
    remove_secret_from_rotation,
    rotate_secret,
    rotate_all_secrets,
    start_secret_rotation,
    stop_secret_rotation,
    get_rotation_status
)

__all__ = [
    # Secrets manager
    "get_secret",
    "set_secret",
    "delete_secret",
    "list_secrets",
    "configure_vault_backend",
    "configure_aws_secrets_manager_backend",
    
    # Configuration
    "configure_from_env",
    "configure_from_file",
    "initialize_secrets",
    
    # Rotation
    "add_secret_for_rotation",
    "remove_secret_from_rotation",
    "rotate_secret",
    "rotate_all_secrets",
    "start_secret_rotation",
    "stop_secret_rotation",
    "get_rotation_status"
]
