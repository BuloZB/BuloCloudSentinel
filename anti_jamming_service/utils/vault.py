"""
HashiCorp Vault integration for the Anti-Jamming Service.

This module provides utilities for securely storing and retrieving secrets
using HashiCorp Vault.
"""

import logging
import os
from typing import Dict, Any, Optional, List
import hvac
import json

from anti_jamming_service.utils.config import get_config

logger = logging.getLogger(__name__)


class VaultClient:
    """
    HashiCorp Vault client for secure secret management.
    
    This class provides methods for securely storing and retrieving secrets
    using HashiCorp Vault.
    """
    
    def __init__(self, url: Optional[str] = None, token: Optional[str] = None, path: Optional[str] = None):
        """
        Initialize the Vault client.
        
        Args:
            url: Vault server URL.
            token: Vault authentication token.
            path: Base path for secrets.
        """
        config = get_config().get('security', {}).get('vault', {})
        
        self.enabled = config.get('enabled', False)
        self.url = url or config.get('url') or os.environ.get('VAULT_ADDR')
        self.token = token or config.get('token') or os.environ.get('VAULT_TOKEN')
        self.path = path or config.get('path', 'secret/anti_jamming_service')
        
        self.client = None
        
        if self.enabled:
            self._initialize_client()
    
    def _initialize_client(self):
        """Initialize the Vault client."""
        if not self.url:
            logger.error("Vault URL not provided")
            return
        
        if not self.token:
            logger.error("Vault token not provided")
            return
        
        try:
            self.client = hvac.Client(url=self.url, token=self.token)
            
            if not self.client.is_authenticated():
                logger.error("Failed to authenticate with Vault")
                self.client = None
                return
            
            logger.info(f"Connected to Vault at {self.url}")
        except Exception as e:
            logger.error(f"Failed to initialize Vault client: {str(e)}")
            self.client = None
    
    def is_available(self) -> bool:
        """
        Check if Vault is available and authenticated.
        
        Returns:
            bool: True if Vault is available and authenticated, False otherwise.
        """
        if not self.enabled or not self.client:
            return False
        
        try:
            return self.client.is_authenticated()
        except Exception as e:
            logger.error(f"Failed to check Vault authentication: {str(e)}")
            return False
    
    def get_secret(self, key: str) -> Optional[str]:
        """
        Get a secret from Vault.
        
        Args:
            key: Secret key.
            
        Returns:
            Optional[str]: Secret value, or None if not found or error.
        """
        if not self.is_available():
            logger.warning(f"Vault not available, cannot get secret {key}")
            return None
        
        try:
            # Construct secret path
            secret_path = f"{self.path}/{key}"
            
            # Read secret
            response = self.client.secrets.kv.v2.read_secret_version(path=secret_path)
            
            # Extract value
            if response and 'data' in response and 'data' in response['data']:
                return response['data']['data'].get('value')
            
            logger.warning(f"Secret {key} not found in Vault")
            return None
        except Exception as e:
            logger.error(f"Failed to get secret {key} from Vault: {str(e)}")
            return None
    
    def set_secret(self, key: str, value: str) -> bool:
        """
        Set a secret in Vault.
        
        Args:
            key: Secret key.
            value: Secret value.
            
        Returns:
            bool: True if successful, False otherwise.
        """
        if not self.is_available():
            logger.warning(f"Vault not available, cannot set secret {key}")
            return False
        
        try:
            # Construct secret path
            secret_path = f"{self.path}/{key}"
            
            # Write secret
            self.client.secrets.kv.v2.create_or_update_secret(
                path=secret_path,
                secret={"value": value}
            )
            
            logger.info(f"Secret {key} set in Vault")
            return True
        except Exception as e:
            logger.error(f"Failed to set secret {key} in Vault: {str(e)}")
            return False
    
    def delete_secret(self, key: str) -> bool:
        """
        Delete a secret from Vault.
        
        Args:
            key: Secret key.
            
        Returns:
            bool: True if successful, False otherwise.
        """
        if not self.is_available():
            logger.warning(f"Vault not available, cannot delete secret {key}")
            return False
        
        try:
            # Construct secret path
            secret_path = f"{self.path}/{key}"
            
            # Delete secret
            self.client.secrets.kv.v2.delete_metadata_and_all_versions(path=secret_path)
            
            logger.info(f"Secret {key} deleted from Vault")
            return True
        except Exception as e:
            logger.error(f"Failed to delete secret {key} from Vault: {str(e)}")
            return False
    
    def list_secrets(self) -> List[str]:
        """
        List all secrets in the configured path.
        
        Returns:
            List[str]: List of secret keys.
        """
        if not self.is_available():
            logger.warning("Vault not available, cannot list secrets")
            return []
        
        try:
            # List secrets
            response = self.client.secrets.kv.v2.list_secrets(path=self.path)
            
            # Extract keys
            if response and 'data' in response and 'keys' in response['data']:
                return response['data']['keys']
            
            return []
        except Exception as e:
            logger.error(f"Failed to list secrets from Vault: {str(e)}")
            return []


# Global Vault client instance
_vault_client = None


def get_vault_client() -> VaultClient:
    """
    Get the global Vault client instance.
    
    Returns:
        VaultClient: Vault client instance.
    """
    global _vault_client
    
    if _vault_client is None:
        _vault_client = VaultClient()
    
    return _vault_client


def get_secret(key: str, default: Optional[str] = None) -> Optional[str]:
    """
    Get a secret from Vault.
    
    Args:
        key: Secret key.
        default: Default value if secret not found.
        
    Returns:
        Optional[str]: Secret value, or default if not found.
    """
    client = get_vault_client()
    
    if not client.is_available():
        return default
    
    value = client.get_secret(key)
    return value if value is not None else default
