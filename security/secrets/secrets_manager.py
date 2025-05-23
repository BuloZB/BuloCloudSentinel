"""
Secrets Manager for Bulo.Cloud Sentinel.

This module provides a centralized interface for accessing secrets from
various backends (environment variables, HashiCorp Vault, AWS Secrets Manager, etc.).
"""

import os
import logging
import json
from typing import Any, Dict, Optional, Union, List
from abc import ABC, abstractmethod
import threading

# Try to import optional dependencies
try:
    import hvac
    HVAC_AVAILABLE = True
except ImportError:
    HVAC_AVAILABLE = False

try:
    import boto3
    BOTO3_AVAILABLE = True
except ImportError:
    BOTO3_AVAILABLE = False

# Configure logging
logger = logging.getLogger(__name__)


class SecretsBackend(ABC):
    """Abstract base class for secrets backends."""
    
    @abstractmethod
    def get_secret(self, key: str) -> Optional[str]:
        """
        Get a secret by key.
        
        Args:
            key: Secret key
            
        Returns:
            Secret value or None if not found
        """
        pass
    
    @abstractmethod
    def set_secret(self, key: str, value: str) -> bool:
        """
        Set a secret.
        
        Args:
            key: Secret key
            value: Secret value
            
        Returns:
            True if successful, False otherwise
        """
        pass
    
    @abstractmethod
    def delete_secret(self, key: str) -> bool:
        """
        Delete a secret.
        
        Args:
            key: Secret key
            
        Returns:
            True if successful, False otherwise
        """
        pass
    
    @abstractmethod
    def list_secrets(self, prefix: Optional[str] = None) -> List[str]:
        """
        List available secrets.
        
        Args:
            prefix: Optional prefix to filter secrets
            
        Returns:
            List of secret keys
        """
        pass


class EnvironmentSecretsBackend(SecretsBackend):
    """Secrets backend that uses environment variables."""
    
    def __init__(self, prefix: str = ""):
        """
        Initialize the environment secrets backend.
        
        Args:
            prefix: Prefix for environment variables
        """
        self.prefix = prefix
        logger.info(f"Initialized environment [REDACTED])
    
    def get_secret(self, key: str) -> Optional[str]:
        """
        Get a secret from environment variables.
        
        Args:
            key: Secret key
            
        Returns:
            Secret value or None if not found
        """
        env_key = f"{self.prefix}{key}"
        value = os.environ.get(env_key)
        if value is None:
            logger.debug(f"Secret '{key}' not found in environment variables")
        return value
    
    def set_secret(self, key: str, value: str) -> bool:
        """
        Set a secret in environment variables.
        
        Note: This is generally not recommended as environment variables
        are not secure for storing secrets in a running application.
        
        Args:
            key: Secret key
            value: Secret value
            
        Returns:
            True if successful, False otherwise
        """
        env_key = f"{self.prefix}{key}"
        os.environ[env_key] = value
        logger.debug(f"Set secret '{key}' in environment variables")
        return True
    
    def delete_secret(self, key: str) -> bool:
        """
        Delete a secret from environment variables.
        
        Args:
            key: Secret key
            
        Returns:
            True if successful, False otherwise
        """
        env_key = f"{self.prefix}{key}"
        if env_key in os.environ:
            del os.environ[env_key]
            logger.debug(f"Deleted secret '{key}' from environment variables")
            return True
        logger.debug(f"Secret '{key}' not found in environment variables")
        return False
    
    def list_secrets(self, prefix: Optional[str] = None) -> List[str]:
        """
        List available secrets in environment variables.
        
        Args:
            prefix: Optional prefix to filter secrets
            
        Returns:
            List of secret keys
        """
        full_prefix = f"{self.prefix}{prefix or ''}"
        secrets = []
        
        for key in os.environ:
            if key.startswith(full_prefix):
                # Remove the prefix to get the actual key
                secret_key = key[len(self.prefix):]
                secrets.append(secret_key)
        
        return secrets


class VaultSecretsBackend(SecretsBackend):
    """Secrets backend that uses HashiCorp Vault."""
    
    def __init__(
        self,
        url: str,
        token: Optional[str] = None,
        path: str = "secret",
        mount_point: str = "secret",
        namespace: Optional[str] = None,
        client_cert: Optional[str] = None,
        client_key: Optional[str] = None,
        ca_cert: Optional[str] = None
    ):
        """
        Initialize the Vault secrets backend.
        
        Args:
            url: Vault server URL
            token: Vault token
            path: Path in Vault
            mount_point: Mount point in Vault
            namespace: Vault namespace
            client_cert: Client certificate path
            client_key: Client key path
            ca_cert: CA certificate path
        """
        if not HVAC_AVAILABLE:
            raise ImportError("hvac package is required for VaultSecretsBackend")
        
        self.url = url
        self.path = path
        self.mount_point = mount_point
        
        # Initialize Vault client
        self.client = hvac.Client(
            url=url,
            token=token,
            namespace=namespace,
            cert=(client_cert, client_key) if client_cert and client_key else None,
            verify=ca_cert if ca_cert else True
        )
        
        # Check if client is authenticated
        if not self.client.is_authenticated():
            raise ValueError("Failed to authenticate with Vault")
        
        logger.info(f"Initialized Vault secrets backend at {url}, path={path}, mount_point={mount_point}")
    
    def get_secret(self, key: str) -> Optional[str]:
        """
        Get a secret from Vault.
        
        Args:
            key: Secret key
            
        Returns:
            Secret value or None if not found
        """
        try:
            # For KV version 2
            try:
                response = self.client.secrets.kv.v2.read_secret_version(
                    path=f"{self.path}/{key}",
                    mount_point=self.mount_point
                )
                return response["data"]["data"].get("value")
            except Exception:
                # Try KV version 1
                response = self.client.secrets.kv.v1.read_secret(
                    path=f"{self.path}/{key}",
                    mount_point=self.mount_point
                )
                return response["data"].get("value")
        except Exception as e:
            logger.debug(f"Failed to get secret '{key}' from Vault: {str(e)}")
            return None
    
    def set_secret(self, key: str, value: str) -> bool:
        """
        Set a secret in Vault.
        
        Args:
            key: Secret key
            value: Secret value
            
        Returns:
            True if successful, False otherwise
        """
        try:
            # For KV version 2
            try:
                self.client.secrets.kv.v2.create_or_update_secret(
                    path=f"{self.path}/{key}",
                    secret={"value": value},
                    mount_point=self.mount_point
                )
                logger.debug(f"Set secret '{key}' in Vault (KV v2)")
                return True
            except Exception:
                # Try KV version 1
                self.client.secrets.kv.v1.create_or_update_secret(
                    path=f"{self.path}/{key}",
                    secret={"value": value},
                    mount_point=self.mount_point
                )
                logger.debug(f"Set secret '{key}' in Vault (KV v1)")
                return True
        except Exception as e:
            logger.error(f"Failed to set secret '{key}' in Vault: {str(e)}")
            return False
    
    def delete_secret(self, key: str) -> bool:
        """
        Delete a secret from Vault.
        
        Args:
            key: Secret key
            
        Returns:
            True if successful, False otherwise
        """
        try:
            # For KV version 2
            try:
                self.client.secrets.kv.v2.delete_metadata_and_all_versions(
                    path=f"{self.path}/{key}",
                    mount_point=self.mount_point
                )
                logger.debug(f"Deleted secret '{key}' from Vault (KV v2)")
                return True
            except Exception:
                # Try KV version 1
                self.client.secrets.kv.v1.delete_secret(
                    path=f"{self.path}/{key}",
                    mount_point=self.mount_point
                )
                logger.debug(f"Deleted secret '{key}' from Vault (KV v1)")
                return True
        except Exception as e:
            logger.error(f"Failed to delete secret '{key}' from Vault: {str(e)}")
            return False
    
    def list_secrets(self, prefix: Optional[str] = None) -> List[str]:
        """
        List available secrets in Vault.
        
        Args:
            prefix: Optional prefix to filter secrets
            
        Returns:
            List of secret keys
        """
        try:
            path = f"{self.path}"
            if prefix:
                path = f"{path}/{prefix}"
            
            # For KV version 2
            try:
                response = self.client.secrets.kv.v2.list_secrets(
                    path=path,
                    mount_point=self.mount_point
                )
            except Exception:
                # Try KV version 1
                response = self.client.secrets.kv.v1.list_secrets(
                    path=path,
                    mount_point=self.mount_point
                )
            
            return response.get("data", {}).get("keys", [])
        except Exception as e:
            logger.error(f"Failed to list secrets in Vault: {str(e)}")
            return []


class AWSSecretsManagerBackend(SecretsBackend):
    """Secrets backend that uses AWS Secrets Manager."""
    
    def __init__(
        self,
        region_name: Optional[str] = None,
        aws_access_key_id: Optional[str] = None,
        aws_secret_access_key: Optional[str] = None,
        prefix: str = ""
    ):
        """
        Initialize the AWS Secrets Manager backend.
        
        Args:
            region_name: AWS region name
            aws_access_key_id: AWS access key ID
            aws_secret_access_key: AWS secret access key
            prefix: Prefix for secret names
        """
        if not BOTO3_AVAILABLE:
            raise ImportError("boto3 package is required for AWSSecretsManagerBackend")
        
        self.prefix = prefix
        
        # Initialize AWS Secrets Manager client
        self.client = boto3.client(
            "secretsmanager",
            region_name=region_name,
            aws_access_key_id=aws_access_key_id,
            aws_secret_access_key=aws_secret_access_key
        )
        
        logger.info(f"Initialized AWS [REDACTED])
    
    def get_secret(self, key: str) -> Optional[str]:
        """
        Get a secret from AWS Secrets Manager.
        
        Args:
            key: Secret key
            
        Returns:
            Secret value or None if not found
        """
        try:
            secret_name = f"{self.prefix}{key}"
            response = self.client.get_secret_value(SecretId=secret_name)
            
            # Parse secret value
            if "SecretString" in response:
                secret = response["SecretString"]
                try:
                    # Try to parse as JSON
                    secret_dict = json.loads(secret)
                    return secret_dict.get("value", secret)
                except json.JSONDecodeError:
                    # Return as is
                    return secret
            elif "SecretBinary" in response:
                # Binary secret
                return response["SecretBinary"]
            
            return None
        except Exception as e:
            logger.debug(f"Failed to get secret '{key}' from AWS Secrets Manager: {str(e)}")
            return None
    
    def set_secret(self, key: str, value: str) -> bool:
        """
        Set a secret in AWS Secrets Manager.
        
        Args:
            key: Secret key
            value: Secret value
            
        Returns:
            True if successful, False otherwise
        """
        try:
            secret_name = f"{self.prefix}{key}"
            
            # Check if secret exists
            try:
                self.client.describe_secret(SecretId=secret_name)
                # Secret exists, update it
                self.client.update_secret(
                    SecretId=secret_name,
                    SecretString=json.dumps({"value": value})
                )
            except self.client.exceptions.ResourceNotFoundException:
                # Secret doesn't exist, create it
                self.client.create_secret(
                    Name=secret_name,
                    SecretString=json.dumps({"value": value})
                )
            
            logger.debug(f"Set secret '{key}' in AWS Secrets Manager")
            return True
        except Exception as e:
            logger.error(f"Failed to set secret '{key}' in AWS Secrets Manager: {str(e)}")
            return False
    
    def delete_secret(self, key: str) -> bool:
        """
        Delete a secret from AWS Secrets Manager.
        
        Args:
            key: Secret key
            
        Returns:
            True if successful, False otherwise
        """
        try:
            secret_name = f"{self.prefix}{key}"
            self.client.delete_secret(
                SecretId=secret_name,
                ForceDeleteWithoutRecovery=True
            )
            logger.debug(f"Deleted secret '{key}' from AWS Secrets Manager")
            return True
        except Exception as e:
            logger.error(f"Failed to delete secret '{key}' from AWS Secrets Manager: {str(e)}")
            return False
    
    def list_secrets(self, prefix: Optional[str] = None) -> List[str]:
        """
        List available secrets in AWS Secrets Manager.
        
        Args:
            prefix: Optional prefix to filter secrets
            
        Returns:
            List of secret keys
        """
        try:
            full_prefix = f"{self.prefix}{prefix or ''}"
            secrets = []
            
            # AWS Secrets Manager doesn't have a native filtering mechanism,
            # so we need to list all secrets and filter them
            paginator = self.client.get_paginator("list_secrets")
            for page in paginator.paginate():
                for secret in page["SecretList"]:
                    name = secret["Name"]
                    if name.startswith(full_prefix):
                        # Remove the prefix to get the actual key
                        secret_key = name[len(self.prefix):]
                        secrets.append(secret_key)
            
            return secrets
        except Exception as e:
            logger.error(f"Failed to list secrets in AWS Secrets Manager: {str(e)}")
            return []


class InMemorySecretsBackend(SecretsBackend):
    """In-memory secrets backend for testing and development."""
    
    def __init__(self):
        """Initialize the in-memory secrets backend."""
        self.secrets = {}
        self._lock = threading.RLock()
        logger.info("Initialized in-memory secrets backend")
    
    def get_secret(self, key: str) -> Optional[str]:
        """
        Get a secret from memory.
        
        Args:
            key: Secret key
            
        Returns:
            Secret value or None if not found
        """
        with self._lock:
            return self.secrets.get(key)
    
    def set_secret(self, key: str, value: str) -> bool:
        """
        Set a secret in memory.
        
        Args:
            key: Secret key
            value: Secret value
            
        Returns:
            True if successful, False otherwise
        """
        with self._lock:
            self.secrets[key] = value
            return True
    
    def delete_secret(self, key: str) -> bool:
        """
        Delete a secret from memory.
        
        Args:
            key: Secret key
            
        Returns:
            True if successful, False otherwise
        """
        with self._lock:
            if key in self.secrets:
                del self.secrets[key]
                return True
            return False
    
    def list_secrets(self, prefix: Optional[str] = None) -> List[str]:
        """
        List available secrets in memory.
        
        Args:
            prefix: Optional prefix to filter secrets
            
        Returns:
            List of secret keys
        """
        with self._lock:
            if prefix:
                return [k for k in self.secrets.keys() if k.startswith(prefix)]
            return list(self.secrets.keys())


class SecretsManager:
    """
    Centralized secrets manager with multiple backends.
    
    This class provides a unified interface for accessing secrets from
    various backends with fallback support.
    """
    
    def __init__(self, backends: Optional[List[SecretsBackend]] = None):
        """
        Initialize the secrets manager.
        
        Args:
            backends: List of secrets backends in priority order
        """
        self.backends = backends or [EnvironmentSecretsBackend()]
        self._lock = threading.RLock()
        logger.info(f"Initialized secrets manager with {len(self.backends)} backends")
    
    def add_backend(self, backend: SecretsBackend, priority: int = -1):
        """
        Add a secrets backend.
        
        Args:
            backend: Secrets backend to add
            priority: Priority (0 = highest, -1 = lowest)
        """
        with self._lock:
            if priority < 0 or priority >= len(self.backends):
                self.backends.append(backend)
            else:
                self.backends.insert(priority, backend)
            logger.info(f"Added {backend.__class__.__name__} backend with priority {priority}")
    
    def get_secret(self, key: str, default: Optional[str] = None) -> Optional[str]:
        """
        Get a secret from the first available backend.
        
        Args:
            key: Secret key
            default: Default value if secret is not found
            
        Returns:
            Secret value or default if not found
        """
        for backend in self.backends:
            value = backend.get_secret(key)
            if value is not None:
                return value
        
        logger.debug(f"[REDACTED])
        return default
    
    def set_secret(self, key: str, value: str, backend_index: int = 0) -> bool:
        """
        Set a secret in a specific backend.
        
        Args:
            key: Secret key
            value: Secret value
            backend_index: Backend index (0 = highest priority)
            
        Returns:
            True if successful, False otherwise
        """
        with self._lock:
            if backend_index < 0 or backend_index >= len(self.backends):
                logger.error(f"Invalid backend index: {backend_index}")
                return False
            
            return self.backends[backend_index].set_secret(key, value)
    
    def delete_secret(self, key: str, backend_index: Optional[int] = None) -> bool:
        """
        Delete a secret from a specific backend or all backends.
        
        Args:
            key: Secret key
            backend_index: Backend index (None = all backends)
            
        Returns:
            True if successful, False otherwise
        """
        with self._lock:
            if backend_index is not None:
                if backend_index < 0 or backend_index >= len(self.backends):
                    logger.error(f"Invalid backend index: {backend_index}")
                    return False
                
                return self.backends[backend_index].delete_secret(key)
            else:
                # Delete from all backends
                success = False
                for backend in self.backends:
                    if backend.delete_secret(key):
                        success = True
                return success
    
    def list_secrets(self, prefix: Optional[str] = None, backend_index: Optional[int] = None) -> List[str]:
        """
        List available secrets.
        
        Args:
            prefix: Optional prefix to filter secrets
            backend_index: Backend index (None = all backends)
            
        Returns:
            List of secret keys
        """
        with self._lock:
            if backend_index is not None:
                if backend_index < 0 or backend_index >= len(self.backends):
                    logger.error(f"Invalid backend index: {backend_index}")
                    return []
                
                return self.backends[backend_index].list_secrets(prefix)
            else:
                # List from all backends
                secrets = set()
                for backend in self.backends:
                    secrets.update(backend.list_secrets(prefix))
                return list(secrets)


# Create default secrets manager with environment variables backend
secrets_manager = SecretsManager([EnvironmentSecretsBackend()])


def get_secret(key: str, default: Optional[str] = None) -> Optional[str]:
    """
    Get a secret from the secrets manager.
    
    Args:
        key: Secret key
        default: Default value if secret is not found
        
    Returns:
        Secret value or default if not found
    """
    return secrets_manager.get_secret(key, default)


def set_secret(key: str, value: str, backend_index: int = 0) -> bool:
    """
    Set a secret in the secrets manager.
    
    Args:
        key: Secret key
        value: Secret value
        backend_index: Backend index (0 = highest priority)
        
    Returns:
        True if successful, False otherwise
    """
    return secrets_manager.set_secret(key, value, backend_index)


def delete_secret(key: str, backend_index: Optional[int] = None) -> bool:
    """
    Delete a secret from the secrets manager.
    
    Args:
        key: Secret key
        backend_index: Backend index (None = all backends)
        
    Returns:
        True if successful, False otherwise
    """
    return secrets_manager.delete_secret(key, backend_index)


def list_secrets(prefix: Optional[str] = None, backend_index: Optional[int] = None) -> List[str]:
    """
    List available secrets in the secrets manager.
    
    Args:
        prefix: Optional prefix to filter secrets
        backend_index: Backend index (None = all backends)
        
    Returns:
        List of secret keys
    """
    return secrets_manager.list_secrets(prefix, backend_index)


def configure_vault_backend(
    url: str,
    token: Optional[str] = None,
    path: str = "secret",
    mount_point: str = "secret",
    namespace: Optional[str] = None,
    client_cert: Optional[str] = None,
    client_key: Optional[str] = None,
    ca_cert: Optional[str] = None,
    priority: int = 0
):
    """
    Configure a HashiCorp Vault backend.
    
    Args:
        url: Vault server URL
        token: Vault token
        path: Path in Vault
        mount_point: Mount point in Vault
        namespace: Vault namespace
        client_cert: Client certificate path
        client_key: Client key path
        ca_cert: CA certificate path
        priority: Priority (0 = highest)
    """
    if not HVAC_AVAILABLE:
        logger.error("hvac package is required for Vault backend")
        return
    
    try:
        backend = VaultSecretsBackend(
            url=url,
            token=token,
            path=path,
            mount_point=mount_point,
            namespace=namespace,
            client_cert=client_cert,
            client_key=client_key,
            ca_cert=ca_cert
        )
        secrets_manager.add_backend(backend, priority)
        logger.info(f"Configured Vault backend at {url}")
    except Exception as e:
        logger.error(f"Failed to configure Vault backend: {str(e)}")


def configure_aws_secrets_manager_backend(
    region_name: Optional[str] = None,
    aws_access_key_id: Optional[str] = None,
    aws_secret_access_key: Optional[str] = None,
    prefix: str = "",
    priority: int = 0
):
    """
    Configure an AWS Secrets Manager backend.
    
    Args:
        region_name: AWS region name
        aws_access_key_id: AWS access key ID
        aws_secret_access_key: AWS secret access key
        prefix: Prefix for secret names
        priority: Priority (0 = highest)
    """
    if not BOTO3_AVAILABLE:
        logger.error("boto3 package is required for AWS Secrets Manager backend")
        return
    
    try:
        backend = AWSSecretsManagerBackend(
            region_name=region_name,
            aws_access_key_id=aws_access_key_id,
            aws_secret_access_key=aws_secret_access_key,
            prefix=prefix
        )
        secrets_manager.add_backend(backend, priority)
        logger.info(f"Configured AWS Secrets Manager backend in region {region_name or 'default'}")
    except Exception as e:
        logger.error(f"Failed to configure AWS Secrets Manager backend: {str(e)}")
