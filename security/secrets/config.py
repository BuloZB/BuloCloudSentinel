"""
Secrets configuration for Bulo.Cloud Sentinel.

This module provides functions for configuring the secrets manager
based on environment variables or configuration files.
"""

import os
import logging
import json
from typing import Dict, Optional, Any

from .secrets_manager import (
    secrets_manager,
    configure_vault_backend,
    configure_aws_secrets_manager_backend,
    EnvironmentSecretsBackend,
    InMemorySecretsBackend
)

# Configure logging
logger = logging.getLogger(__name__)


def configure_from_env():
    """
    Configure secrets manager from environment variables.
    
    This function checks for the following environment variables:
    - SECRETS_BACKEND: Comma-separated list of backends to use (env, vault, aws, memory)
    - VAULT_URL: Vault server URL
    - VAULT_TOKEN: Vault token
    - VAULT_PATH: Path in Vault (default: secret)
    - VAULT_MOUNT_POINT: Mount point in Vault (default: secret)
    - VAULT_NAMESPACE: Vault namespace
    - VAULT_CLIENT_CERT: Client certificate path
    - VAULT_CLIENT_KEY: Client key path
    - VAULT_CA_CERT: CA certificate path
    - AWS_REGION: AWS region name
    - AWS_ACCESS_KEY_ID: AWS access key ID
    - AWS_SECRET_ACCESS_KEY: AWS secret access key
    - AWS_SECRETS_PREFIX: Prefix for AWS Secrets Manager
    """
    # Get backends to configure
    backends = os.environ.get("SECRETS_BACKEND", "env").lower().split(",")
    
    # Clear existing backends
    secrets_manager.backends = []
    
    # Configure backends
    for i, backend in enumerate(backends):
        backend = backend.strip()
        
        if backend == "env":
            # Environment variables backend
            prefix = os.environ.get("ENV_SECRETS_PREFIX", "")
            secrets_manager.add_backend(EnvironmentSecretsBackend(prefix=prefix))
            logger.info(f"Configured environment variables backend with prefix '{prefix}'")
        
        elif backend == "vault":
            # Vault backend
            vault_url = os.environ.get("VAULT_URL")
            if not vault_url:
                logger.error("VAULT_URL environment variable is required for Vault backend")
                continue
            
            vault_token = os.environ.get("VAULT_TOKEN")
            vault_path = os.environ.get("VAULT_PATH", "secret")
            vault_mount_point = os.environ.get("VAULT_MOUNT_POINT", "secret")
            vault_namespace = os.environ.get("VAULT_NAMESPACE")
            vault_client_cert = os.environ.get("VAULT_CLIENT_CERT")
            vault_client_key = os.environ.get("VAULT_CLIENT_KEY")
            vault_ca_cert = os.environ.get("VAULT_CA_CERT")
            
            configure_vault_backend(
                url=vault_url,
                token=vault_token,
                path=vault_path,
                mount_point=vault_mount_point,
                namespace=vault_namespace,
                client_cert=vault_client_cert,
                client_key=vault_client_key,
                ca_cert=vault_ca_cert
            )
        
        elif backend == "aws":
            # AWS Secrets Manager backend
            aws_region = os.environ.get("AWS_REGION")
            aws_access_key_id = os.environ.get("AWS_ACCESS_KEY_ID")
            aws_secret_access_key = os.environ.get("AWS_SECRET_ACCESS_KEY")
            aws_secrets_prefix = os.environ.get("AWS_SECRETS_PREFIX", "")
            
            configure_aws_secrets_manager_backend(
                region_name=aws_region,
                aws_access_key_id=aws_access_key_id,
                aws_secret_access_key=aws_secret_access_key,
                prefix=aws_secrets_prefix
            )
        
        elif backend == "memory":
            # In-memory backend (for testing)
            secrets_manager.add_backend(InMemorySecretsBackend())
            logger.info("Configured in-memory backend")
        
        else:
            logger.warning(f"Unknown secrets backend: {backend}")


def configure_from_file(config_file: str):
    """
    Configure secrets manager from a JSON configuration file.
    
    Args:
        config_file: Path to configuration file
    """
    try:
        with open(config_file, "r") as f:
            config = json.load(f)
        
        # Clear existing backends
        secrets_manager.backends = []
        
        # Configure backends
        backends = config.get("backends", [])
        for backend_config in backends:
            backend_type = backend_config.get("type")
            
            if backend_type == "env":
                # Environment variables backend
                prefix = backend_config.get("prefix", "")
                secrets_manager.add_backend(EnvironmentSecretsBackend(prefix=prefix))
                logger.info(f"Configured environment variables backend with prefix '{prefix}'")
            
            elif backend_type == "vault":
                # Vault backend
                vault_url = backend_config.get("url")
                if not vault_url:
                    logger.error("url is required for Vault backend")
                    continue
                
                configure_vault_backend(
                    url=vault_url,
                    token=backend_config.get("token"),
                    path=backend_config.get("path", "secret"),
                    mount_point=backend_config.get("mount_point", "secret"),
                    namespace=backend_config.get("namespace"),
                    client_cert=backend_config.get("client_cert"),
                    client_key=backend_config.get("client_key"),
                    ca_cert=backend_config.get("ca_cert")
                )
            
            elif backend_type == "aws":
                # AWS Secrets Manager backend
                configure_aws_secrets_manager_backend(
                    region_name=backend_config.get("region"),
                    aws_access_key_id=backend_config.get("access_key_id"),
                    aws_secret_access_key=backend_config.get("secret_access_key"),
                    prefix=backend_config.get("prefix", "")
                )
            
            elif backend_type == "memory":
                # In-memory backend (for testing)
                secrets_manager.add_backend(InMemorySecretsBackend())
                logger.info("Configured in-memory backend")
            
            else:
                logger.warning(f"Unknown secrets backend type: {backend_type}")
        
        logger.info(f"Configured [REDACTED])
    
    except Exception as e:
        logger.error(f"Failed to configure secrets manager from file: {str(e)}")


def initialize_secrets():
    """
    Initialize secrets manager based on environment variables or configuration file.
    
    This function checks for the SECRETS_CONFIG environment variable.
    If set, it loads the configuration from the specified file.
    Otherwise, it configures the secrets manager from environment variables.
    """
    config_file = os.environ.get("SECRETS_CONFIG")
    
    if config_file:
        configure_from_file(config_file)
    else:
        configure_from_env()
    
    logger.info(f"Initialized secrets manager with {len(secrets_manager.backends)} backends")
