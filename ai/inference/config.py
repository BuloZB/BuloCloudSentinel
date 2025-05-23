"""
Secure configuration management for the inference engine.

This module provides secure configuration management for the inference engine,
including loading configuration from files, environment variables, and secrets.
"""

import os
import sys
import json
import logging
import secrets
from typing import Dict, Any, Optional, List, Union
from pathlib import Path

# Setup logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class ConfigManager:
    """
    Secure configuration manager.
    
    This class provides secure configuration management for the inference engine.
    """
    
    def __init__(
        self,
        config_path: Optional[str] = None,
        secrets_path: Optional[str] = None,
        env_prefix: str = "SENTINEL_"
    ):
        """
        Initialize the configuration manager.
        
        Args:
            config_path: Path to the configuration file
            secrets_path: Path to the secrets file
            env_prefix: Prefix for environment variables
        """
        # Set configuration paths
        self.config_path = config_path or os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            "../../config/inference.json"
        )
        
        self.secrets_path = secrets_path or os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            "../../config/secrets.json"
        )
        
        # Set environment variable prefix
        self.env_prefix = env_prefix
        
        # Initialize configuration
        self.config = {}
        
        # Load configuration
        self._load_config()
    
    def _load_config(self):
        """Load configuration from files and environment variables."""
        try:
            # Load configuration from file
            if os.path.exists(self.config_path):
                with open(self.config_path, "r") as f:
                    self.config = json.load(f)
                
                logger.info(f"Loaded configuration from {self.config_path}")
            else:
                logger.warning(f"Configuration file {self.config_path} not found")
            
            # Load secrets from file
            if os.path.exists(self.secrets_path):
                with open(self.secrets_path, "r") as f:
                    secrets_config = json.load(f)
                
                # Merge secrets into configuration
                self._merge_dict(self.config, secrets_config)
                
                logger.info(f"Loaded [REDACTED])
            else:
                logger.warning(f"[REDACTED])
            
            # Load configuration from environment variables
            self._load_env_vars()
            
            # Generate missing secrets
            self._generate_missing_secrets()
            
            # Save configuration
            self._save_config()
        except Exception as e:
            logger.error(f"Error loading configuration: {str(e)}")
    
    def _load_env_vars(self):
        """Load configuration from environment variables."""
        try:
            # Get all environment variables with the prefix
            env_vars = {
                k[len(self.env_prefix):].lower(): v
                for k, v in os.environ.items()
                if k.startswith(self.env_prefix)
            }
            
            # Convert environment variables to nested dictionary
            env_config = {}
            for key, value in env_vars.items():
                # Split key by underscore
                parts = key.split("_")
                
                # Create nested dictionary
                current = env_config
                for part in parts[:-1]:
                    if part not in current:
                        current[part] = {}
                    current = current[part]
                
                # Set value
                current[parts[-1]] = self._parse_value(value)
            
            # Merge environment variables into configuration
            self._merge_dict(self.config, env_config)
            
            logger.info(f"Loaded configuration from environment variables")
        except Exception as e:
            logger.error(f"Error loading environment variables: {str(e)}")
    
    def _parse_value(self, value: str) -> Any:
        """
        Parse a string value to the appropriate type.
        
        Args:
            value: String value
            
        Returns:
            Parsed value
        """
        # Try to parse as JSON
        try:
            return json.loads(value)
        except json.JSONDecodeError:
            # Return as string
            return value
    
    def _merge_dict(self, target: Dict[str, Any], source: Dict[str, Any]):
        """
        Merge a source dictionary into a target dictionary.
        
        Args:
            target: Target dictionary
            source: Source dictionary
        """
        for key, value in source.items():
            if key in target and isinstance(target[key], dict) and isinstance(value, dict):
                # Recursively merge dictionaries
                self._merge_dict(target[key], value)
            else:
                # Set value
                target[key] = value
    
    def _generate_missing_secrets(self):
        """Generate missing secrets."""
        try:
            # Check if JWT secret key exists
            if not self.get("auth.jwt_secret"):
                # Generate JWT secret key
                self.set("auth.jwt_secret", secrets.token_hex(32))
                logger.info("Generated JWT secret key")
            
            # Check if API key exists
            if not self.get("auth.api_key"):
                # Generate API key
                self.set("auth.api_key", secrets.token_hex(32))
                logger.info("Generated API key")
            
            # Check if cookie secret exists
            if not self.get("auth.cookie_secret"):
                # Generate cookie secret
                self.set("auth.cookie_secret", secrets.token_hex(32))
                logger.info("Generated cookie secret")
        except Exception as e:
            logger.error(f"Error generating missing secrets: {str(e)}")
    
    def _save_config(self):
        """Save configuration to files."""
        try:
            # Create configuration directory if it doesn't exist
            os.makedirs(os.path.dirname(self.config_path), exist_ok=True)
            
            # Split configuration into public and secret parts
            public_config = {}
            secret_config = {}
            
            # Extract secret configuration
            self._extract_secrets(self.config, public_config, secret_config)
            
            # Save public configuration
            with open(self.config_path, "w") as f:
                json.dump(public_config, f, indent=2)
            
            # Save secret configuration
            with open(self.secrets_path, "w") as f:
                json.dump(secret_config, f, indent=2)
            
            # Set secure permissions for secrets file
            os.chmod(self.secrets_path, 0o600)
            
            logger.info(f"Saved configuration to {self.config_path} and {self.[REDACTED])
        except Exception as e:
            logger.error(f"Error saving configuration: {str(e)}")
    
    def _extract_secrets(
        self,
        config: Dict[str, Any],
        public_config: Dict[str, Any],
        secret_config: Dict[str, Any],
        path: str = ""
    ):
        """
        Extract secret configuration from a configuration dictionary.
        
        Args:
            config: Configuration dictionary
            public_config: Public configuration dictionary
            secret_config: Secret configuration dictionary
            path: Current path
        """
        for key, value in config.items():
            # Create current path
            current_path = f"{path}.{key}" if path else key
            
            if isinstance(value, dict):
                # Create dictionaries if they don't exist
                if key not in public_config:
                    public_config[key] = {}
                if key not in secret_config:
                    secret_config[key] = {}
                
                # Recursively extract secrets
                self._extract_secrets(value, public_config[key], secret_config[key], current_path)
            elif self._is_secret(current_path, value):
                # Add to secret configuration
                secret_config[key] = value
                
                # Add placeholder to public configuration
                public_config[key] = "<secret>"
            else:
                # Add to public configuration
                public_config[key] = value
    
    def _is_secret(self, path: str, value: Any) -> bool:
        """
        Check if a configuration value is a secret.
        
        Args:
            path: Configuration path
            value: Configuration value
            
        Returns:
            True if the value is a secret, False otherwise
        """
        # Check if path contains secret keywords
        secret_keywords = [
            "secret", "password", "token", "key", "cert", "private", "credential"
        ]
        
        return any(keyword in path.lower() for keyword in secret_keywords)
    
    def get(self, path: str, default: Any = None) -> Any:
        """
        Get a configuration value.
        
        Args:
            path: Configuration path (e.g., "auth.jwt_secret")
            default: Default value if the path doesn't exist
            
        Returns:
            Configuration value
        """
        try:
            # Split path by dot
            parts = path.split(".")
            
            # Get value
            value = self.config
            for part in parts:
                if part not in value:
                    return default
                value = value[part]
            
            return value
        except Exception as e:
            logger.error(f"Error getting configuration value for {path}: {str(e)}")
            return default
    
    def set(self, path: str, value: Any):
        """
        Set a configuration value.
        
        Args:
            path: Configuration path (e.g., "auth.jwt_secret")
            value: Configuration value
        """
        try:
            # Split path by dot
            parts = path.split(".")
            
            # Get parent dictionary
            parent = self.config
            for part in parts[:-1]:
                if part not in parent:
                    parent[part] = {}
                parent = parent[part]
            
            # Set value
            parent[parts[-1]] = value
            
            # Save configuration
            self._save_config()
        except Exception as e:
            logger.error(f"Error setting configuration value for {path}: {str(e)}")
    
    def get_all(self) -> Dict[str, Any]:
        """
        Get all configuration values.
        
        Returns:
            Dictionary with all configuration values
        """
        return self.config.copy()


# Create global configuration manager
config_manager = ConfigManager()
