"""
Authentication and authorization module for the inference engine.

This module provides authentication and authorization utilities for the inference engine,
including JWT token generation and validation, role-based access control, and user management.
"""

import os
import time
import json
import logging
import secrets
import hashlib
import base64
from typing import Dict, Any, Optional, List, Tuple, Union
from datetime import datetime, timedelta

import jwt
from passlib.hash import pbkdf2_sha256

# Setup logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Default roles and permissions
DEFAULT_ROLES = {
    "admin": {
        "description": "Administrator with full access",
        "permissions": [
            "model:read", "model:write", "model:delete",
            "inference:run", "inference:manage",
            "user:read", "user:write", "user:delete",
            "system:read", "system:write"
        ]
    },
    "user": {
        "description": "Regular user with limited access",
        "permissions": [
            "model:read",
            "inference:run"
        ]
    },
    "inference": {
        "description": "Inference-only role for worker nodes",
        "permissions": [
            "inference:run"
        ]
    }
}


class AuthManager:
    """
    Authentication and authorization manager.
    
    This class provides authentication and authorization utilities for the inference engine.
    """
    
    def __init__(
        self,
        secret_key: Optional[str] = None,
        token_expiration: int = 3600,
        refresh_token_expiration: int = 86400 * 7,
        user_db_path: Optional[str] = None
    ):
        """
        Initialize the authentication manager.
        
        Args:
            secret_key: Secret key for JWT token generation and validation
            token_expiration: Token expiration time in seconds (default: 1 hour)
            refresh_token_expiration: Refresh token expiration time in seconds (default: 7 days)
            user_db_path: Path to the user database file
        """
        # Generate secret key if not provided
        self.secret_key = secret_key or secrets.token_hex(32)
        
        # Set token expiration times
        self.token_expiration = token_expiration
        self.refresh_token_expiration = refresh_token_expiration
        
        # Set user database path
        self.user_db_path = user_db_path or os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            "../../config/users.json"
        )
        
        # Initialize user database
        self.users = self._load_users()
        
        # Initialize roles
        self.roles = DEFAULT_ROLES.copy()
    
    def _load_users(self) -> Dict[str, Any]:
        """
        Load users from the database file.
        
        Returns:
            Dictionary of users
        """
        try:
            # Check if user database file exists
            if os.path.exists(self.user_db_path):
                # Load users from file
                with open(self.user_db_path, "r") as f:
                    users = json.load(f)
                
                logger.info(f"Loaded {len(users)} users from {self.user_db_path}")
                return users
            else:
                # Create default admin user
                users = {
                    "admin": {
                        "username": "admin",
                        "password_hash": pbkdf2_sha256.hash("placeholderpassword"),
                        "role": "admin",
                        "created_at": datetime.now().isoformat(),
                        "last_login": None,
                        "enabled": True
                    }
                }
                
                # Create user database directory if it doesn't exist
                os.makedirs(os.path.dirname(self.user_db_path), exist_ok=True)
                
                # Save users to file
                with open(self.user_db_path, "w") as f:
                    json.dump(users, f, indent=2)
                
                logger.info(f"Created default admin user in {self.user_db_path}")
                return users
        except Exception as e:
            logger.error(f"Error loading users: {str(e)}")
            return {}
    
    def _save_users(self) -> bool:
        """
        Save users to the database file.
        
        Returns:
            True if successful, False otherwise
        """
        try:
            # Create user database directory if it doesn't exist
            os.makedirs(os.path.dirname(self.user_db_path), exist_ok=True)
            
            # Save users to file
            with open(self.user_db_path, "w") as f:
                json.dump(self.users, f, indent=2)
            
            logger.info(f"Saved {len(self.users)} users to {self.user_db_path}")
            return True
        except Exception as e:
            logger.error(f"Error saving users: {str(e)}")
            return False
    
    def authenticate(self, username: str, password: str) -> Optional[Dict[str, Any]]:
        """
        Authenticate a user.
        
        Args:
            username: Username
            password: Password
            
        Returns:
            User data if authentication is successful, None otherwise
        """
        try:
            # Check if user exists
            if username not in self.users:
                logger.warning(f"User {username} not found")
                return None
            
            # Get user data
            user = self.users[username]
            
            # Check if user is enabled
            if not user.get("enabled", True):
                logger.warning(f"User {username} is disabled")
                return None
            
            # Verify password
            if not pbkdf2_sha256.verify(password, user["password_hash"]):
                logger.warning(f"Invalid password for user {username}")
                return None
            
            # Update last login time
            user["last_login"] = datetime.now().isoformat()
            self._save_users()
            
            return user
        except Exception as e:
            logger.error(f"Error authenticating user {username}: {str(e)}")
            return None
    
    def create_user(
        self,
        username: str,
        password: str,
        role: str = "user",
        enabled: bool = True
    ) -> bool:
        """
        Create a new user.
        
        Args:
            username: Username
            password: Password
            role: Role (default: user)
            enabled: Whether the user is enabled (default: True)
            
        Returns:
            True if successful, False otherwise
        """
        try:
            # Check if user already exists
            if username in self.users:
                logger.warning(f"User {username} already exists")
                return False
            
            # Check if role exists
            if role not in self.roles:
                logger.warning(f"Role {role} not found")
                return False
            
            # Create user
            self.users[username] = {
                "username": username,
                "password_hash": pbkdf2_sha256.hash(password),
                "role": role,
                "created_at": datetime.now().isoformat(),
                "last_login": None,
                "enabled": enabled
            }
            
            # Save users
            return self._save_users()
        except Exception as e:
            logger.error(f"Error creating user {username}: {str(e)}")
            return False
    
    def update_user(
        self,
        username: str,
        password: Optional[str] = None,
        role: Optional[str] = None,
        enabled: Optional[bool] = None
    ) -> bool:
        """
        Update a user.
        
        Args:
            username: Username
            password: New password (optional)
            role: New role (optional)
            enabled: Whether the user is enabled (optional)
            
        Returns:
            True if successful, False otherwise
        """
        try:
            # Check if user exists
            if username not in self.users:
                logger.warning(f"User {username} not found")
                return False
            
            # Get user data
            user = self.users[username]
            
            # Update password if provided
            if password:
                user["password_hash"] = pbkdf2_sha256.hash(password)
            
            # Update role if provided
            if role:
                # Check if role exists
                if role not in self.roles:
                    logger.warning(f"Role {role} not found")
                    return False
                
                user["role"] = role
            
            # Update enabled status if provided
            if enabled is not None:
                user["enabled"] = enabled
            
            # Save users
            return self._save_users()
        except Exception as e:
            logger.error(f"Error updating user {username}: {str(e)}")
            return False
    
    def delete_user(self, username: str) -> bool:
        """
        Delete a user.
        
        Args:
            username: Username
            
        Returns:
            True if successful, False otherwise
        """
        try:
            # Check if user exists
            if username not in self.users:
                logger.warning(f"User {username} not found")
                return False
            
            # Delete user
            del self.users[username]
            
            # Save users
            return self._save_users()
        except Exception as e:
            logger.error(f"Error deleting user {username}: {str(e)}")
            return False
    
    def get_user(self, username: str) -> Optional[Dict[str, Any]]:
        """
        Get user data.
        
        Args:
            username: Username
            
        Returns:
            User data if found, None otherwise
        """
        try:
            # Check if user exists
            if username not in self.users:
                logger.warning(f"User {username} not found")
                return None
            
            # Get user data
            user = self.users[username].copy()
            
            # Remove sensitive data
            user.pop("password_hash", None)
            
            return user
        except Exception as e:
            logger.error(f"Error getting user {username}: {str(e)}")
            return None
    
    def get_users(self) -> List[Dict[str, Any]]:
        """
        Get all users.
        
        Returns:
            List of user data
        """
        try:
            # Get all users
            users = []
            for username, user_data in self.users.items():
                # Create a copy of user data
                user = user_data.copy()
                
                # Remove sensitive data
                user.pop("password_hash", None)
                
                users.append(user)
            
            return users
        except Exception as e:
            logger.error(f"Error getting users: {str(e)}")
            return []
    
    def create_token(self, username: str) -> Optional[Dict[str, str]]:
        """
        Create JWT tokens for a user.
        
        Args:
            username: Username
            
        Returns:
            Dictionary with access and refresh tokens if successful, None otherwise
        """
        try:
            # Check if user exists
            if username not in self.users:
                logger.warning(f"User {username} not found")
                return None
            
            # Get user data
            user = self.users[username]
            
            # Check if user is enabled
            if not user.get("enabled", True):
                logger.warning(f"User {username} is disabled")
                return None
            
            # Get current time
            now = datetime.utcnow()
            
            # Create access token
            access_token_payload = {
                "sub": username,
                "role": user["role"],
                "iat": now,
                "exp": now + timedelta(seconds=self.token_expiration),
                "type": "access"
            }
            
            access_token = jwt.encode(
                access_token_payload,
                self.secret_key,
                algorithm="HS256"
            )
            
            # Create refresh token
            refresh_token_payload = {
                "sub": username,
                "iat": now,
                "exp": now + timedelta(seconds=self.refresh_token_expiration),
                "type": "refresh"
            }
            
            refresh_token = jwt.encode(
                refresh_token_payload,
                self.secret_key,
                algorithm="HS256"
            )
            
            return {
                "access_token": access_token,
                "refresh_token": refresh_token,
                "token_type": "bearer",
                "expires_in": self.token_expiration
            }
        except Exception as e:
            logger.error(f"Error creating token for user {username}: {str(e)}")
            return None
    
    def refresh_token(self, refresh_token: str) -> Optional[Dict[str, str]]:
        """
        Refresh an access token.
        
        Args:
            refresh_token: Refresh token
            
        Returns:
            Dictionary with new access token if successful, None otherwise
        """
        try:
            # Decode refresh token
            try:
                payload = jwt.decode(
                    refresh_token,
                    self.secret_key,
                    algorithms=["HS256"]
                )
            except jwt.ExpiredSignatureError:
                logger.warning("Refresh token has expired")
                return None
            except jwt.InvalidTokenError:
                logger.warning("Invalid refresh token")
                return None
            
            # Check token type
            if payload.get("type") != "refresh":
                logger.warning("Token is not a refresh token")
                return None
            
            # Get username
            username = payload.get("sub")
            
            # Check if user exists
            if username not in self.users:
                logger.warning(f"User {username} not found")
                return None
            
            # Get user data
            user = self.users[username]
            
            # Check if user is enabled
            if not user.get("enabled", True):
                logger.warning(f"User {username} is disabled")
                return None
            
            # Get current time
            now = datetime.utcnow()
            
            # Create new access token
            access_token_payload = {
                "sub": username,
                "role": user["role"],
                "iat": now,
                "exp": now + timedelta(seconds=self.token_expiration),
                "type": "access"
            }
            
            access_token = jwt.encode(
                access_token_payload,
                self.secret_key,
                algorithm="HS256"
            )
            
            return {
                "access_token": access_token,
                "token_type": "bearer",
                "expires_in": self.token_expiration
            }
        except Exception as e:
            logger.error(f"Error refreshing token: {str(e)}")
            return None
    
    def validate_token(self, token: str) -> Optional[Dict[str, Any]]:
        """
        Validate a JWT token.
        
        Args:
            token: JWT token
            
        Returns:
            Token payload if valid, None otherwise
        """
        try:
            # Decode token
            try:
                payload = jwt.decode(
                    token,
                    self.secret_key,
                    algorithms=["HS256"]
                )
            except jwt.ExpiredSignatureError:
                logger.warning("Token has expired")
                return None
            except jwt.InvalidTokenError:
                logger.warning("Invalid token")
                return None
            
            # Check token type
            if payload.get("type") != "access":
                logger.warning("Token is not an access token")
                return None
            
            # Get username
            username = payload.get("sub")
            
            # Check if user exists
            if username not in self.users:
                logger.warning(f"User {username} not found")
                return None
            
            # Get user data
            user = self.users[username]
            
            # Check if user is enabled
            if not user.get("enabled", True):
                logger.warning(f"User {username} is disabled")
                return None
            
            # Check if role matches
            if payload.get("role") != user["role"]:
                logger.warning(f"Token role {payload.get('role')} does not match user role {user['role']}")
                return None
            
            return payload
        except Exception as e:
            logger.error(f"Error validating token: {str(e)}")
            return None
    
    def has_permission(self, token_payload: Dict[str, Any], permission: str) -> bool:
        """
        Check if a token has a permission.
        
        Args:
            token_payload: Token payload
            permission: Permission to check
            
        Returns:
            True if the token has the permission, False otherwise
        """
        try:
            # Get role
            role = token_payload.get("role")
            
            # Check if role exists
            if role not in self.roles:
                logger.warning(f"Role {role} not found")
                return False
            
            # Get role permissions
            permissions = self.roles[role].get("permissions", [])
            
            # Check if permission is granted
            return permission in permissions
        except Exception as e:
            logger.error(f"Error checking permission: {str(e)}")
            return False
    
    def create_api_key(self, username: str, description: str = "") -> Optional[str]:
        """
        Create an API key for a user.
        
        Args:
            username: Username
            description: API key description
            
        Returns:
            API key if successful, None otherwise
        """
        try:
            # Check if user exists
            if username not in self.users:
                logger.warning(f"User {username} not found")
                return None
            
            # Get user data
            user = self.users[username]
            
            # Check if user is enabled
            if not user.get("enabled", True):
                logger.warning(f"User {username} is disabled")
                return None
            
            # Generate API key
            api_key = secrets.token_hex(32)
            
            # Create API key hash
            api_key_hash = hashlib.sha256(api_key.encode()).hexdigest()
            
            # Initialize API keys if not exists
            if "api_keys" not in user:
                user["api_keys"] = {}
            
            # Add API key
            user["api_keys"][api_key_hash] = {
                "description": description,
                "created_at": datetime.now().isoformat(),
                "last_used": None
            }
            
            # Save users
            if self._save_users():
                return api_key
            else:
                return None
        except Exception as e:
            logger.error(f"Error creating API key for user {username}: {str(e)}")
            return None
    
    def validate_api_key(self, api_key: str) -> Optional[Dict[str, Any]]:
        """
        Validate an API key.
        
        Args:
            api_key: API key
            
        Returns:
            User data if valid, None otherwise
        """
        try:
            # Create API key hash
            api_key_hash = hashlib.sha256(api_key.encode()).hexdigest()
            
            # Check all users
            for username, user_data in self.users.items():
                # Check if user is enabled
                if not user_data.get("enabled", True):
                    continue
                
                # Check if user has API keys
                if "api_keys" not in user_data:
                    continue
                
                # Check if API key exists
                if api_key_hash in user_data["api_keys"]:
                    # Update last used time
                    user_data["api_keys"][api_key_hash]["last_used"] = datetime.now().isoformat()
                    self._save_users()
                    
                    # Return user data
                    return {
                        "username": username,
                        "role": user_data["role"]
                    }
            
            logger.warning("Invalid API key")
            return None
        except Exception as e:
            logger.error(f"Error validating API key: {str(e)}")
            return None
    
    def delete_api_key(self, username: str, api_key: str) -> bool:
        """
        Delete an API key.
        
        Args:
            username: Username
            api_key: API key
            
        Returns:
            True if successful, False otherwise
        """
        try:
            # Check if user exists
            if username not in self.users:
                logger.warning(f"User {username} not found")
                return False
            
            # Get user data
            user = self.users[username]
            
            # Check if user has API keys
            if "api_keys" not in user:
                logger.warning(f"User {username} has no API keys")
                return False
            
            # Create API key hash
            api_key_hash = hashlib.sha256(api_key.encode()).hexdigest()
            
            # Check if API key exists
            if api_key_hash not in user["api_keys"]:
                logger.warning(f"API key not found for user {username}")
                return False
            
            # Delete API key
            del user["api_keys"][api_key_hash]
            
            # Save users
            return self._save_users()
        except Exception as e:
            logger.error(f"Error deleting API key for user {username}: {str(e)}")
            return False
    
    def get_api_keys(self, username: str) -> List[Dict[str, Any]]:
        """
        Get API keys for a user.
        
        Args:
            username: Username
            
        Returns:
            List of API key data
        """
        try:
            # Check if user exists
            if username not in self.users:
                logger.warning(f"User {username} not found")
                return []
            
            # Get user data
            user = self.users[username]
            
            # Check if user has API keys
            if "api_keys" not in user:
                return []
            
            # Get API keys
            api_keys = []
            for api_key_hash, api_key_data in user["api_keys"].items():
                # Create a copy of API key data
                api_key = api_key_data.copy()
                
                # Add API key hash
                api_key["hash"] = api_key_hash
                
                api_keys.append(api_key)
            
            return api_keys
        except Exception as e:
            logger.error(f"Error getting API keys for user {username}: {str(e)}")
            return []
