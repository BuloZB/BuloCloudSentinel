"""
Token blacklist for Bulo.Cloud Sentinel Security Module.

This module provides functionality to blacklist JWT tokens for logout
and token revocation using a persistent Redis storage.
"""

import time
import json
from typing import Dict, Optional, Set, Union, List
import logging
import redis
from redis.exceptions import RedisError

from backend.infrastructure.config.config import settings

# Configure logging
logger = logging.getLogger(__name__)

class RedisTokenBlacklist:
    """
    Redis-backed token blacklist with automatic expiration.
    
    This class provides a persistent token blacklist for JWT tokens
    using Redis as the storage backend. It automatically removes
    expired tokens using Redis's built-in expiration mechanism.
    """
    
    def __init__(self, redis_url: Optional[str] = None):
        """
        Initialize the token blacklist.
        
        Args:
            redis_url: Redis connection URL (optional, uses settings.REDIS_URL if not provided)
        """
        # Use provided Redis URL or get from settings
        self.redis_url = redis_url or settings.REDIS_URL
        
        if not self.redis_url:
            raise ValueError("Redis URL must be provided for token blacklist")
        
        # Initialize Redis connection
        try:
            self.redis = redis.from_url(self.redis_url)
            logger.info("Initialized Redis-backed token blacklist")
        except RedisError as e:
            logger.error(f"Failed to connect to Redis: {str(e)}")
            raise
    
    def add(self, jti: str, expires_at: float):
        """
        Add a token to the blacklist.
        
        Args:
            jti: JWT token ID
            expires_at: Expiration timestamp
        """
        try:
            # Calculate TTL (time to live) in seconds
            now = time.time()
            ttl = max(int(expires_at - now), 1)  # Ensure at least 1 second TTL
            
            # Add to Redis with expiration
            key = f"blacklist:{jti}"
            self.redis.setex(key, ttl, "1")
            logger.debug(f"Added token {jti} to blacklist, expires in {ttl}s")
        except RedisError as e:
            logger.error(f"Failed to add token to blacklist: {str(e)}")
            # Fallback to in-memory storage in case of Redis failure
            in_memory_blacklist[jti] = expires_at
    
    def is_blacklisted(self, jti: str) -> bool:
        """
        Check if a token is blacklisted.
        
        Args:
            jti: JWT token ID
            
        Returns:
            True if token is blacklisted, False otherwise
        """
        try:
            # Check if token is in Redis
            key = f"blacklist:{jti}"
            return bool(self.redis.exists(key))
        except RedisError as e:
            logger.error(f"Failed to check token blacklist: {str(e)}")
            # Fallback to in-memory check in case of Redis failure
            return jti in in_memory_blacklist
    
    def remove(self, jti: str) -> bool:
        """
        Remove a token from the blacklist.
        
        Args:
            jti: JWT token ID
            
        Returns:
            True if token was removed, False if it wasn't in the blacklist
        """
        try:
            # Remove from Redis
            key = f"blacklist:{jti}"
            result = self.redis.delete(key)
            if result > 0:
                logger.debug(f"Removed token {jti} from blacklist")
                return True
            return False
        except RedisError as e:
            logger.error(f"Failed to remove token from blacklist: {str(e)}")
            # Fallback to in-memory removal in case of Redis failure
            if jti in in_memory_blacklist:
                del in_memory_blacklist[jti]
                return True
            return False
    
    def clear(self):
        """Clear the entire blacklist."""
        try:
            # Get all blacklist keys
            keys = self.redis.keys("blacklist:*")
            if keys:
                # Delete all keys
                self.redis.delete(*keys)
                logger.info(f"Cleared token blacklist ({len(keys)} tokens)")
        except RedisError as e:
            logger.error(f"Failed to clear token blacklist: {str(e)}")
            # Fallback to clearing in-memory blacklist
            in_memory_blacklist.clear()
    
    def add_user_tokens(self, user_id: str, jti: str, expires_at: float):
        """
        Add a token to a user's token list for tracking all user tokens.
        
        Args:
            user_id: User ID
            jti: JWT token ID
            expires_at: Expiration timestamp
        """
        try:
            # Add token to user's token set
            user_key = f"user_tokens:{user_id}"
            token_data = json.dumps({"jti": jti, "exp": expires_at})
            self.redis.sadd(user_key, token_data)
            
            # Set expiration on the user's token set if not already set
            if not self.redis.ttl(user_key):
                # Set a long expiration (30 days) for the user's token set
                self.redis.expire(user_key, 30 * 24 * 60 * 60)
            
            logger.debug(f"Added token {jti} to user {user_id}'s token list")
        except RedisError as e:
            logger.error(f"Failed to add token to user's token list: {str(e)}")
    
    def blacklist_all_user_tokens(self, user_id: str):
        """
        Blacklist all tokens for a user.
        
        Args:
            user_id: User ID
        """
        try:
            # Get all tokens for the user
            user_key = f"user_tokens:{user_id}"
            token_data_set = self.redis.smembers(user_key)
            
            count = 0
            for token_data_str in token_data_set:
                try:
                    token_data = json.loads(token_data_str)
                    jti = token_data.get("jti")
                    exp = token_data.get("exp")
                    
                    if jti and exp:
                        # Add token to blacklist
                        self.add(jti, exp)
                        count += 1
                except (json.JSONDecodeError, KeyError) as e:
                    logger.error(f"Invalid token data: {str(e)}")
            
            # Clear the user's token set
            self.redis.delete(user_key)
            
            logger.info(f"Blacklisted {count} tokens for user {user_id}")
            return count
        except RedisError as e:
            logger.error(f"Failed to blacklist all user tokens: {str(e)}")
            return 0
    
    @property
    def size(self) -> int:
        """Get the current size of the blacklist."""
        try:
            # Count all blacklist keys
            return len(self.redis.keys("blacklist:*"))
        except RedisError as e:
            logger.error(f"Failed to get blacklist size: {str(e)}")
            # Fallback to in-memory size
            return len(in_memory_blacklist)


# Fallback in-memory blacklist for Redis failures
in_memory_blacklist: Dict[str, float] = {}

# Global token blacklist instance
try:
    token_blacklist = RedisTokenBlacklist()
except Exception as e:
    logger.error(f"Failed to initialize Redis token blacklist: {str(e)}")
    # Fallback to a simple in-memory implementation
    from security.auth.memory_blacklist import MemoryTokenBlacklist
    token_blacklist = MemoryTokenBlacklist()
    logger.warning("Using in-memory token blacklist as fallback")


def blacklist_token(jti: str, expires_at: float):
    """
    Add a token to the blacklist.
    
    Args:
        jti: JWT token ID
        expires_at: Expiration timestamp
    """
    token_blacklist.add(jti, expires_at)


def is_token_blacklisted(jti: str) -> bool:
    """
    Check if a token is blacklisted.
    
    Args:
        jti: JWT token ID
        
    Returns:
        True if token is blacklisted, False otherwise
    """
    return token_blacklist.is_blacklisted(jti)


def remove_from_blacklist(jti: str) -> bool:
    """
    Remove a token from the blacklist.
    
    Args:
        jti: JWT token ID
        
    Returns:
        True if token was removed, False if it wasn't in the blacklist
    """
    return token_blacklist.remove(jti)


def clear_blacklist():
    """Clear the entire blacklist."""
    token_blacklist.clear()


def get_blacklist_size() -> int:
    """Get the current size of the blacklist."""
    return token_blacklist.size


def blacklist_all_user_tokens(user_id: str) -> int:
    """
    Blacklist all tokens for a user.
    
    Args:
        user_id: User ID
        
    Returns:
        Number of tokens blacklisted
    """
    return token_blacklist.blacklist_all_user_tokens(user_id)


def add_user_token(user_id: str, jti: str, expires_at: float):
    """
    Add a token to a user's token list.
    
    Args:
        user_id: User ID
        jti: JWT token ID
        expires_at: Expiration timestamp
    """
    token_blacklist.add_user_tokens(user_id, jti, expires_at)
