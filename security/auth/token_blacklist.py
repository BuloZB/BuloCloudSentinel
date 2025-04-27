"""
Token blacklist for Bulo.Cloud Sentinel Security Module.

This module provides functionality to blacklist JWT tokens for logout
and token revocation.
"""

import time
from typing import Dict, Optional, Set, Union
import threading
import logging

# Configure logging
logger = logging.getLogger(__name__)

class TokenBlacklist:
    """
    In-memory token blacklist with periodic cleanup.
    
    This class provides a thread-safe token blacklist for JWT tokens.
    It automatically removes expired tokens from the blacklist.
    """
    
    def __init__(self, cleanup_interval: int = 3600):
        """
        Initialize the token blacklist.
        
        Args:
            cleanup_interval: Interval in seconds for cleanup of expired tokens
        """
        self._blacklist: Dict[str, float] = {}  # jti -> expiration time
        self._lock = threading.RLock()
        self._cleanup_interval = cleanup_interval
        self._last_cleanup = time.time()
        
        logger.info(f"Initialized token blacklist with cleanup interval {cleanup_interval}s")
    
    def add(self, jti: str, expires_at: float):
        """
        Add a token to the blacklist.
        
        Args:
            jti: JWT token ID
            expires_at: Expiration timestamp
        """
        with self._lock:
            self._blacklist[jti] = expires_at
            logger.debug(f"Added token {jti} to blacklist, expires at {expires_at}")
            
            # Perform cleanup if needed
            self._maybe_cleanup()
    
    def is_blacklisted(self, jti: str) -> bool:
        """
        Check if a token is blacklisted.
        
        Args:
            jti: JWT token ID
            
        Returns:
            True if token is blacklisted, False otherwise
        """
        with self._lock:
            # Perform cleanup if needed
            self._maybe_cleanup()
            
            # Check if token is in blacklist
            return jti in self._blacklist
    
    def remove(self, jti: str) -> bool:
        """
        Remove a token from the blacklist.
        
        Args:
            jti: JWT token ID
            
        Returns:
            True if token was removed, False if it wasn't in the blacklist
        """
        with self._lock:
            if jti in self._blacklist:
                del self._blacklist[jti]
                logger.debug(f"Removed token {jti} from blacklist")
                return True
            return False
    
    def clear(self):
        """Clear the entire blacklist."""
        with self._lock:
            self._blacklist.clear()
            logger.info("Cleared token blacklist")
    
    def _cleanup(self):
        """Remove expired tokens from the blacklist."""
        now = time.time()
        expired_tokens = [jti for jti, exp in self._blacklist.items() if exp < now]
        
        for jti in expired_tokens:
            del self._blacklist[jti]
        
        if expired_tokens:
            logger.debug(f"Cleaned up {len(expired_tokens)} expired tokens from blacklist")
        
        self._last_cleanup = now
    
    def _maybe_cleanup(self):
        """Perform cleanup if the cleanup interval has passed."""
        now = time.time()
        if now - self._last_cleanup > self._cleanup_interval:
            self._cleanup()
    
    @property
    def size(self) -> int:
        """Get the current size of the blacklist."""
        with self._lock:
            return len(self._blacklist)


# Global token blacklist instance
token_blacklist = TokenBlacklist()


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
