"""
In-memory token blacklist for Bulo.Cloud Sentinel Security Module.

This module provides a fallback in-memory implementation of the token blacklist
when Redis is not available.
"""

import time
from typing import Dict, Optional, Set, Union, List
import threading
import logging

# Configure logging
logger = logging.getLogger(__name__)

class MemoryTokenBlacklist:
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
        self._user_tokens: Dict[str, List[Dict[str, Union[str, float]]]] = {}  # user_id -> list of {jti, exp}
        self._lock = threading.RLock()
        self._cleanup_interval = cleanup_interval
        self._last_cleanup = time.time()
        
        logger.info(f"Initialized in-memory token blacklist with cleanup interval {cleanup_interval}s")
    
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
            self._user_tokens.clear()
            logger.info("Cleared token blacklist")
    
    def _cleanup(self):
        """Remove expired tokens from the blacklist."""
        now = time.time()
        expired_tokens = [jti for jti, exp in self._blacklist.items() if exp < now]
        
        for jti in expired_tokens:
            del self._blacklist[jti]
        
        if expired_tokens:
            logger.debug(f"Cleaned up {len(expired_tokens)} expired tokens from blacklist")
        
        # Also clean up expired tokens from user_tokens
        for user_id in list(self._user_tokens.keys()):
            self._user_tokens[user_id] = [
                token for token in self._user_tokens[user_id]
                if token.get("exp", 0) > now
            ]
            
            # Remove empty user entries
            if not self._user_tokens[user_id]:
                del self._user_tokens[user_id]
        
        self._last_cleanup = now
    
    def _maybe_cleanup(self):
        """Perform cleanup if the cleanup interval has passed."""
        now = time.time()
        if now - self._last_cleanup > self._cleanup_interval:
            self._cleanup()
    
    def add_user_tokens(self, user_id: str, jti: str, expires_at: float):
        """
        Add a token to a user's token list for tracking all user tokens.
        
        Args:
            user_id: User ID
            jti: JWT token ID
            expires_at: Expiration timestamp
        """
        with self._lock:
            if user_id not in self._user_tokens:
                self._user_tokens[user_id] = []
            
            self._user_tokens[user_id].append({
                "jti": jti,
                "exp": expires_at
            })
            
            logger.debug(f"Added token {jti} to user {user_id}'s token list")
    
    def blacklist_all_user_tokens(self, user_id: str) -> int:
        """
        Blacklist all tokens for a user.
        
        Args:
            user_id: User ID
            
        Returns:
            Number of tokens blacklisted
        """
        with self._lock:
            if user_id not in self._user_tokens:
                return 0
            
            count = 0
            for token in self._user_tokens[user_id]:
                jti = token.get("jti")
                exp = token.get("exp")
                
                if jti and exp and jti not in self._blacklist:
                    self._blacklist[jti] = exp
                    count += 1
            
            # Clear the user's tokens
            self._user_tokens[user_id] = []
            
            logger.info(f"Blacklisted {count} tokens for user {user_id}")
            return count
    
    @property
    def size(self) -> int:
        """Get the current size of the blacklist."""
        with self._lock:
            return len(self._blacklist)