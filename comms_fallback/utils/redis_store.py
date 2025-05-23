"""
Redis integration for state management.

This module provides utilities for storing and retrieving state in Redis.
"""

import asyncio
import json
import logging
from typing import Any, Dict, List, Optional, Set, Tuple, TypeVar, Union, Generic

import redis.asyncio as redis

logger = logging.getLogger(__name__)

T = TypeVar("T")


class RedisStore(Generic[T]):
    """Redis store for state management."""

    def __init__(
        self,
        redis_url: str,
        prefix: str = "comms_fallback:",
        ttl: int = 3600,
    ):
        """
        Initialize the Redis store.

        Args:
            redis_url: Redis connection URL
            prefix: Key prefix
            ttl: Time-to-live in seconds
        """
        self.redis_url = redis_url
        self.prefix = prefix
        self.ttl = ttl
        self.redis = None

    async def connect(self) -> bool:
        """
        Connect to Redis.

        Returns:
            True if connected successfully, False otherwise
        """
        try:
            logger.info(f"Connecting to Redis at {self.redis_url}")
            self.redis = redis.from_url(self.redis_url, decode_responses=True)
            
            # Test connection
            await self.redis.ping()
            
            logger.info("Connected to Redis")
            return True
            
        except Exception as e:
            logger.error(f"Error connecting to Redis: {e}")
            self.redis = None
            return False

    async def disconnect(self):
        """Disconnect from Redis."""
        if self.redis:
            await self.redis.close()
            self.redis = None
            logger.info("Disconnected from Redis")

    async def set(self, key: str, value: Any) -> bool:
        """
        Set a value in Redis.

        Args:
            key: Key
            value: Value

        Returns:
            True if set successfully, False otherwise
        """
        try:
            if not self.redis:
                if not await self.connect():
                    return False
            
            # Convert value to JSON
            if isinstance(value, dict) or isinstance(value, list):
                value_str = json.dumps(value)
            else:
                value_str = str(value)
            
            # Set value with TTL
            full_key = f"{self.prefix}{key}"
            await self.redis.setex(full_key, self.ttl, value_str)
            
            return True
            
        except Exception as e:
            logger.error(f"Error setting value in Redis: {e}")
            return False

    async def get(self, key: str) -> Optional[Any]:
        """
        Get a value from Redis.

        Args:
            key: Key

        Returns:
            Value or None if not found
        """
        try:
            if not self.redis:
                if not await self.connect():
                    return None
            
            # Get value
            full_key = f"{self.prefix}{key}"
            value_str = await self.redis.get(full_key)
            
            if not value_str:
                return None
            
            # Try to parse as JSON
            try:
                return json.loads(value_str)
            except json.JSONDecodeError:
                return value_str
            
        except Exception as e:
            logger.error(f"Error getting value from Redis: {e}")
            return None

    async def delete(self, key: str) -> bool:
        """
        Delete a value from Redis.

        Args:
            key: Key

        Returns:
            True if deleted successfully, False otherwise
        """
        try:
            if not self.redis:
                if not await self.connect():
                    return False
            
            # Delete value
            full_key = f"{self.prefix}{key}"
            await self.redis.delete(full_key)
            
            return True
            
        except Exception as e:
            logger.error(f"Error deleting value from Redis: {e}")
            return False

    async def list_append(self, key: str, value: Any) -> bool:
        """
        Append a value to a list in Redis.

        Args:
            key: Key
            value: Value

        Returns:
            True if appended successfully, False otherwise
        """
        try:
            if not self.redis:
                if not await self.connect():
                    return False
            
            # Convert value to JSON
            if isinstance(value, dict) or isinstance(value, list):
                value_str = json.dumps(value)
            else:
                value_str = str(value)
            
            # Append value to list
            full_key = f"{self.prefix}{key}"
            await self.redis.rpush(full_key, value_str)
            
            # Set TTL
            await self.redis.expire(full_key, self.ttl)
            
            return True
            
        except Exception as e:
            logger.error(f"Error appending value to list in Redis: {e}")
            return False

    async def list_get(self, key: str) -> List[Any]:
        """
        Get a list from Redis.

        Args:
            key: Key

        Returns:
            List of values
        """
        try:
            if not self.redis:
                if not await self.connect():
                    return []
            
            # Get list
            full_key = f"{self.prefix}{key}"
            values = await self.redis.lrange(full_key, 0, -1)
            
            # Parse values as JSON
            result = []
            for value_str in values:
                try:
                    result.append(json.loads(value_str))
                except json.JSONDecodeError:
                    result.append(value_str)
            
            return result
            
        except Exception as e:
            logger.error(f"Error getting list from Redis: {e}")
            return []

    async def hash_set(self, key: str, field: str, value: Any) -> bool:
        """
        Set a field in a hash in Redis.

        Args:
            key: Key
            field: Field
            value: Value

        Returns:
            True if set successfully, False otherwise
        """
        try:
            if not self.redis:
                if not await self.connect():
                    return False
            
            # Convert value to JSON
            if isinstance(value, dict) or isinstance(value, list):
                value_str = json.dumps(value)
            else:
                value_str = str(value)
            
            # Set field in hash
            full_key = f"{self.prefix}{key}"
            await self.redis.hset(full_key, field, value_str)
            
            # Set TTL
            await self.redis.expire(full_key, self.ttl)
            
            return True
            
        except Exception as e:
            logger.error(f"Error setting field in hash in Redis: {e}")
            return False

    async def hash_get(self, key: str, field: str) -> Optional[Any]:
        """
        Get a field from a hash in Redis.

        Args:
            key: Key
            field: Field

        Returns:
            Value or None if not found
        """
        try:
            if not self.redis:
                if not await self.connect():
                    return None
            
            # Get field from hash
            full_key = f"{self.prefix}{key}"
            value_str = await self.redis.hget(full_key, field)
            
            if not value_str:
                return None
            
            # Try to parse as JSON
            try:
                return json.loads(value_str)
            except json.JSONDecodeError:
                return value_str
            
        except Exception as e:
            logger.error(f"Error getting field from hash in Redis: {e}")
            return None

    async def hash_get_all(self, key: str) -> Dict[str, Any]:
        """
        Get all fields from a hash in Redis.

        Args:
            key: Key

        Returns:
            Dictionary of fields to values
        """
        try:
            if not self.redis:
                if not await self.connect():
                    return {}
            
            # Get all fields from hash
            full_key = f"{self.prefix}{key}"
            values = await self.redis.hgetall(full_key)
            
            # Parse values as JSON
            result = {}
            for field, value_str in values.items():
                try:
                    result[field] = json.loads(value_str)
                except json.JSONDecodeError:
                    result[field] = value_str
            
            return result
            
        except Exception as e:
            logger.error(f"Error getting all fields from hash in Redis: {e}")
            return {}
