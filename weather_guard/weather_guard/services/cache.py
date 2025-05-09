"""
Redis cache service for Weather Guard.

This module provides a Redis-based cache for weather data with TTL support.
"""

import json
import logging
from datetime import datetime
from typing import Any, Dict, List, Optional, TypeVar, Generic, Type, Union

import redis.asyncio as redis
from pydantic import BaseModel

from weather_guard.core.config import settings
from weather_guard.models.weather import WeatherData, WeatherForecast

logger = logging.getLogger(__name__)

T = TypeVar('T', bound=BaseModel)


class RedisCache(Generic[T]):
    """Redis cache for Pydantic models with TTL support."""
    
    def __init__(
        self,
        model_class: Type[T],
        redis_url: Optional[str] = None,
        prefix: Optional[str] = None,
        ttl: Optional[int] = None,
    ):
        """Initialize the Redis cache.
        
        Args:
            model_class: Pydantic model class to cache
            redis_url: Redis connection URL
            prefix: Key prefix for Redis
            ttl: Time-to-live in seconds
        """
        self.model_class = model_class
        self.redis_url = redis_url or settings.REDIS_URL
        self.prefix = prefix or settings.REDIS_PREFIX
        self.ttl = ttl or settings.REDIS_TTL
        self.redis: Optional[redis.Redis] = None
    
    async def connect(self) -> None:
        """Connect to Redis."""
        if self.redis is None:
            try:
                self.redis = redis.from_url(str(self.redis_url), decode_responses=True)
                # Test connection
                await self.redis.ping()
                logger.info(f"Connected to Redis at {self.redis_url}")
            except Exception as e:
                logger.error(f"Failed to connect to Redis: {str(e)}")
                self.redis = None
                raise
    
    async def close(self) -> None:
        """Close the Redis connection."""
        if self.redis:
            await self.redis.close()
            self.redis = None
    
    async def get(self, key: str) -> Optional[T]:
        """Get a value from the cache.
        
        Args:
            key: Cache key
            
        Returns:
            Cached value or None if not found
        """
        if not self.redis:
            await self.connect()
        
        try:
            full_key = f"{self.prefix}{key}"
            data = await self.redis.get(full_key)
            
            if data:
                # Parse JSON and convert to model
                json_data = json.loads(data)
                return self.model_class.model_validate(json_data)
            
            return None
        except Exception as e:
            logger.error(f"Error getting from cache: {str(e)}")
            return None
    
    async def set(self, key: str, value: T) -> bool:
        """Set a value in the cache.
        
        Args:
            key: Cache key
            value: Value to cache
            
        Returns:
            True if successful, False otherwise
        """
        if not self.redis:
            await self.connect()
        
        try:
            full_key = f"{self.prefix}{key}"
            # Convert model to JSON
            json_data = value.model_dump_json()
            
            # Set with TTL
            result = await self.redis.setex(full_key, self.ttl, json_data)
            return result == "OK"
        except Exception as e:
            logger.error(f"Error setting cache: {str(e)}")
            return False
    
    async def delete(self, key: str) -> bool:
        """Delete a value from the cache.
        
        Args:
            key: Cache key
            
        Returns:
            True if successful, False otherwise
        """
        if not self.redis:
            await self.connect()
        
        try:
            full_key = f"{self.prefix}{key}"
            result = await self.redis.delete(full_key)
            return result > 0
        except Exception as e:
            logger.error(f"Error deleting from cache: {str(e)}")
            return False
    
    async def get_keys(self, pattern: str = "*") -> List[str]:
        """Get keys matching a pattern.
        
        Args:
            pattern: Key pattern
            
        Returns:
            List of matching keys
        """
        if not self.redis:
            await self.connect()
        
        try:
            full_pattern = f"{self.prefix}{pattern}"
            keys = await self.redis.keys(full_pattern)
            # Remove prefix from keys
            return [key[len(self.prefix):] for key in keys]
        except Exception as e:
            logger.error(f"Error getting keys: {str(e)}")
            return []
    
    async def get_cache_stats(self) -> Dict[str, Any]:
        """Get cache statistics.
        
        Returns:
            Cache statistics
        """
        if not self.redis:
            await self.connect()
        
        try:
            # Get all keys with this prefix
            keys = await self.get_keys()
            
            # Get memory usage
            info = await self.redis.info("memory")
            
            return {
                "total_keys": len(keys),
                "memory_usage": info.get("used_memory_human", "unknown"),
                "ttl": self.ttl,
                "prefix": self.prefix,
            }
        except Exception as e:
            logger.error(f"Error getting cache stats: {str(e)}")
            return {
                "total_keys": 0,
                "memory_usage": "unknown",
                "ttl": self.ttl,
                "prefix": self.prefix,
                "error": str(e),
            }


# Create cache instances
weather_cache = RedisCache[WeatherData](WeatherData)
forecast_cache = RedisCache[List[WeatherForecast]](List[WeatherForecast])
