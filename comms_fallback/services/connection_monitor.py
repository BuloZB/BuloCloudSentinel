"""
Connection monitor service.

This module provides a service for monitoring connection quality metrics.
"""

import asyncio
import logging
import time
from typing import Any, Dict, List, Optional, Set, Tuple, Union

from ..adapters.base import CommsAdapter, ConnectionQuality, ConnectionStatus
from ..utils.redis_store import RedisStore

logger = logging.getLogger(__name__)


class ConnectionMonitor:
    """Connection monitor service."""

    def __init__(
        self,
        adapters: Dict[str, CommsAdapter],
        config: Dict[str, Any],
        redis_store: Optional[RedisStore] = None,
    ):
        """
        Initialize the connection monitor.

        Args:
            adapters: Dictionary of communication adapters
            config: Monitor configuration
            redis_store: Redis store for state management
        """
        self.adapters = adapters
        self.config = config
        self.redis_store = redis_store
        self.check_interval = config.get("check_interval", 1.0)
        self.hysteresis_time = config.get("hysteresis_time", 5.0)
        self.hysteresis_count = config.get("hysteresis_count", 5)
        self.degradation_counts = {adapter_id: 0 for adapter_id in adapters}
        self.recovery_counts = {adapter_id: 0 for adapter_id in adapters}
        self.last_check_time = 0
        self.running = False
        self.monitor_task = None
        self.callbacks = []

    async def start(self):
        """Start the connection monitor."""
        if self.running:
            return

        logger.info("Starting connection monitor")
        self.running = True
        self.monitor_task = asyncio.create_task(self._monitor_loop())

    async def stop(self):
        """Stop the connection monitor."""
        if not self.running:
            return

        logger.info("Stopping connection monitor")
        self.running = False
        if self.monitor_task:
            self.monitor_task.cancel()
            try:
                await self.monitor_task
            except asyncio.CancelledError:
                pass
            self.monitor_task = None

    def add_callback(self, callback):
        """
        Add a callback function to be called when connection status changes.

        Args:
            callback: Callback function
        """
        self.callbacks.append(callback)

    def remove_callback(self, callback):
        """
        Remove a callback function.

        Args:
            callback: Callback function to remove
        """
        if callback in self.callbacks:
            self.callbacks.remove(callback)

    async def get_best_adapter(self) -> Optional[str]:
        """
        Get the ID of the best available adapter.

        Returns:
            Adapter ID or None if no adapter is available
        """
        best_adapter_id = None
        best_priority = float('inf')
        best_quality = None

        for adapter_id, adapter in self.adapters.items():
            if adapter.status == ConnectionStatus.CONNECTED:
                # Lower priority number means higher priority
                if adapter.priority < best_priority:
                    best_adapter_id = adapter_id
                    best_priority = adapter.priority
                    best_quality = await adapter.get_connection_quality()

        return best_adapter_id

    async def get_connection_quality(self, adapter_id: str) -> Optional[ConnectionQuality]:
        """
        Get connection quality metrics for an adapter.

        Args:
            adapter_id: Adapter ID

        Returns:
            Connection quality metrics or None if adapter not found
        """
        if adapter_id not in self.adapters:
            return None

        return await self.adapters[adapter_id].get_connection_quality()

    async def get_all_connection_qualities(self) -> Dict[str, ConnectionQuality]:
        """
        Get connection quality metrics for all adapters.

        Returns:
            Dictionary of adapter IDs to connection quality metrics
        """
        result = {}
        for adapter_id, adapter in self.adapters.items():
            result[adapter_id] = await adapter.get_connection_quality()
        return result

    async def _monitor_loop(self):
        """Background task for monitoring connection quality."""
        try:
            while self.running:
                try:
                    await self._check_connections()
                    self.last_check_time = time.time()
                except Exception as e:
                    logger.error(f"Error checking connections: {e}")

                # Wait for next check interval
                await asyncio.sleep(self.check_interval)

        except asyncio.CancelledError:
            logger.debug("Monitor loop cancelled")
            raise
        except Exception as e:
            logger.error(f"Error in monitor loop: {e}")

    async def _check_connections(self):
        """Check connection quality for all adapters."""
        for adapter_id, adapter in self.adapters.items():
            try:
                # Skip if adapter is not connected
                if adapter.status not in [ConnectionStatus.CONNECTED, ConnectionStatus.DEGRADED]:
                    continue

                # Get connection quality
                quality = await adapter.get_connection_quality()

                # Check if connection is degraded
                is_degraded = (
                    quality.rssi < adapter.min_rssi or
                    quality.latency > adapter.max_latency or
                    quality.packet_loss > adapter.max_packet_loss or
                    quality.jitter > adapter.max_jitter
                )

                # Apply hysteresis to prevent rapid switching
                if is_degraded:
                    self.degradation_counts[adapter_id] += 1
                    self.recovery_counts[adapter_id] = 0
                else:
                    self.degradation_counts[adapter_id] = 0
                    self.recovery_counts[adapter_id] += 1

                # Update adapter status
                old_status = adapter.status
                if self.degradation_counts[adapter_id] >= self.hysteresis_count:
                    if adapter.status == ConnectionStatus.CONNECTED:
                        logger.warning(f"Adapter {adapter_id} connection degraded: {quality.to_dict()}")
                        adapter.status = ConnectionStatus.DEGRADED
                elif self.recovery_counts[adapter_id] >= self.hysteresis_count:
                    if adapter.status == ConnectionStatus.DEGRADED:
                        logger.info(f"Adapter {adapter_id} connection restored: {quality.to_dict()}")
                        adapter.status = ConnectionStatus.CONNECTED

                # Store metrics in Redis if available
                if self.redis_store:
                    await self.redis_store.set(
                        f"connection_quality:{adapter_id}",
                        quality.to_dict()
                    )

                # Call callbacks if status changed
                if adapter.status != old_status:
                    for callback in self.callbacks:
                        try:
                            callback(adapter_id, adapter.status)
                        except Exception as e:
                            logger.error(f"Error in callback: {e}")

            except Exception as e:
                logger.error(f"Error checking connection for adapter {adapter_id}: {e}")

    def get_status(self) -> Dict[str, Any]:
        """
        Get the current status of all adapters.

        Returns:
            Dictionary of adapter IDs to status information
        """
        return {
            adapter_id: {
                "status": adapter.status.value,
                "priority": adapter.priority,
                "connected_since": adapter.connected_since,
                "last_error": adapter.last_error,
                "metrics": adapter.metrics,
            }
            for adapter_id, adapter in self.adapters.items()
        }
