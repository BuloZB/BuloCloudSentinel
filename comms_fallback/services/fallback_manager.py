"""
Fallback manager service.

This module provides a service for orchestrating transitions between communication methods.
"""

import asyncio
import json
import logging
import time
import uuid
from typing import Any, Dict, List, Optional, Set, Tuple, Union

from ..adapters.base import CommsAdapter, ConnectionStatus
from ..utils.redis_store import RedisStore
from .connection_monitor import ConnectionMonitor
from .wireguard_manager import WireGuardManager

logger = logging.getLogger(__name__)


class FallbackManager:
    """Fallback manager service."""

    def __init__(
        self,
        config: Dict[str, Any],
        adapters: Optional[Dict[str, CommsAdapter]] = None,
        redis_store: Optional[RedisStore] = None,
    ):
        """
        Initialize the fallback manager.

        Args:
            config: Fallback manager configuration
            adapters: Dictionary of communication adapters
            redis_store: Redis store for state management
        """
        self.config = config
        self.adapters = adapters or {}
        self.redis_store = redis_store
        self.buffer_size = config.get("buffer_size", 1000)
        self.critical_timeout = config.get("critical_timeout", 30)
        self.session_timeout = config.get("session_timeout", 300)
        self.message_buffer = asyncio.Queue(maxsize=self.buffer_size)
        self.active_adapter_id = None
        self.previous_adapter_id = None
        self.transition_time = 0
        self.running = False
        self.manager_task = None
        self.buffer_task = None
        
        # Initialize connection monitor
        monitor_config = config.get("monitor", {})
        self.connection_monitor = ConnectionMonitor(
            adapters=self.adapters,
            config=monitor_config,
            redis_store=self.redis_store,
        )
        self.connection_monitor.add_callback(self._on_connection_status_changed)
        
        # Initialize WireGuard manager
        wireguard_config = config.get("wireguard", {})
        self.wireguard_manager = WireGuardManager(config=wireguard_config)

    async def start(self):
        """Start the fallback manager."""
        if self.running:
            return

        logger.info("Starting fallback manager")
        self.running = True
        
        # Start connection monitor
        await self.connection_monitor.start()
        
        # Start WireGuard tunnel
        await self.wireguard_manager.start()
        
        # Start manager task
        self.manager_task = asyncio.create_task(self._manager_loop())
        
        # Start buffer processing task
        self.buffer_task = asyncio.create_task(self._process_buffer())
        
        # Connect to best available adapter
        await self._connect_best_adapter()

    async def stop(self):
        """Stop the fallback manager."""
        if not self.running:
            return

        logger.info("Stopping fallback manager")
        self.running = False
        
        # Stop tasks
        if self.manager_task:
            self.manager_task.cancel()
            try:
                await self.manager_task
            except asyncio.CancelledError:
                pass
            self.manager_task = None
            
        if self.buffer_task:
            self.buffer_task.cancel()
            try:
                await self.buffer_task
            except asyncio.CancelledError:
                pass
            self.buffer_task = None
        
        # Disconnect from active adapter
        if self.active_adapter_id:
            await self._disconnect_adapter(self.active_adapter_id)
            self.active_adapter_id = None
        
        # Stop WireGuard tunnel
        await self.wireguard_manager.stop()
        
        # Stop connection monitor
        await self.connection_monitor.stop()

    async def send_message(
        self,
        message: Union[str, bytes, Dict[str, Any]],
        critical: bool = False,
    ) -> bool:
        """
        Send a message with fallback support.

        Args:
            message: Message to send
            critical: Whether the message is critical

        Returns:
            True if sent successfully, False otherwise
        """
        # Check if we have an active adapter
        if not self.active_adapter_id:
            if critical:
                # Buffer critical messages
                await self._buffer_message(message, critical)
                return True
            else:
                logger.warning("No active adapter available for sending message")
                return False
        
        # Get active adapter
        adapter = self.adapters.get(self.active_adapter_id)
        if not adapter:
            logger.error(f"Active adapter {self.active_adapter_id} not found")
            return False
        
        # Check if adapter is connected
        if adapter.status != ConnectionStatus.CONNECTED:
            if critical:
                # Buffer critical messages
                await self._buffer_message(message, critical)
                return True
            else:
                logger.warning(f"Active adapter {self.active_adapter_id} is not connected")
                return False
        
        # Send message
        try:
            success = await adapter.send_message(message)
            if not success and critical:
                # Buffer critical messages that failed to send
                await self._buffer_message(message, critical)
                return True
            return success
        except Exception as e:
            logger.error(f"Error sending message: {e}")
            if critical:
                # Buffer critical messages that failed to send
                await self._buffer_message(message, critical)
                return True
            return False

    async def receive_message(self, timeout: Optional[float] = None) -> Optional[Dict[str, Any]]:
        """
        Receive a message.

        Args:
            timeout: Timeout in seconds

        Returns:
            Received message or None if no message received
        """
        # Check if we have an active adapter
        if not self.active_adapter_id:
            logger.warning("No active adapter available for receiving message")
            return None
        
        # Get active adapter
        adapter = self.adapters.get(self.active_adapter_id)
        if not adapter:
            logger.error(f"Active adapter {self.active_adapter_id} not found")
            return None
        
        # Check if adapter is connected
        if adapter.status != ConnectionStatus.CONNECTED:
            logger.warning(f"Active adapter {self.active_adapter_id} is not connected")
            return None
        
        # Receive message
        try:
            return await adapter.receive_message(timeout)
        except Exception as e:
            logger.error(f"Error receiving message: {e}")
            return None

    def get_active_adapter(self) -> Optional[str]:
        """
        Get the ID of the active adapter.

        Returns:
            Active adapter ID or None if no adapter is active
        """
        return self.active_adapter_id

    def get_status(self) -> Dict[str, Any]:
        """
        Get the current status of the fallback manager.

        Returns:
            Status information
        """
        return {
            "active_adapter": self.active_adapter_id,
            "previous_adapter": self.previous_adapter_id,
            "transition_time": self.transition_time,
            "buffer_size": self.message_buffer.qsize(),
            "adapters": self.connection_monitor.get_status(),
            "wireguard": asyncio.run(self.wireguard_manager.get_status()),
        }

    async def _connect_best_adapter(self) -> bool:
        """
        Connect to the best available adapter.

        Returns:
            True if connected successfully, False otherwise
        """
        # Get best adapter
        best_adapter_id = await self.connection_monitor.get_best_adapter()
        if not best_adapter_id:
            # Try to connect to all adapters in priority order
            for adapter_id, adapter in sorted(
                self.adapters.items(), key=lambda x: x[1].priority
            ):
                if await self._connect_adapter(adapter_id):
                    return True
            return False
        
        # Connect to best adapter
        return await self._connect_adapter(best_adapter_id)

    async def _connect_adapter(self, adapter_id: str) -> bool:
        """
        Connect to an adapter.

        Args:
            adapter_id: Adapter ID

        Returns:
            True if connected successfully, False otherwise
        """
        if adapter_id not in self.adapters:
            logger.error(f"Adapter {adapter_id} not found")
            return False
        
        adapter = self.adapters[adapter_id]
        
        # Check if already connected
        if adapter.status == ConnectionStatus.CONNECTED:
            self.active_adapter_id = adapter_id
            return True
        
        # Connect to adapter
        logger.info(f"Connecting to adapter {adapter_id}")
        success = await adapter.connect()
        
        if success:
            self.previous_adapter_id = self.active_adapter_id
            self.active_adapter_id = adapter_id
            self.transition_time = time.time()
            
            # Store state in Redis if available
            if self.redis_store:
                await self.redis_store.set(
                    "active_adapter",
                    {
                        "id": adapter_id,
                        "previous": self.previous_adapter_id,
                        "transition_time": self.transition_time,
                    }
                )
            
            # Update WireGuard endpoint if needed
            if adapter.config.get("wireguard_endpoint"):
                await self.wireguard_manager.update_endpoint(
                    adapter.config["wireguard_endpoint"]
                )
            
            logger.info(f"Connected to adapter {adapter_id}")
            return True
        else:
            logger.warning(f"Failed to connect to adapter {adapter_id}")
            return False

    async def _disconnect_adapter(self, adapter_id: str) -> bool:
        """
        Disconnect from an adapter.

        Args:
            adapter_id: Adapter ID

        Returns:
            True if disconnected successfully, False otherwise
        """
        if adapter_id not in self.adapters:
            logger.error(f"Adapter {adapter_id} not found")
            return False
        
        adapter = self.adapters[adapter_id]
        
        # Check if already disconnected
        if adapter.status == ConnectionStatus.DISCONNECTED:
            return True
        
        # Disconnect from adapter
        logger.info(f"Disconnecting from adapter {adapter_id}")
        success = await adapter.disconnect()
        
        if success:
            if self.active_adapter_id == adapter_id:
                self.previous_adapter_id = self.active_adapter_id
                self.active_adapter_id = None
                self.transition_time = time.time()
                
                # Store state in Redis if available
                if self.redis_store:
                    await self.redis_store.set(
                        "active_adapter",
                        {
                            "id": None,
                            "previous": self.previous_adapter_id,
                            "transition_time": self.transition_time,
                        }
                    )
            
            logger.info(f"Disconnected from adapter {adapter_id}")
            return True
        else:
            logger.warning(f"Failed to disconnect from adapter {adapter_id}")
            return False

    async def _buffer_message(
        self,
        message: Union[str, bytes, Dict[str, Any]],
        critical: bool = False,
    ):
        """
        Buffer a message for later sending.

        Args:
            message: Message to buffer
            critical: Whether the message is critical
        """
        try:
            # Convert message to dictionary if needed
            if isinstance(message, dict):
                message_dict = message
            elif isinstance(message, str):
                message_dict = {"data": message, "type": "text"}
            else:
                message_dict = {"data": message, "type": "binary"}
            
            # Add metadata
            message_dict["id"] = str(uuid.uuid4())
            message_dict["timestamp"] = time.time()
            message_dict["critical"] = critical
            message_dict["expiry"] = time.time() + (
                self.critical_timeout if critical else self.session_timeout
            )
            
            # Add to buffer
            await self.message_buffer.put(message_dict)
            logger.debug(f"Buffered message {message_dict['id']}")
            
        except asyncio.QueueFull:
            logger.warning("Message buffer full, dropping message")
        except Exception as e:
            logger.error(f"Error buffering message: {e}")

    async def _process_buffer(self):
        """Background task for processing buffered messages."""
        try:
            while self.running:
                # Check if we have an active adapter
                if not self.active_adapter_id:
                    await asyncio.sleep(1.0)
                    continue
                
                # Get active adapter
                adapter = self.adapters.get(self.active_adapter_id)
                if not adapter or adapter.status != ConnectionStatus.CONNECTED:
                    await asyncio.sleep(1.0)
                    continue
                
                # Process buffered messages
                try:
                    # Get message from buffer
                    message = self.message_buffer.get_nowait()
                    
                    # Check if message has expired
                    if message["expiry"] < time.time():
                        logger.warning(f"Message {message['id']} expired, dropping")
                        self.message_buffer.task_done()
                        continue
                    
                    # Send message
                    logger.debug(f"Sending buffered message {message['id']}")
                    success = await adapter.send_message(message)
                    
                    if success:
                        logger.debug(f"Sent buffered message {message['id']}")
                        self.message_buffer.task_done()
                    else:
                        # Put message back in buffer
                        logger.warning(f"Failed to send buffered message {message['id']}, re-buffering")
                        await self.message_buffer.put(message)
                        self.message_buffer.task_done()
                        
                        # Wait a bit before trying again
                        await asyncio.sleep(1.0)
                    
                except asyncio.QueueEmpty:
                    # No messages in buffer, wait a bit
                    await asyncio.sleep(0.1)
                except Exception as e:
                    logger.error(f"Error processing buffered message: {e}")
                    await asyncio.sleep(1.0)
                
        except asyncio.CancelledError:
            logger.debug("Buffer processing task cancelled")
            raise
        except Exception as e:
            logger.error(f"Error in buffer processing task: {e}")

    async def _manager_loop(self):
        """Background task for managing adapters."""
        try:
            while self.running:
                try:
                    # Check if we need to switch adapters
                    await self._check_adapters()
                except Exception as e:
                    logger.error(f"Error checking adapters: {e}")
                
                # Wait a bit
                await asyncio.sleep(1.0)
                
        except asyncio.CancelledError:
            logger.debug("Manager task cancelled")
            raise
        except Exception as e:
            logger.error(f"Error in manager task: {e}")

    async def _check_adapters(self):
        """Check if we need to switch adapters."""
        # Get best adapter
        best_adapter_id = await self.connection_monitor.get_best_adapter()
        
        # If no best adapter, try to connect to any adapter
        if not best_adapter_id:
            if not self.active_adapter_id:
                await self._connect_best_adapter()
            return
        
        # If no active adapter, connect to best adapter
        if not self.active_adapter_id:
            await self._connect_adapter(best_adapter_id)
            return
        
        # If best adapter is different from active adapter, switch
        if best_adapter_id != self.active_adapter_id:
            # Get adapters
            best_adapter = self.adapters.get(best_adapter_id)
            active_adapter = self.adapters.get(self.active_adapter_id)
            
            if not best_adapter or not active_adapter:
                return
            
            # Check if best adapter has higher priority
            if best_adapter.priority < active_adapter.priority:
                logger.info(f"Switching from adapter {self.active_adapter_id} to {best_adapter_id}")
                
                # Connect to best adapter
                if await self._connect_adapter(best_adapter_id):
                    # Disconnect from active adapter
                    await self._disconnect_adapter(self.active_adapter_id)

    def _on_connection_status_changed(self, adapter_id: str, status: ConnectionStatus):
        """
        Callback for connection status changes.

        Args:
            adapter_id: Adapter ID
            status: New connection status
        """
        logger.info(f"Adapter {adapter_id} status changed to {status.value}")
        
        # If active adapter is degraded or disconnected, try to switch
        if adapter_id == self.active_adapter_id and status in [
            ConnectionStatus.DEGRADED,
            ConnectionStatus.ERROR,
            ConnectionStatus.DISCONNECTED,
        ]:
            asyncio.create_task(self._connect_best_adapter())
