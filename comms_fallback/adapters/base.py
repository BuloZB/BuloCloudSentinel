"""
Base communication adapter interface.

This module defines the base interface for communication adapters.
"""

import abc
import asyncio
import enum
import time
from typing import Any, Dict, List, Optional, Tuple, Union


class ConnectionStatus(enum.Enum):
    """Connection status enum."""
    DISCONNECTED = "disconnected"
    CONNECTING = "connecting"
    CONNECTED = "connected"
    DEGRADED = "degraded"
    ERROR = "error"


class ConnectionQuality:
    """Connection quality metrics."""

    def __init__(
        self,
        rssi: float = 0.0,
        latency: float = 0.0,
        packet_loss: float = 0.0,
        jitter: float = 0.0,
        bandwidth: float = 0.0,
        timestamp: Optional[float] = None,
    ):
        """
        Initialize connection quality metrics.

        Args:
            rssi: Received Signal Strength Indicator (dBm)
            latency: Round-trip time (ms)
            packet_loss: Packet loss percentage (%)
            jitter: Jitter (ms)
            bandwidth: Available bandwidth (kbps)
            timestamp: Timestamp of the measurement (seconds since epoch)
        """
        self.rssi = rssi
        self.latency = latency
        self.packet_loss = packet_loss
        self.jitter = jitter
        self.bandwidth = bandwidth
        self.timestamp = timestamp or time.time()

    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary."""
        return {
            "rssi": self.rssi,
            "latency": self.latency,
            "packet_loss": self.packet_loss,
            "jitter": self.jitter,
            "bandwidth": self.bandwidth,
            "timestamp": self.timestamp,
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "ConnectionQuality":
        """Create from dictionary."""
        return cls(
            rssi=data.get("rssi", 0.0),
            latency=data.get("latency", 0.0),
            packet_loss=data.get("packet_loss", 0.0),
            jitter=data.get("jitter", 0.0),
            bandwidth=data.get("bandwidth", 0.0),
            timestamp=data.get("timestamp", time.time()),
        )


class CommsAdapter(abc.ABC):
    """Base communication adapter interface."""

    def __init__(self, config: Dict[str, Any]):
        """
        Initialize the adapter.

        Args:
            config: Adapter configuration
        """
        self.config = config
        self.name = config.get("name", self.__class__.__name__)
        self.priority = config.get("priority", 999)
        self.status = ConnectionStatus.DISCONNECTED
        self.last_quality = ConnectionQuality()
        self.connected_since: Optional[float] = None
        self.last_error: Optional[str] = None
        self.metrics: Dict[str, Any] = {
            "messages_sent": 0,
            "messages_received": 0,
            "bytes_sent": 0,
            "bytes_received": 0,
            "connect_attempts": 0,
            "connect_failures": 0,
        }

    @abc.abstractmethod
    async def connect(self) -> bool:
        """
        Connect to the communication channel.

        Returns:
            True if connected successfully, False otherwise
        """
        pass

    @abc.abstractmethod
    async def disconnect(self) -> bool:
        """
        Disconnect from the communication channel.

        Returns:
            True if disconnected successfully, False otherwise
        """
        pass

    @abc.abstractmethod
    async def send_message(self, message: Union[str, bytes, Dict[str, Any]]) -> bool:
        """
        Send a message.

        Args:
            message: Message to send

        Returns:
            True if sent successfully, False otherwise
        """
        pass

    @abc.abstractmethod
    async def receive_message(self, timeout: Optional[float] = None) -> Optional[Dict[str, Any]]:
        """
        Receive a message.

        Args:
            timeout: Timeout in seconds

        Returns:
            Received message or None if no message received
        """
        pass

    @abc.abstractmethod
    async def get_connection_quality(self) -> ConnectionQuality:
        """
        Get connection quality metrics.

        Returns:
            Connection quality metrics
        """
        pass

    async def is_available(self) -> bool:
        """
        Check if the adapter is available.

        Returns:
            True if available, False otherwise
        """
        return True

    def get_status(self) -> ConnectionStatus:
        """
        Get the current connection status.

        Returns:
            Connection status
        """
        return self.status

    def get_metrics(self) -> Dict[str, Any]:
        """
        Get adapter metrics.

        Returns:
            Adapter metrics
        """
        return {
            **self.metrics,
            "status": self.status.value,
            "connected_since": self.connected_since,
            "last_error": self.last_error,
            "last_quality": self.last_quality.to_dict(),
        }

    def __str__(self) -> str:
        """String representation."""
        return f"{self.name} (Priority: {self.priority}, Status: {self.status.value})"
