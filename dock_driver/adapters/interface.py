"""
Dock Adapter Interface

This module defines the interface that all dock adapters must implement.
"""

from abc import ABC, abstractmethod
from typing import Dict, Any, Optional
from enum import Enum


class DockStatus(str, Enum):
    """Dock status enum."""
    UNKNOWN = "unknown"
    ONLINE = "online"
    OFFLINE = "offline"
    OPEN = "open"
    CLOSED = "closed"
    ERROR = "error"


class ChargingStatus(str, Enum):
    """Charging status enum."""
    UNKNOWN = "unknown"
    IDLE = "idle"
    CHARGING = "charging"
    COMPLETE = "complete"
    ERROR = "error"


class DockAdapter(ABC):
    """Base class for all dock adapters."""

    @abstractmethod
    async def initialize(self) -> bool:
        """
        Initialize the adapter.
        
        Returns:
            bool: True if initialization was successful, False otherwise.
        """
        pass

    @abstractmethod
    async def get_status(self) -> Dict[str, Any]:
        """
        Get the current status of the dock.
        
        Returns:
            Dict[str, Any]: Dictionary containing the dock status.
        """
        pass

    @abstractmethod
    async def open_dock(self) -> bool:
        """
        Open the dock.
        
        Returns:
            bool: True if the operation was successful, False otherwise.
        """
        pass

    @abstractmethod
    async def close_dock(self) -> bool:
        """
        Close the dock.
        
        Returns:
            bool: True if the operation was successful, False otherwise.
        """
        pass

    @abstractmethod
    async def start_charging(self) -> bool:
        """
        Start charging the drone.
        
        Returns:
            bool: True if the operation was successful, False otherwise.
        """
        pass

    @abstractmethod
    async def stop_charging(self) -> bool:
        """
        Stop charging the drone.
        
        Returns:
            bool: True if the operation was successful, False otherwise.
        """
        pass

    @abstractmethod
    async def get_telemetry(self) -> Dict[str, Any]:
        """
        Get telemetry data from the dock.
        
        Returns:
            Dict[str, Any]: Dictionary containing telemetry data.
        """
        pass

    @abstractmethod
    async def shutdown(self) -> None:
        """
        Shutdown the adapter and release resources.
        """
        pass


class DockAdapterFactory:
    """Factory for creating dock adapters."""
    
    @staticmethod
    def create_adapter(dock_type: str, config: Dict[str, Any]) -> Optional[DockAdapter]:
        """
        Create a dock adapter based on the dock type.
        
        Args:
            dock_type: Type of the dock (dji, heisha, esp32).
            config: Configuration for the adapter.
            
        Returns:
            DockAdapter: An instance of the appropriate adapter, or None if the type is not supported.
        """
        if dock_type == "dji":
            from dock_driver.adapters.dji.adapter import DJIDockAdapter
            return DJIDockAdapter(config)
        elif dock_type == "heisha":
            from dock_driver.adapters.heisha.adapter import HeishaDockAdapter
            return HeishaDockAdapter(config)
        elif dock_type == "esp32":
            from dock_driver.adapters.esp32.adapter import ESP32DockAdapter
            return ESP32DockAdapter(config)
        else:
            return None
