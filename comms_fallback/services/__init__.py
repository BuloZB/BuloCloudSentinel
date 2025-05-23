"""
Services for the SATCOM / 5G Fallback Connectivity module.

This package provides services for managing communication fallback:
- ConnectionMonitor: Monitors connection quality metrics
- FallbackManager: Orchestrates transitions between communication methods
- WireGuardManager: Manages WireGuard tunnel transitions
"""

from .connection_monitor import ConnectionMonitor
from .fallback_manager import FallbackManager
from .wireguard_manager import WireGuardManager

__all__ = [
    "ConnectionMonitor",
    "FallbackManager",
    "WireGuardManager",
]
