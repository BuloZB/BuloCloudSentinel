"""
Communication adapters for the SATCOM / 5G Fallback Connectivity module.

This package provides adapters for different communication methods:
- WifiMeshAdapter: Primary mesh network communications
- FiveGAdapter: 5G Standalone fallback in urban/suburban areas
- IridiumCertusAdapter: Satellite fallback in remote areas
"""

from .base import CommsAdapter, ConnectionQuality, ConnectionStatus
from .wifi_mesh import WifiMeshAdapter
from .five_g import FiveGAdapter
from .iridium import IridiumCertusAdapter

__all__ = [
    "CommsAdapter",
    "ConnectionQuality",
    "ConnectionStatus",
    "WifiMeshAdapter",
    "FiveGAdapter",
    "IridiumCertusAdapter",
]
