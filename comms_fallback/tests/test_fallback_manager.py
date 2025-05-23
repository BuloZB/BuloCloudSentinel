"""
Tests for fallback manager.

This module provides tests for the fallback manager.
"""

import asyncio
import json
import os
import pytest
import time
from typing import Any, Dict, List, Optional

from comms_fallback.adapters import (
    CommsAdapter,
    ConnectionQuality,
    ConnectionStatus,
    WifiMeshAdapter,
    FiveGAdapter,
    IridiumCertusAdapter,
)
from comms_fallback.services import (
    ConnectionMonitor,
    FallbackManager,
    WireGuardManager,
)


@pytest.fixture
def adapters():
    """Create test adapters."""
    wifi_mesh = WifiMeshAdapter({
        "name": "WiFi Mesh Test",
        "priority": 1,
        "mesh_node_id": "test_node",
        "min_rssi": -75,
        "max_latency": 100,
        "max_packet_loss": 5.0,
        "max_jitter": 20.0,
    })
    
    five_g = FiveGAdapter({
        "name": "5G Test",
        "priority": 2,
        "apn": "internet",
        "device": "/dev/ttyUSB0",
        "min_rssi": -90,
        "max_latency": 150,
        "max_packet_loss": 10.0,
        "max_jitter": 30.0,
    })
    
    iridium = IridiumCertusAdapter({
        "name": "Iridium Test",
        "priority": 3,
        "device": "/dev/ttyUSB1",
        "min_rssi": -100,
        "max_latency": 1000,
        "max_packet_loss": 15.0,
        "max_jitter": 100.0,
    })
    
    return {
        "wifi_mesh": wifi_mesh,
        "five_g": five_g,
        "iridium": iridium,
    }


@pytest.fixture
def config():
    """Create test configuration."""
    return {
        "monitor": {
            "check_interval": 0.1,
            "hysteresis_time": 0.5,
            "hysteresis_count": 3,
        },
        "fallback": {
            "buffer_size": 100,
            "critical_timeout": 5,
            "session_timeout": 10,
        },
        "wireguard": {
            "enabled": False,
        },
    }


@pytest.mark.asyncio
async def test_fallback_manager_init(adapters, config):
    """Test fallback manager initialization."""
    manager = FallbackManager(
        config=config,
        adapters=adapters,
    )
    
    assert manager.adapters == adapters
    assert manager.config == config
    assert manager.active_adapter_id is None
    assert manager.previous_adapter_id is None
    assert manager.transition_time == 0
    assert manager.running is False
    assert manager.manager_task is None
    assert manager.buffer_task is None


@pytest.mark.asyncio
async def test_fallback_manager_start_stop(adapters, config):
    """Test fallback manager start and stop."""
    manager = FallbackManager(
        config=config,
        adapters=adapters,
    )
    
    # Start manager
    await manager.start()
    assert manager.running is True
    assert manager.manager_task is not None
    assert manager.buffer_task is not None
    
    # Wait for manager to connect to an adapter
    for _ in range(10):
        if manager.active_adapter_id:
            break
        await asyncio.sleep(0.1)
    
    assert manager.active_adapter_id is not None
    assert manager.active_adapter_id in adapters
    
    # Stop manager
    await manager.stop()
    assert manager.running is False
    assert manager.manager_task is None
    assert manager.buffer_task is None
    assert manager.active_adapter_id is None


@pytest.mark.asyncio
async def test_fallback_manager_send_receive(adapters, config):
    """Test fallback manager send and receive."""
    manager = FallbackManager(
        config=config,
        adapters=adapters,
    )
    
    # Start manager
    await manager.start()
    
    # Wait for manager to connect to an adapter
    for _ in range(10):
        if manager.active_adapter_id:
            break
        await asyncio.sleep(0.1)
    
    assert manager.active_adapter_id is not None
    
    # Send message
    message = {"type": "test", "data": "Hello, world!"}
    sent = await manager.send_message(message)
    assert sent
    
    # Stop manager
    await manager.stop()


@pytest.mark.asyncio
async def test_fallback_manager_adapter_failure(adapters, config):
    """Test fallback manager adapter failure handling."""
    manager = FallbackManager(
        config=config,
        adapters=adapters,
    )
    
    # Start manager
    await manager.start()
    
    # Wait for manager to connect to an adapter
    for _ in range(10):
        if manager.active_adapter_id:
            break
        await asyncio.sleep(0.1)
    
    assert manager.active_adapter_id is not None
    first_adapter_id = manager.active_adapter_id
    
    # Simulate adapter failure
    adapters[first_adapter_id].status = ConnectionStatus.ERROR
    
    # Wait for manager to switch to another adapter
    for _ in range(20):
        if manager.active_adapter_id and manager.active_adapter_id != first_adapter_id:
            break
        await asyncio.sleep(0.1)
    
    assert manager.active_adapter_id is not None
    assert manager.active_adapter_id != first_adapter_id
    assert manager.previous_adapter_id == first_adapter_id
    
    # Stop manager
    await manager.stop()
