"""
Tests for communication adapters.

This module provides tests for the communication adapters.
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


@pytest.fixture
def wifi_mesh_config():
    """WiFi Mesh adapter configuration."""
    return {
        "name": "WiFi Mesh Test",
        "priority": 1,
        "mesh_node_id": "test_node",
        "min_rssi": -75,
        "max_latency": 100,
        "max_packet_loss": 5.0,
        "max_jitter": 20.0,
    }


@pytest.fixture
def five_g_config():
    """5G adapter configuration."""
    return {
        "name": "5G Test",
        "priority": 2,
        "apn": "internet",
        "device": "/dev/ttyUSB0",
        "min_rssi": -90,
        "max_latency": 150,
        "max_packet_loss": 10.0,
        "max_jitter": 30.0,
    }


@pytest.fixture
def iridium_config():
    """Iridium adapter configuration."""
    return {
        "name": "Iridium Test",
        "priority": 3,
        "device": "/dev/ttyUSB1",
        "min_rssi": -100,
        "max_latency": 1000,
        "max_packet_loss": 15.0,
        "max_jitter": 100.0,
    }


@pytest.mark.asyncio
async def test_wifi_mesh_adapter(wifi_mesh_config):
    """Test WiFi Mesh adapter."""
    adapter = WifiMeshAdapter(wifi_mesh_config)
    
    # Check initial state
    assert adapter.name == "WiFi Mesh Test"
    assert adapter.priority == 1
    assert adapter.status == ConnectionStatus.DISCONNECTED
    
    # Connect
    connected = await adapter.connect()
    assert connected
    assert adapter.status == ConnectionStatus.CONNECTED
    
    # Send message
    message = {"type": "test", "data": "Hello, world!"}
    sent = await adapter.send_message(message)
    assert sent
    
    # Get connection quality
    quality = await adapter.get_connection_quality()
    assert isinstance(quality, ConnectionQuality)
    assert quality.rssi <= 0
    assert quality.latency >= 0
    assert quality.packet_loss >= 0
    assert quality.jitter >= 0
    assert quality.bandwidth >= 0
    
    # Disconnect
    disconnected = await adapter.disconnect()
    assert disconnected
    assert adapter.status == ConnectionStatus.DISCONNECTED


@pytest.mark.asyncio
async def test_five_g_adapter(five_g_config):
    """Test 5G adapter."""
    adapter = FiveGAdapter(five_g_config)
    
    # Check initial state
    assert adapter.name == "5G Test"
    assert adapter.priority == 2
    assert adapter.status == ConnectionStatus.DISCONNECTED
    
    # Connect
    connected = await adapter.connect()
    assert connected
    assert adapter.status == ConnectionStatus.CONNECTED
    
    # Send message
    message = {"type": "test", "data": "Hello, world!"}
    sent = await adapter.send_message(message)
    assert sent
    
    # Get connection quality
    quality = await adapter.get_connection_quality()
    assert isinstance(quality, ConnectionQuality)
    assert quality.rssi <= 0
    assert quality.latency >= 0
    assert quality.packet_loss >= 0
    assert quality.jitter >= 0
    assert quality.bandwidth >= 0
    
    # Disconnect
    disconnected = await adapter.disconnect()
    assert disconnected
    assert adapter.status == ConnectionStatus.DISCONNECTED


@pytest.mark.asyncio
async def test_iridium_adapter(iridium_config):
    """Test Iridium adapter."""
    adapter = IridiumCertusAdapter(iridium_config)
    
    # Check initial state
    assert adapter.name == "Iridium Test"
    assert adapter.priority == 3
    assert adapter.status == ConnectionStatus.DISCONNECTED
    
    # Connect
    connected = await adapter.connect()
    assert connected
    assert adapter.status == ConnectionStatus.CONNECTED
    
    # Send message
    message = {"type": "test", "data": "Hello, world!"}
    sent = await adapter.send_message(message)
    assert sent
    
    # Get connection quality
    quality = await adapter.get_connection_quality()
    assert isinstance(quality, ConnectionQuality)
    assert quality.rssi <= 0
    assert quality.latency >= 0
    assert quality.packet_loss >= 0
    assert quality.jitter >= 0
    assert quality.bandwidth >= 0
    
    # Disconnect
    disconnected = await adapter.disconnect()
    assert disconnected
    assert adapter.status == ConnectionStatus.DISCONNECTED
