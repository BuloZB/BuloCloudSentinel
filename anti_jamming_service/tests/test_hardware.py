"""
Tests for hardware interfaces.

This module provides tests for the hardware interfaces of the Anti-Jamming Service.
"""

import pytest
import numpy as np
from unittest.mock import MagicMock, AsyncMock, patch

from anti_jamming_service.hardware.kraken_sdr import KrakenSDR
from anti_jamming_service.hardware.hackrf import HackRF
from anti_jamming_service.hardware.lora_sx127x import LoRaSX127x


@pytest.mark.asyncio
async def test_kraken_sdr_initialization():
    """Test KrakenSDR initialization."""
    # Create KrakenSDR instance
    kraken = KrakenSDR(device_index=0)
    
    # Mock initialization
    with patch.object(kraken, '_load_config'):
        result = await kraken.initialize()
    
    # Check result
    assert result is True
    assert kraken.initialized is True


@pytest.mark.asyncio
async def test_kraken_sdr_configuration():
    """Test KrakenSDR configuration."""
    # Create KrakenSDR instance
    kraken = KrakenSDR(device_index=0)
    
    # Mock initialization
    with patch.object(kraken, '_load_config'):
        await kraken.initialize()
    
    # Configure KrakenSDR
    config = {
        "center_frequency": 1575420000,
        "sample_rate": 2400000,
        "gain": 30.0,
        "coherent_mode": True,
        "reference_channel": 0
    }
    
    result = await kraken.configure(config)
    
    # Check result
    assert result is True
    assert kraken.center_frequency == 1575420000
    assert kraken.sample_rate == 2400000
    assert kraken.gain == 30.0
    assert kraken.coherent_mode is True
    assert kraken.reference_channel == 0


@pytest.mark.asyncio
async def test_kraken_sdr_get_samples():
    """Test KrakenSDR get_samples."""
    # Create KrakenSDR instance
    kraken = KrakenSDR(device_index=0)
    
    # Mock initialization
    with patch.object(kraken, '_load_config'):
        await kraken.initialize()
    
    # Start receiving
    await kraken.start_rx()
    
    # Get samples
    samples = await kraken.get_samples(1024)
    
    # Check result
    assert isinstance(samples, np.ndarray)
    assert samples.shape == (1024,)
    assert samples.dtype == np.complex128


@pytest.mark.asyncio
async def test_hackrf_initialization():
    """Test HackRF initialization."""
    # Create HackRF instance
    hackrf = HackRF()
    
    # Mock initialization
    with patch.object(hackrf, '_load_config'):
        result = await hackrf.initialize()
    
    # Check result
    assert result is True
    assert hackrf.initialized is True


@pytest.mark.asyncio
async def test_hackrf_configuration():
    """Test HackRF configuration."""
    # Create HackRF instance
    hackrf = HackRF()
    
    # Mock initialization
    with patch.object(hackrf, '_load_config'):
        await hackrf.initialize()
    
    # Configure HackRF
    config = {
        "center_frequency": 2450000000,
        "tx_frequency": 2450000000,
        "sample_rate": 10000000,
        "rx_gain": 30.0,
        "tx_gain": 0.0
    }
    
    result = await hackrf.configure(config)
    
    # Check result
    assert result is True
    assert hackrf.center_frequency == 2450000000
    assert hackrf.tx_frequency == 2450000000
    assert hackrf.sample_rate == 10000000
    assert hackrf.rx_gain == 30.0
    assert hackrf.tx_gain == 0.0


@pytest.mark.asyncio
async def test_hackrf_get_samples():
    """Test HackRF get_samples."""
    # Create HackRF instance
    hackrf = HackRF()
    
    # Mock initialization
    with patch.object(hackrf, '_load_config'):
        await hackrf.initialize()
    
    # Start receiving
    await hackrf.start_rx()
    
    # Get samples
    samples = await hackrf.get_samples(1024)
    
    # Check result
    assert isinstance(samples, np.ndarray)
    assert samples.shape == (1024,)
    assert samples.dtype == np.complex128


@pytest.mark.asyncio
async def test_lora_initialization():
    """Test LoRa initialization."""
    # Create LoRa instance
    lora = LoRaSX127x()
    
    # Mock initialization
    with patch.object(lora, '_load_config'):
        result = await lora.initialize()
    
    # Check result
    assert result is True
    assert lora.initialized is True


@pytest.mark.asyncio
async def test_lora_configuration():
    """Test LoRa configuration."""
    # Create LoRa instance
    lora = LoRaSX127x()
    
    # Mock initialization
    with patch.object(lora, '_load_config'):
        await lora.initialize()
    
    # Configure LoRa
    config = {
        "frequency": 868000000,
        "spreading_factor": 7,
        "bandwidth": 125000,
        "coding_rate": 5,
        "frequency_hopping": False,
        "hop_period": 0,
        "tx_power": 17
    }
    
    result = await lora.configure(config)
    
    # Check result
    assert result is True
    assert lora.frequency == 868000000
    assert lora.spreading_factor == 7
    assert lora.bandwidth == 125000
    assert lora.coding_rate == 5
    assert lora.frequency_hopping is False
    assert lora.hop_period == 0
    assert lora.tx_power == 17


@pytest.mark.asyncio
async def test_lora_send_receive():
    """Test LoRa send and receive."""
    # Create LoRa instance
    lora = LoRaSX127x()
    
    # Mock initialization
    with patch.object(lora, '_load_config'):
        await lora.initialize()
    
    # Send packet
    result = await lora.send_packet(b"Test message")
    
    # Check result
    assert result is True
    
    # Receive packet
    packet = await lora.receive_packet(timeout=1.0)
    
    # Check result
    assert packet is not None
    assert isinstance(packet, bytes)
