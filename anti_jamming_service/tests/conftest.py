"""
Test fixtures for the Anti-Jamming Service.

This module provides pytest fixtures for testing the Anti-Jamming Service.
"""

import pytest
import asyncio
import numpy as np
from typing import Dict, Any, Optional
from unittest.mock import MagicMock, AsyncMock

from anti_jamming_service.hardware.interfaces import IKrakenSDRInterface, IHackRFInterface, ILoRaInterface
from anti_jamming_service.processing.gnss_mitigation import GNSSMitigationProcessor
from anti_jamming_service.processing.doa_estimation import DoAEstimator
from anti_jamming_service.processing.jamming_detection import JammingDetector, JammingType
from anti_jamming_service.processing.fhss import FHSSProtocol


@pytest.fixture
def mock_kraken_sdr():
    """Mock KrakenSDR hardware interface."""
    mock = AsyncMock(spec=IKrakenSDRInterface)
    mock.initialized = True
    mock.get_status.return_value = {
        "device_index": 0,
        "initialized": True,
        "running": False,
        "coherent_mode": True,
        "reference_channel": 0,
        "center_frequency": 1575420000,
        "sample_rate": 2400000,
        "gain": 30.0,
        "doa": {"azimuth": 45.0, "elevation": 0.0, "confidence": 0.8}
    }
    mock.get_samples.return_value = np.random.normal(0, 1, 1024) + 1j * np.random.normal(0, 1, 1024)
    mock.get_channel_samples.return_value = np.random.normal(0, 1, 1024) + 1j * np.random.normal(0, 1, 1024)
    mock.get_doa_estimate.return_value = (45.0, 0.0)
    return mock


@pytest.fixture
def mock_hackrf():
    """Mock HackRF hardware interface."""
    mock = AsyncMock(spec=IHackRFInterface)
    mock.initialized = True
    mock.get_status.return_value = {
        "serial_number": "0123456789",
        "initialized": True,
        "rx_running": False,
        "tx_running": False,
        "center_frequency": 2450000000,
        "tx_frequency": 2450000000,
        "sample_rate": 10000000,
        "rx_gain": 30.0,
        "tx_gain": 0.0
    }
    mock.get_samples.return_value = np.random.normal(0, 1, 1024) + 1j * np.random.normal(0, 1, 1024)
    return mock


@pytest.fixture
def mock_lora():
    """Mock LoRa hardware interface."""
    mock = AsyncMock(spec=ILoRaInterface)
    mock.initialized = True
    mock.get_status.return_value = {
        "port": "/dev/ttyUSB0",
        "initialized": True,
        "frequency": 868000000,
        "spreading_factor": 7,
        "bandwidth": 125000,
        "coding_rate": 5,
        "frequency_hopping": False,
        "hop_period": 0,
        "tx_power": 17
    }
    mock.send_packet.return_value = True
    mock.receive_packet.return_value = b"Test message"
    return mock


@pytest.fixture
def gnss_mitigation_processor():
    """GNSS mitigation processor."""
    return GNSSMitigationProcessor()


@pytest.fixture
def doa_estimator(mock_kraken_sdr):
    """DoA estimator."""
    return DoAEstimator(sdr_interface=mock_kraken_sdr)


@pytest.fixture
def jamming_detector(mock_kraken_sdr):
    """Jamming detector."""
    detector = JammingDetector(sdr_interface=mock_kraken_sdr)
    detector.last_detection = {
        "is_jamming": True,
        "jamming_type": JammingType.CONTINUOUS_WAVE,
        "confidence": 0.8,
        "snr_db": 15.0,
        "timestamp": 1234567890.0
    }
    return detector


@pytest.fixture
def fhss_protocol(mock_lora):
    """FHSS protocol."""
    return FHSSProtocol(lora_interface=mock_lora)
