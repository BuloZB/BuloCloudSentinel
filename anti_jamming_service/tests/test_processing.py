"""
Tests for signal processing components.

This module provides tests for the signal processing components of the Anti-Jamming Service.
"""

import pytest
import numpy as np
from unittest.mock import MagicMock, AsyncMock, patch

from anti_jamming_service.processing.gnss_mitigation import GNSSMitigationProcessor, PulseBlankingFilter, NotchFilter
from anti_jamming_service.processing.doa_estimation import DoAEstimator, MUSICAlgorithm
from anti_jamming_service.processing.jamming_detection import JammingDetector, JammingType, EnergyDetector, SpectralDetector
from anti_jamming_service.processing.fhss import FHSSProtocol, HoppingPattern


def test_pulse_blanking_filter():
    """Test pulse blanking filter."""
    # Create filter
    pb_filter = PulseBlankingFilter(threshold=3.0, window_size=1024)
    
    # Create test signal with pulse
    signal = np.random.normal(0, 1, 1024) + 1j * np.random.normal(0, 1, 1024)
    signal[512] = 10.0 + 10.0j  # Add pulse
    
    # Process signal
    processed = pb_filter.process(signal)
    
    # Check result
    assert isinstance(processed, np.ndarray)
    assert processed.shape == signal.shape
    assert processed[512] == 0.0  # Pulse should be blanked


def test_notch_filter():
    """Test notch filter."""
    # Create filter
    notch_filter = NotchFilter(threshold=10.0, fft_size=1024, overlap=512)
    
    # Create test signal with narrowband interference
    signal = np.random.normal(0, 1, 2048) + 1j * np.random.normal(0, 1, 2048)
    t = np.arange(2048)
    interference = 10.0 * np.exp(1j * 0.1 * t)  # Narrowband interference
    signal += interference
    
    # Process signal
    processed = notch_filter.process(signal)
    
    # Check result
    assert isinstance(processed, np.ndarray)
    assert processed.shape == signal.shape
    
    # Check that interference is reduced
    signal_power = np.mean(np.abs(signal) ** 2)
    processed_power = np.mean(np.abs(processed) ** 2)
    assert processed_power < signal_power


def test_gnss_mitigation_processor():
    """Test GNSS mitigation processor."""
    # Create processor
    processor = GNSSMitigationProcessor()
    
    # Create test signal with pulse and narrowband interference
    signal = np.random.normal(0, 1, 2048) + 1j * np.random.normal(0, 1, 2048)
    signal[512] = 10.0 + 10.0j  # Add pulse
    t = np.arange(2048)
    interference = 10.0 * np.exp(1j * 0.1 * t)  # Narrowband interference
    signal += interference
    
    # Process signal
    processed = processor.process(signal)
    
    # Check result
    assert isinstance(processed, np.ndarray)
    assert processed.shape == signal.shape
    
    # Check that interference is reduced
    signal_power = np.mean(np.abs(signal) ** 2)
    processed_power = np.mean(np.abs(processed) ** 2)
    assert processed_power < signal_power


@pytest.mark.asyncio
async def test_doa_estimator(mock_kraken_sdr):
    """Test DoA estimator."""
    # Create estimator
    estimator = DoAEstimator(sdr_interface=mock_kraken_sdr)
    
    # Estimate DoA
    result = await estimator.estimate()
    
    # Check result
    assert isinstance(result, dict)
    assert "azimuth" in result
    assert "elevation" in result
    assert "confidence" in result
    assert isinstance(result["azimuth"], float)
    assert isinstance(result["elevation"], float)
    assert isinstance(result["confidence"], float)


def test_music_algorithm():
    """Test MUSIC algorithm."""
    # Create algorithm
    music = MUSICAlgorithm(num_antennas=5, num_sources=1, angle_resolution=1.0)
    
    # Create test signal
    samples = np.zeros((5, 1024), dtype=np.complex128)
    for i in range(5):
        samples[i] = np.random.normal(0, 1, 1024) + 1j * np.random.normal(0, 1, 1024)
    
    # Add signal from 45 degrees
    steering_vector = np.exp(1j * np.pi * np.arange(5) * np.sin(np.deg2rad(45)))
    signal = np.random.normal(0, 1, 1024) + 1j * np.random.normal(0, 1, 1024)
    for i in range(5):
        samples[i] += 5.0 * steering_vector[i] * signal
    
    # Estimate DoA
    angles, amplitudes = music.estimate(samples)
    
    # Check result
    assert isinstance(angles, np.ndarray)
    assert isinstance(amplitudes, np.ndarray)
    assert len(angles) == 1
    assert len(amplitudes) == 1
    assert abs(angles[0] - 45.0) < 5.0  # Allow some error


@pytest.mark.asyncio
async def test_jamming_detector(mock_kraken_sdr):
    """Test jamming detector."""
    # Create detector
    detector = JammingDetector(sdr_interface=mock_kraken_sdr)
    
    # Create test signal with jamming
    signal = np.random.normal(0, 1, 1024) + 1j * np.random.normal(0, 1, 1024)
    t = np.arange(1024)
    jamming = 10.0 * np.exp(1j * 0.1 * t)  # CW jamming
    signal += jamming
    
    # Detect jamming
    result = await detector.detect(signal)
    
    # Check result
    assert isinstance(result, dict)
    assert "is_jamming" in result
    assert "jamming_type" in result
    assert "confidence" in result
    assert "snr_db" in result
    assert "timestamp" in result
    assert isinstance(result["is_jamming"], bool)
    assert isinstance(result["jamming_type"], JammingType)
    assert isinstance(result["confidence"], float)
    assert isinstance(result["snr_db"], float)
    assert isinstance(result["timestamp"], float)


def test_energy_detector():
    """Test energy detector."""
    # Create detector
    detector = EnergyDetector(threshold=10.0, window_size=1024)
    
    # Create test signal with jamming
    signal = np.random.normal(0, 1, 1024) + 1j * np.random.normal(0, 1, 1024)
    t = np.arange(1024)
    jamming = 10.0 * np.exp(1j * 0.1 * t)  # CW jamming
    signal += jamming
    
    # Detect jamming
    result = detector.detect(signal)
    
    # Check result
    assert isinstance(result, dict)
    assert "is_jamming" in result
    assert "jamming_type" in result
    assert "snr_db" in result
    assert isinstance(result["is_jamming"], bool)
    assert result["is_jamming"] is True


def test_spectral_detector():
    """Test spectral detector."""
    # Create detector
    detector = SpectralDetector(threshold=10.0, fft_size=1024, overlap=512)
    
    # Create test signal with jamming
    signal = np.random.normal(0, 1, 2048) + 1j * np.random.normal(0, 1, 2048)
    t = np.arange(2048)
    jamming = 10.0 * np.exp(1j * 0.1 * t)  # CW jamming
    signal += jamming
    
    # Detect jamming
    result = detector.detect(signal)
    
    # Check result
    assert isinstance(result, dict)
    assert "is_jamming" in result
    assert "jamming_type" in result
    assert "snr_db" in result
    assert "spectral_peaks" in result
    assert isinstance(result["is_jamming"], bool)
    assert result["is_jamming"] is True


def test_hopping_pattern():
    """Test hopping pattern."""
    # Create pattern
    pattern = HoppingPattern(num_channels=10, seed=b"test")
    
    # Get channels
    channels = [pattern.next_channel() for _ in range(20)]
    
    # Check result
    assert len(channels) == 20
    assert all(0 <= c < 10 for c in channels)
    
    # Reset pattern
    pattern.reset(seed=b"test")
    
    # Get channels again
    channels2 = [pattern.next_channel() for _ in range(10)]
    
    # Check that pattern repeats
    assert channels[:10] == channels2


@pytest.mark.asyncio
async def test_fhss_protocol(mock_lora):
    """Test FHSS protocol."""
    # Create protocol
    protocol = FHSSProtocol(lora_interface=mock_lora)
    
    # Initialize protocol
    await protocol.initialize()
    
    # Send message
    result = await protocol.send_message(b"Test message")
    
    # Check result
    assert result is True
    
    # Receive message
    message = await protocol.receive_message(timeout=1.0)
    
    # Check result
    assert message is not None
    assert isinstance(message, bytes)
