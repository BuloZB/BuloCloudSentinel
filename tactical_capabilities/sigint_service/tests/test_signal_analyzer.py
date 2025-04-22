"""
Tests for the signal analyzer service.
"""

import pytest
import numpy as np
from services.signal_analyzer import SignalAnalyzer

@pytest.fixture
def signal_analyzer():
    """Create a signal analyzer for testing."""
    return SignalAnalyzer()

@pytest.mark.asyncio
async def test_classify_signal_am(signal_analyzer):
    """Test classifying an AM signal."""
    # Create test AM signal
    sample_rate = 44100  # Hz
    duration = 1.0  # seconds
    t = np.arange(0, duration, 1/sample_rate)
    
    # Carrier frequency
    fc = 1000  # Hz
    carrier = np.cos(2 * np.pi * fc * t)
    
    # Modulating signal (message)
    fm = 100  # Hz
    modulator = 0.5 * (1 + 0.8 * np.cos(2 * np.pi * fm * t))
    
    # AM signal
    am_signal = modulator * carrier
    
    # Convert to complex IQ format
    iq_signal = am_signal + 1j * np.zeros_like(am_signal)
    
    # Classify signal
    result = await signal_analyzer.classify_signal(iq_signal, sample_rate, fc)
    
    # Check result
    assert result["signal_type"] == "AM"
    assert result["confidence"] > 0.7
    assert result["modulation"] == "AM"
    assert 80 < result["bandwidth"] < 120  # Should be around 2*fm = 200 Hz

@pytest.mark.asyncio
async def test_classify_signal_fm(signal_analyzer):
    """Test classifying an FM signal."""
    # Create test FM signal
    sample_rate = 44100  # Hz
    duration = 1.0  # seconds
    t = np.arange(0, duration, 1/sample_rate)
    
    # Carrier frequency
    fc = 1000  # Hz
    
    # Modulating signal (message)
    fm = 100  # Hz
    modulator = np.cos(2 * np.pi * fm * t)
    
    # Modulation index
    beta = 0.5
    
    # FM signal (using phase modulation)
    phase = 2 * np.pi * fc * t + beta * modulator
    fm_signal = np.cos(phase)
    
    # Convert to complex IQ format
    iq_signal = fm_signal + 1j * np.sin(phase)
    
    # Classify signal
    result = await signal_analyzer.classify_signal(iq_signal, sample_rate, fc)
    
    # Check result
    assert result["signal_type"] == "FM"
    assert result["confidence"] > 0.7
    assert result["modulation"] == "FM"
    assert result["bandwidth"] > 0  # Should be positive

@pytest.mark.asyncio
async def test_decode_signal(signal_analyzer):
    """Test decoding a signal."""
    # Create test AM signal
    sample_rate = 44100  # Hz
    duration = 1.0  # seconds
    t = np.arange(0, duration, 1/sample_rate)
    
    # Carrier frequency
    fc = 1000  # Hz
    carrier = np.cos(2 * np.pi * fc * t)
    
    # Modulating signal (message)
    fm = 100  # Hz
    modulator = 0.5 * (1 + 0.8 * np.cos(2 * np.pi * fm * t))
    
    # AM signal
    am_signal = modulator * carrier
    
    # Convert to complex IQ format
    iq_signal = am_signal + 1j * np.zeros_like(am_signal)
    
    # Decode signal
    result = await signal_analyzer.decode_signal(iq_signal, sample_rate, "AM")
    
    # Check result
    assert result["signal_type"] == "AM"
    assert result["success"] is True
    assert "content" in result
    assert result["confidence"] > 0.7

@pytest.mark.asyncio
async def test_extract_metadata(signal_analyzer):
    """Test extracting metadata from a signal."""
    # Create test FM signal
    sample_rate = 44100  # Hz
    duration = 1.0  # seconds
    t = np.arange(0, duration, 1/sample_rate)
    
    # Carrier frequency
    fc = 1000  # Hz
    
    # Modulating signal (message)
    fm = 100  # Hz
    modulator = np.cos(2 * np.pi * fm * t)
    
    # Modulation index
    beta = 0.5
    
    # FM signal (using phase modulation)
    phase = 2 * np.pi * fc * t + beta * modulator
    fm_signal = np.cos(phase)
    
    # Convert to complex IQ format
    iq_signal = fm_signal + 1j * np.sin(phase)
    
    # Extract metadata
    result = await signal_analyzer.extract_metadata(iq_signal, sample_rate, "FM")
    
    # Check result
    assert result["signal_type"] == "FM"
    assert result["duration"] == pytest.approx(duration, abs=0.01)
    assert "power" in result
    assert "peak_power" in result
    assert "metadata" in result
    assert "frequency_deviation" in result["metadata"]
    assert result["metadata"]["frequency_deviation"] > 0

@pytest.mark.asyncio
async def test_match_signal_profile(signal_analyzer):
    """Test matching signal features against profiles."""
    # Create test features
    features = {
        "time_domain": {
            "mean_amplitude": 0.5,
            "std_amplitude": 0.2,
            "mean_phase": 0.0,
            "std_phase": 0.1
        },
        "spectral": {
            "peak_frequency": 1000.0,
            "bandwidth": 200.0,
            "spectral_flatness": 0.1
        },
        "modulation": {
            "am_depth": 0.8,
            "fm_deviation": 0.1
        }
    }
    
    # Create test profiles
    profiles = [
        {
            "id": "profile1",
            "name": "Test Profile 1",
            "signal_type": "AM",
            "features": {
                "time_domain": {
                    "mean_amplitude": 0.5,
                    "std_amplitude": 0.2,
                    "mean_phase": 0.0,
                    "std_phase": 0.1
                },
                "spectral": {
                    "peak_frequency": 1000.0,
                    "bandwidth": 200.0,
                    "spectral_flatness": 0.1
                },
                "modulation": {
                    "am_depth": 0.8,
                    "fm_deviation": 0.1
                }
            },
            "threat_level": "medium"
        },
        {
            "id": "profile2",
            "name": "Test Profile 2",
            "signal_type": "FM",
            "features": {
                "time_domain": {
                    "mean_amplitude": 0.5,
                    "std_amplitude": 0.2,
                    "mean_phase": 0.0,
                    "std_phase": 0.5
                },
                "spectral": {
                    "peak_frequency": 2000.0,
                    "bandwidth": 500.0,
                    "spectral_flatness": 0.2
                },
                "modulation": {
                    "am_depth": 0.1,
                    "fm_deviation": 0.5
                }
            },
            "threat_level": "low"
        }
    ]
    
    # Match features against profiles
    profile, confidence = await signal_analyzer.match_signal_profile(features, profiles)
    
    # Check result
    assert profile is not None
    assert profile["id"] == "profile1"
    assert profile["name"] == "Test Profile 1"
    assert profile["signal_type"] == "AM"
    assert profile["threat_level"] == "medium"
    assert confidence > 0.9
