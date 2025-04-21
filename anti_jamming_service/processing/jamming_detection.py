"""
Jamming detection algorithms.

This module provides algorithms for detecting various types of jamming signals.
"""

import logging
import numpy as np
from typing import Dict, List, Optional, Tuple, Any, Union
import asyncio
from enum import Enum, auto

from anti_jamming_service.hardware.interfaces import IJammingDetector, ISDRInterface
from anti_jamming_service.utils.config import get_config

logger = logging.getLogger(__name__)


class JammingType(Enum):
    """Enumeration of jamming signal types."""
    UNKNOWN = auto()
    CONTINUOUS_WAVE = auto()
    WIDEBAND_NOISE = auto()
    SWEPT_FREQUENCY = auto()
    PULSED = auto()
    REPEATER = auto()
    MEACONER = auto()


class EnergyDetector:
    """
    Energy detector for jamming detection.
    
    This class implements a simple energy detector that compares the signal
    energy to a threshold to detect jamming.
    """
    
    def __init__(self, threshold: float = 10.0, window_size: int = 1024):
        """
        Initialize the energy detector.
        
        Args:
            threshold: Threshold for detection (in dB above noise floor).
            window_size: Size of the processing window.
        """
        self.threshold = threshold
        self.window_size = window_size
        self.noise_floor = None
    
    def detect(self, samples: np.ndarray) -> Dict[str, Any]:
        """
        Detect jamming signals using energy detection.
        
        Args:
            samples: Complex samples to analyze.
            
        Returns:
            Dict[str, Any]: Detection results.
        """
        # Calculate signal power
        power = np.mean(np.abs(samples) ** 2)
        
        # Estimate noise floor if not already estimated
        if self.noise_floor is None:
            # Use the minimum power in the window as an estimate of the noise floor
            # In a real implementation, this would be more sophisticated
            self.noise_floor = np.min(np.abs(samples) ** 2)
        
        # Calculate SNR in dB
        snr_db = 10 * np.log10(power / self.noise_floor)
        
        # Check if SNR exceeds threshold
        is_jamming = snr_db > self.threshold
        
        # Determine jamming type based on signal characteristics
        jamming_type = JammingType.UNKNOWN
        if is_jamming:
            # Calculate spectrum
            spectrum = np.abs(np.fft.fft(samples)) ** 2
            
            # Normalize spectrum
            spectrum = spectrum / np.max(spectrum)
            
            # Check for continuous wave (narrow peak in spectrum)
            peak_width = np.sum(spectrum > 0.5) / len(spectrum)
            if peak_width < 0.05:
                jamming_type = JammingType.CONTINUOUS_WAVE
            # Check for wideband noise (flat spectrum)
            elif np.std(spectrum) < 0.1:
                jamming_type = JammingType.WIDEBAND_NOISE
            # Check for pulsed jamming (time domain characteristics)
            elif np.max(np.abs(samples)) / np.mean(np.abs(samples)) > 5.0:
                jamming_type = JammingType.PULSED
        
        return {
            "is_jamming": is_jamming,
            "jamming_type": jamming_type,
            "snr_db": snr_db,
            "power": power,
            "noise_floor": self.noise_floor
        }


class SpectralDetector:
    """
    Spectral detector for jamming detection.
    
    This class implements a spectral detector that analyzes the signal
    spectrum to detect and classify jamming signals.
    """
    
    def __init__(self, threshold: float = 10.0, fft_size: int = 1024, overlap: int = 512):
        """
        Initialize the spectral detector.
        
        Args:
            threshold: Threshold for detection (in dB).
            fft_size: Size of the FFT.
            overlap: Overlap between FFT windows.
        """
        self.threshold = threshold
        self.fft_size = fft_size
        self.overlap = overlap
    
    def detect(self, samples: np.ndarray) -> Dict[str, Any]:
        """
        Detect jamming signals using spectral analysis.
        
        Args:
            samples: Complex samples to analyze.
            
        Returns:
            Dict[str, Any]: Detection results.
        """
        # Ensure samples length is a multiple of (fft_size - overlap)
        step = self.fft_size - self.overlap
        num_steps = (len(samples) - self.overlap) // step
        if num_steps < 1:
            # Not enough samples
            return {
                "is_jamming": False,
                "jamming_type": JammingType.UNKNOWN,
                "snr_db": 0.0,
                "spectral_peaks": []
            }
        
        # Truncate samples to fit
        truncated_length = num_steps * step + self.overlap
        truncated_samples = samples[:truncated_length]
        
        # Initialize spectrogram
        spectrogram = np.zeros((num_steps, self.fft_size))
        
        # Create window function
        window = np.hanning(self.fft_size)
        
        # Compute spectrogram
        for i in range(num_steps):
            # Extract window
            start = i * step
            end = start + self.fft_size
            window_samples = truncated_samples[start:end]
            
            # Apply window function
            windowed_samples = window_samples * window
            
            # Compute FFT
            fft = np.fft.fft(windowed_samples)
            
            # Compute power spectrum
            power_spectrum = np.abs(fft) ** 2
            
            # Store in spectrogram
            spectrogram[i] = power_spectrum
        
        # Compute average spectrum
        avg_spectrum = np.mean(spectrogram, axis=0)
        
        # Compute noise floor (10th percentile)
        noise_floor = np.percentile(avg_spectrum, 10)
        
        # Convert to dB
        spectrum_db = 10 * np.log10(avg_spectrum / noise_floor)
        
        # Find peaks above threshold
        peaks = []
        for i in range(1, len(spectrum_db) - 1):
            if spectrum_db[i] > self.threshold and spectrum_db[i] > spectrum_db[i-1] and spectrum_db[i] > spectrum_db[i+1]:
                peaks.append((i, spectrum_db[i]))
        
        # Sort peaks by amplitude
        peaks.sort(key=lambda x: x[1], reverse=True)
        
        # Determine if jamming is present
        is_jamming = len(peaks) > 0
        
        # Determine jamming type based on spectral characteristics
        jamming_type = JammingType.UNKNOWN
        if is_jamming:
            # Check for continuous wave (single strong peak)
            if len(peaks) == 1 and peaks[0][1] > 20.0:
                jamming_type = JammingType.CONTINUOUS_WAVE
            # Check for wideband noise (many peaks of similar amplitude)
            elif len(peaks) > 10 and np.std([p[1] for p in peaks[:10]]) < 3.0:
                jamming_type = JammingType.WIDEBAND_NOISE
            # Check for swept frequency (peaks change over time)
            elif np.std(np.var(spectrogram, axis=0)) > 10.0:
                jamming_type = JammingType.SWEPT_FREQUENCY
        
        # Calculate SNR in dB
        snr_db = np.max(spectrum_db) if is_jamming else 0.0
        
        return {
            "is_jamming": is_jamming,
            "jamming_type": jamming_type,
            "snr_db": snr_db,
            "spectral_peaks": peaks
        }


class CyclostationaryDetector:
    """
    Cyclostationary detector for jamming detection.
    
    This class implements a cyclostationary detector that analyzes the
    cyclic spectral density to detect and classify jamming signals.
    """
    
    def __init__(self, threshold: float = 0.8, window_size: int = 1024, num_lags: int = 128):
        """
        Initialize the cyclostationary detector.
        
        Args:
            threshold: Threshold for detection (0-1).
            window_size: Size of the processing window.
            num_lags: Number of lags for autocorrelation.
        """
        self.threshold = threshold
        self.window_size = window_size
        self.num_lags = num_lags
    
    def detect(self, samples: np.ndarray) -> Dict[str, Any]:
        """
        Detect jamming signals using cyclostationary analysis.
        
        Args:
            samples: Complex samples to analyze.
            
        Returns:
            Dict[str, Any]: Detection results.
        """
        # Ensure enough samples
        if len(samples) < self.window_size:
            return {
                "is_jamming": False,
                "jamming_type": JammingType.UNKNOWN,
                "cyclic_features": []
            }
        
        # Compute autocorrelation for different lags
        autocorr = np.zeros(self.num_lags, dtype=np.complex128)
        for lag in range(self.num_lags):
            if lag == 0:
                autocorr[lag] = np.mean(np.abs(samples) ** 2)
            else:
                autocorr[lag] = np.mean(samples[:-lag] * np.conj(samples[lag:]))
        
        # Normalize autocorrelation
        autocorr = autocorr / autocorr[0]
        
        # Compute cyclic spectral density
        csd = np.abs(np.fft.fft(autocorr))
        
        # Normalize CSD
        csd = csd / np.max(csd)
        
        # Find peaks in CSD
        peaks = []
        for i in range(1, len(csd) - 1):
            if csd[i] > self.threshold and csd[i] > csd[i-1] and csd[i] > csd[i+1]:
                peaks.append((i, csd[i]))
        
        # Sort peaks by amplitude
        peaks.sort(key=lambda x: x[1], reverse=True)
        
        # Determine if jamming is present
        is_jamming = len(peaks) > 0
        
        # Determine jamming type based on cyclostationary characteristics
        jamming_type = JammingType.UNKNOWN
        if is_jamming:
            # Check for repeater/meaconer (strong cyclic features)
            if len(peaks) > 3 and peaks[0][1] > 0.9:
                jamming_type = JammingType.REPEATER
        
        return {
            "is_jamming": is_jamming,
            "jamming_type": jamming_type,
            "cyclic_features": peaks
        }


class JammingDetector(IJammingDetector):
    """
    Jamming detector implementation.
    
    This class implements the jamming detector interface and combines multiple
    detection algorithms to provide comprehensive jamming detection capabilities.
    """
    
    def __init__(self, sdr_interface: ISDRInterface, config: Optional[Dict[str, Any]] = None):
        """
        Initialize the jamming detector.
        
        Args:
            sdr_interface: SDR interface for getting samples.
            config: Optional configuration parameters.
        """
        self.sdr = sdr_interface
        self.config = config or {}
        self.energy_detector = None
        self.spectral_detector = None
        self.cyclostationary_detector = None
        self.enabled = True
        self.sample_size = 1024
        self.detection_interval = 1.0  # seconds
        self.detection_task = None
        self.last_detection = {
            "is_jamming": False,
            "jamming_type": JammingType.UNKNOWN,
            "confidence": 0.0,
            "snr_db": 0.0,
            "timestamp": 0.0
        }
        
        # Load configuration
        self._load_config()
        
        # Initialize detectors
        self._initialize_detectors()
    
    def _load_config(self):
        """Load configuration from the config file."""
        config = get_config().get("processing", {}).get("jamming_detection", {})
        self.config.update(config)
        
        # Update parameters from config
        self.enabled = self.config.get("enabled", True)
        self.sample_size = self.config.get("sample_size", 1024)
        self.detection_interval = self.config.get("detection_interval", 1.0)
    
    def _initialize_detectors(self):
        """Initialize the jamming detectors based on configuration."""
        # Energy detector
        energy_config = self.config.get("energy_detector", {})
        if energy_config.get("enabled", True):
            self.energy_detector = EnergyDetector(
                threshold=energy_config.get("threshold", 10.0),
                window_size=energy_config.get("window_size", self.sample_size)
            )
        
        # Spectral detector
        spectral_config = self.config.get("spectral_detector", {})
        if spectral_config.get("enabled", True):
            self.spectral_detector = SpectralDetector(
                threshold=spectral_config.get("threshold", 10.0),
                fft_size=spectral_config.get("fft_size", self.sample_size),
                overlap=spectral_config.get("overlap", self.sample_size // 2)
            )
        
        # Cyclostationary detector
        cyclo_config = self.config.get("cyclostationary_detector", {})
        if cyclo_config.get("enabled", False):
            self.cyclostationary_detector = CyclostationaryDetector(
                threshold=cyclo_config.get("threshold", 0.8),
                window_size=cyclo_config.get("window_size", self.sample_size),
                num_lags=cyclo_config.get("num_lags", 128)
            )
    
    async def initialize(self) -> bool:
        """
        Initialize the jamming detector.
        
        Returns:
            bool: True if initialization was successful, False otherwise.
        """
        logger.info("Initializing jamming detector")
        
        try:
            # Start detection task
            if self.enabled:
                self.detection_task = asyncio.create_task(self._detection_loop())
            
            return True
        except Exception as e:
            logger.error(f"Failed to initialize jamming detector: {str(e)}")
            return False
    
    async def shutdown(self) -> bool:
        """
        Shutdown the jamming detector.
        
        Returns:
            bool: True if shutdown was successful, False otherwise.
        """
        logger.info("Shutting down jamming detector")
        
        try:
            # Cancel detection task
            if self.detection_task:
                self.detection_task.cancel()
                try:
                    await self.detection_task
                except asyncio.CancelledError:
                    pass
                self.detection_task = None
            
            return True
        except Exception as e:
            logger.error(f"Failed to shut down jamming detector: {str(e)}")
            return False
    
    async def configure(self, config: Dict[str, Any]) -> bool:
        """
        Configure the jamming detector.
        
        Args:
            config: Configuration parameters.
            
        Returns:
            bool: True if configuration was successful, False otherwise.
        """
        logger.info("Configuring jamming detector")
        
        try:
            # Update configuration
            self.config.update(config)
            
            # Update enabled state
            if "enabled" in config:
                self.enabled = config["enabled"]
            
            # Update parameters
            if "sample_size" in config:
                self.sample_size = config["sample_size"]
            
            if "detection_interval" in config:
                self.detection_interval = config["detection_interval"]
            
            # Reinitialize detectors
            self._initialize_detectors()
            
            # Restart detection task if enabled
            if self.detection_task:
                self.detection_task.cancel()
                try:
                    await self.detection_task
                except asyncio.CancelledError:
                    pass
                self.detection_task = None
            
            if self.enabled:
                self.detection_task = asyncio.create_task(self._detection_loop())
            
            return True
        except Exception as e:
            logger.error(f"Failed to configure jamming detector: {str(e)}")
            return False
    
    async def detect(self, samples: np.ndarray) -> Dict[str, Any]:
        """
        Detect jamming signals in the provided samples.
        
        Args:
            samples: Complex samples to analyze.
            
        Returns:
            Dict[str, Any]: Detection results.
        """
        if not self.enabled:
            return self.last_detection
        
        try:
            # Initialize results
            results = {
                "is_jamming": False,
                "jamming_type": JammingType.UNKNOWN,
                "confidence": 0.0,
                "snr_db": 0.0,
                "timestamp": time.time()
            }
            
            # Run energy detector
            if self.energy_detector:
                energy_results = self.energy_detector.detect(samples)
                results["is_jamming"] |= energy_results["is_jamming"]
                if energy_results["is_jamming"]:
                    results["jamming_type"] = energy_results["jamming_type"]
                    results["snr_db"] = energy_results["snr_db"]
                    results["confidence"] = 0.6  # Base confidence for energy detector
            
            # Run spectral detector
            if self.spectral_detector:
                spectral_results = self.spectral_detector.detect(samples)
                results["is_jamming"] |= spectral_results["is_jamming"]
                if spectral_results["is_jamming"]:
                    # If spectral detector has higher confidence, use its results
                    if spectral_results["snr_db"] > results["snr_db"]:
                        results["jamming_type"] = spectral_results["jamming_type"]
                        results["snr_db"] = spectral_results["snr_db"]
                        results["confidence"] = 0.8  # Higher confidence for spectral detector
            
            # Run cyclostationary detector
            if self.cyclostationary_detector:
                cyclo_results = self.cyclostationary_detector.detect(samples)
                results["is_jamming"] |= cyclo_results["is_jamming"]
                if cyclo_results["is_jamming"]:
                    # If cyclostationary detector detects repeater/meaconer, use its results
                    if cyclo_results["jamming_type"] in [JammingType.REPEATER, JammingType.MEACONER]:
                        results["jamming_type"] = cyclo_results["jamming_type"]
                        results["confidence"] = 0.9  # Highest confidence for cyclostationary detector
            
            # Update last detection
            self.last_detection = results
            
            return results
        except Exception as e:
            logger.error(f"Failed to detect jamming: {str(e)}")
            return self.last_detection
    
    async def get_status(self) -> Dict[str, Any]:
        """
        Get the status of the jamming detector.
        
        Returns:
            Dict[str, Any]: Status information.
        """
        return {
            "enabled": self.enabled,
            "sample_size": self.sample_size,
            "detection_interval": self.detection_interval,
            "energy_detector_enabled": self.energy_detector is not None,
            "spectral_detector_enabled": self.spectral_detector is not None,
            "cyclostationary_detector_enabled": self.cyclostationary_detector is not None,
            "last_detection": {
                "is_jamming": self.last_detection["is_jamming"],
                "jamming_type": self.last_detection["jamming_type"].name,
                "confidence": self.last_detection["confidence"],
                "snr_db": self.last_detection["snr_db"],
                "timestamp": self.last_detection["timestamp"]
            }
        }
    
    async def _detection_loop(self):
        """Background task for continuous jamming detection."""
        logger.info("Starting jamming detection loop")
        
        while True:
            try:
                # Get samples from SDR
                samples = await self.sdr.get_samples(self.sample_size)
                
                # Detect jamming
                await self.detect(samples)
                
                # Sleep for detection interval
                await asyncio.sleep(self.detection_interval)
            except asyncio.CancelledError:
                logger.info("Jamming detection loop cancelled")
                break
            except Exception as e:
                logger.error(f"Error in jamming detection loop: {str(e)}")
                await asyncio.sleep(1)
