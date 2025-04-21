"""
Direction of Arrival (DoA) estimation algorithms.

This module provides algorithms for estimating the direction of arrival of jamming signals.
"""

import logging
import numpy as np
from typing import Dict, List, Optional, Tuple, Any, Union
import asyncio

from anti_jamming_service.hardware.interfaces import IKrakenSDRInterface
from anti_jamming_service.utils.config import get_config

logger = logging.getLogger(__name__)


class MUSICAlgorithm:
    """
    MUSIC (MUltiple SIgnal Classification) algorithm for DoA estimation.
    
    This class implements the MUSIC algorithm for estimating the direction of arrival
    of signals using an antenna array.
    """
    
    def __init__(self, num_antennas: int = 5, num_sources: int = 1, angle_resolution: float = 1.0):
        """
        Initialize the MUSIC algorithm.
        
        Args:
            num_antennas: Number of antennas in the array.
            num_sources: Number of signal sources to detect.
            angle_resolution: Angular resolution in degrees.
        """
        self.num_antennas = num_antennas
        self.num_sources = num_sources
        self.angle_resolution = angle_resolution
        
        # Generate angle grid
        self.angles = np.arange(0, 360, angle_resolution)
        
        # Generate steering vectors
        self.steering_vectors = self._generate_steering_vectors()
    
    def _generate_steering_vectors(self) -> np.ndarray:
        """
        Generate steering vectors for the antenna array.
        
        Returns:
            np.ndarray: Steering vectors for each angle.
        """
        # Assume uniform linear array with half-wavelength spacing
        # In a real implementation, this would be based on the actual antenna array geometry
        
        # Convert angles to radians
        angles_rad = np.deg2rad(self.angles)
        
        # Generate steering vectors
        steering_vectors = np.exp(1j * np.pi * np.outer(np.arange(self.num_antennas), np.sin(angles_rad)))
        
        return steering_vectors
    
    def estimate(self, samples: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Estimate the direction of arrival using the MUSIC algorithm.
        
        Args:
            samples: Complex samples from the antenna array (num_antennas x num_samples).
            
        Returns:
            Tuple[np.ndarray, np.ndarray]: Estimated angles and corresponding spectrum.
        """
        # Compute covariance matrix
        cov_matrix = np.cov(samples)
        
        # Compute eigenvalues and eigenvectors
        eigenvalues, eigenvectors = np.linalg.eigh(cov_matrix)
        
        # Sort eigenvalues and eigenvectors
        idx = eigenvalues.argsort()
        eigenvalues = eigenvalues[idx]
        eigenvectors = eigenvectors[:, idx]
        
        # Separate signal and noise subspaces
        noise_eigenvectors = eigenvectors[:, :self.num_antennas - self.num_sources]
        
        # Compute MUSIC spectrum
        spectrum = np.zeros(len(self.angles))
        for i, angle in enumerate(self.angles):
            steering_vector = self.steering_vectors[:, i]
            spectrum[i] = 1.0 / np.abs(np.dot(np.dot(steering_vector.conj(), noise_eigenvectors),
                                             np.dot(noise_eigenvectors.conj().T, steering_vector)))
        
        # Normalize spectrum
        spectrum = spectrum / np.max(spectrum)
        
        # Find peaks in spectrum
        peaks = []
        for i in range(1, len(spectrum) - 1):
            if spectrum[i] > spectrum[i-1] and spectrum[i] > spectrum[i+1]:
                peaks.append((self.angles[i], spectrum[i]))
        
        # Sort peaks by amplitude
        peaks.sort(key=lambda x: x[1], reverse=True)
        
        # Get top peaks
        top_peaks = peaks[:self.num_sources]
        
        # Extract angles and amplitudes
        estimated_angles = np.array([angle for angle, _ in top_peaks])
        estimated_amplitudes = np.array([amplitude for _, amplitude in top_peaks])
        
        return estimated_angles, estimated_amplitudes


class ESPRITAlgorithm:
    """
    ESPRIT (Estimation of Signal Parameters via Rotational Invariance Techniques) algorithm for DoA estimation.
    
    This class implements the ESPRIT algorithm for estimating the direction of arrival
    of signals using an antenna array.
    """
    
    def __init__(self, num_antennas: int = 5, num_sources: int = 1):
        """
        Initialize the ESPRIT algorithm.
        
        Args:
            num_antennas: Number of antennas in the array.
            num_sources: Number of signal sources to detect.
        """
        self.num_antennas = num_antennas
        self.num_sources = num_sources
    
    def estimate(self, samples: np.ndarray) -> np.ndarray:
        """
        Estimate the direction of arrival using the ESPRIT algorithm.
        
        Args:
            samples: Complex samples from the antenna array (num_antennas x num_samples).
            
        Returns:
            np.ndarray: Estimated angles.
        """
        # Compute covariance matrix
        cov_matrix = np.cov(samples)
        
        # Compute eigenvalues and eigenvectors
        eigenvalues, eigenvectors = np.linalg.eigh(cov_matrix)
        
        # Sort eigenvalues and eigenvectors
        idx = eigenvalues.argsort()[::-1]  # Descending order
        eigenvalues = eigenvalues[idx]
        eigenvectors = eigenvectors[:, idx]
        
        # Separate signal subspace
        signal_eigenvectors = eigenvectors[:, :self.num_sources]
        
        # Define selection matrices
        selection1 = np.zeros((self.num_antennas - 1, self.num_antennas))
        selection2 = np.zeros((self.num_antennas - 1, self.num_antennas))
        
        for i in range(self.num_antennas - 1):
            selection1[i, i] = 1
            selection2[i, i + 1] = 1
        
        # Apply selection matrices
        s1 = np.dot(selection1, signal_eigenvectors)
        s2 = np.dot(selection2, signal_eigenvectors)
        
        # Compute pseudo-inverse
        s1_pinv = np.linalg.pinv(s1)
        
        # Compute rotation matrix
        phi = np.dot(s1_pinv, s2)
        
        # Compute eigenvalues of rotation matrix
        phi_eigenvalues = np.linalg.eigvals(phi)
        
        # Compute angles
        estimated_angles = np.rad2deg(np.arcsin(np.angle(phi_eigenvalues) / np.pi))
        
        return estimated_angles


class DoAEstimator:
    """
    Direction of Arrival (DoA) estimator.
    
    This class combines multiple DoA estimation algorithms to provide
    robust direction finding capabilities.
    """
    
    def __init__(self, sdr_interface: IKrakenSDRInterface, config: Optional[Dict[str, Any]] = None):
        """
        Initialize the DoA estimator.
        
        Args:
            sdr_interface: KrakenSDR interface for getting samples.
            config: Optional configuration parameters.
        """
        self.sdr = sdr_interface
        self.config = config or {}
        self.music = None
        self.esprit = None
        self.enabled = True
        self.num_antennas = 5  # KrakenSDR has 5 channels
        self.num_sources = 1
        self.angle_resolution = 1.0
        self.sample_size = 1024
        self.last_estimate = {"azimuth": 0.0, "elevation": 0.0, "confidence": 0.0}
        
        # Load configuration
        self._load_config()
        
        # Initialize algorithms
        self._initialize_algorithms()
    
    def _load_config(self):
        """Load configuration from the config file."""
        config = get_config().get("processing", {}).get("doa_estimation", {})
        self.config.update(config)
        
        # Update parameters from config
        self.enabled = self.config.get("enabled", True)
        self.num_sources = self.config.get("num_sources", 1)
        self.angle_resolution = self.config.get("angle_resolution", 1.0)
        self.sample_size = self.config.get("sample_size", 1024)
    
    def _initialize_algorithms(self):
        """Initialize the DoA estimation algorithms based on configuration."""
        # MUSIC algorithm
        music_config = self.config.get("music", {})
        if music_config.get("enabled", True):
            self.music = MUSICAlgorithm(
                num_antennas=self.num_antennas,
                num_sources=self.num_sources,
                angle_resolution=self.angle_resolution
            )
        
        # ESPRIT algorithm
        esprit_config = self.config.get("esprit", {})
        if esprit_config.get("enabled", False):
            self.esprit = ESPRITAlgorithm(
                num_antennas=self.num_antennas,
                num_sources=self.num_sources
            )
    
    async def estimate(self) -> Dict[str, Any]:
        """
        Estimate the direction of arrival of jamming signals.
        
        Returns:
            Dict[str, Any]: Estimated direction (azimuth, elevation) and confidence.
        """
        if not self.enabled or not self.sdr:
            return self.last_estimate
        
        try:
            # Get samples from all channels
            samples = np.zeros((self.num_antennas, self.sample_size), dtype=np.complex128)
            for i in range(self.num_antennas):
                samples[i] = await self.sdr.get_channel_samples(i, self.sample_size)
            
            # Estimate using MUSIC algorithm
            if self.music:
                angles, amplitudes = self.music.estimate(samples)
                if len(angles) > 0:
                    azimuth = angles[0]
                    confidence = amplitudes[0]
                    
                    # Update last estimate
                    self.last_estimate = {
                        "azimuth": azimuth,
                        "elevation": 0.0,  # KrakenSDR only provides azimuth
                        "confidence": float(confidence)
                    }
            
            # Estimate using ESPRIT algorithm
            if self.esprit:
                angles = self.esprit.estimate(samples)
                if len(angles) > 0:
                    azimuth = angles[0]
                    
                    # Update last estimate if MUSIC not used
                    if not self.music:
                        self.last_estimate = {
                            "azimuth": azimuth,
                            "elevation": 0.0,  # KrakenSDR only provides azimuth
                            "confidence": 0.8  # Default confidence
                        }
            
            return self.last_estimate
        except Exception as e:
            logger.error(f"Failed to estimate DoA: {str(e)}")
            return self.last_estimate
    
    def configure(self, config: Dict[str, Any]) -> bool:
        """
        Configure the DoA estimator.
        
        Args:
            config: Configuration parameters.
            
        Returns:
            bool: True if configuration was successful, False otherwise.
        """
        try:
            # Update configuration
            self.config.update(config)
            
            # Update enabled state
            if "enabled" in config:
                self.enabled = config["enabled"]
            
            # Update parameters
            if "num_sources" in config:
                self.num_sources = config["num_sources"]
            
            if "angle_resolution" in config:
                self.angle_resolution = config["angle_resolution"]
            
            if "sample_size" in config:
                self.sample_size = config["sample_size"]
            
            # Reinitialize algorithms
            self._initialize_algorithms()
            
            return True
        except Exception as e:
            logger.error(f"Failed to configure DoA estimator: {str(e)}")
            return False
    
    def get_status(self) -> Dict[str, Any]:
        """
        Get the status of the DoA estimator.
        
        Returns:
            Dict[str, Any]: Status information.
        """
        return {
            "enabled": self.enabled,
            "num_antennas": self.num_antennas,
            "num_sources": self.num_sources,
            "angle_resolution": self.angle_resolution,
            "sample_size": self.sample_size,
            "music_enabled": self.music is not None,
            "esprit_enabled": self.esprit is not None,
            "last_estimate": self.last_estimate
        }
