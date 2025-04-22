"""
Signal analyzer service for the SIGINT service.

This service is responsible for analyzing signals, including:
- Classifying signal types
- Decoding signal content
- Extracting signal metadata
- Identifying signal features
"""

import logging
import numpy as np
from typing import Dict, List, Optional, Any, Tuple
from scipy import signal as scipy_signal

from core.config import settings

logger = logging.getLogger(__name__)

class SignalAnalyzer:
    """Signal analyzer service."""
    
    def __init__(self):
        """Initialize the signal analyzer."""
        self.confidence_threshold = settings.SIGNAL_ANALYSIS_CONFIDENCE_THRESHOLD
    
    async def classify_signal(self, signal_data: np.ndarray, sample_rate: int, center_frequency: float) -> Dict[str, Any]:
        """
        Classify a signal based on its characteristics.
        
        Args:
            signal_data: Raw signal data (IQ samples)
            sample_rate: Sample rate in Hz
            center_frequency: Center frequency in Hz
            
        Returns:
            Classification results
        """
        # Extract signal features
        features = self._extract_signal_features(signal_data, sample_rate)
        
        # Classify signal type
        signal_type, confidence = self._classify_signal_type(features)
        
        # Determine modulation type
        modulation = self._determine_modulation(signal_data, sample_rate)
        
        # Estimate bandwidth
        bandwidth = self._estimate_bandwidth(signal_data, sample_rate)
        
        # Return classification results
        return {
            "signal_type": signal_type,
            "confidence": confidence,
            "modulation": modulation,
            "bandwidth": bandwidth,
            "center_frequency": center_frequency,
            "features": features
        }
    
    async def decode_signal(self, signal_data: np.ndarray, sample_rate: int, signal_type: str) -> Dict[str, Any]:
        """
        Decode a signal to extract its content.
        
        Args:
            signal_data: Raw signal data (IQ samples)
            sample_rate: Sample rate in Hz
            signal_type: Type of signal
            
        Returns:
            Decoding results
        """
        # In a real implementation, this would use signal-specific decoders
        # For now, we'll just return a placeholder
        
        # Demodulate signal
        if signal_type == "AM":
            demodulated = self._demodulate_am(signal_data)
        elif signal_type == "FM":
            demodulated = self._demodulate_fm(signal_data, sample_rate)
        else:
            demodulated = np.abs(signal_data)  # Simple envelope detection
        
        # Extract content (in a real implementation, this would decode the actual content)
        content = "Simulated signal content"
        
        # Return decoding results
        return {
            "signal_type": signal_type,
            "success": True,
            "content_type": "text",
            "content": content,
            "confidence": 0.8
        }
    
    async def extract_metadata(self, signal_data: np.ndarray, sample_rate: int, signal_type: str) -> Dict[str, Any]:
        """
        Extract metadata from a signal.
        
        Args:
            signal_data: Raw signal data (IQ samples)
            sample_rate: Sample rate in Hz
            signal_type: Type of signal
            
        Returns:
            Metadata extraction results
        """
        # In a real implementation, this would extract signal-specific metadata
        # For now, we'll just return a placeholder
        
        # Extract basic signal properties
        duration = len(signal_data) / sample_rate
        power = np.mean(np.abs(signal_data)**2)
        peak_power = np.max(np.abs(signal_data)**2)
        
        # Extract signal-specific metadata
        if signal_type == "FM":
            # For FM, extract frequency deviation
            demodulated = self._demodulate_fm(signal_data, sample_rate)
            deviation = np.std(demodulated) * sample_rate / (2 * np.pi)
            metadata = {
                "frequency_deviation": deviation,
                "modulation_index": deviation / 1000  # Assuming 1 kHz modulation
            }
        elif signal_type == "AM":
            # For AM, extract modulation depth
            demodulated = self._demodulate_am(signal_data)
            modulation_depth = (np.max(demodulated) - np.min(demodulated)) / (np.max(demodulated) + np.min(demodulated))
            metadata = {
                "modulation_depth": modulation_depth
            }
        else:
            metadata = {}
        
        # Return metadata extraction results
        return {
            "signal_type": signal_type,
            "duration": duration,
            "power": power,
            "peak_power": peak_power,
            "metadata": metadata
        }
    
    async def match_signal_profile(self, features: Dict[str, Any], profiles: List[Dict[str, Any]]) -> Tuple[Optional[Dict[str, Any]], float]:
        """
        Match signal features against known signal profiles.
        
        Args:
            features: Signal features
            profiles: Known signal profiles
            
        Returns:
            Tuple of (matching profile, confidence) or (None, 0.0) if no match
        """
        if not profiles:
            return None, 0.0
        
        best_match = None
        best_confidence = 0.0
        
        for profile in profiles:
            # Calculate match confidence
            confidence = self._calculate_profile_match(features, profile["features"])
            
            # Update best match if this profile is better
            if confidence > best_confidence:
                best_match = profile
                best_confidence = confidence
        
        # Return best match if confidence is above threshold
        if best_confidence >= self.confidence_threshold:
            return best_match, best_confidence
        else:
            return None, 0.0
    
    def _extract_signal_features(self, signal_data: np.ndarray, sample_rate: int) -> Dict[str, Any]:
        """
        Extract features from a signal.
        
        Args:
            signal_data: Raw signal data (IQ samples)
            sample_rate: Sample rate in Hz
            
        Returns:
            Signal features
        """
        # Calculate power spectral density
        f, psd = scipy_signal.welch(signal_data, fs=sample_rate, nperseg=1024, scaling='spectrum')
        
        # Calculate time-domain features
        amplitude = np.abs(signal_data)
        phase = np.angle(signal_data)
        
        # Calculate statistical features
        mean_amplitude = np.mean(amplitude)
        std_amplitude = np.std(amplitude)
        mean_phase = np.mean(phase)
        std_phase = np.std(phase)
        
        # Calculate spectral features
        peak_frequency = f[np.argmax(psd)]
        bandwidth = self._estimate_bandwidth(signal_data, sample_rate)
        spectral_flatness = scipy_signal.spectral_flatness(psd)
        
        # Calculate modulation features
        am_depth = self._calculate_am_depth(signal_data)
        fm_deviation = self._calculate_fm_deviation(signal_data, sample_rate)
        
        # Return features
        return {
            "time_domain": {
                "mean_amplitude": float(mean_amplitude),
                "std_amplitude": float(std_amplitude),
                "mean_phase": float(mean_phase),
                "std_phase": float(std_phase)
            },
            "spectral": {
                "peak_frequency": float(peak_frequency),
                "bandwidth": float(bandwidth),
                "spectral_flatness": float(spectral_flatness)
            },
            "modulation": {
                "am_depth": float(am_depth),
                "fm_deviation": float(fm_deviation)
            }
        }
    
    def _classify_signal_type(self, features: Dict[str, Any]) -> Tuple[str, float]:
        """
        Classify signal type based on features.
        
        Args:
            features: Signal features
            
        Returns:
            Tuple of (signal type, confidence)
        """
        # In a real implementation, this would use a machine learning model
        # For now, we'll use a simple rule-based approach
        
        # Extract relevant features
        spectral_flatness = features["spectral"]["spectral_flatness"]
        am_depth = features["modulation"]["am_depth"]
        fm_deviation = features["modulation"]["fm_deviation"]
        
        # Classify signal type
        if fm_deviation > 0.1:
            return "FM", 0.8
        elif am_depth > 0.2:
            return "AM", 0.8
        elif spectral_flatness > 0.5:
            return "DIGITAL", 0.7
        else:
            return "UNKNOWN", 0.5
    
    def _determine_modulation(self, signal_data: np.ndarray, sample_rate: int) -> str:
        """
        Determine modulation type of a signal.
        
        Args:
            signal_data: Raw signal data (IQ samples)
            sample_rate: Sample rate in Hz
            
        Returns:
            Modulation type
        """
        # In a real implementation, this would use modulation recognition algorithms
        # For now, we'll use a simple approach based on the signal's properties
        
        # Calculate amplitude and phase
        amplitude = np.abs(signal_data)
        phase = np.unwrap(np.angle(signal_data))
        
        # Calculate amplitude and phase variations
        amplitude_var = np.var(amplitude)
        phase_var = np.var(np.diff(phase))
        
        # Determine modulation type
        if phase_var > 0.1 and amplitude_var < 0.01:
            return "PSK"
        elif phase_var > 0.1 and amplitude_var > 0.01:
            return "QAM"
        elif phase_var < 0.01 and amplitude_var > 0.01:
            return "AM"
        elif phase_var > 0.01 and amplitude_var < 0.01:
            return "FM"
        else:
            return "UNKNOWN"
    
    def _estimate_bandwidth(self, signal_data: np.ndarray, sample_rate: int) -> float:
        """
        Estimate bandwidth of a signal.
        
        Args:
            signal_data: Raw signal data (IQ samples)
            sample_rate: Sample rate in Hz
            
        Returns:
            Bandwidth in Hz
        """
        # Calculate power spectral density
        f, psd = scipy_signal.welch(signal_data, fs=sample_rate, nperseg=1024, scaling='spectrum')
        
        # Normalize PSD
        psd_norm = psd / np.max(psd)
        
        # Find frequencies where PSD is above threshold
        threshold = 0.1
        mask = psd_norm > threshold
        
        if not np.any(mask):
            return 0.0
        
        # Calculate bandwidth
        f_min = f[mask][0]
        f_max = f[mask][-1]
        bandwidth = f_max - f_min
        
        return bandwidth
    
    def _calculate_am_depth(self, signal_data: np.ndarray) -> float:
        """
        Calculate AM modulation depth.
        
        Args:
            signal_data: Raw signal data (IQ samples)
            
        Returns:
            AM modulation depth
        """
        # Calculate amplitude
        amplitude = np.abs(signal_data)
        
        # Calculate modulation depth
        if np.mean(amplitude) == 0:
            return 0.0
        
        modulation_depth = (np.max(amplitude) - np.min(amplitude)) / (np.max(amplitude) + np.min(amplitude))
        
        return modulation_depth
    
    def _calculate_fm_deviation(self, signal_data: np.ndarray, sample_rate: int) -> float:
        """
        Calculate FM frequency deviation.
        
        Args:
            signal_data: Raw signal data (IQ samples)
            sample_rate: Sample rate in Hz
            
        Returns:
            FM frequency deviation in Hz
        """
        # Calculate phase
        phase = np.unwrap(np.angle(signal_data))
        
        # Calculate instantaneous frequency
        inst_freq = np.diff(phase) * sample_rate / (2 * np.pi)
        
        # Calculate frequency deviation
        if len(inst_freq) == 0:
            return 0.0
        
        deviation = np.std(inst_freq)
        
        return deviation
    
    def _demodulate_am(self, signal_data: np.ndarray) -> np.ndarray:
        """
        Demodulate AM signal.
        
        Args:
            signal_data: Raw signal data (IQ samples)
            
        Returns:
            Demodulated signal
        """
        # Simple envelope detection
        return np.abs(signal_data)
    
    def _demodulate_fm(self, signal_data: np.ndarray, sample_rate: int) -> np.ndarray:
        """
        Demodulate FM signal.
        
        Args:
            signal_data: Raw signal data (IQ samples)
            sample_rate: Sample rate in Hz
            
        Returns:
            Demodulated signal
        """
        # Calculate phase
        phase = np.unwrap(np.angle(signal_data))
        
        # Calculate instantaneous frequency
        inst_freq = np.diff(phase) * sample_rate / (2 * np.pi)
        
        # Pad to original length
        inst_freq = np.pad(inst_freq, (0, 1), mode='edge')
        
        return inst_freq
    
    def _calculate_profile_match(self, features: Dict[str, Any], profile_features: Dict[str, Any]) -> float:
        """
        Calculate match confidence between signal features and profile features.
        
        Args:
            features: Signal features
            profile_features: Profile features
            
        Returns:
            Match confidence (0.0 to 1.0)
        """
        # In a real implementation, this would use a more sophisticated matching algorithm
        # For now, we'll use a simple similarity metric
        
        # Calculate similarity for each feature category
        time_domain_similarity = self._calculate_dict_similarity(
            features.get("time_domain", {}),
            profile_features.get("time_domain", {})
        )
        
        spectral_similarity = self._calculate_dict_similarity(
            features.get("spectral", {}),
            profile_features.get("spectral", {})
        )
        
        modulation_similarity = self._calculate_dict_similarity(
            features.get("modulation", {}),
            profile_features.get("modulation", {})
        )
        
        # Calculate overall similarity
        similarity = (time_domain_similarity + spectral_similarity + modulation_similarity) / 3
        
        return similarity
    
    def _calculate_dict_similarity(self, dict1: Dict[str, float], dict2: Dict[str, float]) -> float:
        """
        Calculate similarity between two dictionaries of features.
        
        Args:
            dict1: First dictionary
            dict2: Second dictionary
            
        Returns:
            Similarity (0.0 to 1.0)
        """
        if not dict1 or not dict2:
            return 0.0
        
        # Find common keys
        common_keys = set(dict1.keys()) & set(dict2.keys())
        
        if not common_keys:
            return 0.0
        
        # Calculate similarity for each common key
        similarities = []
        for key in common_keys:
            value1 = dict1[key]
            value2 = dict2[key]
            
            # Avoid division by zero
            if value1 == 0 and value2 == 0:
                similarities.append(1.0)
            elif value1 == 0 or value2 == 0:
                similarities.append(0.0)
            else:
                # Calculate relative difference
                diff = abs(value1 - value2) / max(abs(value1), abs(value2))
                # Convert to similarity
                similarities.append(max(0.0, 1.0 - diff))
        
        # Calculate average similarity
        return sum(similarities) / len(similarities)
