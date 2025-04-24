"""
Waveform generator service for the EW service.

This service is responsible for generating various types of waveforms for electronic attacks, including:
- Noise waveforms (white, pink, etc.)
- Tone waveforms (single tone, multi-tone)
- Sweep waveforms (linear, exponential)
- Pulse waveforms
- Custom waveforms
"""

import logging
import numpy as np
from typing import Dict, List, Optional, Any, Tuple
import scipy.signal as signal

logger = logging.getLogger(__name__)

class WaveformGenerator:
    """Waveform generator service."""
    
    def __init__(self):
        """Initialize the waveform generator."""
        pass
    
    def generate_noise_waveform(
        self,
        waveform_type: str,
        sample_rate: int,
        duration: float,
        amplitude: float = 1.0,
        seed: Optional[int] = None
    ) -> Tuple[np.ndarray, Dict[str, Any]]:
        """
        Generate a noise waveform.
        
        Args:
            waveform_type: Type of noise (white, pink, etc.)
            sample_rate: Sample rate in Hz
            duration: Duration in seconds
            amplitude: Amplitude (0.0 to 1.0)
            seed: Random seed for reproducibility
            
        Returns:
            Tuple of (waveform samples, metadata)
        """
        # Set random seed if provided
        if seed is not None:
            np.random.seed(seed)
        
        # Calculate number of samples
        num_samples = int(sample_rate * duration)
        
        # Generate waveform based on type
        if waveform_type == "white":
            # Generate white noise
            samples = np.random.normal(0, 1, num_samples)
        
        elif waveform_type == "pink":
            # Generate white noise first
            white_noise = np.random.normal(0, 1, num_samples)
            
            # Apply pink filter (1/f spectrum)
            # Create filter coefficients for pink noise
            b = [0.049922035, -0.095993537, 0.050612699, -0.004408786]
            a = [1, -2.494956002, 2.017265875, -0.522189400]
            samples = signal.lfilter(b, a, white_noise)
        
        elif waveform_type == "brown":
            # Generate white noise first
            white_noise = np.random.normal(0, 1, num_samples)
            
            # Apply brown filter (1/f^2 spectrum)
            # Simple implementation: cumulative sum of white noise
            samples = np.cumsum(white_noise)
            
            # Normalize
            samples = samples / np.max(np.abs(samples))
        
        elif waveform_type == "blue":
            # Generate white noise first
            white_noise = np.random.normal(0, 1, num_samples)
            
            # Apply blue filter (f spectrum)
            # Simple implementation: differentiate white noise
            samples = np.diff(white_noise, prepend=0)
            
            # Normalize
            samples = samples / np.max(np.abs(samples))
        
        else:
            raise ValueError(f"Unknown noise type: {waveform_type}")
        
        # Normalize and apply amplitude
        samples = amplitude * samples / np.max(np.abs(samples))
        
        # Create metadata
        metadata = {
            "waveform_type": f"noise_{waveform_type}",
            "sample_rate": sample_rate,
            "duration": duration,
            "amplitude": amplitude,
            "num_samples": num_samples,
            "seed": seed
        }
        
        return samples, metadata
    
    def generate_tone_waveform(
        self,
        frequencies: List[float],
        amplitudes: Optional[List[float]] = None,
        phases: Optional[List[float]] = None,
        sample_rate: int = 44100,
        duration: float = 1.0
    ) -> Tuple[np.ndarray, Dict[str, Any]]:
        """
        Generate a tone waveform (single or multi-tone).
        
        Args:
            frequencies: List of frequencies in Hz
            amplitudes: List of amplitudes (0.0 to 1.0), one per frequency
            phases: List of phases in radians, one per frequency
            sample_rate: Sample rate in Hz
            duration: Duration in seconds
            
        Returns:
            Tuple of (waveform samples, metadata)
        """
        # Calculate number of samples
        num_samples = int(sample_rate * duration)
        
        # Create time array
        t = np.arange(num_samples) / sample_rate
        
        # Default amplitudes and phases if not provided
        if amplitudes is None:
            amplitudes = [1.0 / len(frequencies)] * len(frequencies)
        if phases is None:
            phases = [0.0] * len(frequencies)
        
        # Ensure lengths match
        if len(frequencies) != len(amplitudes) or len(frequencies) != len(phases):
            raise ValueError("Frequencies, amplitudes, and phases must have the same length")
        
        # Generate waveform
        samples = np.zeros(num_samples)
        for freq, amp, phase in zip(frequencies, amplitudes, phases):
            samples += amp * np.sin(2 * np.pi * freq * t + phase)
        
        # Normalize
        if np.max(np.abs(samples)) > 0:
            samples = samples / np.max(np.abs(samples))
        
        # Create metadata
        metadata = {
            "waveform_type": "tone",
            "sample_rate": sample_rate,
            "duration": duration,
            "frequencies": frequencies,
            "amplitudes": amplitudes,
            "phases": phases,
            "num_samples": num_samples
        }
        
        return samples, metadata
    
    def generate_sweep_waveform(
        self,
        start_frequency: float,
        end_frequency: float,
        sweep_type: str = "linear",
        sample_rate: int = 44100,
        duration: float = 1.0,
        amplitude: float = 1.0
    ) -> Tuple[np.ndarray, Dict[str, Any]]:
        """
        Generate a frequency sweep waveform.
        
        Args:
            start_frequency: Start frequency in Hz
            end_frequency: End frequency in Hz
            sweep_type: Type of sweep (linear, exponential, logarithmic)
            sample_rate: Sample rate in Hz
            duration: Duration in seconds
            amplitude: Amplitude (0.0 to 1.0)
            
        Returns:
            Tuple of (waveform samples, metadata)
        """
        # Calculate number of samples
        num_samples = int(sample_rate * duration)
        
        # Create time array
        t = np.arange(num_samples) / sample_rate
        
        # Generate frequency array based on sweep type
        if sweep_type == "linear":
            # Linear sweep
            freq = np.linspace(start_frequency, end_frequency, num_samples)
        
        elif sweep_type == "exponential" or sweep_type == "logarithmic":
            # Exponential/logarithmic sweep
            if start_frequency <= 0 or end_frequency <= 0:
                raise ValueError("Frequencies must be positive for exponential/logarithmic sweep")
            
            log_start = np.log(start_frequency)
            log_end = np.log(end_frequency)
            freq = np.exp(np.linspace(log_start, log_end, num_samples))
        
        else:
            raise ValueError(f"Unknown sweep type: {sweep_type}")
        
        # Calculate instantaneous phase by integrating frequency
        phase = 2 * np.pi * np.cumsum(freq) / sample_rate
        
        # Generate waveform
        samples = amplitude * np.sin(phase)
        
        # Create metadata
        metadata = {
            "waveform_type": f"sweep_{sweep_type}",
            "sample_rate": sample_rate,
            "duration": duration,
            "start_frequency": start_frequency,
            "end_frequency": end_frequency,
            "amplitude": amplitude,
            "num_samples": num_samples
        }
        
        return samples, metadata
    
    def generate_pulse_waveform(
        self,
        pulse_width: float,
        pulse_repetition_interval: float,
        frequency: float = 0.0,
        sample_rate: int = 44100,
        duration: float = 1.0,
        amplitude: float = 1.0,
        pulse_shape: str = "rectangular"
    ) -> Tuple[np.ndarray, Dict[str, Any]]:
        """
        Generate a pulse waveform.
        
        Args:
            pulse_width: Pulse width in seconds
            pulse_repetition_interval: Pulse repetition interval in seconds
            frequency: Carrier frequency in Hz (0 for baseband)
            sample_rate: Sample rate in Hz
            duration: Duration in seconds
            amplitude: Amplitude (0.0 to 1.0)
            pulse_shape: Shape of pulse (rectangular, gaussian, triangular)
            
        Returns:
            Tuple of (waveform samples, metadata)
        """
        # Calculate number of samples
        num_samples = int(sample_rate * duration)
        
        # Create time array
        t = np.arange(num_samples) / sample_rate
        
        # Generate pulse train
        pulse_train = np.zeros(num_samples)
        
        # Calculate pulse width and PRI in samples
        pulse_width_samples = int(pulse_width * sample_rate)
        pri_samples = int(pulse_repetition_interval * sample_rate)
        
        # Generate pulse shape
        if pulse_shape == "rectangular":
            pulse = np.ones(pulse_width_samples)
        
        elif pulse_shape == "gaussian":
            # Gaussian pulse
            x = np.linspace(-3, 3, pulse_width_samples)
            pulse = np.exp(-0.5 * x**2)
        
        elif pulse_shape == "triangular":
            # Triangular pulse
            ramp_up = np.linspace(0, 1, pulse_width_samples // 2)
            ramp_down = np.linspace(1, 0, pulse_width_samples - len(ramp_up))
            pulse = np.concatenate((ramp_up, ramp_down))
        
        else:
            raise ValueError(f"Unknown pulse shape: {pulse_shape}")
        
        # Create pulse train
        for i in range(0, num_samples, pri_samples):
            end_idx = min(i + pulse_width_samples, num_samples)
            pulse_len = end_idx - i
            pulse_train[i:end_idx] = pulse[:pulse_len]
        
        # Apply carrier if frequency > 0
        if frequency > 0:
            carrier = np.sin(2 * np.pi * frequency * t)
            samples = amplitude * pulse_train * carrier
        else:
            samples = amplitude * pulse_train
        
        # Create metadata
        metadata = {
            "waveform_type": f"pulse_{pulse_shape}",
            "sample_rate": sample_rate,
            "duration": duration,
            "pulse_width": pulse_width,
            "pulse_repetition_interval": pulse_repetition_interval,
            "frequency": frequency,
            "amplitude": amplitude,
            "num_samples": num_samples
        }
        
        return samples, metadata
    
    def generate_chirp_waveform(
        self,
        start_frequency: float,
        end_frequency: float,
        chirp_period: float,
        sample_rate: int = 44100,
        duration: float = 1.0,
        amplitude: float = 1.0
    ) -> Tuple[np.ndarray, Dict[str, Any]]:
        """
        Generate a chirp waveform.
        
        Args:
            start_frequency: Start frequency in Hz
            end_frequency: End frequency in Hz
            chirp_period: Chirp period in seconds
            sample_rate: Sample rate in Hz
            duration: Duration in seconds
            amplitude: Amplitude (0.0 to 1.0)
            
        Returns:
            Tuple of (waveform samples, metadata)
        """
        # Calculate number of samples
        num_samples = int(sample_rate * duration)
        
        # Create time array
        t = np.arange(num_samples) / sample_rate
        
        # Calculate chirp rate
        chirp_rate = (end_frequency - start_frequency) / chirp_period
        
        # Generate chirp waveform
        samples = amplitude * signal.chirp(
            t, 
            f0=start_frequency, 
            f1=end_frequency, 
            t1=chirp_period, 
            method='linear'
        )
        
        # Repeat chirp for the entire duration
        num_chirps = int(np.ceil(duration / chirp_period))
        chirp_samples = int(chirp_period * sample_rate)
        
        if num_chirps > 1:
            # Create a single chirp
            single_chirp = samples[:chirp_samples]
            
            # Repeat the chirp
            samples = np.tile(single_chirp, num_chirps)[:num_samples]
        
        # Create metadata
        metadata = {
            "waveform_type": "chirp",
            "sample_rate": sample_rate,
            "duration": duration,
            "start_frequency": start_frequency,
            "end_frequency": end_frequency,
            "chirp_period": chirp_period,
            "chirp_rate": chirp_rate,
            "amplitude": amplitude,
            "num_samples": num_samples
        }
        
        return samples, metadata
    
    def generate_custom_waveform(
        self,
        waveform_function,
        sample_rate: int = 44100,
        duration: float = 1.0,
        **kwargs
    ) -> Tuple[np.ndarray, Dict[str, Any]]:
        """
        Generate a custom waveform using a provided function.
        
        Args:
            waveform_function: Function that takes time array and returns samples
            sample_rate: Sample rate in Hz
            duration: Duration in seconds
            **kwargs: Additional arguments to pass to the waveform function
            
        Returns:
            Tuple of (waveform samples, metadata)
        """
        # Calculate number of samples
        num_samples = int(sample_rate * duration)
        
        # Create time array
        t = np.arange(num_samples) / sample_rate
        
        # Generate waveform
        samples = waveform_function(t, **kwargs)
        
        # Ensure samples is a numpy array
        samples = np.asarray(samples)
        
        # Normalize if needed
        if np.max(np.abs(samples)) > 1.0:
            samples = samples / np.max(np.abs(samples))
        
        # Create metadata
        metadata = {
            "waveform_type": "custom",
            "sample_rate": sample_rate,
            "duration": duration,
            "num_samples": num_samples,
            "function_name": waveform_function.__name__,
            **kwargs
        }
        
        return samples, metadata
    
    def combine_waveforms(
        self,
        waveforms: List[np.ndarray],
        weights: Optional[List[float]] = None,
        normalize: bool = True
    ) -> Tuple[np.ndarray, Dict[str, Any]]:
        """
        Combine multiple waveforms.
        
        Args:
            waveforms: List of waveform arrays
            weights: List of weights for each waveform
            normalize: Whether to normalize the result
            
        Returns:
            Tuple of (combined waveform, metadata)
        """
        # Check if waveforms list is empty
        if not waveforms:
            raise ValueError("Waveforms list is empty")
        
        # Check if all waveforms have the same length
        lengths = [len(w) for w in waveforms]
        if len(set(lengths)) > 1:
            raise ValueError("All waveforms must have the same length")
        
        # Default weights if not provided
        if weights is None:
            weights = [1.0 / len(waveforms)] * len(waveforms)
        
        # Ensure weights and waveforms have the same length
        if len(weights) != len(waveforms):
            raise ValueError("Weights and waveforms must have the same length")
        
        # Combine waveforms
        combined = np.zeros_like(waveforms[0])
        for waveform, weight in zip(waveforms, weights):
            combined += weight * waveform
        
        # Normalize if requested
        if normalize and np.max(np.abs(combined)) > 0:
            combined = combined / np.max(np.abs(combined))
        
        # Create metadata
        metadata = {
            "waveform_type": "combined",
            "num_waveforms": len(waveforms),
            "weights": weights,
            "normalized": normalize,
            "num_samples": len(combined)
        }
        
        return combined, metadata
    
    def apply_modulation(
        self,
        carrier: np.ndarray,
        modulator: np.ndarray,
        modulation_type: str,
        modulation_index: float = 1.0,
        sample_rate: int = 44100
    ) -> Tuple[np.ndarray, Dict[str, Any]]:
        """
        Apply modulation to a carrier signal.
        
        Args:
            carrier: Carrier signal
            modulator: Modulating signal
            modulation_type: Type of modulation (AM, FM, PM)
            modulation_index: Modulation index
            sample_rate: Sample rate in Hz
            
        Returns:
            Tuple of (modulated signal, metadata)
        """
        # Check if signals have the same length
        if len(carrier) != len(modulator):
            raise ValueError("Carrier and modulator must have the same length")
        
        # Apply modulation based on type
        if modulation_type == "AM":
            # Amplitude modulation
            # Ensure modulator is in range [0, 1]
            mod_normalized = 0.5 * (1 + modulation_index * modulator / np.max(np.abs(modulator)))
            modulated = mod_normalized * carrier
        
        elif modulation_type == "FM":
            # Frequency modulation
            # Integrate modulator to get phase
            phase = modulation_index * np.cumsum(modulator) / sample_rate
            # Apply phase modulation
            t = np.arange(len(carrier)) / sample_rate
            modulated = np.sin(2 * np.pi * t * carrier + phase)
        
        elif modulation_type == "PM":
            # Phase modulation
            t = np.arange(len(carrier)) / sample_rate
            modulated = np.sin(2 * np.pi * t * carrier + modulation_index * modulator)
        
        else:
            raise ValueError(f"Unknown modulation type: {modulation_type}")
        
        # Normalize
        if np.max(np.abs(modulated)) > 0:
            modulated = modulated / np.max(np.abs(modulated))
        
        # Create metadata
        metadata = {
            "waveform_type": f"modulated_{modulation_type}",
            "modulation_type": modulation_type,
            "modulation_index": modulation_index,
            "sample_rate": sample_rate,
            "num_samples": len(modulated)
        }
        
        return modulated, metadata
    
    def save_waveform(
        self,
        samples: np.ndarray,
        metadata: Dict[str, Any],
        file_path: str
    ) -> bool:
        """
        Save a waveform to a file.
        
        Args:
            samples: Waveform samples
            metadata: Waveform metadata
            file_path: Path to save the waveform
            
        Returns:
            True if successful, False otherwise
        """
        try:
            # Save as NumPy file (.npz)
            np.savez(
                file_path,
                samples=samples,
                **metadata
            )
            
            logger.info(f"Saved waveform to {file_path}")
            
            return True
        
        except Exception as e:
            logger.error(f"Failed to save waveform: {e}")
            return False
    
    def load_waveform(self, file_path: str) -> Tuple[np.ndarray, Dict[str, Any]]:
        """
        Load a waveform from a file.
        
        Args:
            file_path: Path to the waveform file
            
        Returns:
            Tuple of (waveform samples, metadata)
        """
        try:
            # Load NumPy file
            data = np.load(file_path, allow_pickle=True)
            
            # Extract samples
            samples = data["samples"]
            
            # Extract metadata
            metadata = {key: data[key] for key in data.files if key != "samples"}
            
            logger.info(f"Loaded waveform from {file_path}")
            
            return samples, metadata
        
        except Exception as e:
            logger.error(f"Failed to load waveform: {e}")
            raise
