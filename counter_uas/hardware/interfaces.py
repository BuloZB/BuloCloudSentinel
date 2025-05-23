"""
Hardware interfaces for the Counter-UAS module.

This module defines abstract interfaces for hardware components used in the Counter-UAS module.
"""

from abc import ABC, abstractmethod
from typing import Dict, List, Optional, Tuple, Any, Union
import numpy as np


class IHardwareInterface(ABC):
    """
    Abstract interface for hardware components.
    
    This interface defines the common methods that all hardware components must implement.
    """
    
    @abstractmethod
    async def initialize(self) -> bool:
        """
        Initialize the hardware component.
        
        Returns:
            bool: True if initialization was successful, False otherwise.
        """
        pass
    
    @abstractmethod
    async def shutdown(self) -> bool:
        """
        Shutdown the hardware component.
        
        Returns:
            bool: True if shutdown was successful, False otherwise.
        """
        pass
    
    @abstractmethod
    async def get_status(self) -> Dict[str, Any]:
        """
        Get the status of the hardware component.
        
        Returns:
            Dict[str, Any]: Status information.
        """
        pass
    
    @abstractmethod
    async def configure(self, config: Dict[str, Any]) -> bool:
        """
        Configure the hardware component.
        
        Args:
            config: Configuration parameters.
            
        Returns:
            bool: True if configuration was successful, False otherwise.
        """
        pass


class IKerberosSDRInterface(IHardwareInterface):
    """
    Interface for KerberosSDR hardware.
    
    This interface defines methods specific to KerberosSDR hardware, a 4-channel coherent RTL-SDR array.
    """
    
    @abstractmethod
    async def set_frequency(self, frequency: int) -> bool:
        """
        Set the center frequency of the KerberosSDR.
        
        Args:
            frequency: Center frequency in Hz.
            
        Returns:
            bool: True if successful, False otherwise.
        """
        pass
    
    @abstractmethod
    async def set_sample_rate(self, sample_rate: int) -> bool:
        """
        Set the sample rate of the KerberosSDR.
        
        Args:
            sample_rate: Sample rate in samples per second.
            
        Returns:
            bool: True if successful, False otherwise.
        """
        pass
    
    @abstractmethod
    async def set_gain(self, gain: float) -> bool:
        """
        Set the gain of the KerberosSDR.
        
        Args:
            gain: Gain in dB.
            
        Returns:
            bool: True if successful, False otherwise.
        """
        pass
    
    @abstractmethod
    async def start_rx(self) -> bool:
        """
        Start receiving samples.
        
        Returns:
            bool: True if successful, False otherwise.
        """
        pass
    
    @abstractmethod
    async def stop_rx(self) -> bool:
        """
        Stop receiving samples.
        
        Returns:
            bool: True if successful, False otherwise.
        """
        pass
    
    @abstractmethod
    async def get_samples(self, num_samples: int) -> np.ndarray:
        """
        Get samples from the KerberosSDR.
        
        Args:
            num_samples: Number of samples to get.
            
        Returns:
            np.ndarray: Complex samples.
        """
        pass
    
    @abstractmethod
    async def set_coherent_mode(self, enabled: bool) -> bool:
        """
        Enable or disable coherent mode.
        
        Args:
            enabled: True to enable coherent mode, False to disable.
            
        Returns:
            bool: True if successful, False otherwise.
        """
        pass
    
    @abstractmethod
    async def set_reference_channel(self, channel: int) -> bool:
        """
        Set the reference channel for coherent mode.
        
        Args:
            channel: Channel number (0-3).
            
        Returns:
            bool: True if successful, False otherwise.
        """
        pass
    
    @abstractmethod
    async def get_doa_estimate(self) -> Tuple[float, float, float]:
        """
        Get the direction of arrival (DoA) estimate.
        
        Returns:
            Tuple[float, float, float]: Azimuth angle, elevation angle, and confidence (0-1).
        """
        pass
    
    @abstractmethod
    async def get_channel_samples(self, channel: int, num_samples: int) -> np.ndarray:
        """
        Get samples from a specific channel.
        
        Args:
            channel: Channel number (0-3).
            num_samples: Number of samples to get.
            
        Returns:
            np.ndarray: Complex samples.
        """
        pass


class IAcconeerRadarInterface(IHardwareInterface):
    """
    Interface for Acconeer radar hardware.
    
    This interface defines methods specific to Acconeer A111 60GHz radar sensors.
    """
    
    @abstractmethod
    async def set_mode(self, mode: str) -> bool:
        """
        Set the operating mode of the radar.
        
        Args:
            mode: Operating mode ('power_bins', 'envelope', 'iq', 'sparse').
            
        Returns:
            bool: True if successful, False otherwise.
        """
        pass
    
    @abstractmethod
    async def set_range(self, start_m: float, end_m: float) -> bool:
        """
        Set the range of the radar.
        
        Args:
            start_m: Start range in meters.
            end_m: End range in meters.
            
        Returns:
            bool: True if successful, False otherwise.
        """
        pass
    
    @abstractmethod
    async def set_update_rate(self, rate_hz: float) -> bool:
        """
        Set the update rate of the radar.
        
        Args:
            rate_hz: Update rate in Hz.
            
        Returns:
            bool: True if successful, False otherwise.
        """
        pass
    
    @abstractmethod
    async def start_measurement(self) -> bool:
        """
        Start radar measurement.
        
        Returns:
            bool: True if successful, False otherwise.
        """
        pass
    
    @abstractmethod
    async def stop_measurement(self) -> bool:
        """
        Stop radar measurement.
        
        Returns:
            bool: True if successful, False otherwise.
        """
        pass
    
    @abstractmethod
    async def get_next_frame(self) -> Dict[str, Any]:
        """
        Get the next radar frame.
        
        Returns:
            Dict[str, Any]: Radar frame data.
        """
        pass
