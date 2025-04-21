"""
Hardware interfaces for the Anti-Jamming Service.

This module defines abstract interfaces for hardware components used in the anti-jamming service.
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


class ISDRInterface(IHardwareInterface):
    """
    Interface for Software-Defined Radio (SDR) hardware.
    
    This interface defines methods specific to SDR hardware.
    """
    
    @abstractmethod
    async def set_frequency(self, frequency: int) -> bool:
        """
        Set the center frequency of the SDR.
        
        Args:
            frequency: Center frequency in Hz.
            
        Returns:
            bool: True if successful, False otherwise.
        """
        pass
    
    @abstractmethod
    async def set_sample_rate(self, sample_rate: int) -> bool:
        """
        Set the sample rate of the SDR.
        
        Args:
            sample_rate: Sample rate in samples per second.
            
        Returns:
            bool: True if successful, False otherwise.
        """
        pass
    
    @abstractmethod
    async def set_gain(self, gain: float) -> bool:
        """
        Set the gain of the SDR.
        
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
        Get samples from the SDR.
        
        Args:
            num_samples: Number of samples to get.
            
        Returns:
            np.ndarray: Complex samples.
        """
        pass


class IKrakenSDRInterface(ISDRInterface):
    """
    Interface for KrakenSDR hardware.
    
    This interface defines methods specific to KrakenSDR hardware.
    """
    
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
            channel: Channel number (0-4).
            
        Returns:
            bool: True if successful, False otherwise.
        """
        pass
    
    @abstractmethod
    async def get_doa_estimate(self) -> Tuple[float, float]:
        """
        Get the direction of arrival (DoA) estimate.
        
        Returns:
            Tuple[float, float]: Azimuth and elevation angles in degrees.
        """
        pass
    
    @abstractmethod
    async def get_channel_samples(self, channel: int, num_samples: int) -> np.ndarray:
        """
        Get samples from a specific channel.
        
        Args:
            channel: Channel number (0-4).
            num_samples: Number of samples to get.
            
        Returns:
            np.ndarray: Complex samples.
        """
        pass


class IHackRFInterface(ISDRInterface):
    """
    Interface for HackRF hardware.
    
    This interface defines methods specific to HackRF hardware.
    """
    
    @abstractmethod
    async def set_tx_frequency(self, frequency: int) -> bool:
        """
        Set the transmit frequency of the HackRF.
        
        Args:
            frequency: Transmit frequency in Hz.
            
        Returns:
            bool: True if successful, False otherwise.
        """
        pass
    
    @abstractmethod
    async def set_tx_gain(self, gain: float) -> bool:
        """
        Set the transmit gain of the HackRF.
        
        Args:
            gain: Transmit gain in dB.
            
        Returns:
            bool: True if successful, False otherwise.
        """
        pass
    
    @abstractmethod
    async def start_tx(self) -> bool:
        """
        Start transmitting.
        
        Returns:
            bool: True if successful, False otherwise.
        """
        pass
    
    @abstractmethod
    async def stop_tx(self) -> bool:
        """
        Stop transmitting.
        
        Returns:
            bool: True if successful, False otherwise.
        """
        pass
    
    @abstractmethod
    async def transmit_samples(self, samples: np.ndarray) -> bool:
        """
        Transmit samples.
        
        Args:
            samples: Complex samples to transmit.
            
        Returns:
            bool: True if successful, False otherwise.
        """
        pass


class ILoRaInterface(IHardwareInterface):
    """
    Interface for LoRa hardware.
    
    This interface defines methods specific to LoRa hardware.
    """
    
    @abstractmethod
    async def set_frequency(self, frequency: int) -> bool:
        """
        Set the frequency of the LoRa module.
        
        Args:
            frequency: Frequency in Hz.
            
        Returns:
            bool: True if successful, False otherwise.
        """
        pass
    
    @abstractmethod
    async def set_spreading_factor(self, sf: int) -> bool:
        """
        Set the spreading factor of the LoRa module.
        
        Args:
            sf: Spreading factor (7-12).
            
        Returns:
            bool: True if successful, False otherwise.
        """
        pass
    
    @abstractmethod
    async def set_bandwidth(self, bw: int) -> bool:
        """
        Set the bandwidth of the LoRa module.
        
        Args:
            bw: Bandwidth in Hz.
            
        Returns:
            bool: True if successful, False otherwise.
        """
        pass
    
    @abstractmethod
    async def set_coding_rate(self, cr: int) -> bool:
        """
        Set the coding rate of the LoRa module.
        
        Args:
            cr: Coding rate (5-8, representing 4/5 to 4/8).
            
        Returns:
            bool: True if successful, False otherwise.
        """
        pass
    
    @abstractmethod
    async def send_packet(self, data: bytes) -> bool:
        """
        Send a packet.
        
        Args:
            data: Packet data.
            
        Returns:
            bool: True if successful, False otherwise.
        """
        pass
    
    @abstractmethod
    async def receive_packet(self, timeout: float = 0.0) -> Optional[bytes]:
        """
        Receive a packet.
        
        Args:
            timeout: Timeout in seconds (0 = no timeout).
            
        Returns:
            Optional[bytes]: Packet data, or None if no packet was received.
        """
        pass
    
    @abstractmethod
    async def set_frequency_hopping(self, enabled: bool, hop_period: int = 0) -> bool:
        """
        Enable or disable frequency hopping.
        
        Args:
            enabled: True to enable frequency hopping, False to disable.
            hop_period: Hop period in symbols.
            
        Returns:
            bool: True if successful, False otherwise.
        """
        pass


class IJammingDetector(ABC):
    """
    Interface for jamming detection algorithms.
    
    This interface defines methods for detecting jamming signals.
    """
    
    @abstractmethod
    async def initialize(self) -> bool:
        """
        Initialize the jamming detector.
        
        Returns:
            bool: True if initialization was successful, False otherwise.
        """
        pass
    
    @abstractmethod
    async def shutdown(self) -> bool:
        """
        Shutdown the jamming detector.
        
        Returns:
            bool: True if shutdown was successful, False otherwise.
        """
        pass
    
    @abstractmethod
    async def configure(self, config: Dict[str, Any]) -> bool:
        """
        Configure the jamming detector.
        
        Args:
            config: Configuration parameters.
            
        Returns:
            bool: True if configuration was successful, False otherwise.
        """
        pass
    
    @abstractmethod
    async def detect(self, samples: np.ndarray) -> Dict[str, Any]:
        """
        Detect jamming signals in the provided samples.
        
        Args:
            samples: Complex samples to analyze.
            
        Returns:
            Dict[str, Any]: Detection results, including jamming probability,
                            jamming type, and signal characteristics.
        """
        pass
    
    @abstractmethod
    async def get_status(self) -> Dict[str, Any]:
        """
        Get the status of the jamming detector.
        
        Returns:
            Dict[str, Any]: Status information.
        """
        pass
