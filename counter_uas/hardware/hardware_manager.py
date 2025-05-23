"""
Hardware manager for the Counter-UAS module.

This module provides a manager for hardware devices used in the Counter-UAS module.
"""

import asyncio
import logging
from typing import Dict, List, Optional, Tuple, Any, Union

from counter_uas.hardware.interfaces import IHardwareInterface, IKerberosSDRInterface, IAcconeerRadarInterface
from counter_uas.hardware.kerberos_sdr import KerberosSDR
from counter_uas.hardware.acconeer_radar import AcconeerRadar
from counter_uas.utils.config import get_config

logger = logging.getLogger(__name__)


class HardwareManager:
    """
    Hardware manager for the Counter-UAS module.
    
    This class manages hardware devices used in the Counter-UAS module.
    """
    
    def __init__(self, config: Optional[Dict[str, Any]] = None):
        """
        Initialize the hardware manager.
        
        Args:
            config: Optional configuration parameters.
        """
        self.config = config or {}
        self.devices = {}
        self.initialized = False
        
        # Load configuration
        self._load_config()
    
    def _load_config(self):
        """Load configuration from the config file."""
        config = get_config().get("hardware", {})
        self.config.update(config)
    
    async def initialize(self) -> bool:
        """
        Initialize all hardware devices.
        
        Returns:
            bool: True if initialization was successful, False otherwise.
        """
        logger.info("Initializing hardware devices")
        
        try:
            # Initialize KerberosSDR devices
            kerberos_configs = self.config.get("kerberos_sdr_devices", [])
            for i, config in enumerate(kerberos_configs):
                device_id = config.get("device_id", str(i))
                device = KerberosSDR(device_index=i, config=config)
                self.devices[f"kerberos_sdr_{device_id}"] = device
                logger.info(f"Added KerberosSDR device {device_id}")
            
            # Initialize Acconeer radar devices
            acconeer_configs = self.config.get("acconeer_radar_devices", [])
            for i, config in enumerate(acconeer_configs):
                device_id = config.get("device_id", str(i))
                device = AcconeerRadar(device_id=device_id, config=config)
                self.devices[f"acconeer_radar_{device_id}"] = device
                logger.info(f"Added Acconeer radar device {device_id}")
            
            # Initialize all devices
            for device_id, device in self.devices.items():
                success = await device.initialize()
                if not success:
                    logger.error(f"Failed to initialize device {device_id}")
                    return False
            
            self.initialized = True
            logger.info("All hardware devices initialized successfully")
            return True
        except Exception as e:
            logger.error(f"Failed to initialize hardware devices: {str(e)}")
            return False
    
    async def shutdown(self) -> bool:
        """
        Shutdown all hardware devices.
        
        Returns:
            bool: True if shutdown was successful, False otherwise.
        """
        logger.info("Shutting down hardware devices")
        
        try:
            # Shutdown all devices
            for device_id, device in self.devices.items():
                success = await device.shutdown()
                if not success:
                    logger.error(f"Failed to shutdown device {device_id}")
            
            self.initialized = False
            logger.info("All hardware devices shut down successfully")
            return True
        except Exception as e:
            logger.error(f"Failed to shutdown hardware devices: {str(e)}")
            return False
    
    async def get_status(self) -> Dict[str, Any]:
        """
        Get the status of all hardware devices.
        
        Returns:
            Dict[str, Any]: Status information.
        """
        status = {
            "initialized": self.initialized,
            "devices": {}
        }
        
        for device_id, device in self.devices.items():
            status["devices"][device_id] = await device.get_status()
        
        return status
    
    def get_device(self, device_id: str) -> Optional[IHardwareInterface]:
        """
        Get a hardware device by ID.
        
        Args:
            device_id: Device ID.
            
        Returns:
            Optional[IHardwareInterface]: Hardware device, or None if not found.
        """
        return self.devices.get(device_id)
    
    def get_kerberos_sdr_devices(self) -> List[IKerberosSDRInterface]:
        """
        Get all KerberosSDR devices.
        
        Returns:
            List[IKerberosSDRInterface]: List of KerberosSDR devices.
        """
        return [device for device_id, device in self.devices.items() 
                if device_id.startswith("kerberos_sdr_")]
    
    def get_acconeer_radar_devices(self) -> List[IAcconeerRadarInterface]:
        """
        Get all Acconeer radar devices.
        
        Returns:
            List[IAcconeerRadarInterface]: List of Acconeer radar devices.
        """
        return [device for device_id, device in self.devices.items() 
                if device_id.startswith("acconeer_radar_")]
    
    async def configure_device(self, device_id: str, config: Dict[str, Any]) -> bool:
        """
        Configure a hardware device.
        
        Args:
            device_id: Device ID.
            config: Configuration parameters.
            
        Returns:
            bool: True if configuration was successful, False otherwise.
        """
        device = self.get_device(device_id)
        if not device:
            logger.error(f"Device {device_id} not found")
            return False
        
        return await device.configure(config)
    
    async def start_all_devices(self) -> bool:
        """
        Start all hardware devices.
        
        Returns:
            bool: True if all devices were started successfully, False otherwise.
        """
        logger.info("Starting all hardware devices")
        
        success = True
        
        # Start KerberosSDR devices
        for device in self.get_kerberos_sdr_devices():
            if not await device.start_rx():
                success = False
        
        # Start Acconeer radar devices
        for device in self.get_acconeer_radar_devices():
            if not await device.start_measurement():
                success = False
        
        return success
    
    async def stop_all_devices(self) -> bool:
        """
        Stop all hardware devices.
        
        Returns:
            bool: True if all devices were stopped successfully, False otherwise.
        """
        logger.info("Stopping all hardware devices")
        
        success = True
        
        # Stop KerberosSDR devices
        for device in self.get_kerberos_sdr_devices():
            if not await device.stop_rx():
                success = False
        
        # Stop Acconeer radar devices
        for device in self.get_acconeer_radar_devices():
            if not await device.stop_measurement():
                success = False
        
        return success
