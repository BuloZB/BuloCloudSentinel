"""
Dock Manager Service

This module provides a service for managing dock adapters.
"""

import logging
import asyncio
from typing import Dict, Any, List, Optional, Tuple
import yaml
import os
import json
import redis.asyncio as redis
from datetime import datetime

from dock_driver.adapters.interface import DockAdapter, DockAdapterFactory, DockStatus, ChargingStatus
from dock_driver.models.schemas import DockItem, DockType

logger = logging.getLogger(__name__)


class DockManager:
    """Service for managing dock adapters."""
    
    _instance = None
    
    @classmethod
    def get_instance(cls):
        """
        Get the singleton instance of the dock manager.
        
        Returns:
            DockManager: The singleton instance.
        """
        if cls._instance is None:
            cls._instance = cls()
        return cls._instance
    
    def __init__(self):
        """Initialize the dock manager."""
        self.config = self._load_config()
        self.adapters = {}
        self.redis_client = None
        self.initialized = False
    
    async def initialize(self):
        """
        Initialize the dock manager.
        
        Returns:
            bool: True if initialization was successful, False otherwise.
        """
        if self.initialized:
            return True
        
        try:
            # Initialize Redis client
            redis_url = self.config.get("redis", {}).get("url", "redis://localhost:6379/0")
            self.redis_client = redis.from_url(redis_url)
            
            # Initialize DJI dock adapters
            if self.config.get("dji_dock", {}).get("enabled", False):
                await self._initialize_dji_docks()
            
            # Initialize Heisha dock adapters
            if self.config.get("heisha_dock", {}).get("enabled", False):
                await self._initialize_heisha_docks()
            
            # Initialize ESP32 dock adapters
            if self.config.get("esp32_dock", {}).get("enabled", False):
                await self._initialize_esp32_docks()
            
            self.initialized = True
            logger.info("Dock manager initialized successfully")
            return True
        except Exception as e:
            logger.error(f"Error initializing dock manager: {str(e)}")
            return False
    
    async def shutdown(self):
        """Shutdown the dock manager and release resources."""
        try:
            # Shutdown all adapters
            for dock_type, dock_id_map in self.adapters.items():
                for dock_id, adapter in dock_id_map.items():
                    try:
                        await adapter.shutdown()
                    except Exception as e:
                        logger.error(f"Error shutting down adapter for dock {dock_id}: {str(e)}")
            
            # Close Redis client
            if self.redis_client:
                await self.redis_client.close()
            
            logger.info("Dock manager shut down successfully")
        except Exception as e:
            logger.error(f"Error shutting down dock manager: {str(e)}")
    
    def get_adapter(self, dock_type: str, dock_id: str) -> Optional[DockAdapter]:
        """
        Get a dock adapter by type and ID.
        
        Args:
            dock_type: Type of the dock (dji, heisha, esp32).
            dock_id: ID of the dock.
            
        Returns:
            DockAdapter: The dock adapter, or None if not found.
        """
        return self.adapters.get(dock_type, {}).get(dock_id)
    
    def get_all_docks(self) -> List[DockItem]:
        """
        Get a list of all configured docks.
        
        Returns:
            List[DockItem]: List of dock items.
        """
        docks = []
        
        for dock_type, dock_id_map in self.adapters.items():
            for dock_id, adapter in dock_id_map.items():
                try:
                    # Get cached status if available
                    status = adapter.status_cache or {}
                    
                    # Create dock item
                    dock = DockItem(
                        dock_id=dock_id,
                        dock_type=dock_type,
                        name=f"{dock_type.capitalize()} Dock {dock_id}",
                        status=status.get("status", DockStatus.UNKNOWN),
                        charging_status=status.get("charging_status", ChargingStatus.UNKNOWN),
                        last_updated=datetime.fromisoformat(status.get("timestamp", datetime.now().isoformat())) if status.get("timestamp") else None
                    )
                    
                    docks.append(dock)
                except Exception as e:
                    logger.error(f"Error creating dock item for dock {dock_id}: {str(e)}")
        
        return docks
    
    async def _initialize_dji_docks(self):
        """Initialize DJI dock adapters."""
        try:
            # Get DJI dock configuration
            dji_config = self.config.get("dji_dock", {})
            
            # Create adapter
            dock_id = dji_config.get("dock_sn", "dji-dock-1")
            adapter = DockAdapterFactory.create_adapter("dji", dji_config)
            
            if adapter:
                # Initialize adapter
                success = await adapter.initialize()
                if success:
                    # Store adapter
                    if "dji" not in self.adapters:
                        self.adapters["dji"] = {}
                    self.adapters["dji"][dock_id] = adapter
                    logger.info(f"DJI dock adapter for dock {dock_id} initialized successfully")
                else:
                    logger.error(f"Failed to initialize DJI dock adapter for dock {dock_id}")
            else:
                logger.error("Failed to create DJI dock adapter")
        except Exception as e:
            logger.error(f"Error initializing DJI docks: {str(e)}")
    
    async def _initialize_heisha_docks(self):
        """Initialize Heisha dock adapters."""
        try:
            # Get Heisha dock configuration
            heisha_config = self.config.get("heisha_dock", {})
            
            # Create adapter
            dock_id = f"heisha-{heisha_config.get('modbus_host', 'localhost')}"
            adapter = DockAdapterFactory.create_adapter("heisha", heisha_config)
            
            if adapter:
                # Initialize adapter
                success = await adapter.initialize()
                if success:
                    # Store adapter
                    if "heisha" not in self.adapters:
                        self.adapters["heisha"] = {}
                    self.adapters["heisha"][dock_id] = adapter
                    logger.info(f"Heisha dock adapter for dock {dock_id} initialized successfully")
                else:
                    logger.error(f"Failed to initialize Heisha dock adapter for dock {dock_id}")
            else:
                logger.error("Failed to create Heisha dock adapter")
        except Exception as e:
            logger.error(f"Error initializing Heisha docks: {str(e)}")
    
    async def _initialize_esp32_docks(self):
        """Initialize ESP32 dock adapters."""
        try:
            # Get ESP32 dock configuration
            esp32_config = self.config.get("esp32_dock", {})
            
            # Create adapter
            dock_id = esp32_config.get("dock_id", "esp32-dock-1")
            adapter = DockAdapterFactory.create_adapter("esp32", esp32_config)
            
            if adapter:
                # Initialize adapter
                success = await adapter.initialize()
                if success:
                    # Store adapter
                    if "esp32" not in self.adapters:
                        self.adapters["esp32"] = {}
                    self.adapters["esp32"][dock_id] = adapter
                    logger.info(f"ESP32 dock adapter for dock {dock_id} initialized successfully")
                else:
                    logger.error(f"Failed to initialize ESP32 dock adapter for dock {dock_id}")
            else:
                logger.error("Failed to create ESP32 dock adapter")
        except Exception as e:
            logger.error(f"Error initializing ESP32 docks: {str(e)}")
    
    def _load_config(self) -> Dict[str, Any]:
        """
        Load configuration from file.
        
        Returns:
            Dict[str, Any]: Configuration dictionary.
        """
        try:
            config_path = os.environ.get("DOCK_DRIVER_CONFIG", "config/config.yaml")
            
            with open(config_path, "r") as f:
                config = yaml.safe_load(f)
            
            # Replace environment variables in config
            config_str = json.dumps(config)
            for key, value in os.environ.items():
                config_str = config_str.replace(f"${{{key}}}", value)
            
            config = json.loads(config_str)
            
            return config
        except Exception as e:
            logger.error(f"Error loading configuration: {str(e)}")
            return {}
