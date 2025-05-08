"""
Power Management Integration Service

This module provides integration with the Power Management module for automated charging.
"""

import logging
import asyncio
import json
from typing import Dict, Any, Optional, List, Callable
from datetime import datetime
import httpx

from dock_driver.adapters.interface import DockStatus, ChargingStatus

logger = logging.getLogger(__name__)


class PowerManagementService:
    """Service for integrating with the Power Management module."""

    def __init__(self, config: Dict[str, Any]):
        """
        Initialize the Power Management integration service.
        
        Args:
            config: Configuration dictionary for the service.
        """
        self.config = config
        self.api_url = config.get("api_url", "")
        self.auto_charge_threshold = config.get("auto_charge_threshold", 30)
        self.charge_complete_threshold = config.get("charge_complete_threshold", 90)
        self.enabled = config.get("integration_enabled", True)
        self.jwt_token = config.get("jwt_token", "")
        
        self.update_task = None
        self.running = False
        self.dock_manager = None

    async def initialize(self, dock_manager) -> bool:
        """
        Initialize the service.
        
        Args:
            dock_manager: Dock manager instance.
            
        Returns:
            bool: True if initialization was successful, False otherwise.
        """
        try:
            if not self.enabled:
                logger.info("Power Management integration is disabled")
                return True
            
            # Store dock manager reference
            self.dock_manager = dock_manager
            
            # Test connection to Power Management API
            async with httpx.AsyncClient() as client:
                response = await client.get(
                    f"{self.api_url}/battery/fleet",
                    headers=self._get_headers(),
                    timeout=10.0
                )
                
                if response.status_code != 200:
                    logger.error(f"Failed to connect to Power Management API at {self.api_url}")
                    return False
            
            # Start background task for monitoring battery levels
            self.running = True
            self.update_task = asyncio.create_task(self._monitor_battery_levels())
            
            logger.info("Power Management integration service initialized")
            return True
        except Exception as e:
            logger.error(f"Error initializing Power Management integration service: {str(e)}")
            return False

    async def shutdown(self) -> None:
        """
        Shutdown the service and release resources.
        """
        self.running = False
        if self.update_task:
            self.update_task.cancel()
            try:
                await self.update_task
            except asyncio.CancelledError:
                pass
        
        logger.info("Power Management integration service shut down")

    async def _monitor_battery_levels(self) -> None:
        """
        Background task for monitoring battery levels and triggering automated charging.
        """
        while self.running:
            try:
                if not self.enabled or not self.dock_manager:
                    await asyncio.sleep(60)
                    continue
                
                # Get fleet battery status
                fleet_status = await self._get_fleet_battery_status()
                
                # Process each drone
                for drone_id, battery_status in fleet_status.items():
                    await self._process_drone_battery(drone_id, battery_status)
                
                # Sleep for 60 seconds
                await asyncio.sleep(60)
            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error(f"Error in battery monitoring: {str(e)}")
                await asyncio.sleep(60)

    async def _process_drone_battery(self, drone_id: str, battery_status: Dict[str, Any]) -> None:
        """
        Process battery status for a drone and trigger charging if needed.
        
        Args:
            drone_id: ID of the drone.
            battery_status: Battery status dictionary.
        """
        try:
            # Extract battery level
            battery_level = battery_status.get("capacity_percent", 100)
            
            # Check if battery level is below auto-charge threshold
            if battery_level <= self.auto_charge_threshold:
                logger.info(f"Drone {drone_id} battery level ({battery_level}%) is below auto-charge threshold ({self.auto_charge_threshold}%)")
                
                # Find available dock
                dock = await self._find_available_dock()
                if not dock:
                    logger.warning(f"No available dock found for drone {drone_id}")
                    return
                
                # Trigger return-to-home
                await self._trigger_return_to_home(drone_id, f"Battery level ({battery_level}%) below threshold")
                
                # Open dock (if supported)
                dock_adapter = self.dock_manager.get_adapter(dock["dock_type"], dock["dock_id"])
                if dock_adapter:
                    await dock_adapter.open_dock()
                
                # Wait for drone to land
                # In a real implementation, this would wait for a landing confirmation
                # For now, we'll just wait for a fixed time
                await asyncio.sleep(30)
                
                # Start charging
                if dock_adapter:
                    await dock_adapter.start_charging()
                    
                    # Monitor charging until complete
                    await self._monitor_charging(drone_id, dock_adapter)
            
        except Exception as e:
            logger.error(f"Error processing drone battery: {str(e)}")

    async def _monitor_charging(self, drone_id: str, dock_adapter) -> None:
        """
        Monitor charging until complete.
        
        Args:
            drone_id: ID of the drone.
            dock_adapter: Dock adapter instance.
        """
        try:
            while True:
                # Get battery status
                battery_status = await self._get_drone_battery_status(drone_id)
                battery_level = battery_status.get("capacity_percent", 0)
                
                # Check if charging is complete
                if battery_level >= self.charge_complete_threshold:
                    logger.info(f"Charging complete for drone {drone_id} (battery level: {battery_level}%)")
                    
                    # Stop charging
                    await dock_adapter.stop_charging()
                    
                    # Close dock (if supported)
                    await dock_adapter.close_dock()
                    
                    break
                
                # Sleep for 60 seconds
                await asyncio.sleep(60)
        except Exception as e:
            logger.error(f"Error monitoring charging: {str(e)}")

    async def _find_available_dock(self) -> Optional[Dict[str, Any]]:
        """
        Find an available dock.
        
        Returns:
            Dict[str, Any]: Dictionary containing dock information, or None if no dock is available.
        """
        try:
            # Get all docks
            docks = self.dock_manager.get_all_docks()
            
            # Find an available dock
            for dock in docks:
                dock_adapter = self.dock_manager.get_adapter(dock["dock_type"], dock["dock_id"])
                if dock_adapter:
                    status = await dock_adapter.get_status()
                    if status.get("status") == DockStatus.ONLINE and status.get("charging_status") == ChargingStatus.IDLE:
                        return dock
            
            return None
        except Exception as e:
            logger.error(f"Error finding available dock: {str(e)}")
            return None

    async def _trigger_return_to_home(self, drone_id: str, reason: str) -> bool:
        """
        Trigger return-to-home for a drone.
        
        Args:
            drone_id: ID of the drone.
            reason: Reason for RTH.
            
        Returns:
            bool: True if the operation was successful, False otherwise.
        """
        try:
            async with httpx.AsyncClient() as client:
                response = await client.post(
                    f"{self.api_url}/battery/trigger-rth/{drone_id}",
                    headers=self._get_headers(),
                    params={"reason": reason},
                    timeout=10.0
                )
                
                if response.status_code == 200:
                    logger.info(f"Successfully triggered RTH for drone {drone_id}")
                    return True
                else:
                    logger.error(f"HTTP error: {response.status_code} - {response.text}")
                    return False
        except Exception as e:
            logger.error(f"Error triggering RTH: {str(e)}")
            return False

    async def _get_fleet_battery_status(self) -> Dict[str, Dict[str, Any]]:
        """
        Get battery status for all drones in the fleet.
        
        Returns:
            Dict[str, Dict[str, Any]]: Dictionary mapping drone IDs to battery status.
        """
        try:
            async with httpx.AsyncClient() as client:
                response = await client.get(
                    f"{self.api_url}/battery/fleet",
                    headers=self._get_headers(),
                    timeout=10.0
                )
                
                if response.status_code == 200:
                    return response.json()
                else:
                    logger.error(f"HTTP error: {response.status_code} - {response.text}")
                    return {}
        except Exception as e:
            logger.error(f"Error getting fleet battery status: {str(e)}")
            return {}

    async def _get_drone_battery_status(self, drone_id: str) -> Dict[str, Any]:
        """
        Get battery status for a specific drone.
        
        Args:
            drone_id: ID of the drone.
            
        Returns:
            Dict[str, Any]: Dictionary containing battery status.
        """
        try:
            async with httpx.AsyncClient() as client:
                response = await client.get(
                    f"{self.api_url}/battery/{drone_id}/current",
                    headers=self._get_headers(),
                    timeout=10.0
                )
                
                if response.status_code == 200:
                    return response.json()
                else:
                    logger.error(f"HTTP error: {response.status_code} - {response.text}")
                    return {}
        except Exception as e:
            logger.error(f"Error getting drone battery status: {str(e)}")
            return {}

    def _get_headers(self) -> Dict[str, str]:
        """
        Get the headers for API requests.
        
        Returns:
            Dict[str, str]: Headers dictionary.
        """
        return {
            "Authorization": f"Bearer {self.jwt_token}",
            "Content-Type": "application/json",
            "Accept": "application/json"
        }
