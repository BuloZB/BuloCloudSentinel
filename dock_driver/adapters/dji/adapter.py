"""
DJI Dock Adapter

This module implements the adapter for DJI Dock 2 using DJI Cloud API v2.
"""

import logging
import asyncio
import time
from typing import Dict, Any, Optional, List
import httpx
import json
import hmac
import hashlib
import base64
from datetime import datetime, timedelta

from dock_driver.adapters.interface import DockAdapter, DockStatus, ChargingStatus

logger = logging.getLogger(__name__)


class DJIDockAdapter(DockAdapter):
    """Adapter for DJI Dock 2."""

    def __init__(self, config: Dict[str, Any]):
        """
        Initialize the DJI Dock adapter.
        
        Args:
            config: Configuration dictionary for the adapter.
        """
        self.config = config
        self.api_key = config.get("api_key", "")
        self.api_secret = config.get("api_secret", "")
        self.dock_sn = config.get("dock_sn", "")
        self.region = config.get("region", "us-east-1")
        self.refresh_interval = config.get("refresh_interval", 30)
        
        self.base_url = self._get_base_url()
        self.access_token = None
        self.token_expiry = datetime.now()
        self.status_cache = {}
        self.telemetry_cache = {}
        self.update_task = None
        self.running = False

    async def initialize(self) -> bool:
        """
        Initialize the adapter.
        
        Returns:
            bool: True if initialization was successful, False otherwise.
        """
        try:
            # Get initial access token
            success = await self._refresh_token()
            if not success:
                logger.error("Failed to get initial access token")
                return False
            
            # Start background task for updating status
            self.running = True
            self.update_task = asyncio.create_task(self._update_loop())
            
            logger.info(f"DJI Dock adapter initialized for dock {self.dock_sn}")
            return True
        except Exception as e:
            logger.error(f"Error initializing DJI Dock adapter: {str(e)}")
            return False

    async def get_status(self) -> Dict[str, Any]:
        """
        Get the current status of the dock.
        
        Returns:
            Dict[str, Any]: Dictionary containing the dock status.
        """
        # Return cached status if available
        if self.status_cache:
            return self.status_cache
        
        # Otherwise, fetch status
        try:
            await self._ensure_token()
            
            async with httpx.AsyncClient() as client:
                response = await client.get(
                    f"{self.base_url}/dock/status",
                    headers=self._get_headers(),
                    params={"sn": self.dock_sn},
                    timeout=10.0
                )
                
                if response.status_code == 200:
                    data = response.json()
                    if data.get("code") == 0:
                        status_data = data.get("data", {})
                        
                        # Parse and cache status
                        self.status_cache = {
                            "dock_id": self.dock_sn,
                            "status": self._parse_dock_status(status_data.get("status", 0)),
                            "charging_status": self._parse_charging_status(status_data.get("charging_status", 0)),
                            "door_state": status_data.get("door_state", "unknown"),
                            "drone_connected": status_data.get("drone_connected", False),
                            "error_code": status_data.get("error_code", 0),
                            "error_message": status_data.get("error_message", ""),
                            "timestamp": datetime.now().isoformat()
                        }
                        
                        return self.status_cache
                    else:
                        logger.error(f"API error: {data.get('message', 'Unknown error')}")
                else:
                    logger.error(f"HTTP error: {response.status_code} - {response.text}")
            
            # Return last known status or default
            return self.status_cache or {
                "dock_id": self.dock_sn,
                "status": DockStatus.UNKNOWN,
                "charging_status": ChargingStatus.UNKNOWN,
                "door_state": "unknown",
                "drone_connected": False,
                "error_code": 0,
                "error_message": "",
                "timestamp": datetime.now().isoformat()
            }
        except Exception as e:
            logger.error(f"Error getting dock status: {str(e)}")
            return {
                "dock_id": self.dock_sn,
                "status": DockStatus.ERROR,
                "charging_status": ChargingStatus.UNKNOWN,
                "door_state": "unknown",
                "drone_connected": False,
                "error_code": -1,
                "error_message": str(e),
                "timestamp": datetime.now().isoformat()
            }

    async def open_dock(self) -> bool:
        """
        Open the dock.
        
        Returns:
            bool: True if the operation was successful, False otherwise.
        """
        try:
            await self._ensure_token()
            
            async with httpx.AsyncClient() as client:
                response = await client.post(
                    f"{self.base_url}/dock/control",
                    headers=self._get_headers(),
                    json={
                        "sn": self.dock_sn,
                        "action": "open_door"
                    },
                    timeout=10.0
                )
                
                if response.status_code == 200:
                    data = response.json()
                    if data.get("code") == 0:
                        logger.info(f"Successfully opened dock {self.dock_sn}")
                        return True
                    else:
                        logger.error(f"API error: {data.get('message', 'Unknown error')}")
                else:
                    logger.error(f"HTTP error: {response.status_code} - {response.text}")
            
            return False
        except Exception as e:
            logger.error(f"Error opening dock: {str(e)}")
            return False

    async def close_dock(self) -> bool:
        """
        Close the dock.
        
        Returns:
            bool: True if the operation was successful, False otherwise.
        """
        try:
            await self._ensure_token()
            
            async with httpx.AsyncClient() as client:
                response = await client.post(
                    f"{self.base_url}/dock/control",
                    headers=self._get_headers(),
                    json={
                        "sn": self.dock_sn,
                        "action": "close_door"
                    },
                    timeout=10.0
                )
                
                if response.status_code == 200:
                    data = response.json()
                    if data.get("code") == 0:
                        logger.info(f"Successfully closed dock {self.dock_sn}")
                        return True
                    else:
                        logger.error(f"API error: {data.get('message', 'Unknown error')}")
                else:
                    logger.error(f"HTTP error: {response.status_code} - {response.text}")
            
            return False
        except Exception as e:
            logger.error(f"Error closing dock: {str(e)}")
            return False

    async def start_charging(self) -> bool:
        """
        Start charging the drone.
        
        Returns:
            bool: True if the operation was successful, False otherwise.
        """
        try:
            await self._ensure_token()
            
            async with httpx.AsyncClient() as client:
                response = await client.post(
                    f"{self.base_url}/dock/control",
                    headers=self._get_headers(),
                    json={
                        "sn": self.dock_sn,
                        "action": "start_charging"
                    },
                    timeout=10.0
                )
                
                if response.status_code == 200:
                    data = response.json()
                    if data.get("code") == 0:
                        logger.info(f"Successfully started charging for dock {self.dock_sn}")
                        return True
                    else:
                        logger.error(f"API error: {data.get('message', 'Unknown error')}")
                else:
                    logger.error(f"HTTP error: {response.status_code} - {response.text}")
            
            return False
        except Exception as e:
            logger.error(f"Error starting charging: {str(e)}")
            return False

    async def stop_charging(self) -> bool:
        """
        Stop charging the drone.
        
        Returns:
            bool: True if the operation was successful, False otherwise.
        """
        try:
            await self._ensure_token()
            
            async with httpx.AsyncClient() as client:
                response = await client.post(
                    f"{self.base_url}/dock/control",
                    headers=self._get_headers(),
                    json={
                        "sn": self.dock_sn,
                        "action": "stop_charging"
                    },
                    timeout=10.0
                )
                
                if response.status_code == 200:
                    data = response.json()
                    if data.get("code") == 0:
                        logger.info(f"Successfully stopped charging for dock {self.dock_sn}")
                        return True
                    else:
                        logger.error(f"API error: {data.get('message', 'Unknown error')}")
                else:
                    logger.error(f"HTTP error: {response.status_code} - {response.text}")
            
            return False
        except Exception as e:
            logger.error(f"Error stopping charging: {str(e)}")
            return False

    async def get_telemetry(self) -> Dict[str, Any]:
        """
        Get telemetry data from the dock.
        
        Returns:
            Dict[str, Any]: Dictionary containing telemetry data.
        """
        # Return cached telemetry if available
        if self.telemetry_cache:
            return self.telemetry_cache
        
        # Otherwise, fetch telemetry
        try:
            await self._ensure_token()
            
            async with httpx.AsyncClient() as client:
                response = await client.get(
                    f"{self.base_url}/dock/telemetry",
                    headers=self._get_headers(),
                    params={"sn": self.dock_sn},
                    timeout=10.0
                )
                
                if response.status_code == 200:
                    data = response.json()
                    if data.get("code") == 0:
                        telemetry_data = data.get("data", {})
                        
                        # Parse and cache telemetry
                        self.telemetry_cache = {
                            "dock_id": self.dock_sn,
                            "temperature": telemetry_data.get("temperature", 0.0),
                            "humidity": telemetry_data.get("humidity", 0.0),
                            "charging_voltage": telemetry_data.get("charging_voltage", 0.0),
                            "charging_current": telemetry_data.get("charging_current", 0.0),
                            "battery_level": telemetry_data.get("battery_level", 0),
                            "network_signal": telemetry_data.get("network_signal", 0),
                            "timestamp": datetime.now().isoformat()
                        }
                        
                        return self.telemetry_cache
                    else:
                        logger.error(f"API error: {data.get('message', 'Unknown error')}")
                else:
                    logger.error(f"HTTP error: {response.status_code} - {response.text}")
            
            # Return last known telemetry or default
            return self.telemetry_cache or {
                "dock_id": self.dock_sn,
                "temperature": 0.0,
                "humidity": 0.0,
                "charging_voltage": 0.0,
                "charging_current": 0.0,
                "battery_level": 0,
                "network_signal": 0,
                "timestamp": datetime.now().isoformat()
            }
        except Exception as e:
            logger.error(f"Error getting dock telemetry: {str(e)}")
            return {
                "dock_id": self.dock_sn,
                "temperature": 0.0,
                "humidity": 0.0,
                "charging_voltage": 0.0,
                "charging_current": 0.0,
                "battery_level": 0,
                "network_signal": 0,
                "timestamp": datetime.now().isoformat()
            }

    async def shutdown(self) -> None:
        """
        Shutdown the adapter and release resources.
        """
        self.running = False
        if self.update_task:
            self.update_task.cancel()
            try:
                await self.update_task
            except asyncio.CancelledError:
                pass
        logger.info(f"DJI Dock adapter for dock {self.dock_sn} shut down")

    def _get_base_url(self) -> str:
        """
        Get the base URL for the DJI Cloud API based on the region.
        
        Returns:
            str: Base URL for the API.
        """
        region_urls = {
            "us-east-1": "https://api-us-east-1.dji.com/v2",
            "eu-central-1": "https://api-eu-central-1.dji.com/v2",
            "ap-southeast-1": "https://api-ap-southeast-1.dji.com/v2"
        }
        return region_urls.get(self.region, region_urls["us-east-1"])

    def _get_headers(self) -> Dict[str, str]:
        """
        Get the headers for API requests.
        
        Returns:
            Dict[str, str]: Headers dictionary.
        """
        return {
            "Authorization": f"Bearer {self.access_token}",
            "Content-Type": "application/json",
            "Accept": "application/json"
        }

    async def _ensure_token(self) -> None:
        """
        Ensure that the access token is valid, refreshing it if necessary.
        """
        if not self.access_token or datetime.now() >= self.token_expiry:
            await self._refresh_token()

    async def _refresh_token(self) -> bool:
        """
        Refresh the access token.
        
        Returns:
            bool: True if the token was refreshed successfully, False otherwise.
        """
        try:
            # Generate signature
            timestamp = int(time.time())
            signature_string = f"{self.api_key}{timestamp}"
            signature = hmac.new(
                self.api_secret.encode(),
                signature_string.encode(),
                hashlib.sha256
            ).digest()
            signature_b64 = base64.b64encode(signature).decode()
            
            async with httpx.AsyncClient() as client:
                response = await client.post(
                    f"{self.base_url}/auth/token",
                    json={
                        "api_key": self.api_key,
                        "timestamp": timestamp,
                        "signature": signature_b64
                    },
                    timeout=10.0
                )
                
                if response.status_code == 200:
                    data = response.json()
                    if data.get("code") == 0:
                        token_data = data.get("data", {})
                        self.access_token = token_data.get("access_token")
                        expires_in = token_data.get("expires_in", 7200)
                        self.token_expiry = datetime.now() + timedelta(seconds=expires_in - 300)  # Refresh 5 minutes before expiry
                        logger.info("Successfully refreshed access token")
                        return True
                    else:
                        logger.error(f"API error: {data.get('message', 'Unknown error')}")
                else:
                    logger.error(f"HTTP error: {response.status_code} - {response.text}")
            
            return False
        except Exception as e:
            logger.error(f"Error refreshing token: {str(e)}")
            return False

    async def _update_loop(self) -> None:
        """
        Background task for updating status and telemetry.
        """
        while self.running:
            try:
                # Update status
                await self.get_status()
                
                # Update telemetry
                await self.get_telemetry()
                
                # Sleep for refresh interval
                await asyncio.sleep(self.refresh_interval)
            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error(f"Error in update loop: {str(e)}")
                await asyncio.sleep(self.refresh_interval)

    def _parse_dock_status(self, status_code: int) -> DockStatus:
        """
        Parse the dock status code.
        
        Args:
            status_code: Status code from the API.
            
        Returns:
            DockStatus: Parsed dock status.
        """
        status_map = {
            0: DockStatus.OFFLINE,
            1: DockStatus.ONLINE,
            2: DockStatus.OPEN,
            3: DockStatus.CLOSED,
            4: DockStatus.ERROR
        }
        return status_map.get(status_code, DockStatus.UNKNOWN)

    def _parse_charging_status(self, status_code: int) -> ChargingStatus:
        """
        Parse the charging status code.
        
        Args:
            status_code: Status code from the API.
            
        Returns:
            ChargingStatus: Parsed charging status.
        """
        status_map = {
            0: ChargingStatus.IDLE,
            1: ChargingStatus.CHARGING,
            2: ChargingStatus.COMPLETE,
            3: ChargingStatus.ERROR
        }
        return status_map.get(status_code, ChargingStatus.UNKNOWN)
