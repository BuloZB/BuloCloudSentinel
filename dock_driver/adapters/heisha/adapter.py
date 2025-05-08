"""
Heisha Charging Pad Adapter

This module implements the adapter for Heisha Charging Pad using REST API and Modbus TCP.
"""

import logging
import asyncio
from typing import Dict, Any, Optional, List, Tuple
import httpx
import json
from datetime import datetime
from pymodbus.client import AsyncModbusTcpClient
from pymodbus.exceptions import ModbusException

from dock_driver.adapters.interface import DockAdapter, DockStatus, ChargingStatus

logger = logging.getLogger(__name__)


class HeishaDockAdapter(DockAdapter):
    """Adapter for Heisha Charging Pad."""

    def __init__(self, config: Dict[str, Any]):
        """
        Initialize the Heisha Charging Pad adapter.
        
        Args:
            config: Configuration dictionary for the adapter.
        """
        self.config = config
        self.rest_api_url = config.get("rest_api_url", "")
        self.modbus_host = config.get("modbus_host", "")
        self.modbus_port = config.get("modbus_port", 502)
        self.modbus_unit_id = config.get("modbus_unit_id", 1)
        self.refresh_interval = config.get("refresh_interval", 15)
        self.username = config.get("username", "")
        self.password = config.get("password", "")
        
        self.modbus_client = None
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
            # Initialize Modbus client
            self.modbus_client = AsyncModbusTcpClient(
                host=self.modbus_host,
                port=self.modbus_port
            )
            
            # Connect to Modbus server
            connected = await self.modbus_client.connect()
            if not connected:
                logger.error(f"Failed to connect to Modbus server at {self.modbus_host}:{self.modbus_port}")
                return False
            
            # Test REST API connection
            async with httpx.AsyncClient() as client:
                auth = None
                if self.username and self.password:
                    auth = (self.username, self.password)
                
                response = await client.get(
                    f"{self.rest_api_url}/status",
                    auth=auth,
                    timeout=10.0
                )
                
                if response.status_code != 200:
                    logger.error(f"Failed to connect to REST API at {self.rest_api_url}")
                    return False
            
            # Start background task for updating status
            self.running = True
            self.update_task = asyncio.create_task(self._update_loop())
            
            logger.info(f"Heisha Charging Pad adapter initialized for dock at {self.modbus_host}")
            return True
        except Exception as e:
            logger.error(f"Error initializing Heisha Charging Pad adapter: {str(e)}")
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
            # Read status registers from Modbus
            if not self.modbus_client or not self.modbus_client.connected:
                await self._reconnect_modbus()
                if not self.modbus_client.connected:
                    raise Exception("Failed to connect to Modbus server")
            
            # Read dock status (register 0)
            result = await self.modbus_client.read_holding_registers(0, 2, slave=self.modbus_unit_id)
            if result.isError():
                raise ModbusException(f"Modbus error: {result}")
            
            dock_status_code = result.registers[0]
            charging_status_code = result.registers[1]
            
            # Parse status codes
            dock_status = self._parse_dock_status(dock_status_code)
            charging_status = self._parse_charging_status(charging_status_code)
            
            # Get additional status from REST API
            rest_status = await self._get_rest_status()
            
            # Combine and cache status
            self.status_cache = {
                "dock_id": f"heisha-{self.modbus_host}",
                "status": dock_status,
                "charging_status": charging_status,
                "door_state": rest_status.get("door_state", "unknown"),
                "drone_connected": rest_status.get("drone_connected", False),
                "error_code": rest_status.get("error_code", 0),
                "error_message": rest_status.get("error_message", ""),
                "timestamp": datetime.now().isoformat()
            }
            
            return self.status_cache
        except Exception as e:
            logger.error(f"Error getting dock status: {str(e)}")
            return {
                "dock_id": f"heisha-{self.modbus_host}",
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
            # Write to dock control register (register 9)
            if not self.modbus_client or not self.modbus_client.connected:
                await self._reconnect_modbus()
                if not self.modbus_client.connected:
                    raise Exception("Failed to connect to Modbus server")
            
            # Write 1 to register 9 to open the dock
            result = await self.modbus_client.write_register(9, 1, slave=self.modbus_unit_id)
            if result.isError():
                raise ModbusException(f"Modbus error: {result}")
            
            logger.info(f"Successfully opened dock at {self.modbus_host}")
            return True
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
            # Write to dock control register (register 9)
            if not self.modbus_client or not self.modbus_client.connected:
                await self._reconnect_modbus()
                if not self.modbus_client.connected:
                    raise Exception("Failed to connect to Modbus server")
            
            # Write 2 to register 9 to close the dock
            result = await self.modbus_client.write_register(9, 2, slave=self.modbus_unit_id)
            if result.isError():
                raise ModbusException(f"Modbus error: {result}")
            
            logger.info(f"Successfully closed dock at {self.modbus_host}")
            return True
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
            # Write to charging control register (register 10)
            if not self.modbus_client or not self.modbus_client.connected:
                await self._reconnect_modbus()
                if not self.modbus_client.connected:
                    raise Exception("Failed to connect to Modbus server")
            
            # Write 1 to register 10 to start charging
            result = await self.modbus_client.write_register(10, 1, slave=self.modbus_unit_id)
            if result.isError():
                raise ModbusException(f"Modbus error: {result}")
            
            logger.info(f"Successfully started charging for dock at {self.modbus_host}")
            return True
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
            # Write to charging control register (register 10)
            if not self.modbus_client or not self.modbus_client.connected:
                await self._reconnect_modbus()
                if not self.modbus_client.connected:
                    raise Exception("Failed to connect to Modbus server")
            
            # Write 0 to register 10 to stop charging
            result = await self.modbus_client.write_register(10, 0, slave=self.modbus_unit_id)
            if result.isError():
                raise ModbusException(f"Modbus error: {result}")
            
            logger.info(f"Successfully stopped charging for dock at {self.modbus_host}")
            return True
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
            # Read telemetry registers from Modbus
            if not self.modbus_client or not self.modbus_client.connected:
                await self._reconnect_modbus()
                if not self.modbus_client.connected:
                    raise Exception("Failed to connect to Modbus server")
            
            # Read registers 2-7 (battery level, charging current, charging voltage, temperature, humidity, fan speed)
            result = await self.modbus_client.read_holding_registers(2, 6, slave=self.modbus_unit_id)
            if result.isError():
                raise ModbusException(f"Modbus error: {result}")
            
            # Parse telemetry data
            battery_level = result.registers[0]
            charging_current = result.registers[1] / 10.0  # Convert to A
            charging_voltage = result.registers[2] / 10.0  # Convert to V
            temperature = result.registers[3] / 10.0  # Convert to °C
            humidity = result.registers[4]
            fan_speed = result.registers[5]
            
            # Get additional telemetry from REST API
            rest_telemetry = await self._get_rest_telemetry()
            
            # Combine and cache telemetry
            self.telemetry_cache = {
                "dock_id": f"heisha-{self.modbus_host}",
                "battery_level": battery_level,
                "charging_current": charging_current,
                "charging_voltage": charging_voltage,
                "temperature": temperature,
                "humidity": humidity,
                "fan_speed": fan_speed,
                "network_signal": rest_telemetry.get("network_signal", 0),
                "timestamp": datetime.now().isoformat()
            }
            
            return self.telemetry_cache
        except Exception as e:
            logger.error(f"Error getting dock telemetry: {str(e)}")
            return {
                "dock_id": f"heisha-{self.modbus_host}",
                "battery_level": 0,
                "charging_current": 0.0,
                "charging_voltage": 0.0,
                "temperature": 0.0,
                "humidity": 0,
                "fan_speed": 0,
                "network_signal": 0,
                "timestamp": datetime.now().isoformat()
            }

    async def set_fan_speed(self, speed: int) -> bool:
        """
        Set the fan speed.
        
        Args:
            speed: Fan speed (0-100%).
            
        Returns:
            bool: True if the operation was successful, False otherwise.
        """
        try:
            # Validate speed
            if speed < 0 or speed > 100:
                raise ValueError("Fan speed must be between 0 and 100")
            
            # Write to fan speed register (register 7)
            if not self.modbus_client or not self.modbus_client.connected:
                await self._reconnect_modbus()
                if not self.modbus_client.connected:
                    raise Exception("Failed to connect to Modbus server")
            
            # Write speed to register 7
            result = await self.modbus_client.write_register(7, speed, slave=self.modbus_unit_id)
            if result.isError():
                raise ModbusException(f"Modbus error: {result}")
            
            logger.info(f"Successfully set fan speed to {speed}% for dock at {self.modbus_host}")
            return True
        except Exception as e:
            logger.error(f"Error setting fan speed: {str(e)}")
            return False

    async def set_heater_state(self, state: bool) -> bool:
        """
        Set the heater state.
        
        Args:
            state: True to turn on the heater, False to turn it off.
            
        Returns:
            bool: True if the operation was successful, False otherwise.
        """
        try:
            # Write to heater status register (register 8)
            if not self.modbus_client or not self.modbus_client.connected:
                await self._reconnect_modbus()
                if not self.modbus_client.connected:
                    raise Exception("Failed to connect to Modbus server")
            
            # Write 1 to register 8 to turn on the heater, 0 to turn it off
            result = await self.modbus_client.write_register(8, 1 if state else 0, slave=self.modbus_unit_id)
            if result.isError():
                raise ModbusException(f"Modbus error: {result}")
            
            logger.info(f"Successfully {'enabled' if state else 'disabled'} heater for dock at {self.modbus_host}")
            return True
        except Exception as e:
            logger.error(f"Error setting heater state: {str(e)}")
            return False

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
        
        if self.modbus_client and self.modbus_client.connected:
            await self.modbus_client.close()
        
        logger.info(f"Heisha Charging Pad adapter for dock at {self.modbus_host} shut down")

    async def _reconnect_modbus(self) -> bool:
        """
        Reconnect to the Modbus server.
        
        Returns:
            bool: True if reconnection was successful, False otherwise.
        """
        try:
            if self.modbus_client:
                await self.modbus_client.close()
            
            self.modbus_client = AsyncModbusTcpClient(
                host=self.modbus_host,
                port=self.modbus_port
            )
            
            connected = await self.modbus_client.connect()
            return connected
        except Exception as e:
            logger.error(f"Error reconnecting to Modbus server: {str(e)}")
            return False

    async def _get_rest_status(self) -> Dict[str, Any]:
        """
        Get status from the REST API.
        
        Returns:
            Dict[str, Any]: Dictionary containing status data.
        """
        try:
            async with httpx.AsyncClient() as client:
                auth = None
                if self.username and self.password:
                    auth = (self.username, self.password)
                
                response = await client.get(
                    f"{self.rest_api_url}/status",
                    auth=auth,
                    timeout=10.0
                )
                
                if response.status_code == 200:
                    return response.json()
                else:
                    logger.error(f"HTTP error: {response.status_code} - {response.text}")
                    return {}
        except Exception as e:
            logger.error(f"Error getting REST status: {str(e)}")
            return {}

    async def _get_rest_telemetry(self) -> Dict[str, Any]:
        """
        Get telemetry from the REST API.
        
        Returns:
            Dict[str, Any]: Dictionary containing telemetry data.
        """
        try:
            async with httpx.AsyncClient() as client:
                auth = None
                if self.username and self.password:
                    auth = (self.username, self.password)
                
                response = await client.get(
                    f"{self.rest_api_url}/telemetry",
                    auth=auth,
                    timeout=10.0
                )
                
                if response.status_code == 200:
                    return response.json()
                else:
                    logger.error(f"HTTP error: {response.status_code} - {response.text}")
                    return {}
        except Exception as e:
            logger.error(f"Error getting REST telemetry: {str(e)}")
            return {}

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
            status_code: Status code from Modbus.
            
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
            status_code: Status code from Modbus.
            
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
