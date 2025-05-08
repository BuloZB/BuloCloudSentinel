"""
ESP32 Dock Adapter

This module implements the adapter for DIY ESP32-powered docks using MQTT.
"""

import logging
import asyncio
import json
from typing import Dict, Any, Optional, List, Callable
from datetime import datetime
import aiomqtt
from aiomqtt.client import Client as MQTTClient

from dock_driver.adapters.interface import DockAdapter, DockStatus, ChargingStatus

logger = logging.getLogger(__name__)


class ESP32DockAdapter(DockAdapter):
    """Adapter for DIY ESP32-powered docks."""

    def __init__(self, config: Dict[str, Any]):
        """
        Initialize the ESP32 Dock adapter.
        
        Args:
            config: Configuration dictionary for the adapter.
        """
        self.config = config
        self.mqtt_broker = config.get("mqtt_broker", "")
        self.mqtt_port = config.get("mqtt_port", 1883)
        self.mqtt_username = config.get("mqtt_username", "")
        self.mqtt_password = config.get("mqtt_password", "")
        self.mqtt_topic_prefix = config.get("mqtt_topic_prefix", "esp32_dock")
        self.dock_id = config.get("dock_id", "esp32-dock-1")
        self.refresh_interval = config.get("refresh_interval", 10)
        
        self.mqtt_client = None
        self.status_cache = {}
        self.telemetry_cache = {}
        self.update_task = None
        self.running = False
        self.available = False
        
        # Initialize telemetry values
        self.voltage = 0.0
        self.current = 0.0
        self.temperature = 0.0
        self.relay_state = False

    async def initialize(self) -> bool:
        """
        Initialize the adapter.
        
        Returns:
            bool: True if initialization was successful, False otherwise.
        """
        try:
            # Connect to MQTT broker
            self.mqtt_client = aiomqtt.Client(
                hostname=self.mqtt_broker,
                port=self.mqtt_port,
                username=self.mqtt_username or None,
                password=self.mqtt_password or None,
                identifier=f"dock-driver-{self.dock_id}"
            )
            
            # Start MQTT client
            await self.mqtt_client.connect()
            
            # Subscribe to topics
            await self._subscribe_to_topics()
            
            # Start background task for updating status
            self.running = True
            self.update_task = asyncio.create_task(self._update_loop())
            
            logger.info(f"ESP32 Dock adapter initialized for dock {self.dock_id}")
            return True
        except Exception as e:
            logger.error(f"Error initializing ESP32 Dock adapter: {str(e)}")
            return False

    async def get_status(self) -> Dict[str, Any]:
        """
        Get the current status of the dock.
        
        Returns:
            Dict[str, Any]: Dictionary containing the dock status.
        """
        # Return cached status if available and recent
        if self.status_cache:
            return self.status_cache
        
        # Otherwise, create status from current values
        try:
            # Determine dock status
            if not self.available:
                dock_status = DockStatus.OFFLINE
            else:
                dock_status = DockStatus.ONLINE
            
            # Determine charging status
            if not self.relay_state:
                charging_status = ChargingStatus.IDLE
            else:
                if self.current > 0.1:  # Current above 0.1A indicates charging
                    charging_status = ChargingStatus.CHARGING
                else:
                    charging_status = ChargingStatus.COMPLETE
            
            # Create and cache status
            self.status_cache = {
                "dock_id": self.dock_id,
                "status": dock_status,
                "charging_status": charging_status,
                "relay_state": self.relay_state,
                "available": self.available,
                "timestamp": datetime.now().isoformat()
            }
            
            return self.status_cache
        except Exception as e:
            logger.error(f"Error getting dock status: {str(e)}")
            return {
                "dock_id": self.dock_id,
                "status": DockStatus.ERROR,
                "charging_status": ChargingStatus.UNKNOWN,
                "relay_state": False,
                "available": False,
                "timestamp": datetime.now().isoformat()
            }

    async def open_dock(self) -> bool:
        """
        Open the dock.
        
        Note: ESP32 docks don't have a door, so this is a no-op.
        
        Returns:
            bool: True if the operation was successful, False otherwise.
        """
        logger.warning(f"ESP32 dock {self.dock_id} doesn't support open_dock operation")
        return True

    async def close_dock(self) -> bool:
        """
        Close the dock.
        
        Note: ESP32 docks don't have a door, so this is a no-op.
        
        Returns:
            bool: True if the operation was successful, False otherwise.
        """
        logger.warning(f"ESP32 dock {self.dock_id} doesn't support close_dock operation")
        return True

    async def start_charging(self) -> bool:
        """
        Start charging the drone.
        
        Returns:
            bool: True if the operation was successful, False otherwise.
        """
        return await self.set_relay_state(True)

    async def stop_charging(self) -> bool:
        """
        Stop charging the drone.
        
        Returns:
            bool: True if the operation was successful, False otherwise.
        """
        return await self.set_relay_state(False)

    async def set_relay_state(self, state: bool) -> bool:
        """
        Set the relay state.
        
        Args:
            state: True to turn on the relay, False to turn it off.
            
        Returns:
            bool: True if the operation was successful, False otherwise.
        """
        try:
            if not self.mqtt_client:
                raise Exception("MQTT client not initialized")
            
            # Publish relay state
            topic = f"{self.mqtt_topic_prefix}/{self.dock_id}/relay/set"
            payload = "ON" if state else "OFF"
            
            await self.mqtt_client.publish(topic, payload)
            
            logger.info(f"Successfully {'enabled' if state else 'disabled'} relay for dock {self.dock_id}")
            return True
        except Exception as e:
            logger.error(f"Error setting relay state: {str(e)}")
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
        
        # Otherwise, create telemetry from current values
        try:
            # Create and cache telemetry
            self.telemetry_cache = {
                "dock_id": self.dock_id,
                "voltage": self.voltage,
                "current": self.current,
                "temperature": self.temperature,
                "relay_state": self.relay_state,
                "timestamp": datetime.now().isoformat()
            }
            
            return self.telemetry_cache
        except Exception as e:
            logger.error(f"Error getting dock telemetry: {str(e)}")
            return {
                "dock_id": self.dock_id,
                "voltage": 0.0,
                "current": 0.0,
                "temperature": 0.0,
                "relay_state": False,
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
        
        if self.mqtt_client:
            await self.mqtt_client.disconnect()
        
        logger.info(f"ESP32 Dock adapter for dock {self.dock_id} shut down")

    async def _subscribe_to_topics(self) -> None:
        """
        Subscribe to MQTT topics.
        """
        if not self.mqtt_client:
            return
        
        # Subscribe to status topic
        await self.mqtt_client.subscribe(f"{self.mqtt_topic_prefix}/{self.dock_id}/status")
        
        # Subscribe to voltage topic
        await self.mqtt_client.subscribe(f"{self.mqtt_topic_prefix}/{self.dock_id}/voltage")
        
        # Subscribe to current topic
        await self.mqtt_client.subscribe(f"{self.mqtt_topic_prefix}/{self.dock_id}/current")
        
        # Subscribe to temperature topic
        await self.mqtt_client.subscribe(f"{self.mqtt_topic_prefix}/{self.dock_id}/temperature")
        
        # Subscribe to relay state topic
        await self.mqtt_client.subscribe(f"{self.mqtt_topic_prefix}/{self.dock_id}/relay")
        
        # Subscribe to availability topic
        await self.mqtt_client.subscribe(f"{self.mqtt_topic_prefix}/{self.dock_id}/available")

    async def _update_loop(self) -> None:
        """
        Background task for updating status and telemetry.
        """
        while self.running:
            try:
                # Process MQTT messages
                await self._process_mqtt_messages()
                
                # Clear caches to force refresh
                self.status_cache = {}
                self.telemetry_cache = {}
                
                # Sleep for refresh interval
                await asyncio.sleep(self.refresh_interval)
            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error(f"Error in update loop: {str(e)}")
                await asyncio.sleep(self.refresh_interval)

    async def _process_mqtt_messages(self) -> None:
        """
        Process MQTT messages.
        """
        if not self.mqtt_client:
            return
        
        try:
            async with self.mqtt_client.messages() as messages:
                async for message in messages:
                    topic = message.topic.value
                    payload = message.payload.decode()
                    
                    # Process message based on topic
                    if topic.endswith("/status"):
                        # Process status message
                        self._process_status_message(payload)
                    elif topic.endswith("/voltage"):
                        # Process voltage message
                        self._process_voltage_message(payload)
                    elif topic.endswith("/current"):
                        # Process current message
                        self._process_current_message(payload)
                    elif topic.endswith("/temperature"):
                        # Process temperature message
                        self._process_temperature_message(payload)
                    elif topic.endswith("/relay"):
                        # Process relay state message
                        self._process_relay_message(payload)
                    elif topic.endswith("/available"):
                        # Process availability message
                        self._process_availability_message(payload)
        except Exception as e:
            logger.error(f"Error processing MQTT messages: {str(e)}")

    def _process_status_message(self, payload: str) -> None:
        """
        Process status message.
        
        Args:
            payload: Message payload.
        """
        try:
            # Parse JSON payload
            status_data = json.loads(payload)
            
            # Update status values
            if "relay_state" in status_data:
                self.relay_state = status_data["relay_state"]
            
            # Clear status cache to force refresh
            self.status_cache = {}
        except Exception as e:
            logger.error(f"Error processing status message: {str(e)}")

    def _process_voltage_message(self, payload: str) -> None:
        """
        Process voltage message.
        
        Args:
            payload: Message payload.
        """
        try:
            # Parse payload as float
            self.voltage = float(payload)
            
            # Clear telemetry cache to force refresh
            self.telemetry_cache = {}
        except Exception as e:
            logger.error(f"Error processing voltage message: {str(e)}")

    def _process_current_message(self, payload: str) -> None:
        """
        Process current message.
        
        Args:
            payload: Message payload.
        """
        try:
            # Parse payload as float
            self.current = float(payload)
            
            # Clear telemetry cache to force refresh
            self.telemetry_cache = {}
        except Exception as e:
            logger.error(f"Error processing current message: {str(e)}")

    def _process_temperature_message(self, payload: str) -> None:
        """
        Process temperature message.
        
        Args:
            payload: Message payload.
        """
        try:
            # Parse payload as float
            self.temperature = float(payload)
            
            # Clear telemetry cache to force refresh
            self.telemetry_cache = {}
        except Exception as e:
            logger.error(f"Error processing temperature message: {str(e)}")

    def _process_relay_message(self, payload: str) -> None:
        """
        Process relay state message.
        
        Args:
            payload: Message payload.
        """
        try:
            # Parse payload as boolean
            self.relay_state = payload.upper() == "ON"
            
            # Clear caches to force refresh
            self.status_cache = {}
            self.telemetry_cache = {}
        except Exception as e:
            logger.error(f"Error processing relay message: {str(e)}")

    def _process_availability_message(self, payload: str) -> None:
        """
        Process availability message.
        
        Args:
            payload: Message payload.
        """
        try:
            # Parse payload as boolean
            self.available = payload.upper() == "ONLINE"
            
            # Clear status cache to force refresh
            self.status_cache = {}
        except Exception as e:
            logger.error(f"Error processing availability message: {str(e)}")
