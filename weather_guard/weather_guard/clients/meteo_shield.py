"""
MeteoShield MQTT client for Weather Guard service.

This module provides a client for the MeteoShield weather station, which uses
an ESP32 with BME280 sensor to collect local weather data and publish it via MQTT.
"""

import asyncio
import json
import logging
import time
from datetime import datetime
from typing import Dict, List, Optional, Any, Callable, Tuple

import aiomqtt
from pydantic import ValidationError

from weather_guard.core.config import settings
from weather_guard.models.weather import (
    WeatherCondition,
    WeatherData,
    WeatherProvider,
)

logger = logging.getLogger(__name__)


class MeteoShieldClient:
    """Client for the MeteoShield weather station via MQTT."""
    
    def __init__(
        self,
        broker: Optional[str] = None,
        port: Optional[int] = None,
        username: Optional[str] = None,
        password: Optional[str] = None,
        topic_prefix: Optional[str] = None,
        qos: Optional[int] = None,
        callback: Optional[Callable[[WeatherData], None]] = None,
    ):
        """Initialize the MeteoShield MQTT client.
        
        Args:
            broker: MQTT broker hostname or IP
            port: MQTT broker port
            username: MQTT username
            password: MQTT password
            topic_prefix: MQTT topic prefix
            qos: MQTT QoS level
            callback: Callback function for new weather data
        """
        self.broker = broker or settings.MQTT_BROKER
        self.port = port or settings.MQTT_PORT
        self.username = username or settings.MQTT_USERNAME
        self.password = password or settings.MQTT_PASSWORD
        self.topic_prefix = topic_prefix or settings.MQTT_TOPIC_PREFIX
        self.qos = qos or settings.MQTT_QOS
        self.callback = callback
        
        self.client = None
        self.task = None
        self.running = False
        self.connected = False
        
        # Store latest values
        self.latest_data: Dict[str, Dict[str, Any]] = {}
        self.last_update_time: Dict[str, float] = {}
    
    async def connect(self) -> bool:
        """Connect to the MQTT broker.
        
        Returns:
            True if connected successfully, False otherwise
        """
        try:
            # Create MQTT client
            mqtt_options = {
                "hostname": self.broker,
                "port": self.port,
            }
            
            if self.username and self.password:
                mqtt_options["username"] = self.username
                mqtt_options["password"] = self.password
            
            self.client = aiomqtt.Client(**mqtt_options)
            await self.client.__aenter__()
            
            # Subscribe to topics
            await self.client.subscribe(f"{self.topic_prefix}/+/+", qos=self.qos)
            
            self.connected = True
            logger.info(f"Connected to MQTT broker at {self.broker}:{self.port}")
            return True
        
        except Exception as e:
            logger.error(f"Error connecting to MQTT broker: {str(e)}")
            self.connected = False
            return False
    
    async def disconnect(self) -> None:
        """Disconnect from the MQTT broker."""
        if self.client:
            try:
                await self.client.__aexit__(None, None, None)
                self.client = None
                self.connected = False
                logger.info("Disconnected from MQTT broker")
            except Exception as e:
                logger.error(f"Error disconnecting from MQTT broker: {str(e)}")
    
    async def start(self) -> None:
        """Start the MQTT client."""
        if self.running:
            return
        
        self.running = True
        self.task = asyncio.create_task(self._run())
    
    async def stop(self) -> None:
        """Stop the MQTT client."""
        if not self.running:
            return
        
        self.running = False
        if self.task:
            self.task.cancel()
            try:
                await self.task
            except asyncio.CancelledError:
                pass
            self.task = None
        
        await self.disconnect()
    
    async def _run(self) -> None:
        """Run the MQTT client loop."""
        while self.running:
            try:
                if not self.connected:
                    success = await self.connect()
                    if not success:
                        # Wait before retrying
                        await asyncio.sleep(5)
                        continue
                
                # Process messages
                async for message in self.client.messages:
                    try:
                        # Parse topic
                        topic_parts = message.topic.value.decode().split('/')
                        if len(topic_parts) < 3:
                            continue
                        
                        device_id = topic_parts[1]
                        measurement = topic_parts[2]
                        
                        # Parse payload
                        try:
                            payload = message.payload.decode()
                            value = float(payload)
                        except (ValueError, UnicodeDecodeError):
                            logger.warning(f"Invalid payload for topic {message.topic.value.decode()}: {message.payload}")
                            continue
                        
                        # Store value
                        if device_id not in self.latest_data:
                            self.latest_data[device_id] = {}
                        
                        self.latest_data[device_id][measurement] = value
                        self.last_update_time[device_id] = time.time()
                        
                        # Check if we have all required measurements
                        if self._has_required_measurements(device_id):
                            # Create weather data
                            weather_data = self._create_weather_data(device_id)
                            
                            # Call callback if provided
                            if self.callback and weather_data:
                                self.callback(weather_data)
                    
                    except Exception as e:
                        logger.error(f"Error processing MQTT message: {str(e)}")
            
            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error(f"Error in MQTT client loop: {str(e)}")
                await self.disconnect()
                # Wait before retrying
                await asyncio.sleep(5)
    
    def _has_required_measurements(self, device_id: str) -> bool:
        """Check if we have all required measurements for a device.
        
        Args:
            device_id: Device ID
            
        Returns:
            True if we have all required measurements, False otherwise
        """
        required = ["temperature", "humidity", "pressure"]
        
        # Wind and rain are optional but preferred
        if device_id in self.latest_data:
            return all(m in self.latest_data[device_id] for m in required)
        
        return False
    
    def _create_weather_data(self, device_id: str) -> Optional[WeatherData]:
        """Create weather data from latest measurements.
        
        Args:
            device_id: Device ID
            
        Returns:
            Weather data or None if not enough data
        """
        if device_id not in self.latest_data:
            return None
        
        data = self.latest_data[device_id]
        
        # Get location from settings or use default
        # In a real implementation, this would be stored in a database
        # or provided by the device itself
        latitude = 0.0
        longitude = 0.0
        
        # Determine weather condition based on available data
        condition = WeatherCondition.UNKNOWN
        if "rain" in data and data["rain"] > 0:
            if data["rain"] > 5.0:
                condition = WeatherCondition.HEAVY_RAIN
            elif data["rain"] > 2.0:
                condition = WeatherCondition.RAIN
            else:
                condition = WeatherCondition.LIGHT_RAIN
        elif "wind_speed" in data and data["wind_speed"] > 8.0:
            condition = WeatherCondition.WINDY
        elif "humidity" in data and data["humidity"] > 90:
            condition = WeatherCondition.FOG
        else:
            condition = WeatherCondition.CLEAR
        
        try:
            # Create weather data
            weather_data = WeatherData(
                latitude=latitude,
                longitude=longitude,
                timestamp=datetime.utcnow(),
                provider=WeatherProvider.METEO_SHIELD,
                condition=condition,
                temperature=data.get("temperature", 20.0),
                feels_like=None,  # Not provided by MeteoShield
                humidity=data.get("humidity", None),
                pressure=data.get("pressure", None),
                wind_speed=data.get("wind_speed", 0.0),
                wind_direction=data.get("wind_direction", None),
                wind_gust=data.get("wind_gust", None),
                visibility=None,  # Not provided by MeteoShield
                cloud_cover=None,  # Not provided by MeteoShield
                precipitation=data.get("rain", 0.0),
                uv_index=None,  # Not provided by MeteoShield
            )
            
            return weather_data
        
        except ValidationError as e:
            logger.error(f"Error creating weather data: {str(e)}")
            return None
    
    def get_latest_weather(self, device_id: Optional[str] = None) -> Optional[WeatherData]:
        """Get latest weather data from a device.
        
        Args:
            device_id: Device ID (if None, use the first available device)
            
        Returns:
            Latest weather data or None if not available
        """
        if device_id:
            if device_id in self.latest_data and self._has_required_measurements(device_id):
                return self._create_weather_data(device_id)
        else:
            # Use the first device with required measurements
            for device_id in self.latest_data:
                if self._has_required_measurements(device_id):
                    return self._create_weather_data(device_id)
        
        return None
    
    def get_device_ids(self) -> List[str]:
        """Get list of available device IDs.
        
        Returns:
            List of device IDs
        """
        return list(self.latest_data.keys())
    
    def get_device_status(self, device_id: str) -> Dict[str, Any]:
        """Get status of a device.
        
        Args:
            device_id: Device ID
            
        Returns:
            Device status
        """
        if device_id not in self.latest_data:
            return {
                "device_id": device_id,
                "connected": False,
                "last_update": None,
                "measurements": {},
            }
        
        return {
            "device_id": device_id,
            "connected": True,
            "last_update": self.last_update_time.get(device_id, 0),
            "measurements": self.latest_data[device_id],
        }
