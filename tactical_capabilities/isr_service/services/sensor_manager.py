"""
Sensor manager service for the ISR service.

This service is responsible for managing sensors, including:
- Connecting to sensors
- Polling sensors for data
- Processing sensor data
- Publishing sensor events
"""

import asyncio
import logging
from typing import Dict, List, Optional, Any
from uuid import UUID

from core.config import settings

logger = logging.getLogger(__name__)

class SensorManager:
    """Sensor manager service."""
    
    def __init__(self):
        """Initialize the sensor manager."""
        self.sensors = {}
        self.polling_tasks = {}
        self.polling_interval = settings.SENSOR_POLLING_INTERVAL
        self.running = False
    
    async def connect(self):
        """Connect to the sensor manager."""
        logger.info("Connecting to sensor manager")
        self.running = True
    
    async def disconnect(self):
        """Disconnect from the sensor manager."""
        logger.info("Disconnecting from sensor manager")
        self.running = False
        
        # Cancel all polling tasks
        for task in self.polling_tasks.values():
            task.cancel()
        
        # Wait for all tasks to complete
        if self.polling_tasks:
            await asyncio.gather(*self.polling_tasks.values(), return_exceptions=True)
        
        self.polling_tasks = {}
    
    async def register_sensor(self, sensor_id: UUID, sensor_config: Dict[str, Any]):
        """
        Register a sensor with the manager.
        
        Args:
            sensor_id: Sensor ID
            sensor_config: Sensor configuration
        """
        logger.info(f"Registering sensor {sensor_id}")
        
        # Store sensor configuration
        self.sensors[sensor_id] = sensor_config
        
        # Start polling task if not already running
        if sensor_id not in self.polling_tasks and self.running:
            self.polling_tasks[sensor_id] = asyncio.create_task(
                self._poll_sensor(sensor_id)
            )
    
    async def unregister_sensor(self, sensor_id: UUID):
        """
        Unregister a sensor from the manager.
        
        Args:
            sensor_id: Sensor ID
        """
        logger.info(f"Unregistering sensor {sensor_id}")
        
        # Remove sensor configuration
        if sensor_id in self.sensors:
            del self.sensors[sensor_id]
        
        # Cancel polling task
        if sensor_id in self.polling_tasks:
            self.polling_tasks[sensor_id].cancel()
            try:
                await self.polling_tasks[sensor_id]
            except asyncio.CancelledError:
                pass
            del self.polling_tasks[sensor_id]
    
    async def get_sensor_data(self, sensor_id: UUID) -> Optional[Dict[str, Any]]:
        """
        Get data from a sensor.
        
        Args:
            sensor_id: Sensor ID
            
        Returns:
            Sensor data or None if sensor not found
        """
        if sensor_id not in self.sensors:
            logger.warning(f"Sensor {sensor_id} not found")
            return None
        
        # In a real implementation, this would connect to the sensor and get data
        # For now, we'll just return a placeholder
        return {
            "sensor_id": str(sensor_id),
            "timestamp": "2023-01-01T00:00:00Z",
            "data": {
                "value": 0.0
            }
        }
    
    async def _poll_sensor(self, sensor_id: UUID):
        """
        Poll a sensor for data.
        
        Args:
            sensor_id: Sensor ID
        """
        logger.info(f"Starting polling task for sensor {sensor_id}")
        
        try:
            while self.running and sensor_id in self.sensors:
                # Get sensor data
                data = await self.get_sensor_data(sensor_id)
                
                if data:
                    # Process data (in a real implementation, this would do something with the data)
                    logger.debug(f"Received data from sensor {sensor_id}: {data}")
                
                # Wait for next polling interval
                await asyncio.sleep(self.polling_interval)
        except asyncio.CancelledError:
            logger.info(f"Polling task for sensor {sensor_id} cancelled")
            raise
        except Exception as e:
            logger.exception(f"Error in polling task for sensor {sensor_id}: {e}")
    
    async def update_sensor_config(self, sensor_id: UUID, config: Dict[str, Any]):
        """
        Update sensor configuration.
        
        Args:
            sensor_id: Sensor ID
            config: Sensor configuration
        """
        if sensor_id not in self.sensors:
            logger.warning(f"Sensor {sensor_id} not found")
            return
        
        # Update sensor configuration
        self.sensors[sensor_id].update(config)
        
        # In a real implementation, this would apply the configuration to the sensor
        logger.info(f"Updated configuration for sensor {sensor_id}")
    
    async def get_sensor_status(self, sensor_id: UUID) -> Optional[str]:
        """
        Get sensor status.
        
        Args:
            sensor_id: Sensor ID
            
        Returns:
            Sensor status or None if sensor not found
        """
        if sensor_id not in self.sensors:
            logger.warning(f"Sensor {sensor_id} not found")
            return None
        
        # In a real implementation, this would check the actual sensor status
        return "online" if self.running else "offline"
