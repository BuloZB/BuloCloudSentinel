#!/usr/bin/env python3
"""
Simple example of using the SATCOM / 5G Fallback Connectivity module.

This example demonstrates how to initialize the fallback manager and send messages.
"""

import asyncio
import logging
import os
import sys
import time
from typing import Dict, Any

# Add the parent directory to the path so we can import the module
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../..")))

from comms_fallback.adapters import WifiMeshAdapter, FiveGAdapter, IridiumCertusAdapter
from comms_fallback.services import FallbackManager
from comms_fallback.utils import setup_logging


async def main():
    """Main entry point."""
    # Set up logging
    setup_logging(log_level="DEBUG")
    logger = logging.getLogger("comms_fallback.examples.simple_example")
    
    # Create adapter configurations
    wifi_mesh_config = {
        "name": "WiFi Mesh",
        "priority": 1,
        "mesh_node_id": "drone1",
        "min_rssi": -75,
        "max_latency": 100,
        "max_packet_loss": 5.0,
        "max_jitter": 20.0,
    }
    
    five_g_config = {
        "name": "5G",
        "priority": 2,
        "apn": "internet",
        "device": "/dev/ttyUSB0",
        "min_rssi": -90,
        "max_latency": 150,
        "max_packet_loss": 10.0,
        "max_jitter": 30.0,
    }
    
    iridium_config = {
        "name": "Iridium Certus",
        "priority": 3,
        "device": "/dev/ttyUSB1",
        "min_rssi": -100,
        "max_latency": 1000,
        "max_packet_loss": 15.0,
        "max_jitter": 100.0,
    }
    
    # Create adapters
    adapters = {
        "wifi_mesh": WifiMeshAdapter(wifi_mesh_config),
        "five_g": FiveGAdapter(five_g_config),
        "iridium": IridiumCertusAdapter(iridium_config),
    }
    
    # Create fallback manager configuration
    manager_config = {
        "monitor": {
            "check_interval": 1.0,
            "hysteresis_time": 5.0,
            "hysteresis_count": 5,
        },
        "fallback": {
            "buffer_size": 100,
            "critical_timeout": 30,
            "session_timeout": 300,
        },
        "wireguard": {
            "enabled": False,
        },
    }
    
    # Create fallback manager
    fallback_manager = FallbackManager(
        config=manager_config,
        adapters=adapters,
    )
    
    # Start fallback manager
    await fallback_manager.start()
    logger.info("Fallback manager started")
    
    try:
        # Wait for manager to connect to an adapter
        for _ in range(10):
            if fallback_manager.active_adapter_id:
                break
            await asyncio.sleep(0.5)
        
        if not fallback_manager.active_adapter_id:
            logger.error("No active adapter available")
            return
        
        logger.info(f"Active adapter: {fallback_manager.active_adapter_id}")
        
        # Send some messages
        for i in range(5):
            message = {
                "type": "telemetry",
                "data": {
                    "timestamp": time.time(),
                    "sequence": i,
                    "position": {
                        "latitude": 37.7749 + (i * 0.001),
                        "longitude": -122.4194 + (i * 0.001),
                        "altitude": 100 + (i * 10),
                    },
                    "attitude": {
                        "roll": 0.0,
                        "pitch": 0.0,
                        "yaw": 0.0,
                    },
                    "battery": {
                        "voltage": 12.0 - (i * 0.1),
                        "current": 5.0,
                        "remaining": 80 - (i * 2),
                    },
                },
            }
            
            # Send message
            logger.info(f"Sending message: {message}")
            success = await fallback_manager.send_message(message, critical=(i % 2 == 0))
            logger.info(f"Message sent: {success}")
            
            # Wait a bit
            await asyncio.sleep(1.0)
        
        # Simulate adapter failure
        logger.info("Simulating adapter failure...")
        active_adapter_id = fallback_manager.active_adapter_id
        adapters[active_adapter_id].status = adapters[active_adapter_id].status.__class__.ERROR
        
        # Wait for manager to switch to another adapter
        for _ in range(20):
            if fallback_manager.active_adapter_id and fallback_manager.active_adapter_id != active_adapter_id:
                break
            await asyncio.sleep(0.5)
        
        logger.info(f"New active adapter: {fallback_manager.active_adapter_id}")
        
        # Send some more messages
        for i in range(5):
            message = {
                "type": "command",
                "data": {
                    "timestamp": time.time(),
                    "sequence": i + 5,
                    "command": "move",
                    "parameters": {
                        "latitude": 37.7749 + ((i + 5) * 0.001),
                        "longitude": -122.4194 + ((i + 5) * 0.001),
                        "altitude": 100 + ((i + 5) * 10),
                    },
                },
            }
            
            # Send message
            logger.info(f"Sending message: {message}")
            success = await fallback_manager.send_message(message, critical=(i % 2 == 0))
            logger.info(f"Message sent: {success}")
            
            # Wait a bit
            await asyncio.sleep(1.0)
        
        # Get status
        status = fallback_manager.get_status()
        logger.info(f"Fallback manager status: {status}")
        
    finally:
        # Stop fallback manager
        await fallback_manager.stop()
        logger.info("Fallback manager stopped")


if __name__ == "__main__":
    asyncio.run(main())
