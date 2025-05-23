#!/usr/bin/env python3
"""
Network simulation example for the SATCOM / 5G Fallback Connectivity module.

This example demonstrates how to simulate various network conditions and test the fallback system.
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
from comms_fallback.tests.test_network_simulation import NetworkSimulator


async def main():
    """Main entry point."""
    # Set up logging
    setup_logging(log_level="DEBUG")
    logger = logging.getLogger("comms_fallback.examples.network_simulation_example")
    
    # Create network simulator
    network_simulator = NetworkSimulator()
    await network_simulator.setup()
    
    if not network_simulator.active:
        logger.warning("Network simulator not active, continuing without simulation")
    
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
        
        # Send some messages with normal network conditions
        logger.info("=== Normal Network Conditions ===")
        for i in range(3):
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
                },
            }
            
            # Send message
            logger.info(f"Sending message: {message}")
            success = await fallback_manager.send_message(message, critical=(i % 2 == 0))
            logger.info(f"Message sent: {success}")
            
            # Wait a bit
            await asyncio.sleep(1.0)
        
        # Simulate high packet loss
        if network_simulator.active:
            logger.info("=== Simulating High Packet Loss (20%) ===")
            await network_simulator.simulate_packet_loss(20.0)
            
            # Wait for the network conditions to take effect
            await asyncio.sleep(2.0)
            
            # Send some messages with high packet loss
            for i in range(3):
                message = {
                    "type": "telemetry",
                    "data": {
                        "timestamp": time.time(),
                        "sequence": i + 3,
                        "position": {
                            "latitude": 37.7749 + ((i + 3) * 0.001),
                            "longitude": -122.4194 + ((i + 3) * 0.001),
                            "altitude": 100 + ((i + 3) * 10),
                        },
                    },
                }
                
                # Send message
                logger.info(f"Sending message: {message}")
                success = await fallback_manager.send_message(message, critical=(i % 2 == 0))
                logger.info(f"Message sent: {success}")
                
                # Wait a bit
                await asyncio.sleep(1.0)
        
        # Simulate high latency
        if network_simulator.active:
            logger.info("=== Simulating High Latency (500ms) ===")
            await network_simulator.simulate_high_latency(500.0)
            
            # Wait for the network conditions to take effect
            await asyncio.sleep(2.0)
            
            # Send some messages with high latency
            for i in range(3):
                message = {
                    "type": "telemetry",
                    "data": {
                        "timestamp": time.time(),
                        "sequence": i + 6,
                        "position": {
                            "latitude": 37.7749 + ((i + 6) * 0.001),
                            "longitude": -122.4194 + ((i + 6) * 0.001),
                            "altitude": 100 + ((i + 6) * 10),
                        },
                    },
                }
                
                # Send message
                logger.info(f"Sending message: {message}")
                success = await fallback_manager.send_message(message, critical=(i % 2 == 0))
                logger.info(f"Message sent: {success}")
                
                # Wait a bit
                await asyncio.sleep(1.0)
        
        # Simulate complex network conditions
        if network_simulator.active:
            logger.info("=== Simulating Complex Network Conditions ===")
            await network_simulator.simulate_complex_conditions(10.0, 200.0, 50.0)
            
            # Wait for the network conditions to take effect
            await asyncio.sleep(2.0)
            
            # Send some messages with complex network conditions
            for i in range(3):
                message = {
                    "type": "telemetry",
                    "data": {
                        "timestamp": time.time(),
                        "sequence": i + 9,
                        "position": {
                            "latitude": 37.7749 + ((i + 9) * 0.001),
                            "longitude": -122.4194 + ((i + 9) * 0.001),
                            "altitude": 100 + ((i + 9) * 10),
                        },
                    },
                }
                
                # Send message
                logger.info(f"Sending message: {message}")
                success = await fallback_manager.send_message(message, critical=(i % 2 == 0))
                logger.info(f"Message sent: {success}")
                
                # Wait a bit
                await asyncio.sleep(1.0)
        
        # Reset network conditions
        if network_simulator.active:
            logger.info("=== Resetting Network Conditions ===")
            await network_simulator.teardown()
            
            # Wait for the network conditions to take effect
            await asyncio.sleep(2.0)
        
        # Get status
        status = fallback_manager.get_status()
        logger.info(f"Fallback manager status: {status}")
        
    finally:
        # Stop fallback manager
        await fallback_manager.stop()
        logger.info("Fallback manager stopped")
        
        # Clean up network simulator
        if network_simulator.active:
            await network_simulator.teardown()


if __name__ == "__main__":
    asyncio.run(main())
