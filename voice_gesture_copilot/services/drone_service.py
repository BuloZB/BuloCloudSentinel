"""
Drone Service for Voice & Gesture Co-Pilot.

This service provides an interface for interacting with drones through
the Bulo.Cloud Sentinel API.
"""

import os
import time
import logging
import asyncio
from typing import Dict, Any, List, Optional, Tuple
from datetime import datetime

import httpx
from loguru import logger

from voice_gesture_copilot.core.config import settings

class DroneService:
    """
    Service for interacting with drones.
    
    This service provides methods for sending commands to drones and
    retrieving drone information through the Bulo.Cloud Sentinel API.
    """
    
    def __init__(self, api_url: str, api_token: str):
        """
        Initialize the drone service.
        
        Args:
            api_url: URL of the Bulo.Cloud Sentinel API
            api_token: API token for authentication
        """
        self.api_url = api_url
        self.api_token = api_token
        self.headers = {
            "Authorization": f"Bearer {api_token}",
            "Content-Type": "application/json",
        }
        self.is_initialized = True
        self.active_drones = {}
        self.command_history = []
        
        # Performance metrics
        self.performance_metrics = {
            "command_execution": {
                "latency_ms": [],
                "success_rate": [],
            },
        }
    
    async def cleanup(self):
        """
        Clean up resources used by the drone service.
        """
        logger.info("Cleaning up drone service resources")
        self.is_initialized = False
        logger.info("Drone service resources cleaned up")
    
    async def get_drones(self) -> List[Dict[str, Any]]:
        """
        Get a list of available drones.
        
        Returns:
            List of drone information dictionaries
        """
        if not self.is_initialized:
            logger.error("Drone service not initialized")
            return []
        
        try:
            async with httpx.AsyncClient() as client:
                response = await client.get(
                    f"{self.api_url}/api/drones",
                    headers=self.headers,
                    timeout=10.0,
                )
                response.raise_for_status()
                
                drones = response.json()
                
                # Update active drones cache
                for drone in drones:
                    drone_id = drone.get("id", "")
                    if drone_id:
                        self.active_drones[drone_id] = drone
                
                return drones
        
        except Exception as e:
            logger.error(f"Error getting drones: {str(e)}")
            return []
    
    async def get_drone(self, drone_id: str) -> Optional[Dict[str, Any]]:
        """
        Get information about a specific drone.
        
        Args:
            drone_id: ID of the drone
            
        Returns:
            Drone information dictionary, or None if not found
        """
        if not self.is_initialized:
            logger.error("Drone service not initialized")
            return None
        
        try:
            # Check if drone is in cache
            if drone_id in self.active_drones:
                return self.active_drones[drone_id]
            
            # If not in cache, fetch from API
            async with httpx.AsyncClient() as client:
                response = await client.get(
                    f"{self.api_url}/api/drones/{drone_id}",
                    headers=self.headers,
                    timeout=10.0,
                )
                response.raise_for_status()
                
                drone = response.json()
                
                # Update cache
                self.active_drones[drone_id] = drone
                
                return drone
        
        except Exception as e:
            logger.error(f"Error getting drone {drone_id}: {str(e)}")
            return None
    
    async def send_command(self, drone_id: str, command: str, parameters: Dict[str, Any] = None) -> Dict[str, Any]:
        """
        Send a command to a drone.
        
        Args:
            drone_id: ID of the drone
            command: Command to send
            parameters: Optional command parameters
            
        Returns:
            Command execution result
        """
        if not self.is_initialized:
            logger.error("Drone service not initialized")
            return {"success": False, "error": "Drone service not initialized"}
        
        if not drone_id:
            logger.error("Drone ID is required")
            return {"success": False, "error": "Drone ID is required"}
        
        if not command:
            logger.error("Command is required")
            return {"success": False, "error": "Command is required"}
        
        try:
            # Measure latency
            start_time = time.time()
            
            # Prepare command data
            command_data = {
                "command": command,
                "parameters": parameters or {},
            }
            
            # Send command to drone
            async with httpx.AsyncClient() as client:
                response = await client.post(
                    f"{self.api_url}/api/drones/{drone_id}/command",
                    headers=self.headers,
                    json=command_data,
                    timeout=30.0,
                )
                response.raise_for_status()
                
                result = response.json()
                
                # Calculate latency
                latency_ms = (time.time() - start_time) * 1000
                self.performance_metrics["command_execution"]["latency_ms"].append(latency_ms)
                
                # Update success rate
                self.performance_metrics["command_execution"]["success_rate"].append(
                    1.0 if result.get("success", False) else 0.0
                )
                
                # Add command to history
                self.command_history.append({
                    "timestamp": datetime.now().isoformat(),
                    "drone_id": drone_id,
                    "command": command,
                    "parameters": parameters or {},
                    "success": result.get("success", False),
                    "latency_ms": latency_ms,
                })
                
                # Add latency to result
                result["latency_ms"] = latency_ms
                
                return result
        
        except Exception as e:
            logger.error(f"Error sending command to drone {drone_id}: {str(e)}")
            
            # Add command to history with error
            self.command_history.append({
                "timestamp": datetime.now().isoformat(),
                "drone_id": drone_id,
                "command": command,
                "parameters": parameters or {},
                "success": False,
                "error": str(e),
            })
            
            return {"success": False, "error": str(e)}
    
    async def get_telemetry(self, drone_id: str) -> Dict[str, Any]:
        """
        Get telemetry data for a drone.
        
        Args:
            drone_id: ID of the drone
            
        Returns:
            Telemetry data dictionary
        """
        if not self.is_initialized:
            logger.error("Drone service not initialized")
            return {"error": "Drone service not initialized"}
        
        try:
            async with httpx.AsyncClient() as client:
                response = await client.get(
                    f"{self.api_url}/api/telemetry/{drone_id}",
                    headers=self.headers,
                    timeout=10.0,
                )
                response.raise_for_status()
                
                return response.json()
        
        except Exception as e:
            logger.error(f"Error getting telemetry for drone {drone_id}: {str(e)}")
            return {"error": str(e)}
    
    async def status(self) -> Dict[str, Any]:
        """
        Get the status of the drone service.
        
        Returns:
            Dictionary containing service status
        """
        import numpy as np
        
        status_info = {
            "initialized": self.is_initialized,
            "api_url": self.api_url,
            "active_drones_count": len(self.active_drones),
            "command_history_count": len(self.command_history),
            "performance": {
                "command_execution": {
                    "avg_latency_ms": np.mean(self.performance_metrics["command_execution"]["latency_ms"]).item() if self.performance_metrics["command_execution"]["latency_ms"] else None,
                    "success_rate": np.mean(self.performance_metrics["command_execution"]["success_rate"]).item() if self.performance_metrics["command_execution"]["success_rate"] else None,
                },
            },
        }
        
        return status_info
