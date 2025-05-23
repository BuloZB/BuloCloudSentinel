"""
Simulator CLI for the Remote ID & Regulatory Compliance Service.

This module provides a command-line interface for simulating Remote ID broadcasts.
"""

import argparse
import asyncio
import json
import logging
import math
import random
import sys
import time
from datetime import datetime, timedelta
from typing import Dict, List, Optional, Any, Tuple

import httpx

from remoteid_service.api.schemas.remoteid import (
    RemoteIDMode,
    BroadcastMethod,
    Position,
    Velocity,
)

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
)
logger = logging.getLogger("simulator_cli")

class SimulatorCLI:
    """
    Simulator CLI.
    
    This class provides a command-line interface for simulating Remote ID broadcasts.
    """
    
    def __init__(self):
        """
        Initialize the simulator CLI.
        """
        self.parser = self._create_parser()
        self.running = False
        self.drones = {}
    
    def _create_parser(self) -> argparse.ArgumentParser:
        """
        Create the argument parser.
        
        Returns:
            argparse.ArgumentParser: Argument parser
        """
        parser = argparse.ArgumentParser(
            description="Simulator CLI for the Remote ID & Regulatory Compliance Service",
        )
        
        # Add arguments
        parser.add_argument("--mode", choices=["basic", "full", "random"], default="basic", help="Simulation mode")
        parser.add_argument("--drones", type=int, default=1, help="Number of drones to simulate")
        parser.add_argument("--interval", type=float, default=1.0, help="Update interval in seconds")
        parser.add_argument("--duration", type=float, default=0, help="Simulation duration in seconds (0 for infinite)")
        parser.add_argument("--server", default="http://localhost:8080", help="Server URL")
        
        return parser
    
    async def run(self, args: Optional[List[str]] = None) -> None:
        """
        Run the CLI.
        
        Args:
            args: Command-line arguments
        """
        # Parse arguments
        args = self.parser.parse_args(args)
        
        # Start simulation
        await self._run_simulation(args)
    
    async def _run_simulation(self, args: argparse.Namespace) -> None:
        """
        Run the simulation.
        
        Args:
            args: Command-line arguments
        """
        try:
            # Set running flag
            self.running = True
            
            # Initialize drones
            for i in range(args.drones):
                drone_id = f"SIM-{i+1:03d}"
                
                # Create drone data
                self.drones[drone_id] = {
                    "id": drone_id,
                    "mode": "faa",
                    "methods": ["wifi_nan", "bluetooth_le"],
                    "operator_id": f"OP-{i+1:03d}",
                    "serial_number": f"SN-{i+1:03d}",
                    "position": self._get_random_position(),
                    "velocity": {
                        "speed_horizontal": random.uniform(0, 10),
                        "heading": random.uniform(0, 359),
                    },
                    "session_id": None,
                    "path": self._generate_path(args.mode),
                    "path_index": 0,
                }
                
                # Start broadcasting
                await self._start_broadcast(args.server, drone_id)
            
            # Calculate end time
            end_time = None
            if args.duration > 0:
                end_time = time.time() + args.duration
            
            # Run simulation loop
            while self.running:
                # Check if simulation should end
                if end_time and time.time() >= end_time:
                    break
                
                # Update drones
                for drone_id, drone_data in self.drones.items():
                    # Update position and velocity
                    self._update_drone(drone_data, args.mode)
                    
                    # Send update
                    await self._update_broadcast(args.server, drone_id)
                
                # Wait for next update
                await asyncio.sleep(args.interval)
            
            # Stop broadcasting
            for drone_id in self.drones:
                await self._stop_broadcast(args.server, drone_id)
            
            logger.info("Simulation completed")
        except KeyboardInterrupt:
            logger.info("Simulation interrupted")
            
            # Stop broadcasting
            for drone_id in self.drones:
                await self._stop_broadcast(args.server, drone_id)
        except Exception as e:
            logger.error(f"Error running simulation: {str(e)}")
            sys.exit(1)
    
    async def _start_broadcast(self, server: str, drone_id: str) -> None:
        """
        Start broadcasting for a drone.
        
        Args:
            server: Server URL
            drone_id: Drone ID
        """
        try:
            # Get drone data
            drone_data = self.drones[drone_id]
            
            # Create request data
            request_data = {
                "drone_id": drone_id,
                "mode": drone_data["mode"],
                "methods": drone_data["methods"],
                "operator_id": {
                    "id": drone_data["operator_id"],
                    "type": "other",
                },
                "serial_number": drone_data["serial_number"],
                "initial_position": drone_data["position"],
                "initial_velocity": drone_data["velocity"],
            }
            
            # Send request
            async with httpx.AsyncClient() as client:
                response = await client.post(
                    f"{server}/api/v1/remoteid/broadcast/start",
                    json=request_data,
                    timeout=30,
                )
                
                # Check response
                response.raise_for_status()
                
                # Get session ID
                response_data = response.json()
                self.drones[drone_id]["session_id"] = response_data.get("session_id")
                
                logger.info(f"Started broadcasting for drone {drone_id}")
        except Exception as e:
            logger.error(f"Error starting broadcast for drone {drone_id}: {str(e)}")
    
    async def _update_broadcast(self, server: str, drone_id: str) -> None:
        """
        Update broadcast for a drone.
        
        Args:
            server: Server URL
            drone_id: Drone ID
        """
        try:
            # Get drone data
            drone_data = self.drones[drone_id]
            
            # Create request data
            request_data = {
                "drone_id": drone_id,
                "position": drone_data["position"],
                "velocity": drone_data["velocity"],
                "session_id": drone_data["session_id"],
            }
            
            # Send request
            async with httpx.AsyncClient() as client:
                response = await client.post(
                    f"{server}/api/v1/remoteid/broadcast/update",
                    json=request_data,
                    timeout=30,
                )
                
                # Check response
                response.raise_for_status()
                
                logger.debug(f"Updated broadcast for drone {drone_id}")
        except Exception as e:
            logger.error(f"Error updating broadcast for drone {drone_id}: {str(e)}")
    
    async def _stop_broadcast(self, server: str, drone_id: str) -> None:
        """
        Stop broadcasting for a drone.
        
        Args:
            server: Server URL
            drone_id: Drone ID
        """
        try:
            # Get drone data
            drone_data = self.drones[drone_id]
            
            # Create request data
            request_data = {
                "drone_id": drone_id,
                "session_id": drone_data["session_id"],
            }
            
            # Send request
            async with httpx.AsyncClient() as client:
                response = await client.post(
                    f"{server}/api/v1/remoteid/broadcast/stop",
                    json=request_data,
                    timeout=30,
                )
                
                # Check response
                response.raise_for_status()
                
                logger.info(f"Stopped broadcasting for drone {drone_id}")
        except Exception as e:
            logger.error(f"Error stopping broadcast for drone {drone_id}: {str(e)}")
    
    def _get_random_position(self) -> Dict[str, Any]:
        """
        Get a random position.
        
        Returns:
            Dict[str, Any]: Random position
        """
        return {
            "latitude": random.uniform(37.7, 37.8),
            "longitude": random.uniform(-122.5, -122.4),
            "altitude": random.uniform(10, 100),
        }
    
    def _generate_path(self, mode: str) -> List[Dict[str, Any]]:
        """
        Generate a path for a drone.
        
        Args:
            mode: Simulation mode
            
        Returns:
            List[Dict[str, Any]]: Path
        """
        if mode == "basic":
            # Simple circular path
            center = self._get_random_position()
            radius = random.uniform(0.001, 0.005)  # Approximately 100-500 meters
            points = []
            
            for i in range(36):
                angle = math.radians(i * 10)
                lat = center["latitude"] + radius * math.cos(angle)
                lon = center["longitude"] + radius * math.sin(angle)
                
                points.append({
                    "latitude": lat,
                    "longitude": lon,
                    "altitude": center["altitude"],
                })
            
            return points
        elif mode == "full":
            # More complex path with altitude changes
            center = self._get_random_position()
            radius = random.uniform(0.001, 0.005)  # Approximately 100-500 meters
            points = []
            
            for i in range(36):
                angle = math.radians(i * 10)
                lat = center["latitude"] + radius * math.cos(angle)
                lon = center["longitude"] + radius * math.sin(angle)
                alt = center["altitude"] + 20 * math.sin(angle)
                
                points.append({
                    "latitude": lat,
                    "longitude": lon,
                    "altitude": alt,
                })
            
            return points
        else:  # random
            # Random path
            points = []
            position = self._get_random_position()
            
            for i in range(20):
                # Add some random movement
                position["latitude"] += random.uniform(-0.001, 0.001)
                position["longitude"] += random.uniform(-0.001, 0.001)
                position["altitude"] += random.uniform(-10, 10)
                
                # Ensure altitude is positive
                position["altitude"] = max(10, position["altitude"])
                
                points.append(position.copy())
            
            return points
    
    def _update_drone(self, drone_data: Dict[str, Any], mode: str) -> None:
        """
        Update drone position and velocity.
        
        Args:
            drone_data: Drone data
            mode: Simulation mode
        """
        if mode == "random":
            # Random movement
            drone_data["position"]["latitude"] += random.uniform(-0.0001, 0.0001)
            drone_data["position"]["longitude"] += random.uniform(-0.0001, 0.0001)
            drone_data["position"]["altitude"] += random.uniform(-1, 1)
            
            # Ensure altitude is positive
            drone_data["position"]["altitude"] = max(10, drone_data["position"]["altitude"])
            
            # Update velocity
            drone_data["velocity"]["speed_horizontal"] = random.uniform(0, 10)
            drone_data["velocity"]["heading"] = random.uniform(0, 359)
        else:
            # Follow path
            path = drone_data["path"]
            index = drone_data["path_index"]
            
            # Get next position
            next_position = path[index]
            
            # Update position
            drone_data["position"] = next_position.copy()
            
            # Calculate velocity
            if index > 0:
                prev_position = path[index - 1]
                
                # Calculate distance and heading
                distance, heading = self._calculate_distance_and_heading(
                    prev_position["latitude"],
                    prev_position["longitude"],
                    next_position["latitude"],
                    next_position["longitude"],
                )
                
                # Update velocity
                drone_data["velocity"]["speed_horizontal"] = distance / 1.0  # Assuming 1 second between updates
                drone_data["velocity"]["heading"] = heading
            
            # Update path index
            drone_data["path_index"] = (index + 1) % len(path)
    
    def _calculate_distance_and_heading(
        self,
        lat1: float,
        lon1: float,
        lat2: float,
        lon2: float,
    ) -> Tuple[float, float]:
        """
        Calculate distance and heading between two points.
        
        Args:
            lat1: Latitude of point 1
            lon1: Longitude of point 1
            lat2: Latitude of point 2
            lon2: Longitude of point 2
            
        Returns:
            Tuple[float, float]: Distance (meters) and heading (degrees)
        """
        # Convert to radians
        lat1_rad = math.radians(lat1)
        lon1_rad = math.radians(lon1)
        lat2_rad = math.radians(lat2)
        lon2_rad = math.radians(lon2)
        
        # Calculate distance (Haversine formula)
        dlon = lon2_rad - lon1_rad
        dlat = lat2_rad - lat1_rad
        a = math.sin(dlat/2)**2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon/2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        distance = 6371000 * c  # Earth radius in meters
        
        # Calculate heading
        y = math.sin(dlon) * math.cos(lat2_rad)
        x = math.cos(lat1_rad) * math.sin(lat2_rad) - math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(dlon)
        heading = math.degrees(math.atan2(y, x))
        heading = (heading + 360) % 360  # Normalize to 0-359
        
        return distance, heading

def main():
    """
    Main entry point.
    """
    cli = SimulatorCLI()
    asyncio.run(cli.run())

if __name__ == "__main__":
    main()
