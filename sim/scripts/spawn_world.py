#!/usr/bin/env python3
"""
spawn_world.py - CLI tool to parameterize world size, weather, number of drones.

This script provides a command-line interface to spawn a Gazebo world with
customizable parameters such as world size, weather conditions, and number of drones.
It communicates with the Gazebo server through the gz-transport API.
"""

import os
import sys
import time
import argparse
import yaml
import json
import logging
from pathlib import Path
from typing import Dict, List, Any, Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# Import Gazebo Python bindings
try:
    import gz.transport
    import gz.msgs
except ImportError:
    print("Error: Gazebo Python bindings not found. Please install gz-python.")
    sys.exit(1)

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger("spawn_world")

# Default values
DEFAULT_WORLD = "urban_small"
DEFAULT_NUM_DRONES = 1
DEFAULT_WEATHER = "clear"
DEFAULT_TIME_OF_DAY = "noon"
DEFAULT_WIND_SPEED = 0.0
DEFAULT_WIND_DIRECTION = 0.0
DEFAULT_RAIN_INTENSITY = 0.0
DEFAULT_FOG_DENSITY = 0.0
DEFAULT_DRONE_TYPE = "quadcopter"
DEFAULT_SENSOR_SUITE = "basic"

# Available options
AVAILABLE_WORLDS = ["urban_small", "urban_large", "suburban", "rural", "indoor_warehouse", "indoor_office"]
AVAILABLE_WEATHER = ["clear", "cloudy", "rainy", "foggy", "stormy"]
AVAILABLE_TIMES = ["dawn", "morning", "noon", "afternoon", "dusk", "night"]
AVAILABLE_DRONE_TYPES = ["quadcopter", "hexacopter", "octocopter", "fixed_wing"]
AVAILABLE_SENSOR_SUITES = ["basic", "advanced", "lidar", "thermal", "full"]

class WorldSpawner:
    """Class to spawn and configure Gazebo worlds with drones."""
    
    def __init__(self):
        """Initialize the world spawner."""
        # Initialize Gazebo transport
        self.gz_node = gz.transport.Node()
        
        # Initialize ROS 2
        rclpy.init()
        self.ros_node = rclpy.create_node('world_spawner')
        
        # Set up QoS profile
        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Get simulation directory
        self.sim_dir = Path(__file__).parent.parent.absolute()
        
        # Load world configurations
        self.world_configs = self._load_world_configs()
        
        # Load drone configurations
        self.drone_configs = self._load_drone_configs()
        
        # Initialize publishers
        self._init_publishers()
    
    def _load_world_configs(self) -> Dict[str, Any]:
        """Load world configurations from YAML files."""
        config_path = self.sim_dir / "models" / "environments" / "config"
        configs = {}
        
        for world in AVAILABLE_WORLDS:
            config_file = config_path / f"{world}.yaml"
            if config_file.exists():
                with open(config_file, 'r') as f:
                    configs[world] = yaml.safe_load(f)
            else:
                logger.warning(f"Config file for world '{world}' not found: {config_file}")
                configs[world] = {}
        
        return configs
    
    def _load_drone_configs(self) -> Dict[str, Any]:
        """Load drone configurations from YAML files."""
        config_path = self.sim_dir / "models" / "drones" / "config"
        configs = {}
        
        for drone_type in AVAILABLE_DRONE_TYPES:
            config_file = config_path / f"{drone_type}.yaml"
            if config_file.exists():
                with open(config_file, 'r') as f:
                    configs[drone_type] = yaml.safe_load(f)
            else:
                logger.warning(f"Config file for drone type '{drone_type}' not found: {config_file}")
                configs[drone_type] = {}
        
        return configs
    
    def _init_publishers(self):
        """Initialize Gazebo publishers."""
        # World control publisher
        self.world_control_pub = self.gz_node.advertise("/world/control", "gz.msgs.WorldControl")
        
        # Model spawn publisher
        self.model_spawn_pub = self.gz_node.advertise("/world/model/spawn", "gz.msgs.EntityFactory")
        
        # Physics publisher
        self.physics_pub = self.gz_node.advertise("/world/physics", "gz.msgs.Physics")
        
        # Weather publisher
        self.weather_pub = self.gz_node.advertise("/world/weather", "gz.msgs.Weather")
        
        # Wait for publishers to be ready
        time.sleep(1.0)
    
    def spawn_world(self, args: argparse.Namespace):
        """Spawn a world with the given parameters."""
        # Log the parameters
        logger.info(f"Spawning world '{args.world}' with {args.drones} drones")
        logger.info(f"Weather: {args.weather}, Time: {args.time_of_day}")
        logger.info(f"Wind: {args.wind_speed} m/s at {args.wind_direction} degrees")
        logger.info(f"Drone type: {args.drone_type}, Sensor suite: {args.sensor_suite}")
        
        # Load world
        self._load_world(args.world)
        
        # Set weather
        self._set_weather(args.weather, args.wind_speed, args.wind_direction, 
                         args.rain_intensity, args.fog_density)
        
        # Set time of day
        self._set_time_of_day(args.time_of_day)
        
        # Spawn drones
        self._spawn_drones(args.drones, args.drone_type, args.sensor_suite)
        
        # Start simulation
        self._start_simulation()
        
        logger.info("World spawned successfully")
    
    def _load_world(self, world_name: str):
        """Load a world by name."""
        # TODO: Implement world loading through Gazebo API
        logger.info(f"Loading world: {world_name}")
    
    def _set_weather(self, weather: str, wind_speed: float, wind_direction: float,
                    rain_intensity: float, fog_density: float):
        """Set weather conditions."""
        # TODO: Implement weather setting through Gazebo API
        logger.info(f"Setting weather: {weather}")
    
    def _set_time_of_day(self, time_of_day: str):
        """Set time of day."""
        # TODO: Implement time of day setting through Gazebo API
        logger.info(f"Setting time of day: {time_of_day}")
    
    def _spawn_drones(self, num_drones: int, drone_type: str, sensor_suite: str):
        """Spawn drones in the world."""
        # TODO: Implement drone spawning through Gazebo API
        logger.info(f"Spawning {num_drones} drones of type {drone_type} with {sensor_suite} sensors")
    
    def _start_simulation(self):
        """Start the simulation."""
        # TODO: Implement simulation starting through Gazebo API
        logger.info("Starting simulation")
    
    def cleanup(self):
        """Clean up resources."""
        # Shutdown ROS 2
        self.ros_node.destroy_node()
        rclpy.shutdown()
        
        logger.info("Resources cleaned up")


def parse_args():
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(description="Spawn a Gazebo world with drones")
    
    parser.add_argument("--world", type=str, default=DEFAULT_WORLD,
                        choices=AVAILABLE_WORLDS,
                        help="World to spawn")
    
    parser.add_argument("--drones", type=int, default=DEFAULT_NUM_DRONES,
                        help="Number of drones to spawn")
    
    parser.add_argument("--weather", type=str, default=DEFAULT_WEATHER,
                        choices=AVAILABLE_WEATHER,
                        help="Weather conditions")
    
    parser.add_argument("--time", dest="time_of_day", type=str, default=DEFAULT_TIME_OF_DAY,
                        choices=AVAILABLE_TIMES,
                        help="Time of day")
    
    parser.add_argument("--wind-speed", type=float, default=DEFAULT_WIND_SPEED,
                        help="Wind speed in m/s")
    
    parser.add_argument("--wind-direction", type=float, default=DEFAULT_WIND_DIRECTION,
                        help="Wind direction in degrees")
    
    parser.add_argument("--rain", dest="rain_intensity", type=float, default=DEFAULT_RAIN_INTENSITY,
                        help="Rain intensity (0.0-1.0)")
    
    parser.add_argument("--fog", dest="fog_density", type=float, default=DEFAULT_FOG_DENSITY,
                        help="Fog density (0.0-1.0)")
    
    parser.add_argument("--drone-type", type=str, default=DEFAULT_DRONE_TYPE,
                        choices=AVAILABLE_DRONE_TYPES,
                        help="Type of drone to spawn")
    
    parser.add_argument("--sensor-suite", type=str, default=DEFAULT_SENSOR_SUITE,
                        choices=AVAILABLE_SENSOR_SUITES,
                        help="Sensor suite to equip on drones")
    
    return parser.parse_args()


def main():
    """Main function."""
    # Parse arguments
    args = parse_args()
    
    # Create world spawner
    spawner = WorldSpawner()
    
    try:
        # Spawn world
        spawner.spawn_world(args)
        
        # Keep the script running
        logger.info("Press Ctrl+C to exit")
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        logger.info("Exiting...")
    finally:
        # Clean up
        spawner.cleanup()


if __name__ == "__main__":
    main()
