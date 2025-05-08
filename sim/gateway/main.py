#!/usr/bin/env python3
"""
Simulation Swarm Gateway

This module provides a bridge between the ROS 2 simulation environment and the
Bulo.CloudSentinel platform. It relays ROS 2 topics to the existing drone_core
over the local network, allowing the SentinelWeb UI to display simulation data
without modifications.
"""

import os
import sys
import time
import asyncio
import logging
import json
import yaml
from typing import Dict, List, Any, Optional
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
from sensor_msgs.msg import Image, CameraInfo, NavSatFix, Imu
from std_msgs.msg import String, Float32, Bool

import fastapi
from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
import httpx
import uvicorn

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger("sim_swarm_gateway")

# Default values
DEFAULT_CONFIG_PATH = "/gateway/config/gateway.yaml"
DEFAULT_SENTINEL_API_URL = "http://bulocloud-sentinel-api:8000"
DEFAULT_SENTINEL_API_TOKEN = ""
DEFAULT_PORT = 8070
DEFAULT_HOST = "0.0.0.0"

class SimSwarmGateway:
    """
    Gateway between ROS 2 simulation and Bulo.CloudSentinel platform.
    
    This class provides a bridge between the ROS 2 simulation environment and
    the Bulo.CloudSentinel platform. It relays ROS 2 topics to the existing
    drone_core over the local network, allowing the SentinelWeb UI to display
    simulation data without modifications.
    """
    
    def __init__(self, config_path: str = DEFAULT_CONFIG_PATH):
        """Initialize the gateway."""
        # Load configuration
        self.config = self._load_config(config_path)
        
        # Initialize ROS 2
        rclpy.init()
        self.ros_node = rclpy.create_node('sim_swarm_gateway')
        
        # Set up QoS profile
        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Initialize state variables
        self.drones = {}
        self.telemetry = {}
        self.missions = {}
        self.telemetry_subscribers = {}
        
        # Initialize FastAPI
        self.app = FastAPI(
            title="Simulation Swarm Gateway",
            description="Bridge between ROS 2 simulation and Bulo.CloudSentinel platform",
            version="0.1.0",
        )
        
        # Add CORS middleware
        self.app.add_middleware(
            CORSMiddleware,
            allow_origins=["*"],
            allow_credentials=True,
            allow_methods=["*"],
            allow_headers=["*"],
        )
        
        # Set up API routes
        self._setup_routes()
        
        # Initialize HTTP client for Sentinel API
        self.sentinel_api_url = self.config.get("sentinel_api_url", DEFAULT_SENTINEL_API_URL)
        self.sentinel_api_token = self.config.get("sentinel_api_token", DEFAULT_SENTINEL_API_TOKEN)
        self.sentinel_headers = {
            "Authorization": f"Bearer {self.sentinel_api_token}",
            "Content-Type": "application/json",
        }
        
        # Initialize ROS 2 subscribers and publishers
        self._setup_ros_interfaces()
        
        # Start ROS 2 spinning task
        self.ros_spinning = True
        self.ros_spin_task = asyncio.create_task(self._ros_spin())
    
    def _load_config(self, config_path: str) -> Dict[str, Any]:
        """Load configuration from YAML file."""
        config = {}
        
        # Check if config file exists
        if os.path.exists(config_path):
            try:
                with open(config_path, 'r') as f:
                    config = yaml.safe_load(f)
                logger.info(f"Loaded configuration from {config_path}")
            except Exception as e:
                logger.error(f"Error loading configuration from {config_path}: {str(e)}")
        else:
            logger.warning(f"Configuration file not found: {config_path}")
            logger.warning("Using default configuration")
        
        # Set default values if not in config
        if "sentinel_api_url" not in config:
            config["sentinel_api_url"] = os.environ.get("SENTINEL_API_URL", DEFAULT_SENTINEL_API_URL)
        
        if "sentinel_api_token" not in config:
            config["sentinel_api_token"] = os.environ.get("SENTINEL_API_TOKEN", DEFAULT_SENTINEL_API_TOKEN)
        
        if "port" not in config:
            config["port"] = int(os.environ.get("GATEWAY_PORT", DEFAULT_PORT))
        
        if "host" not in config:
            config["host"] = os.environ.get("GATEWAY_HOST", DEFAULT_HOST)
        
        return config
    
    def _setup_routes(self):
        """Set up FastAPI routes."""
        # Health check
        @self.app.get("/health")
        async def health_check():
            return {"status": "healthy"}
        
        # Drones
        @self.app.get("/api/drones")
        async def get_drones():
            return {"drones": list(self.drones.values())}
        
        @self.app.get("/api/drones/{drone_id}")
        async def get_drone(drone_id: str):
            if drone_id not in self.drones:
                raise HTTPException(status_code=404, detail=f"Drone {drone_id} not found")
            return self.drones[drone_id]
        
        @self.app.post("/api/drones/{drone_id}/command")
        async def send_command(drone_id: str, command: Dict[str, Any]):
            if drone_id not in self.drones:
                raise HTTPException(status_code=404, detail=f"Drone {drone_id} not found")
            
            # Publish command to ROS 2
            await self._publish_command(drone_id, command)
            
            return {"status": "command_sent"}
        
        # Missions
        @self.app.get("/api/missions")
        async def get_missions():
            return {"missions": list(self.missions.values())}
        
        @self.app.post("/api/missions")
        async def create_mission(mission: Dict[str, Any]):
            mission_id = f"mission_{len(self.missions) + 1}"
            mission["id"] = mission_id
            self.missions[mission_id] = mission
            
            # Publish mission to ROS 2
            await self._publish_mission(mission)
            
            return mission
        
        @self.app.get("/api/missions/{mission_id}")
        async def get_mission(mission_id: str):
            if mission_id not in self.missions:
                raise HTTPException(status_code=404, detail=f"Mission {mission_id} not found")
            return self.missions[mission_id]
        
        # Telemetry
        @self.app.get("/api/telemetry/{drone_id}")
        async def get_telemetry(drone_id: str):
            if drone_id not in self.telemetry:
                raise HTTPException(status_code=404, detail=f"Telemetry for drone {drone_id} not found")
            return self.telemetry[drone_id]
        
        # WebSocket for telemetry
        @self.app.websocket("/api/ws/telemetry/{drone_id}")
        async def websocket_telemetry(websocket: WebSocket, drone_id: str):
            await websocket.accept()
            
            # Create queue for telemetry updates
            queue = asyncio.Queue()
            
            # Add to subscribers
            if drone_id not in self.telemetry_subscribers:
                self.telemetry_subscribers[drone_id] = []
            self.telemetry_subscribers[drone_id].append(queue)
            
            try:
                while True:
                    # Wait for telemetry update
                    telemetry = await queue.get()
                    
                    # Send to client
                    await websocket.send_json(telemetry)
            except WebSocketDisconnect:
                # Remove from subscribers
                if drone_id in self.telemetry_subscribers:
                    self.telemetry_subscribers[drone_id].remove(queue)
            except Exception as e:
                logger.error(f"Error in WebSocket: {str(e)}")
                await websocket.close(code=1000, reason=str(e))
    
    def _setup_ros_interfaces(self):
        """Set up ROS 2 subscribers and publishers."""
        # Subscribers
        self.pose_subscribers = {}
        self.telemetry_subscribers_ros = {}
        self.mission_status_subscribers = {}
        
        # Publishers
        self.command_publishers = {}
        self.mission_publishers = {}
        
        # Discover drones
        self._discover_drones()
    
    def _discover_drones(self):
        """Discover drones in the simulation."""
        # TODO: Implement drone discovery
        # For now, just create a few drones
        for i in range(1, 6):
            drone_id = f"drone_{i}"
            self.drones[drone_id] = {
                "id": drone_id,
                "name": f"Simulation Drone {i}",
                "type": "quadcopter",
                "status": "ready",
            }
            
            # Create subscribers for this drone
            self._create_drone_subscribers(drone_id)
            
            # Create publishers for this drone
            self._create_drone_publishers(drone_id)
            
            logger.info(f"Discovered drone: {drone_id}")
    
    def _create_drone_subscribers(self, drone_id: str):
        """Create ROS 2 subscribers for a drone."""
        # Pose subscriber
        self.pose_subscribers[drone_id] = self.ros_node.create_subscription(
            PoseStamped,
            f'/{drone_id}/pose',
            lambda msg, drone_id=drone_id: self._pose_callback(msg, drone_id),
            self.qos_profile
        )
        
        # Telemetry subscriber
        self.telemetry_subscribers_ros[drone_id] = {}
        
        # GPS
        self.telemetry_subscribers_ros[drone_id]["gps"] = self.ros_node.create_subscription(
            NavSatFix,
            f'/{drone_id}/gps',
            lambda msg, drone_id=drone_id: self._gps_callback(msg, drone_id),
            self.qos_profile
        )
        
        # IMU
        self.telemetry_subscribers_ros[drone_id]["imu"] = self.ros_node.create_subscription(
            Imu,
            f'/{drone_id}/imu',
            lambda msg, drone_id=drone_id: self._imu_callback(msg, drone_id),
            self.qos_profile
        )
        
        # Battery
        self.telemetry_subscribers_ros[drone_id]["battery"] = self.ros_node.create_subscription(
            Float32,
            f'/{drone_id}/battery',
            lambda msg, drone_id=drone_id: self._battery_callback(msg, drone_id),
            self.qos_profile
        )
        
        # Mission status
        self.mission_status_subscribers[drone_id] = self.ros_node.create_subscription(
            String,
            f'/{drone_id}/mission_status',
            lambda msg, drone_id=drone_id: self._mission_status_callback(msg, drone_id),
            self.qos_profile
        )
    
    def _create_drone_publishers(self, drone_id: str):
        """Create ROS 2 publishers for a drone."""
        # Command publisher
        self.command_publishers[drone_id] = self.ros_node.create_publisher(
            String,
            f'/{drone_id}/command',
            self.qos_profile
        )
        
        # Mission publisher
        self.mission_publishers[drone_id] = self.ros_node.create_publisher(
            String,
            f'/{drone_id}/mission',
            self.qos_profile
        )
    
    def _pose_callback(self, msg: PoseStamped, drone_id: str):
        """Callback for drone pose updates."""
        # Update drone position
        if drone_id in self.drones:
            self.drones[drone_id]["position"] = {
                "x": msg.pose.position.x,
                "y": msg.pose.position.y,
                "z": msg.pose.position.z,
            }
            self.drones[drone_id]["orientation"] = {
                "x": msg.pose.orientation.x,
                "y": msg.pose.orientation.y,
                "z": msg.pose.orientation.z,
                "w": msg.pose.orientation.w,
            }
    
    def _gps_callback(self, msg: NavSatFix, drone_id: str):
        """Callback for drone GPS updates."""
        # Update drone GPS
        if drone_id not in self.telemetry:
            self.telemetry[drone_id] = {}
        
        self.telemetry[drone_id]["gps"] = {
            "latitude": msg.latitude,
            "longitude": msg.longitude,
            "altitude": msg.altitude,
        }
        
        # Notify subscribers
        self._notify_telemetry_subscribers(drone_id)
    
    def _imu_callback(self, msg: Imu, drone_id: str):
        """Callback for drone IMU updates."""
        # Update drone IMU
        if drone_id not in self.telemetry:
            self.telemetry[drone_id] = {}
        
        self.telemetry[drone_id]["imu"] = {
            "orientation": {
                "x": msg.orientation.x,
                "y": msg.orientation.y,
                "z": msg.orientation.z,
                "w": msg.orientation.w,
            },
            "angular_velocity": {
                "x": msg.angular_velocity.x,
                "y": msg.angular_velocity.y,
                "z": msg.angular_velocity.z,
            },
            "linear_acceleration": {
                "x": msg.linear_acceleration.x,
                "y": msg.linear_acceleration.y,
                "z": msg.linear_acceleration.z,
            },
        }
        
        # Notify subscribers
        self._notify_telemetry_subscribers(drone_id)
    
    def _battery_callback(self, msg: Float32, drone_id: str):
        """Callback for drone battery updates."""
        # Update drone battery
        if drone_id not in self.telemetry:
            self.telemetry[drone_id] = {}
        
        self.telemetry[drone_id]["battery"] = {
            "percentage": msg.data,
        }
        
        # Notify subscribers
        self._notify_telemetry_subscribers(drone_id)
    
    def _mission_status_callback(self, msg: String, drone_id: str):
        """Callback for mission status updates."""
        # Update mission status
        if drone_id in self.drones:
            self.drones[drone_id]["mission_status"] = msg.data
    
    def _notify_telemetry_subscribers(self, drone_id: str):
        """Notify telemetry subscribers for a drone."""
        if drone_id in self.telemetry_subscribers:
            telemetry = self.telemetry[drone_id]
            for queue in self.telemetry_subscribers[drone_id]:
                if not queue.full():
                    queue.put_nowait(telemetry)
    
    async def _publish_command(self, drone_id: str, command: Dict[str, Any]):
        """Publish a command to a drone."""
        if drone_id in self.command_publishers:
            # Convert command to JSON string
            command_str = json.dumps(command)
            
            # Create ROS message
            msg = String()
            msg.data = command_str
            
            # Publish command
            self.command_publishers[drone_id].publish(msg)
            
            logger.info(f"Published command to drone {drone_id}: {command}")
    
    async def _publish_mission(self, mission: Dict[str, Any]):
        """Publish a mission to all drones."""
        # Convert mission to JSON string
        mission_str = json.dumps(mission)
        
        # Create ROS message
        msg = String()
        msg.data = mission_str
        
        # Publish mission to all drones
        for drone_id, publisher in self.mission_publishers.items():
            publisher.publish(msg)
            logger.info(f"Published mission to drone {drone_id}: {mission['id']}")
    
    async def _ros_spin(self):
        """Spin ROS 2 node to process callbacks."""
        while self.ros_spinning:
            rclpy.spin_once(self.ros_node, timeout_sec=0.1)
            await asyncio.sleep(0.01)
    
    async def start(self):
        """Start the gateway."""
        # Start FastAPI server
        config = uvicorn.Config(
            app=self.app,
            host=self.config.get("host", DEFAULT_HOST),
            port=self.config.get("port", DEFAULT_PORT),
            log_level="info",
        )
        server = uvicorn.Server(config)
        await server.serve()
    
    async def stop(self):
        """Stop the gateway."""
        # Stop ROS 2 spinning
        self.ros_spinning = False
        if self.ros_spin_task:
            self.ros_spin_task.cancel()
            try:
                await self.ros_spin_task
            except asyncio.CancelledError:
                pass
        
        # Shutdown ROS 2
        self.ros_node.destroy_node()
        rclpy.shutdown()
        
        logger.info("Gateway stopped")


async def main():
    """Main function."""
    # Get configuration path from environment variable
    config_path = os.environ.get("GATEWAY_CONFIG", DEFAULT_CONFIG_PATH)
    
    # Create gateway
    gateway = SimSwarmGateway(config_path)
    
    try:
        # Start gateway
        await gateway.start()
    finally:
        # Stop gateway
        await gateway.stop()


if __name__ == "__main__":
    asyncio.run(main())
