"""
ROS Bridge for Indoor Drone System

This module provides a bridge between the FastAPI application and ROS2 nodes.
It handles communication with various ROS2 services for SLAM, path planning, and drone control.
"""

import asyncio
import logging
import json
from typing import List, Dict, Any, Optional
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Path
from sensor_msgs.msg import PointCloud2, Image

# Import custom message types
from indoor_drone_msgs.msg import DroneState as DroneStateMsg
from indoor_drone_msgs.msg import MissionPlan as MissionPlanMsg
from indoor_drone_msgs.msg import SensorData as SensorDataMsg
from indoor_drone_msgs.msg import MapData as MapDataMsg

# Import custom service types
from indoor_drone_msgs.srv import ExecuteMission, CreateMap

# Import local models
from models import DroneState, MissionPlan, SensorData, MapData

logger = logging.getLogger(__name__)

class ROSBridge:
    """Bridge between FastAPI and ROS2."""
    
    def __init__(self):
        """Initialize the ROS bridge."""
        # Create ROS2 node
        self.node = rclpy.create_node('indoor_drone_api_bridge')
        
        # Set up QoS profiles
        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Set up publishers
        self.command_pub = self.node.create_publisher(
            String, 
            '/indoor_drone/command', 
            self.qos_profile
        )
        
        # Set up subscribers
        self.drone_state_sub = self.node.create_subscription(
            DroneStateMsg,
            '/indoor_drone/state',
            self._drone_state_callback,
            self.qos_profile
        )
        
        # Set up clients
        self.execute_mission_client = self.node.create_client(
            ExecuteMission,
            '/indoor_drone/execute_mission'
        )
        
        self.create_map_client = self.node.create_client(
            CreateMap,
            '/indoor_drone/create_map'
        )
        
        # Initialize state
        self.drones = {}
        self.missions = {}
        self.maps = {}
        self.telemetry_subscriptions = {}
        
        # Health check flag
        self.healthy = True
        
        logger.info("ROS Bridge initialized")
    
    async def spin(self):
        """Spin the ROS node in a separate thread."""
        def _spin_thread():
            while rclpy.ok():
                rclpy.spin_once(self.node, timeout_sec=0.1)
        
        # Run the spin function in a separate thread
        loop = asyncio.get_event_loop()
        await loop.run_in_executor(None, _spin_thread)
    
    async def shutdown(self):
        """Shutdown the ROS node."""
        self.node.destroy_node()
        logger.info("ROS Bridge shutdown")
    
    def is_healthy(self) -> bool:
        """Check if the ROS bridge is healthy."""
        return self.healthy and rclpy.ok()
    
    def _drone_state_callback(self, msg: DroneStateMsg):
        """Callback for drone state messages."""
        drone_id = msg.drone_id
        
        # Convert ROS message to our model
        drone_state = DroneState(
            drone_id=drone_id,
            position={
                'x': msg.position.x,
                'y': msg.position.y,
                'z': msg.position.z
            },
            orientation={
                'x': msg.orientation.x,
                'y': msg.orientation.y,
                'z': msg.orientation.z,
                'w': msg.orientation.w
            },
            velocity={
                'linear': {
                    'x': msg.velocity.linear.x,
                    'y': msg.velocity.linear.y,
                    'z': msg.velocity.linear.z
                },
                'angular': {
                    'x': msg.velocity.angular.x,
                    'y': msg.velocity.angular.y,
                    'z': msg.velocity.angular.z
                }
            },
            battery_level=msg.battery_level,
            status=msg.status,
            timestamp=datetime.fromtimestamp(
                msg.timestamp.sec + msg.timestamp.nanosec / 1e9
            ).isoformat()
        )
        
        # Update drone state
        self.drones[drone_id] = drone_state
        
        # Update telemetry subscriptions
        if drone_id in self.telemetry_subscriptions:
            for queue in self.telemetry_subscriptions[drone_id]:
                if not queue.full():
                    queue.put_nowait(drone_state.dict())
    
    async def get_drones(self) -> List[DroneState]:
        """Get all available drones."""
        return list(self.drones.values())
    
    async def get_drone(self, drone_id: str) -> Optional[DroneState]:
        """Get a specific drone by ID."""
        return self.drones.get(drone_id)
    
    async def send_command(self, drone_id: str, command: Dict[str, Any]) -> bool:
        """Send a command to a drone."""
        # Add drone ID to command
        command['drone_id'] = drone_id
        
        # Convert command to JSON string
        command_str = json.dumps(command)
        
        # Create ROS message
        msg = String()
        msg.data = command_str
        
        # Publish command
        self.command_pub.publish(msg)
        
        return True
    
    async def create_mission(self, mission: MissionPlan) -> str:
        """Create a new mission plan."""
        # Generate mission ID
        mission_id = f"mission_{len(self.missions) + 1}"
        
        # Store mission
        self.missions[mission_id] = mission
        
        return mission_id
    
    async def get_mission(self, mission_id: str) -> Optional[MissionPlan]:
        """Get a specific mission by ID."""
        return self.missions.get(mission_id)
    
    async def execute_mission(self, mission_id: str, drone_id: str) -> bool:
        """Execute a mission with a specific drone."""
        # Check if mission exists
        if mission_id not in self.missions:
            return False
        
        # Check if drone exists
        if drone_id not in self.drones:
            return False
        
        # Create service request
        request = ExecuteMission.Request()
        request.mission_id = mission_id
        request.drone_id = drone_id
        
        # Call service
        future = self.execute_mission_client.call_async(request)
        
        # Wait for response
        await asyncio.wait_for(future, timeout=5.0)
        
        # Check response
        response = future.result()
        return response.success
    
    async def get_maps(self) -> List[Dict[str, Any]]:
        """Get all available maps."""
        return [
            {
                'map_id': map_id,
                'name': map_data.name,
                'created_at': map_data.created_at,
                'resolution': map_data.resolution,
                'width': map_data.width,
                'height': map_data.height
            }
            for map_id, map_data in self.maps.items()
        ]
    
    async def get_map(self, map_id: str) -> Optional[MapData]:
        """Get a specific map by ID."""
        return self.maps.get(map_id)
    
    async def start_mapping(self, drone_id: str, map_name: str) -> str:
        """Start creating a new map with a specific drone."""
        # Generate map ID
        map_id = f"map_{len(self.maps) + 1}"
        
        # Create service request
        request = CreateMap.Request()
        request.drone_id = drone_id
        request.map_id = map_id
        request.map_name = map_name
        
        # Call service
        future = self.create_map_client.call_async(request)
        
        # Wait for response
        await asyncio.wait_for(future, timeout=5.0)
        
        # Check response
        response = future.result()
        if not response.success:
            raise Exception(f"Failed to start mapping: {response.message}")
        
        # Initialize map data
        self.maps[map_id] = MapData(
            map_id=map_id,
            name=map_name,
            created_at=datetime.now().isoformat(),
            resolution=0.05,
            width=0,
            height=0,
            data=None,
            status="in_progress"
        )
        
        return map_id
    
    async def monitor_mapping(self, map_id: str):
        """Monitor mapping progress."""
        # This would be implemented to update map status as mapping progresses
        pass
    
    async def subscribe_telemetry(self, drone_id: str) -> asyncio.Queue:
        """Subscribe to telemetry updates for a specific drone."""
        # Create queue for telemetry data
        queue = asyncio.Queue(maxsize=100)
        
        # Add queue to subscriptions
        if drone_id not in self.telemetry_subscriptions:
            self.telemetry_subscriptions[drone_id] = []
        
        self.telemetry_subscriptions[drone_id].append(queue)
        
        return queue
    
    async def unsubscribe_telemetry(self, queue: asyncio.Queue):
        """Unsubscribe from telemetry updates."""
        # Remove queue from subscriptions
        for drone_id, queues in self.telemetry_subscriptions.items():
            if queue in queues:
                queues.remove(queue)
                break
