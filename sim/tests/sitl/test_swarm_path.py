#!/usr/bin/env python3
"""
test_swarm_path.py - Test that five drones complete route without collision.

This test verifies that a swarm of five drones can complete a predefined route
in a simulated urban environment without collisions.
"""

import os
import sys
import time
import pytest
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import String

# Add sim directory to Python path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))

# Import simulation modules
from scripts.spawn_world import WorldSpawner

# Constants
NUM_DRONES = 5
WORLD_NAME = "urban_small"
DRONE_TYPE = "quadcopter"
SENSOR_SUITE = "basic"
MISSION_DURATION = 300  # seconds
COLLISION_THRESHOLD = 1.0  # meters
COMPLETION_THRESHOLD = 2.0  # meters

class SwarmPathTest(Node):
    """Test node for swarm path testing."""
    
    def __init__(self):
        """Initialize the test node."""
        super().__init__('swarm_path_test')
        
        # Set up QoS profile
        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Initialize state variables
        self.drone_poses = {}
        self.drone_paths = {}
        self.mission_complete = {}
        self.collisions = []
        
        # Set up subscribers for each drone
        for i in range(1, NUM_DRONES + 1):
            drone_id = f"drone_{i}"
            
            # Subscribe to drone pose
            self.create_subscription(
                PoseStamped,
                f'/{drone_id}/pose',
                lambda msg, drone_id=drone_id: self._pose_callback(msg, drone_id),
                self.qos_profile
            )
            
            # Subscribe to drone path
            self.create_subscription(
                Path,
                f'/{drone_id}/path',
                lambda msg, drone_id=drone_id: self._path_callback(msg, drone_id),
                self.qos_profile
            )
            
            # Subscribe to mission status
            self.create_subscription(
                String,
                f'/{drone_id}/mission_status',
                lambda msg, drone_id=drone_id: self._mission_status_callback(msg, drone_id),
                self.qos_profile
            )
            
            # Initialize state
            self.drone_poses[drone_id] = None
            self.drone_paths[drone_id] = None
            self.mission_complete[drone_id] = False
    
    def _pose_callback(self, msg, drone_id):
        """Callback for drone pose updates."""
        self.drone_poses[drone_id] = msg
        
        # Check for collisions
        self._check_collisions()
    
    def _path_callback(self, msg, drone_id):
        """Callback for drone path updates."""
        self.drone_paths[drone_id] = msg
    
    def _mission_status_callback(self, msg, drone_id):
        """Callback for mission status updates."""
        if msg.data == "MISSION_COMPLETE":
            self.mission_complete[drone_id] = True
    
    def _check_collisions(self):
        """Check for collisions between drones."""
        for i, (drone_id1, pose1) in enumerate(self.drone_poses.items()):
            if pose1 is None:
                continue
            
            for drone_id2, pose2 in list(self.drone_poses.items())[i+1:]:
                if pose2 is None:
                    continue
                
                # Calculate distance between drones
                dx = pose1.pose.position.x - pose2.pose.position.x
                dy = pose1.pose.position.y - pose2.pose.position.y
                dz = pose1.pose.position.z - pose2.pose.position.z
                distance = np.sqrt(dx*dx + dy*dy + dz*dz)
                
                # Check for collision
                if distance < COLLISION_THRESHOLD:
                    collision = (drone_id1, drone_id2, distance)
                    if collision not in self.collisions:
                        self.get_logger().error(f"Collision detected between {drone_id1} and {drone_id2} (distance: {distance:.2f}m)")
                        self.collisions.append(collision)
    
    def all_missions_complete(self):
        """Check if all missions are complete."""
        return all(self.mission_complete.values())
    
    def get_collisions(self):
        """Get list of collisions."""
        return self.collisions


@pytest.fixture
def ros_setup():
    """Set up ROS 2 for testing."""
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def world_spawner():
    """Set up the world spawner."""
    spawner = WorldSpawner()
    yield spawner
    spawner.cleanup()


@pytest.fixture
def test_node(ros_setup):
    """Set up the test node."""
    node = SwarmPathTest()
    yield node
    node.destroy_node()


def test_swarm_path(world_spawner, test_node):
    """Test that five drones complete route without collision."""
    # Spawn the world with five drones
    class Args:
        world = WORLD_NAME
        drones = NUM_DRONES
        drone_type = DRONE_TYPE
        sensor_suite = SENSOR_SUITE
        weather = "clear"
        time_of_day = "noon"
        wind_speed = 0.0
        wind_direction = 0.0
        rain_intensity = 0.0
        fog_density = 0.0
    
    world_spawner.spawn_world(Args())
    
    # Wait for the simulation to start
    time.sleep(5)
    
    # Run the simulation for the specified duration
    start_time = time.time()
    while time.time() - start_time < MISSION_DURATION:
        # Spin ROS 2 node to process callbacks
        rclpy.spin_once(test_node, timeout_sec=0.1)
        
        # Check if all missions are complete
        if test_node.all_missions_complete():
            break
    
    # Check for collisions
    collisions = test_node.get_collisions()
    assert len(collisions) == 0, f"Detected {len(collisions)} collisions between drones"
    
    # Check if all missions are complete
    assert test_node.all_missions_complete(), "Not all drones completed their missions"


if __name__ == "__main__":
    # This allows running the test directly
    rclpy.init()
    try:
        spawner = WorldSpawner()
        node = SwarmPathTest()
        
        # Spawn the world with five drones
        class Args:
            world = WORLD_NAME
            drones = NUM_DRONES
            drone_type = DRONE_TYPE
            sensor_suite = SENSOR_SUITE
            weather = "clear"
            time_of_day = "noon"
            wind_speed = 0.0
            wind_direction = 0.0
            rain_intensity = 0.0
            fog_density = 0.0
        
        spawner.spawn_world(Args())
        
        # Wait for the simulation to start
        time.sleep(5)
        
        # Run the simulation for the specified duration
        start_time = time.time()
        while time.time() - start_time < MISSION_DURATION:
            # Spin ROS 2 node to process callbacks
            rclpy.spin_once(node, timeout_sec=0.1)
            
            # Check if all missions are complete
            if node.all_missions_complete():
                break
        
        # Print results
        collisions = node.get_collisions()
        if len(collisions) == 0:
            print("No collisions detected")
        else:
            print(f"Detected {len(collisions)} collisions between drones")
        
        if node.all_missions_complete():
            print("All drones completed their missions")
        else:
            print("Not all drones completed their missions")
    finally:
        # Clean up
        if 'node' in locals():
            node.destroy_node()
        if 'spawner' in locals():
            spawner.cleanup()
        rclpy.shutdown()
