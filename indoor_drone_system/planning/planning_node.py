#!/usr/bin/env python3
"""
Path Planning Node for Indoor Drone System

This node implements the EGO-Swarm algorithm and other path planning methods
for autonomous navigation in indoor environments.
"""

import os
import sys
import time
import logging
import numpy as np
import threading
import yaml
from typing import Dict, List, Any, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String, Header
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray

# Import custom message types
from indoor_drone_msgs.msg import MissionPlan as MissionPlanMsg
from indoor_drone_msgs.srv import PlanPath, ExecuteMission

# Import planning algorithms
from planners.ego_swarm import EgoSwarmPlanner
from planners.rrt_star import RRTStarPlanner
from planners.a_star import AStarPlanner
from planners.trajectory_optimizer import TrajectoryOptimizer

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class PlanningNode(Node):
    """ROS2 node for path planning."""
    
    def __init__(self):
        """Initialize the planning node."""
        super().__init__('indoor_drone_planning_node')
        
        # Load configuration
        self.config = self._load_config()
        
        # Set up QoS profile
        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Initialize planners
        self.ego_swarm = EgoSwarmPlanner(self.config['ego_swarm'])
        self.rrt_star = RRTStarPlanner(self.config['rrt_star'])
        self.a_star = AStarPlanner(self.config['a_star'])
        self.trajectory_optimizer = TrajectoryOptimizer(self.config['trajectory_optimizer'])
        
        # Initialize state variables
        self.current_map = None
        self.current_mission = None
        self.current_path = None
        self.active_drones = {}
        
        # Set up publishers
        self.path_pub = self.create_publisher(
            Path,
            '/indoor_drone/planning/path',
            self.qos_profile
        )
        
        self.visualization_pub = self.create_publisher(
            MarkerArray,
            '/indoor_drone/planning/visualization',
            self.qos_profile
        )
        
        # Set up subscribers
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/indoor_drone/slam/map',
            self._map_callback,
            self.qos_profile
        )
        
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/indoor_drone/slam/pose',
            self._pose_callback,
            self.qos_profile
        )
        
        self.mission_sub = self.create_subscription(
            MissionPlanMsg,
            '/indoor_drone/mission',
            self._mission_callback,
            self.qos_profile
        )
        
        # Set up services
        self.plan_path_service = self.create_service(
            PlanPath,
            '/indoor_drone/planning/plan_path',
            self._plan_path_callback
        )
        
        self.execute_mission_service = self.create_service(
            ExecuteMission,
            '/indoor_drone/execute_mission',
            self._execute_mission_callback
        )
        
        # Set up timers
        self.visualization_timer = self.create_timer(1.0, self._publish_visualization)
        
        logger.info("Planning node initialized")
    
    def _load_config(self) -> Dict[str, Any]:
        """Load configuration from YAML file."""
        config_path = os.environ.get('PLANNING_CONFIG_PATH', '/app/config/planning_config.yaml')
        
        try:
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)
            
            logger.info(f"Loaded configuration from {config_path}")
            return config
        except Exception as e:
            logger.error(f"Failed to load configuration: {str(e)}")
            
            # Return default configuration
            return {
                'ego_swarm': {
                    'max_velocity': 1.0,
                    'max_acceleration': 0.5,
                    'safety_distance': 0.5,
                    'planning_horizon': 3.0,
                    'update_rate': 10.0
                },
                'rrt_star': {
                    'max_iterations': 1000,
                    'step_size': 0.2,
                    'goal_sample_rate': 0.1,
                    'search_radius': 1.0
                },
                'a_star': {
                    'heuristic': 'euclidean',
                    'diagonal_movement': True,
                    'weight': 1.0
                },
                'trajectory_optimizer': {
                    'max_velocity': 1.0,
                    'max_acceleration': 0.5,
                    'continuity_weight': 1.0,
                    'smoothness_weight': 1.0,
                    'obstacle_weight': 10.0
                }
            }
    
    def _map_callback(self, msg: OccupancyGrid):
        """Process map data."""
        try:
            # Convert ROS map message to internal format
            self.current_map = {
                'resolution': msg.info.resolution,
                'width': msg.info.width,
                'height': msg.info.height,
                'origin': {
                    'x': msg.info.origin.position.x,
                    'y': msg.info.origin.position.y,
                    'z': msg.info.origin.position.z
                },
                'data': list(msg.data)
            }
            
            # Update planners with new map
            self.ego_swarm.update_map(self.current_map)
            self.rrt_star.update_map(self.current_map)
            self.a_star.update_map(self.current_map)
            self.trajectory_optimizer.update_map(self.current_map)
            
            logger.info("Updated map data")
        except Exception as e:
            logger.error(f"Error processing map data: {str(e)}")
    
    def _pose_callback(self, msg: PoseStamped):
        """Process pose data."""
        try:
            # Extract drone ID from frame ID (assuming format 'drone_X')
            frame_parts = msg.header.frame_id.split('_')
            if len(frame_parts) > 1 and frame_parts[0] == 'drone':
                drone_id = frame_parts[1]
            else:
                drone_id = 'default'
            
            # Update drone position
            self.active_drones[drone_id] = {
                'position': {
                    'x': msg.pose.position.x,
                    'y': msg.pose.position.y,
                    'z': msg.pose.position.z
                },
                'orientation': {
                    'x': msg.pose.orientation.x,
                    'y': msg.pose.orientation.y,
                    'z': msg.pose.orientation.z,
                    'w': msg.pose.orientation.w
                },
                'timestamp': self.get_clock().now().to_msg()
            }
            
            # Update planners with new drone positions
            self.ego_swarm.update_drone_positions(self.active_drones)
        except Exception as e:
            logger.error(f"Error processing pose data: {str(e)}")
    
    def _mission_callback(self, msg: MissionPlanMsg):
        """Process mission data."""
        try:
            # Convert ROS mission message to internal format
            self.current_mission = {
                'mission_id': msg.mission_id,
                'drone_id': msg.drone_id,
                'waypoints': [
                    {
                        'position': {
                            'x': waypoint.position.x,
                            'y': waypoint.position.y,
                            'z': waypoint.position.z
                        },
                        'orientation': {
                            'x': waypoint.orientation.x,
                            'y': waypoint.orientation.y,
                            'z': waypoint.orientation.z,
                            'w': waypoint.orientation.w
                        } if waypoint.has_orientation else None,
                        'wait_time': waypoint.wait_time,
                        'action': waypoint.action if waypoint.has_action else None
                    }
                    for waypoint in msg.waypoints
                ],
                'status': 'received'
            }
            
            logger.info(f"Received mission {msg.mission_id} for drone {msg.drone_id}")
            
            # Plan path for the mission
            self._plan_path_for_mission()
        except Exception as e:
            logger.error(f"Error processing mission data: {str(e)}")
    
    def _plan_path_callback(self, request, response):
        """Handle path planning service requests."""
        try:
            # Extract start and goal positions
            start = {
                'x': request.start.x,
                'y': request.start.y,
                'z': request.start.z
            }
            
            goal = {
                'x': request.goal.x,
                'y': request.goal.y,
                'z': request.goal.z
            }
            
            # Select planner based on request
            if request.planner == 'ego_swarm':
                planner = self.ego_swarm
            elif request.planner == 'rrt_star':
                planner = self.rrt_star
            elif request.planner == 'a_star':
                planner = self.a_star
            else:
                planner = self.ego_swarm  # Default
            
            # Plan path
            path, success = planner.plan(start, goal)
            
            if success:
                # Optimize trajectory
                optimized_path = self.trajectory_optimizer.optimize(path)
                
                # Convert path to ROS message
                path_msg = self._create_path_message(optimized_path)
                
                # Set response
                response.success = True
                response.path = path_msg
            else:
                response.success = False
                response.message = "Failed to find path"
            
            return response
        except Exception as e:
            logger.error(f"Error handling plan path request: {str(e)}")
            response.success = False
            response.message = f"Error: {str(e)}"
            return response
    
    def _execute_mission_callback(self, request, response):
        """Handle mission execution service requests."""
        try:
            mission_id = request.mission_id
            drone_id = request.drone_id
            
            logger.info(f"Executing mission {mission_id} for drone {drone_id}")
            
            # Check if mission exists
            if self.current_mission is None or self.current_mission['mission_id'] != mission_id:
                response.success = False
                response.message = f"Mission {mission_id} not found"
                return response
            
            # Check if drone exists
            if drone_id not in self.active_drones:
                response.success = False
                response.message = f"Drone {drone_id} not found"
                return response
            
            # Plan path for the mission
            success = self._plan_path_for_mission()
            
            if success:
                # Publish path
                self._publish_path()
                
                # Update mission status
                self.current_mission['status'] = 'executing'
                
                response.success = True
            else:
                response.success = False
                response.message = "Failed to plan path for mission"
            
            return response
        except Exception as e:
            logger.error(f"Error handling execute mission request: {str(e)}")
            response.success = False
            response.message = f"Error: {str(e)}"
            return response
    
    def _plan_path_for_mission(self) -> bool:
        """Plan path for the current mission."""
        if self.current_mission is None or self.current_map is None:
            return False
        
        try:
            # Get drone position
            drone_id = self.current_mission['drone_id']
            if drone_id not in self.active_drones:
                logger.error(f"Drone {drone_id} not found")
                return False
            
            drone_position = self.active_drones[drone_id]['position']
            
            # Extract waypoints
            waypoints = [drone_position] + [wp['position'] for wp in self.current_mission['waypoints']]
            
            # Plan path through all waypoints
            path = []
            for i in range(len(waypoints) - 1):
                start = waypoints[i]
                goal = waypoints[i + 1]
                
                # Plan segment
                segment, success = self.ego_swarm.plan(start, goal)
                
                if not success:
                    logger.error(f"Failed to plan path segment from {start} to {goal}")
                    return False
                
                # Add segment to path (avoiding duplicates)
                if i > 0:
                    segment = segment[1:]  # Skip first point as it's the last point of the previous segment
                
                path.extend(segment)
            
            # Optimize trajectory
            self.current_path = self.trajectory_optimizer.optimize(path)
            
            # Publish path
            self._publish_path()
            
            logger.info(f"Planned path for mission {self.current_mission['mission_id']} with {len(self.current_path)} points")
            
            return True
        except Exception as e:
            logger.error(f"Error planning path for mission: {str(e)}")
            return False
    
    def _publish_path(self):
        """Publish the current path."""
        if self.current_path is None:
            return
        
        # Create path message
        path_msg = self._create_path_message(self.current_path)
        
        # Publish path
        self.path_pub.publish(path_msg)
    
    def _create_path_message(self, path: List[Dict[str, float]]) -> Path:
        """Create a Path message from a list of positions."""
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'
        
        for point in path:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = point['x']
            pose.pose.position.y = point['y']
            pose.pose.position.z = point['z']
            pose.pose.orientation.w = 1.0  # Default orientation
            
            path_msg.poses.append(pose)
        
        return path_msg
    
    def _publish_visualization(self):
        """Publish visualization markers."""
        if self.current_path is None:
            return
        
        # Create marker array
        marker_array = MarkerArray()
        
        # Add path marker
        path_marker = Marker()
        path_marker.header.stamp = self.get_clock().now().to_msg()
        path_marker.header.frame_id = 'map'
        path_marker.ns = 'path'
        path_marker.id = 0
        path_marker.type = Marker.LINE_STRIP
        path_marker.action = Marker.ADD
        path_marker.scale.x = 0.1  # Line width
        path_marker.color.r = 0.0
        path_marker.color.g = 1.0
        path_marker.color.b = 0.0
        path_marker.color.a = 1.0
        path_marker.pose.orientation.w = 1.0
        
        for point in self.current_path:
            p = Point()
            p.x = point['x']
            p.y = point['y']
            p.z = point['z']
            path_marker.points.append(p)
        
        marker_array.markers.append(path_marker)
        
        # Add waypoint markers
        if self.current_mission is not None:
            for i, waypoint in enumerate(self.current_mission['waypoints']):
                waypoint_marker = Marker()
                waypoint_marker.header.stamp = self.get_clock().now().to_msg()
                waypoint_marker.header.frame_id = 'map'
                waypoint_marker.ns = 'waypoints'
                waypoint_marker.id = i
                waypoint_marker.type = Marker.SPHERE
                waypoint_marker.action = Marker.ADD
                waypoint_marker.scale.x = 0.3
                waypoint_marker.scale.y = 0.3
                waypoint_marker.scale.z = 0.3
                waypoint_marker.color.r = 1.0
                waypoint_marker.color.g = 0.0
                waypoint_marker.color.b = 0.0
                waypoint_marker.color.a = 1.0
                waypoint_marker.pose.position.x = waypoint['position']['x']
                waypoint_marker.pose.position.y = waypoint['position']['y']
                waypoint_marker.pose.position.z = waypoint['position']['z']
                waypoint_marker.pose.orientation.w = 1.0
                
                marker_array.markers.append(waypoint_marker)
        
        # Publish marker array
        self.visualization_pub.publish(marker_array)

def main(args=None):
    """Main function."""
    rclpy.init(args=args)
    
    planning_node = PlanningNode()
    
    try:
        rclpy.spin(planning_node)
    except KeyboardInterrupt:
        pass
    finally:
        planning_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
