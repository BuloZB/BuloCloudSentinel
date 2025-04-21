#!/usr/bin/env python3
"""
SLAM Node for Indoor Drone System

This node integrates LiDAR-based SLAM (FAST-LIO) and Visual SLAM (ORB-SLAM3)
for robust indoor positioning and mapping.
"""

import os
import sys
import time
import logging
import numpy as np
import threading
import yaml
from typing import Dict, List, Any, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String, Header
from sensor_msgs.msg import PointCloud2, Image, Imu
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster

# Import custom message types
from indoor_drone_msgs.msg import SlamStatus

# Import SLAM wrappers
from slam_wrappers.fast_lio_wrapper import FastLioWrapper
from slam_wrappers.orb_slam3_wrapper import OrbSlam3Wrapper
from slam_wrappers.sensor_fusion import SensorFusion

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class SlamNode(Node):
    """ROS2 node for SLAM processing."""
    
    def __init__(self):
        """Initialize the SLAM node."""
        super().__init__('indoor_drone_slam_node')
        
        # Load configuration
        self.config = self._load_config()
        
        # Set up QoS profile
        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Initialize SLAM systems
        self.fast_lio = FastLioWrapper(self.config['fast_lio'])
        self.orb_slam3 = OrbSlam3Wrapper(self.config['orb_slam3'])
        self.sensor_fusion = SensorFusion(self.config['sensor_fusion'])
        
        # Initialize state variables
        self.current_pose = None
        self.current_map = None
        self.slam_status = {
            'status': 'initializing',
            'position_uncertainty': 0.0,
            'map_quality': 0.0,
            'loop_closures': 0,
            'processing_time': 0.0
        }
        
        # Set up publishers
        self.pose_pub = self.create_publisher(
            PoseStamped,
            '/indoor_drone/slam/pose',
            self.qos_profile
        )
        
        self.map_pub = self.create_publisher(
            OccupancyGrid,
            '/indoor_drone/slam/map',
            self.qos_profile
        )
        
        self.point_cloud_pub = self.create_publisher(
            PointCloud2,
            '/indoor_drone/slam/point_cloud',
            self.qos_profile
        )
        
        self.status_pub = self.create_publisher(
            SlamStatus,
            '/indoor_drone/slam/status',
            self.qos_profile
        )
        
        # Set up subscribers
        self.lidar_sub = self.create_subscription(
            PointCloud2,
            '/indoor_drone/sensors/lidar',
            self._lidar_callback,
            self.qos_profile
        )
        
        self.camera_sub = self.create_subscription(
            Image,
            '/indoor_drone/sensors/camera',
            self._camera_callback,
            self.qos_profile
        )
        
        self.imu_sub = self.create_subscription(
            Imu,
            '/indoor_drone/sensors/imu',
            self._imu_callback,
            self.qos_profile
        )
        
        # Set up transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Set up timers
        self.status_timer = self.create_timer(1.0, self._publish_status)
        self.map_timer = self.create_timer(5.0, self._publish_map)
        
        logger.info("SLAM node initialized")
    
    def _load_config(self) -> Dict[str, Any]:
        """Load configuration from YAML file."""
        config_path = os.environ.get('SLAM_CONFIG_PATH', '/app/config/slam_config.yaml')
        
        try:
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)
            
            logger.info(f"Loaded configuration from {config_path}")
            return config
        except Exception as e:
            logger.error(f"Failed to load configuration: {str(e)}")
            
            # Return default configuration
            return {
                'fast_lio': {
                    'enabled': True,
                    'lidar_topic': '/indoor_drone/sensors/lidar',
                    'imu_topic': '/indoor_drone/sensors/imu',
                    'map_resolution': 0.05
                },
                'orb_slam3': {
                    'enabled': True,
                    'camera_topic': '/indoor_drone/sensors/camera',
                    'vocabulary_path': '/app/src/orb_slam3/Vocabulary/ORBvoc.txt',
                    'settings_path': '/app/config/camera_config.yaml'
                },
                'sensor_fusion': {
                    'method': 'complementary_filter',
                    'lidar_weight': 0.7,
                    'visual_weight': 0.3
                }
            }
    
    def _lidar_callback(self, msg: PointCloud2):
        """Process LiDAR data."""
        start_time = time.time()
        
        try:
            # Process LiDAR data with FAST-LIO
            lidar_pose, lidar_map = self.fast_lio.process_lidar(msg)
            
            # Update fusion with LiDAR results
            if lidar_pose is not None:
                self.sensor_fusion.update_lidar(lidar_pose, lidar_map)
                
                # Get fused pose and map
                self.current_pose = self.sensor_fusion.get_fused_pose()
                self.current_map = self.sensor_fusion.get_fused_map()
                
                # Publish pose
                self._publish_pose()
                
                # Publish point cloud
                self._publish_point_cloud(lidar_map)
                
                # Update status
                self.slam_status['processing_time'] = time.time() - start_time
                self.slam_status['status'] = 'running'
        except Exception as e:
            logger.error(f"Error processing LiDAR data: {str(e)}")
            self.slam_status['status'] = 'error'
    
    def _camera_callback(self, msg: Image):
        """Process camera data."""
        start_time = time.time()
        
        try:
            # Process camera data with ORB-SLAM3
            visual_pose, visual_map, loop_closure = self.orb_slam3.process_image(msg)
            
            # Update fusion with visual results
            if visual_pose is not None:
                self.sensor_fusion.update_visual(visual_pose, visual_map)
                
                # Get fused pose and map
                self.current_pose = self.sensor_fusion.get_fused_pose()
                self.current_map = self.sensor_fusion.get_fused_map()
                
                # Publish pose
                self._publish_pose()
                
                # Update status
                if loop_closure:
                    self.slam_status['loop_closures'] += 1
                
                self.slam_status['processing_time'] = time.time() - start_time
                self.slam_status['status'] = 'running'
        except Exception as e:
            logger.error(f"Error processing camera data: {str(e)}")
            self.slam_status['status'] = 'error'
    
    def _imu_callback(self, msg: Imu):
        """Process IMU data."""
        try:
            # Process IMU data with FAST-LIO
            self.fast_lio.process_imu(msg)
        except Exception as e:
            logger.error(f"Error processing IMU data: {str(e)}")
    
    def _publish_pose(self):
        """Publish current pose."""
        if self.current_pose is None:
            return
        
        # Create pose message
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'
        
        # Set position
        pose_msg.pose.position.x = self.current_pose['position']['x']
        pose_msg.pose.position.y = self.current_pose['position']['y']
        pose_msg.pose.position.z = self.current_pose['position']['z']
        
        # Set orientation
        pose_msg.pose.orientation.x = self.current_pose['orientation']['x']
        pose_msg.pose.orientation.y = self.current_pose['orientation']['y']
        pose_msg.pose.orientation.z = self.current_pose['orientation']['z']
        pose_msg.pose.orientation.w = self.current_pose['orientation']['w']
        
        # Publish pose
        self.pose_pub.publish(pose_msg)
        
        # Broadcast transform
        self._broadcast_transform(pose_msg)
    
    def _broadcast_transform(self, pose_msg: PoseStamped):
        """Broadcast transform from map to drone."""
        transform = TransformStamped()
        transform.header = pose_msg.header
        transform.child_frame_id = 'drone'
        
        # Set translation
        transform.transform.translation.x = pose_msg.pose.position.x
        transform.transform.translation.y = pose_msg.pose.position.y
        transform.transform.translation.z = pose_msg.pose.position.z
        
        # Set rotation
        transform.transform.rotation = pose_msg.pose.orientation
        
        # Broadcast transform
        self.tf_broadcaster.sendTransform(transform)
    
    def _publish_map(self):
        """Publish current map."""
        if self.current_map is None:
            return
        
        # Create map message
        map_msg = OccupancyGrid()
        map_msg.header.stamp = self.get_clock().now().to_msg()
        map_msg.header.frame_id = 'map'
        
        # Set map metadata
        map_msg.info.resolution = self.current_map['resolution']
        map_msg.info.width = self.current_map['width']
        map_msg.info.height = self.current_map['height']
        map_msg.info.origin.position.x = self.current_map['origin']['x']
        map_msg.info.origin.position.y = self.current_map['origin']['y']
        map_msg.info.origin.position.z = self.current_map['origin']['z']
        map_msg.info.origin.orientation.w = 1.0
        
        # Set map data
        map_msg.data = self.current_map['data']
        
        # Publish map
        self.map_pub.publish(map_msg)
    
    def _publish_point_cloud(self, point_cloud):
        """Publish point cloud."""
        if point_cloud is None:
            return
        
        # Convert point cloud to ROS message
        # This is a simplified version; in a real implementation,
        # you would convert the point cloud data to a PointCloud2 message
        
        # For now, we'll just create a dummy message
        pc_msg = PointCloud2()
        pc_msg.header.stamp = self.get_clock().now().to_msg()
        pc_msg.header.frame_id = 'map'
        
        # Publish point cloud
        self.point_cloud_pub.publish(pc_msg)
    
    def _publish_status(self):
        """Publish SLAM status."""
        # Create status message
        status_msg = SlamStatus()
        status_msg.header.stamp = self.get_clock().now().to_msg()
        status_msg.status = self.slam_status['status']
        status_msg.position_uncertainty = self.slam_status['position_uncertainty']
        status_msg.map_quality = self.slam_status['map_quality']
        status_msg.loop_closures = self.slam_status['loop_closures']
        status_msg.processing_time = self.slam_status['processing_time']
        
        # Publish status
        self.status_pub.publish(status_msg)

def main(args=None):
    """Main function."""
    rclpy.init(args=args)
    
    slam_node = SlamNode()
    
    try:
        rclpy.spin(slam_node)
    except KeyboardInterrupt:
        pass
    finally:
        slam_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
