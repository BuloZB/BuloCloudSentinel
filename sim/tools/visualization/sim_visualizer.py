#!/usr/bin/env python3
"""
Simulation Visualizer

This tool provides real-time visualization of simulation data, including:
- Drone trajectories
- Sensor data visualization
- Mission planning visualization
- Performance metrics
"""

import os
import sys
import time
import json
import yaml
import logging
import argparse
import threading
import numpy as np
from pathlib import Path
from typing import Dict, List, Any, Optional, Tuple
from dataclasses import dataclass, field

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
from sensor_msgs.msg import Image, CameraInfo, NavSatFix, Imu, PointCloud2
from std_msgs.msg import String, Float32, Bool

import cv2
import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui, QtWidgets
import pyqtgraph.opengl as gl

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger("sim_visualizer")


@dataclass
class DroneState:
    """Drone state data class."""
    id: str
    position: np.ndarray = field(default_factory=lambda: np.zeros(3))
    orientation: np.ndarray = field(default_factory=lambda: np.zeros(4))
    velocity: np.ndarray = field(default_factory=lambda: np.zeros(3))
    angular_velocity: np.ndarray = field(default_factory=lambda: np.zeros(3))
    trajectory: List[np.ndarray] = field(default_factory=list)
    waypoints: List[np.ndarray] = field(default_factory=list)
    battery: float = 100.0
    status: str = "idle"
    sensors: Dict[str, Any] = field(default_factory=dict)


class SimulationVisualizer:
    """
    Real-time visualization tool for simulation data.
    
    This class provides functionality for:
    - 3D visualization of drone trajectories
    - Sensor data visualization (camera, LiDAR, etc.)
    - Mission planning visualization
    - Performance metrics display
    """
    
    def __init__(self, config_path: str = None):
        """Initialize the visualization tool."""
        # Initialize ROS 2
        rclpy.init()
        self.node = Node('sim_visualizer')
        
        # Set up QoS profile
        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Load configuration
        self.config = self._load_config(config_path)
        
        # Initialize drone states
        self.drones = {}
        
        # Initialize visualization windows
        self.windows = {}
        
        # Initialize ROS subscribers
        self._setup_subscribers()
        
        # Initialize visualization
        self._setup_visualization()
        
        # Start ROS spinning thread
        self.spinning = True
        self.spin_thread = threading.Thread(target=self._spin)
        self.spin_thread.start()
    
    def _load_config(self, config_path: str) -> Dict[str, Any]:
        """Load configuration from YAML file."""
        config = {
            "visualization": {
                "trajectory": True,
                "waypoints": True,
                "sensors": True,
                "metrics": True,
                "update_rate": 30  # Hz
            },
            "display": {
                "3d_view": True,
                "camera_view": True,
                "lidar_view": True,
                "metrics_view": True
            },
            "recording": {
                "enabled": False,
                "output_dir": "recordings",
                "format": "mp4"
            }
        }
        
        if config_path and os.path.exists(config_path):
            try:
                with open(config_path, 'r') as f:
                    loaded_config = yaml.safe_load(f)
                    config.update(loaded_config)
                logger.info(f"Loaded configuration from {config_path}")
            except Exception as e:
                logger.error(f"Error loading configuration from {config_path}: {str(e)}")
        
        return config
    
    def _setup_subscribers(self):
        """Set up ROS subscribers."""
        # Drone pose subscribers
        self.pose_subs = {}
        
        # Drone telemetry subscribers
        self.telemetry_subs = {}
        
        # Drone sensor subscribers
        self.sensor_subs = {}
        
        # Drone mission subscribers
        self.mission_subs = {}
        
        # Simulation status subscriber
        self.status_sub = self.node.create_subscription(
            String,
            '/sim/status',
            self._status_callback,
            self.qos_profile
        )
    
    def _status_callback(self, msg: String):
        """Callback for simulation status updates."""
        try:
            status = json.loads(msg.data)
            
            # Check for drone list updates
            if "drones" in status:
                for drone_info in status["drones"]:
                    drone_id = drone_info["id"]
                    
                    # Add new drone if not already tracked
                    if drone_id not in self.drones:
                        self.drones[drone_id] = DroneState(id=drone_id)
                        self._setup_drone_subscribers(drone_id)
                        logger.info(f"Added drone: {drone_id}")
        except Exception as e:
            logger.error(f"Error processing status message: {str(e)}")
    
    def _setup_drone_subscribers(self, drone_id: str):
        """Set up subscribers for a specific drone."""
        # Pose subscriber
        self.pose_subs[drone_id] = self.node.create_subscription(
            PoseStamped,
            f'/{drone_id}/pose',
            lambda msg, id=drone_id: self._pose_callback(msg, id),
            self.qos_profile
        )
        
        # Telemetry subscribers
        self.telemetry_subs[drone_id] = {}
        
        # IMU subscriber
        self.telemetry_subs[drone_id]["imu"] = self.node.create_subscription(
            Imu,
            f'/{drone_id}/imu',
            lambda msg, id=drone_id: self._imu_callback(msg, id),
            self.qos_profile
        )
        
        # Battery subscriber
        self.telemetry_subs[drone_id]["battery"] = self.node.create_subscription(
            Float32,
            f'/{drone_id}/battery',
            lambda msg, id=drone_id: self._battery_callback(msg, id),
            self.qos_profile
        )
        
        # Status subscriber
        self.telemetry_subs[drone_id]["status"] = self.node.create_subscription(
            String,
            f'/{drone_id}/status',
            lambda msg, id=drone_id: self._drone_status_callback(msg, id),
            self.qos_profile
        )
        
        # Sensor subscribers
        self.sensor_subs[drone_id] = {}
        
        # Camera subscriber
        self.sensor_subs[drone_id]["camera"] = self.node.create_subscription(
            Image,
            f'/{drone_id}/camera',
            lambda msg, id=drone_id: self._camera_callback(msg, id),
            self.qos_profile
        )
        
        # LiDAR subscriber
        self.sensor_subs[drone_id]["lidar"] = self.node.create_subscription(
            PointCloud2,
            f'/{drone_id}/lidar',
            lambda msg, id=drone_id: self._lidar_callback(msg, id),
            self.qos_profile
        )
        
        # Mission subscribers
        self.mission_subs[drone_id] = {}
        
        # Path subscriber
        self.mission_subs[drone_id]["path"] = self.node.create_subscription(
            Path,
            f'/{drone_id}/path',
            lambda msg, id=drone_id: self._path_callback(msg, id),
            self.qos_profile
        )
    
    def _pose_callback(self, msg: PoseStamped, drone_id: str):
        """Callback for drone pose updates."""
        if drone_id not in self.drones:
            return
        
        # Update position
        self.drones[drone_id].position = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ])
        
        # Update orientation
        self.drones[drone_id].orientation = np.array([
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        ])
        
        # Add to trajectory
        self.drones[drone_id].trajectory.append(self.drones[drone_id].position.copy())
        
        # Limit trajectory length
        max_trajectory_length = 1000
        if len(self.drones[drone_id].trajectory) > max_trajectory_length:
            self.drones[drone_id].trajectory = self.drones[drone_id].trajectory[-max_trajectory_length:]
    
    def _imu_callback(self, msg: Imu, drone_id: str):
        """Callback for drone IMU updates."""
        if drone_id not in self.drones:
            return
        
        # Update angular velocity
        self.drones[drone_id].angular_velocity = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ])
        
        # Update linear acceleration
        if "imu" not in self.drones[drone_id].sensors:
            self.drones[drone_id].sensors["imu"] = {}
        
        self.drones[drone_id].sensors["imu"]["linear_acceleration"] = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ])
    
    def _battery_callback(self, msg: Float32, drone_id: str):
        """Callback for drone battery updates."""
        if drone_id not in self.drones:
            return
        
        # Update battery level
        self.drones[drone_id].battery = msg.data
    
    def _drone_status_callback(self, msg: String, drone_id: str):
        """Callback for drone status updates."""
        if drone_id not in self.drones:
            return
        
        # Update status
        self.drones[drone_id].status = msg.data
    
    def _camera_callback(self, msg: Image, drone_id: str):
        """Callback for drone camera updates."""
        if drone_id not in self.drones:
            return
        
        # Process camera image
        try:
            # Convert ROS image to OpenCV image
            if msg.encoding == "rgb8":
                cv_image = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                    msg.height, msg.width, 3)
            elif msg.encoding == "bgr8":
                cv_image = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                    msg.height, msg.width, 3)
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            elif msg.encoding == "mono8":
                cv_image = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                    msg.height, msg.width)
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2RGB)
            else:
                logger.warning(f"Unsupported image encoding: {msg.encoding}")
                return
            
            # Store image
            if "camera" not in self.drones[drone_id].sensors:
                self.drones[drone_id].sensors["camera"] = {}
            
            self.drones[drone_id].sensors["camera"]["image"] = cv_image
            
            # Update camera view if enabled
            if self.config["display"]["camera_view"] and "camera_view" in self.windows:
                self._update_camera_view(drone_id)
        except Exception as e:
            logger.error(f"Error processing camera image: {str(e)}")
    
    def _lidar_callback(self, msg: PointCloud2, drone_id: str):
        """Callback for drone LiDAR updates."""
        if drone_id not in self.drones:
            return
        
        # Process LiDAR point cloud
        try:
            # TODO: Implement point cloud conversion
            # For now, just store the raw message
            if "lidar" not in self.drones[drone_id].sensors:
                self.drones[drone_id].sensors["lidar"] = {}
            
            self.drones[drone_id].sensors["lidar"]["point_cloud"] = msg
            
            # Update LiDAR view if enabled
            if self.config["display"]["lidar_view"] and "lidar_view" in self.windows:
                self._update_lidar_view(drone_id)
        except Exception as e:
            logger.error(f"Error processing LiDAR data: {str(e)}")
    
    def _path_callback(self, msg: Path, drone_id: str):
        """Callback for drone path updates."""
        if drone_id not in self.drones:
            return
        
        # Extract waypoints
        waypoints = []
        for pose in msg.poses:
            waypoints.append(np.array([
                pose.pose.position.x,
                pose.pose.position.y,
                pose.pose.position.z
            ]))
        
        # Update waypoints
        self.drones[drone_id].waypoints = waypoints
    
    def _setup_visualization(self):
        """Set up visualization windows."""
        # Create PyQtGraph application
        self.app = QtWidgets.QApplication([])
        
        # Create main window
        self.main_window = QtWidgets.QMainWindow()
        self.main_window.setWindowTitle("Simulation Visualizer")
        self.main_window.resize(1600, 900)
        
        # Create central widget
        self.central_widget = QtWidgets.QWidget()
        self.main_window.setCentralWidget(self.central_widget)
        
        # Create layout
        self.layout = QtWidgets.QGridLayout()
        self.central_widget.setLayout(self.layout)
        
        # Create 3D view
        if self.config["display"]["3d_view"]:
            self._setup_3d_view()
        
        # Create camera view
        if self.config["display"]["camera_view"]:
            self._setup_camera_view()
        
        # Create LiDAR view
        if self.config["display"]["lidar_view"]:
            self._setup_lidar_view()
        
        # Create metrics view
        if self.config["display"]["metrics_view"]:
            self._setup_metrics_view()
        
        # Show main window
        self.main_window.show()
        
        # Set up update timer
        self.update_timer = QtCore.QTimer()
        self.update_timer.timeout.connect(self._update_visualization)
        self.update_timer.start(1000 // self.config["visualization"]["update_rate"])
    
    def _setup_3d_view(self):
        """Set up 3D visualization view."""
        # Create 3D view widget
        self.windows["3d_view"] = gl.GLViewWidget()
        self.layout.addWidget(self.windows["3d_view"], 0, 0, 2, 2)
        
        # Add grid
        grid = gl.GLGridItem()
        grid.setSize(500, 500)
        grid.setSpacing(10, 10)
        self.windows["3d_view"].addItem(grid)
        
        # Add coordinate axes
        axis_x = gl.GLLinePlotItem(pos=np.array([[0, 0, 0], [10, 0, 0]]), color=(1, 0, 0, 1), width=2)
        axis_y = gl.GLLinePlotItem(pos=np.array([[0, 0, 0], [0, 10, 0]]), color=(0, 1, 0, 1), width=2)
        axis_z = gl.GLLinePlotItem(pos=np.array([[0, 0, 0], [0, 0, 10]]), color=(0, 0, 1, 1), width=2)
        self.windows["3d_view"].addItem(axis_x)
        self.windows["3d_view"].addItem(axis_y)
        self.windows["3d_view"].addItem(axis_z)
        
        # Initialize drone visualization items
        self.drone_items = {}
    
    def _setup_camera_view(self):
        """Set up camera visualization view."""
        # Create camera view widget
        self.windows["camera_view"] = pg.GraphicsLayoutWidget()
        self.layout.addWidget(self.windows["camera_view"], 0, 2, 1, 1)
        
        # Add view box
        self.camera_viewbox = self.windows["camera_view"].addViewBox()
        self.camera_viewbox.setAspectLocked(True)
        
        # Add image item
        self.camera_image = pg.ImageItem()
        self.camera_viewbox.addItem(self.camera_image)
        
        # Add drone selector
        self.camera_drone_selector = QtWidgets.QComboBox()
        self.camera_drone_selector.currentTextChanged.connect(self._on_camera_drone_changed)
        self.layout.addWidget(self.camera_drone_selector, 1, 2, 1, 1)
    
    def _setup_lidar_view(self):
        """Set up LiDAR visualization view."""
        # Create LiDAR view widget
        self.windows["lidar_view"] = gl.GLViewWidget()
        self.layout.addWidget(self.windows["lidar_view"], 2, 0, 1, 1)
        
        # Add grid
        grid = gl.GLGridItem()
        grid.setSize(100, 100)
        grid.setSpacing(10, 10)
        self.windows["lidar_view"].addItem(grid)
        
        # Initialize point cloud item
        self.lidar_points = gl.GLScatterPlotItem(pos=np.zeros((1, 3)), color=(1, 1, 1, 1), size=0.1)
        self.windows["lidar_view"].addItem(self.lidar_points)
        
        # Add drone selector
        self.lidar_drone_selector = QtWidgets.QComboBox()
        self.lidar_drone_selector.currentTextChanged.connect(self._on_lidar_drone_changed)
        self.layout.addWidget(self.lidar_drone_selector, 3, 0, 1, 1)
    
    def _setup_metrics_view(self):
        """Set up metrics visualization view."""
        # Create metrics view widget
        self.windows["metrics_view"] = pg.GraphicsLayoutWidget()
        self.layout.addWidget(self.windows["metrics_view"], 2, 1, 2, 2)
        
        # Add plots
        self.metrics_plots = {}
        
        # Battery plot
        self.metrics_plots["battery"] = self.windows["metrics_view"].addPlot(title="Battery Level")
        self.metrics_plots["battery"].setLabel("left", "Battery", "%")
        self.metrics_plots["battery"].setLabel("bottom", "Time", "s")
        self.metrics_plots["battery"].setYRange(0, 100)
        self.metrics_plots["battery"].addLegend()
        
        self.windows["metrics_view"].nextRow()
        
        # Velocity plot
        self.metrics_plots["velocity"] = self.windows["metrics_view"].addPlot(title="Velocity")
        self.metrics_plots["velocity"].setLabel("left", "Velocity", "m/s")
        self.metrics_plots["velocity"].setLabel("bottom", "Time", "s")
        self.metrics_plots["velocity"].addLegend()
        
        # Initialize plot data
        self.metrics_data = {}
        self.metrics_curves = {}
    
    def _update_visualization(self):
        """Update visualization with latest data."""
        # Update drone selectors
        self._update_drone_selectors()
        
        # Update 3D view
        if self.config["display"]["3d_view"] and "3d_view" in self.windows:
            self._update_3d_view()
        
        # Update metrics view
        if self.config["display"]["metrics_view"] and "metrics_view" in self.windows:
            self._update_metrics_view()
    
    def _update_drone_selectors(self):
        """Update drone selector combo boxes."""
        # Get current selections
        current_camera_drone = self.camera_drone_selector.currentText() if self.camera_drone_selector.count() > 0 else ""
        current_lidar_drone = self.lidar_drone_selector.currentText() if self.lidar_drone_selector.count() > 0 else ""
        
        # Update camera drone selector
        if self.config["display"]["camera_view"] and "camera_view" in self.windows:
            self.camera_drone_selector.clear()
            for drone_id in self.drones:
                self.camera_drone_selector.addItem(drone_id)
            
            # Restore selection if possible
            index = self.camera_drone_selector.findText(current_camera_drone)
            if index >= 0:
                self.camera_drone_selector.setCurrentIndex(index)
        
        # Update LiDAR drone selector
        if self.config["display"]["lidar_view"] and "lidar_view" in self.windows:
            self.lidar_drone_selector.clear()
            for drone_id in self.drones:
                self.lidar_drone_selector.addItem(drone_id)
            
            # Restore selection if possible
            index = self.lidar_drone_selector.findText(current_lidar_drone)
            if index >= 0:
                self.lidar_drone_selector.setCurrentIndex(index)
    
    def _update_3d_view(self):
        """Update 3D visualization view."""
        # Update drone visualizations
        for drone_id, drone in self.drones.items():
            # Create visualization items if they don't exist
            if drone_id not in self.drone_items:
                self.drone_items[drone_id] = {}
                
                # Drone position marker
                self.drone_items[drone_id]["position"] = gl.GLScatterPlotItem(
                    pos=np.array([drone.position]),
                    color=pg.glColor((drone_id.hash() % 8) + 1),
                    size=10
                )
                self.windows["3d_view"].addItem(self.drone_items[drone_id]["position"])
                
                # Drone trajectory
                self.drone_items[drone_id]["trajectory"] = gl.GLLinePlotItem(
                    pos=np.array(drone.trajectory),
                    color=pg.glColor((drone_id.hash() % 8) + 1, 0.5),
                    width=2
                )
                self.windows["3d_view"].addItem(self.drone_items[drone_id]["trajectory"])
                
                # Drone waypoints
                self.drone_items[drone_id]["waypoints"] = gl.GLScatterPlotItem(
                    pos=np.array(drone.waypoints) if drone.waypoints else np.zeros((1, 3)),
                    color=pg.glColor((drone_id.hash() % 8) + 1, 0.8),
                    size=5
                )
                self.windows["3d_view"].addItem(self.drone_items[drone_id]["waypoints"])
                
                # Drone waypoint lines
                self.drone_items[drone_id]["waypoint_lines"] = gl.GLLinePlotItem(
                    pos=np.array(drone.waypoints) if drone.waypoints else np.zeros((1, 3)),
                    color=pg.glColor((drone_id.hash() % 8) + 1, 0.3),
                    width=1
                )
                self.windows["3d_view"].addItem(self.drone_items[drone_id]["waypoint_lines"])
            
            # Update position
            self.drone_items[drone_id]["position"].setData(pos=np.array([drone.position]))
            
            # Update trajectory
            if self.config["visualization"]["trajectory"] and drone.trajectory:
                self.drone_items[drone_id]["trajectory"].setData(pos=np.array(drone.trajectory))
            
            # Update waypoints
            if self.config["visualization"]["waypoints"] and drone.waypoints:
                self.drone_items[drone_id]["waypoints"].setData(pos=np.array(drone.waypoints))
                self.drone_items[drone_id]["waypoint_lines"].setData(pos=np.array(drone.waypoints))
    
    def _update_camera_view(self, drone_id: str):
        """Update camera visualization view."""
        if drone_id not in self.drones:
            return
        
        # Get camera image
        if "camera" in self.drones[drone_id].sensors and "image" in self.drones[drone_id].sensors["camera"]:
            image = self.drones[drone_id].sensors["camera"]["image"]
            
            # Update image item
            self.camera_image.setImage(image.transpose(1, 0, 2) if len(image.shape) == 3 else image.T)
    
    def _update_lidar_view(self, drone_id: str):
        """Update LiDAR visualization view."""
        if drone_id not in self.drones:
            return
        
        # Get LiDAR point cloud
        if "lidar" in self.drones[drone_id].sensors and "point_cloud" in self.drones[drone_id].sensors["lidar"]:
            # TODO: Implement point cloud visualization
            pass
    
    def _update_metrics_view(self):
        """Update metrics visualization view."""
        # Initialize time axis if needed
        if "time" not in self.metrics_data:
            self.metrics_data["time"] = np.arange(100) * 0.1  # 10 Hz for 10 seconds
        
        # Update metrics for each drone
        for drone_id, drone in self.drones.items():
            # Initialize drone metrics if needed
            if drone_id not in self.metrics_data:
                self.metrics_data[drone_id] = {
                    "battery": np.ones(100) * 100.0,
                    "velocity": np.zeros(100)
                }
            
            if drone_id not in self.metrics_curves:
                self.metrics_curves[drone_id] = {
                    "battery": self.metrics_plots["battery"].plot(
                        self.metrics_data["time"],
                        self.metrics_data[drone_id]["battery"],
                        pen=pg.mkPen(pg.glColor((drone_id.hash() % 8) + 1), width=2),
                        name=drone_id
                    ),
                    "velocity": self.metrics_plots["velocity"].plot(
                        self.metrics_data["time"],
                        self.metrics_data[drone_id]["velocity"],
                        pen=pg.mkPen(pg.glColor((drone_id.hash() % 8) + 1), width=2),
                        name=drone_id
                    )
                }
            
            # Update battery data
            self.metrics_data[drone_id]["battery"] = np.roll(self.metrics_data[drone_id]["battery"], -1)
            self.metrics_data[drone_id]["battery"][-1] = drone.battery
            self.metrics_curves[drone_id]["battery"].setData(
                self.metrics_data["time"],
                self.metrics_data[drone_id]["battery"]
            )
            
            # Update velocity data
            velocity = np.linalg.norm(drone.velocity) if hasattr(drone, "velocity") else 0.0
            self.metrics_data[drone_id]["velocity"] = np.roll(self.metrics_data[drone_id]["velocity"], -1)
            self.metrics_data[drone_id]["velocity"][-1] = velocity
            self.metrics_curves[drone_id]["velocity"].setData(
                self.metrics_data["time"],
                self.metrics_data[drone_id]["velocity"]
            )
    
    def _on_camera_drone_changed(self, drone_id: str):
        """Handle camera drone selection change."""
        if drone_id in self.drones:
            self._update_camera_view(drone_id)
    
    def _on_lidar_drone_changed(self, drone_id: str):
        """Handle LiDAR drone selection change."""
        if drone_id in self.drones:
            self._update_lidar_view(drone_id)
    
    def _spin(self):
        """Spin ROS node to process callbacks."""
        while self.spinning and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.01)
    
    def run(self):
        """Run the visualization tool."""
        # Start Qt event loop
        self.app.exec_()
    
    def cleanup(self):
        """Clean up resources."""
        # Stop ROS spinning
        self.spinning = False
        if hasattr(self, "spin_thread") and self.spin_thread.is_alive():
            self.spin_thread.join()
        
        # Destroy ROS node
        self.node.destroy_node()
        rclpy.shutdown()


def main():
    """Main function."""
    parser = argparse.ArgumentParser(description="Simulation Visualizer")
    parser.add_argument("--config", type=str, help="Path to configuration file")
    args = parser.parse_args()
    
    visualizer = None
    try:
        # Create visualizer
        visualizer = SimulationVisualizer(args.config)
        
        # Run visualizer
        visualizer.run()
    except KeyboardInterrupt:
        logger.info("Interrupted by user")
    except Exception as e:
        logger.error(f"Error: {str(e)}")
    finally:
        # Clean up
        if visualizer:
            visualizer.cleanup()


if __name__ == "__main__":
    main()
