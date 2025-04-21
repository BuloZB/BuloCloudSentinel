"""
Heat Mapper for the Vision System.

This module provides heat map generation for visualization of analysis results.
"""

import logging
import time
import numpy as np
import cv2
import matplotlib
matplotlib.use('Agg')  # Use Agg backend (non-interactive)
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap
from typing import Dict, List, Any, Optional, Tuple
import io
from datetime import datetime

# Setup logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class HeatMapper:
    """Generator for heat maps and flow visualizations."""
    
    def __init__(self, config: Dict[str, Any]):
        """Initialize the heat mapper."""
        self.config = config
        self.color_maps = {
            "jet": plt.cm.jet,
            "viridis": plt.cm.viridis,
            "plasma": plt.cm.plasma,
            "inferno": plt.cm.inferno,
            "magma": plt.cm.magma,
            "cividis": plt.cm.cividis,
            "hot": plt.cm.hot,
            "cool": plt.cm.cool,
            "rainbow": plt.cm.rainbow
        }
        
        # Add custom color maps if defined in config
        if "custom_color_maps" in config:
            for name, colors in config["custom_color_maps"].items():
                self.color_maps[name] = self._create_custom_colormap(name, colors)
    
    def _create_custom_colormap(self, name: str, colors: List[str]) -> matplotlib.colors.Colormap:
        """
        Create a custom colormap from a list of colors.
        
        Args:
            name: Name of the colormap
            colors: List of color strings (hex or named colors)
            
        Returns:
            Matplotlib colormap
        """
        try:
            # Convert colors to RGB
            rgb_colors = []
            for color in colors:
                rgb_colors.append(matplotlib.colors.to_rgb(color))
            
            # Create colormap
            n_bins = 256  # Number of bins
            cmap = LinearSegmentedColormap.from_list(name, rgb_colors, N=n_bins)
            
            return cmap
        except Exception as e:
            logger.error(f"Error creating custom colormap: {str(e)}")
            # Return default colormap
            return plt.cm.jet
    
    async def generate_heat_map(
        self,
        results: List[Dict[str, Any]],
        analysis_type: str,
        resolution: Optional[Dict[str, int]] = None,
        color_map: Optional[str] = None,
        parameters: Optional[Dict[str, Any]] = None
    ) -> bytes:
        """
        Generate a heat map visualization from analysis results.
        
        Args:
            results: List of analysis results
            analysis_type: Type of analysis (crowd, vehicle, flow)
            resolution: Optional resolution for the heat map
            color_map: Optional color map name
            parameters: Optional parameters for heat map generation
            
        Returns:
            Heat map image as bytes
        """
        try:
            # Get parameters
            params = parameters or {}
            opacity = params.get("opacity", 0.7)
            interpolation = params.get("interpolation", "bicubic")
            normalize = params.get("normalize", True)
            show_legend = params.get("show_legend", True)
            
            # Get resolution
            if resolution:
                width = resolution.get("width", 640)
                height = resolution.get("height", 480)
            else:
                width = self.config.get("default_width", 640)
                height = self.config.get("default_height", 480)
            
            # Get color map
            cmap_name = color_map or self.config.get("default_color_map", "jet")
            cmap = self.color_maps.get(cmap_name, plt.cm.jet)
            
            # Create empty heat map
            heat_map = np.zeros((height, width), dtype=np.float32)
            
            if analysis_type == "crowd":
                # Generate crowd density heat map
                heat_map = await self._generate_crowd_heat_map(results, width, height)
            elif analysis_type == "vehicle":
                # Generate vehicle density heat map
                heat_map = await self._generate_vehicle_heat_map(results, width, height)
            elif analysis_type == "flow":
                # Generate flow magnitude heat map
                heat_map = await self._generate_flow_heat_map(results, width, height)
            
            # Normalize heat map if requested
            if normalize and np.max(heat_map) > 0:
                heat_map = heat_map / np.max(heat_map)
            
            # Create figure and axis
            fig, ax = plt.subplots(figsize=(width/100, height/100), dpi=100)
            
            # Plot heat map
            im = ax.imshow(heat_map, cmap=cmap, interpolation=interpolation, alpha=opacity)
            
            # Add colorbar if requested
            if show_legend:
                cbar = plt.colorbar(im, ax=ax)
                if analysis_type == "crowd":
                    cbar.set_label('Density (people/m²)')
                elif analysis_type == "vehicle":
                    cbar.set_label('Vehicle Count')
                elif analysis_type == "flow":
                    cbar.set_label('Movement Magnitude')
            
            # Remove axes
            ax.axis('off')
            
            # Tight layout
            plt.tight_layout()
            
            # Save figure to bytes
            buf = io.BytesIO()
            plt.savefig(buf, format='png', bbox_inches='tight', pad_inches=0)
            buf.seek(0)
            
            # Close figure to free memory
            plt.close(fig)
            
            return buf.getvalue()
        except Exception as e:
            logger.error(f"Error generating heat map: {str(e)}")
            raise
    
    async def _generate_crowd_heat_map(
        self,
        results: List[Dict[str, Any]],
        width: int,
        height: int
    ) -> np.ndarray:
        """
        Generate a crowd density heat map.
        
        Args:
            results: List of crowd analysis results
            width: Width of the heat map
            height: Height of the heat map
            
        Returns:
            Heat map as numpy array
        """
        try:
            # Create empty heat map
            heat_map = np.zeros((height, width), dtype=np.float32)
            
            # Accumulate density maps from results
            for result in results:
                if "crowd_result" in result and result["crowd_result"]:
                    crowd_result = result["crowd_result"]
                    
                    if "density_map" in crowd_result and crowd_result["density_map"]:
                        density_map = crowd_result["density_map"]
                        
                        # Extract density map data
                        map_width = density_map["width"]
                        map_height = density_map["height"]
                        map_data = np.array(density_map["data"]).reshape(map_height, map_width)
                        
                        # Resize to heat map dimensions
                        resized_map = cv2.resize(map_data, (width, height))
                        
                        # Add to heat map
                        heat_map += resized_map
            
            # Average heat map if multiple results
            if len(results) > 0:
                heat_map /= len(results)
            
            return heat_map
        except Exception as e:
            logger.error(f"Error generating crowd heat map: {str(e)}")
            # Return empty heat map
            return np.zeros((height, width), dtype=np.float32)
    
    async def _generate_vehicle_heat_map(
        self,
        results: List[Dict[str, Any]],
        width: int,
        height: int
    ) -> np.ndarray:
        """
        Generate a vehicle density heat map.
        
        Args:
            results: List of vehicle analysis results
            width: Width of the heat map
            height: Height of the heat map
            
        Returns:
            Heat map as numpy array
        """
        try:
            # Create empty heat map
            heat_map = np.zeros((height, width), dtype=np.float32)
            
            # Accumulate vehicle detections from results
            for result in results:
                if "vehicle_result" in result and result["vehicle_result"]:
                    vehicle_result = result["vehicle_result"]
                    
                    if "detected_vehicles" in vehicle_result and vehicle_result["detected_vehicles"]:
                        for vehicle in vehicle_result["detected_vehicles"]:
                            bbox = vehicle["bounding_box"]
                            
                            # Calculate center and size
                            center_x = int((bbox["x"] + bbox["width"] / 2) * width)
                            center_y = int((bbox["y"] + bbox["height"] / 2) * height)
                            size_x = max(1, int(bbox["width"] * width / 10))
                            size_y = max(1, int(bbox["height"] * height / 10))
                            
                            # Add Gaussian blob to heat map
                            self._add_gaussian_blob(heat_map, center_x, center_y, size_x, size_y)
            
            # Smooth heat map
            heat_map = cv2.GaussianBlur(heat_map, (15, 15), 0)
            
            return heat_map
        except Exception as e:
            logger.error(f"Error generating vehicle heat map: {str(e)}")
            # Return empty heat map
            return np.zeros((height, width), dtype=np.float32)
    
    async def _generate_flow_heat_map(
        self,
        results: List[Dict[str, Any]],
        width: int,
        height: int
    ) -> np.ndarray:
        """
        Generate a flow magnitude heat map.
        
        Args:
            results: List of flow analysis results
            width: Width of the heat map
            height: Height of the heat map
            
        Returns:
            Heat map as numpy array
        """
        try:
            # Create empty heat map
            heat_map = np.zeros((height, width), dtype=np.float32)
            
            # Accumulate flow vectors from results
            for result in results:
                if "flow_result" in result and result["flow_result"]:
                    flow_result = result["flow_result"]
                    
                    if "flow_vectors" in flow_result and flow_result["flow_vectors"]:
                        for vector in flow_result["flow_vectors"]:
                            # Get vector properties
                            start_x = int(vector["start_x"] * width)
                            start_y = int(vector["start_y"] * height)
                            magnitude = vector["magnitude"]
                            
                            # Add to heat map
                            self._add_gaussian_blob(heat_map, start_x, start_y, 20, 20, magnitude)
            
            # Smooth heat map
            heat_map = cv2.GaussianBlur(heat_map, (15, 15), 0)
            
            return heat_map
        except Exception as e:
            logger.error(f"Error generating flow heat map: {str(e)}")
            # Return empty heat map
            return np.zeros((height, width), dtype=np.float32)
    
    def _add_gaussian_blob(
        self,
        heat_map: np.ndarray,
        center_x: int,
        center_y: int,
        size_x: int,
        size_y: int,
        intensity: float = 1.0
    ):
        """
        Add a Gaussian blob to a heat map.
        
        Args:
            heat_map: Heat map to add blob to
            center_x: X coordinate of blob center
            center_y: Y coordinate of blob center
            size_x: X size of blob
            size_y: Y size of blob
            intensity: Intensity of blob
        """
        # Get heat map dimensions
        height, width = heat_map.shape
        
        # Ensure center is within bounds
        if center_x < 0 or center_x >= width or center_y < 0 or center_y >= height:
            return
        
        # Calculate blob bounds
        x_min = max(0, center_x - size_x * 2)
        x_max = min(width, center_x + size_x * 2)
        y_min = max(0, center_y - size_y * 2)
        y_max = min(height, center_y + size_y * 2)
        
        # Create meshgrid for blob
        x, y = np.meshgrid(np.arange(x_min, x_max), np.arange(y_min, y_max))
        
        # Calculate Gaussian blob
        blob = np.exp(-((x - center_x) ** 2 / (2 * size_x ** 2) + (y - center_y) ** 2 / (2 * size_y ** 2))) * intensity
        
        # Add blob to heat map
        heat_map[y_min:y_max, x_min:x_max] += blob
    
    async def generate_flow_map(
        self,
        results: List[Dict[str, Any]],
        resolution: Optional[Dict[str, int]] = None,
        min_track_length: Optional[int] = None,
        parameters: Optional[Dict[str, Any]] = None
    ) -> bytes:
        """
        Generate a flow map visualization from analysis results.
        
        Args:
            results: List of flow analysis results
            resolution: Optional resolution for the flow map
            min_track_length: Optional minimum track length to include
            parameters: Optional parameters for flow map generation
            
        Returns:
            Flow map image as bytes
        """
        try:
            # Get parameters
            params = parameters or {}
            line_color = params.get("line_color", "rainbow")
            line_width = params.get("line_width", 2)
            line_opacity = params.get("line_opacity", 0.8)
            max_tracks = params.get("max_tracks", 100)
            show_direction = params.get("show_direction", True)
            show_speed = params.get("show_speed", True)
            arrow_scale = params.get("arrow_scale", 1.0)
            
            # Get resolution
            if resolution:
                width = resolution.get("width", 640)
                height = resolution.get("height", 480)
            else:
                width = self.config.get("default_width", 640)
                height = self.config.get("default_height", 480)
            
            # Create empty image
            flow_map = np.zeros((height, width, 3), dtype=np.uint8)
            
            # Get tracks from results
            all_tracks = []
            for result in results:
                if "flow_result" in result and result["flow_result"]:
                    flow_result = result["flow_result"]
                    
                    if "tracks" in flow_result and flow_result["tracks"]:
                        all_tracks.extend(flow_result["tracks"])
            
            # Filter tracks by length if specified
            if min_track_length is not None:
                all_tracks = [track for track in all_tracks if len(track["points"]) >= min_track_length]
            
            # Sort tracks by length (longest first)
            all_tracks.sort(key=lambda x: len(x["points"]), reverse=True)
            
            # Limit number of tracks
            all_tracks = all_tracks[:max_tracks]
            
            # Draw tracks
            for track in all_tracks:
                points = track["points"]
                
                # Need at least 2 points to draw a track
                if len(points) < 2:
                    continue
                
                # Convert points to pixel coordinates
                track_points = []
                for point in points:
                    x = int(point["x"] * width)
                    y = int(point["y"] * height)
                    track_points.append((x, y))
                
                # Draw track
                if line_color == "rainbow":
                    # Use rainbow coloring based on direction
                    dx = points[-1]["x"] - points[0]["x"]
                    dy = points[-1]["y"] - points[0]["y"]
                    angle = np.degrees(np.arctan2(dy, dx)) % 360
                    color_value = angle / 360.0
                    
                    # Get color from HSV (hue based on direction)
                    color = tuple(int(c * 255) for c in matplotlib.colors.hsv_to_rgb([color_value, 1.0, 1.0]))
                else:
                    # Use specified color
                    color = matplotlib.colors.to_rgb(line_color)
                    color = tuple(int(c * 255) for c in color)
                
                # Draw track line
                for i in range(1, len(track_points)):
                    cv2.line(flow_map, track_points[i-1], track_points[i], color, line_width)
                
                # Draw direction arrow if requested
                if show_direction and len(track_points) >= 2:
                    # Get last two points
                    p1 = track_points[-2]
                    p2 = track_points[-1]
                    
                    # Calculate arrow properties
                    dx = p2[0] - p1[0]
                    dy = p2[1] - p1[1]
                    angle = np.arctan2(dy, dx)
                    
                    # Draw arrow
                    arrow_length = 15 * arrow_scale
                    arrow_x = p2[0] - arrow_length * np.cos(angle)
                    arrow_y = p2[1] - arrow_length * np.sin(angle)
                    cv2.arrowedLine(flow_map, (int(arrow_x), int(arrow_y)), p2, color, line_width + 1)
                
                # Draw speed indicator if requested
                if show_speed and "average_speed" in track:
                    # Get speed
                    speed = track["average_speed"]
                    
                    # Draw speed text at end of track
                    end_point = track_points[-1]
                    cv2.putText(
                        flow_map,
                        f"{speed:.1f}",
                        (end_point[0] + 5, end_point[1] + 5),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        color,
                        1,
                        cv2.LINE_AA
                    )
            
            # Create figure and axis
            fig, ax = plt.subplots(figsize=(width/100, height/100), dpi=100)
            
            # Convert BGR to RGB
            flow_map_rgb = cv2.cvtColor(flow_map, cv2.COLOR_BGR2RGB)
            
            # Plot flow map
            ax.imshow(flow_map_rgb)
            
            # Remove axes
            ax.axis('off')
            
            # Tight layout
            plt.tight_layout()
            
            # Save figure to bytes
            buf = io.BytesIO()
            plt.savefig(buf, format='png', bbox_inches='tight', pad_inches=0)
            buf.seek(0)
            
            # Close figure to free memory
            plt.close(fig)
            
            return buf.getvalue()
        except Exception as e:
            logger.error(f"Error generating flow map: {str(e)}")
            raise
    
    async def get_available_color_maps(self) -> List[str]:
        """
        Get all available color maps.
        
        Returns:
            List of color map names
        """
        return list(self.color_maps.keys())
