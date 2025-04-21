"""
Flow Analyzer for the Vision System.

This module provides object tracking and flow analysis functionality.
"""

import logging
import time
import numpy as np
import cv2
import torch
from typing import Dict, List, Any, Optional, Tuple
import uuid
from datetime import datetime
from collections import defaultdict

# Setup logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class FlowAnalyzer:
    """Analyzer for object tracking and flow analysis."""
    
    def __init__(self, config: Dict[str, Any]):
        """Initialize the flow analyzer."""
        self.config = config
        self.models = {}
        self.device = torch.device("cuda" if torch.cuda.is_available() and config.get("enable_gpu", True) else "cpu")
        self.tracks = defaultdict(list)  # Dictionary to store tracks: {track_id: [points]}
        self.last_frame_objects = {}  # Store objects from last frame for tracking
        self.next_track_id = 0  # Counter for generating track IDs
        
        # Initialize models
        self._initialize_models()
    
    def _initialize_models(self):
        """Initialize tracking models."""
        try:
            logger.info("Initializing tracking models")
            
            # Load DeepSORT model for tracking
            if self.config.get("use_deepsort", True):
                self._load_deepsort_model()
            
            logger.info(f"Tracking models initialized on {self.device}")
        except Exception as e:
            logger.error(f"Error initializing tracking models: {str(e)}")
            raise
    
    def _load_deepsort_model(self):
        """Load DeepSORT model for object tracking."""
        try:
            # In a real implementation, you would load the actual DeepSORT model
            # For example:
            # from deep_sort.deep_sort import DeepSort
            # model_path = self.config.get("deepsort_model_path", "models/tracking/deepsort.pth")
            # self.models["deepsort"] = DeepSort(model_path, max_dist=0.2, min_confidence=0.3)
            
            # For now, we'll just simulate model loading
            self.models["deepsort"] = {
                "name": "DeepSORT",
                "type": "object_tracking",
                "loaded": True,
                "device": self.device
            }
            
            logger.info("DeepSORT model loaded successfully")
        except Exception as e:
            logger.error(f"Error loading DeepSORT model: {str(e)}")
            raise
    
    async def analyze_sequence(
        self,
        frames: List[np.ndarray],
        detections: List[List[Dict[str, Any]]],
        region_of_interest: Optional[Dict[str, Any]] = None,
        parameters: Optional[Dict[str, Any]] = None
    ) -> Dict[str, Any]:
        """
        Analyze a sequence of frames for object tracking and flow analysis.
        
        Args:
            frames: List of frames to analyze
            detections: List of detections for each frame
            region_of_interest: Optional region of interest to analyze
            parameters: Optional parameters for analysis
            
        Returns:
            Dictionary containing analysis results
        """
        try:
            start_time = time.time()
            
            # Get parameters
            params = parameters or {}
            min_track_length = params.get("min_track_length", 5)
            max_age = params.get("max_age", 30)  # Maximum frames to keep a track alive without detection
            
            # Reset tracks for new sequence
            self.tracks = defaultdict(list)
            self.last_frame_objects = {}
            self.next_track_id = 0
            
            # Process each frame
            for i, (frame, frame_detections) in enumerate(zip(frames, detections)):
                # Apply region of interest if provided
                if region_of_interest:
                    # Apply ROI to detections instead of frame
                    frame_detections = self._filter_detections_by_roi(
                        frame_detections,
                        region_of_interest,
                        frame.shape[1],  # width
                        frame.shape[0]   # height
                    )
                
                # Update tracks with new detections
                self._update_tracks(frame_detections, frame.shape[1], frame.shape[0], i, max_age)
            
            # Filter tracks by length
            valid_tracks = self._filter_tracks(min_track_length)
            
            # Calculate flow vectors
            flow_vectors = self._calculate_flow_vectors(valid_tracks, frames[0].shape[1], frames[0].shape[0])
            
            # Calculate flow statistics
            flow_stats = self._calculate_flow_statistics(valid_tracks, flow_vectors)
            
            # Calculate processing time
            processing_time = (time.time() - start_time) * 1000  # Convert to milliseconds
            
            # Prepare results
            results = {
                "timestamp": datetime.now(),
                "start_frame_id": 0,
                "end_frame_id": len(frames) - 1,
                "tracks": valid_tracks,
                "flow_vectors": flow_vectors,
                "dominant_direction": flow_stats.get("dominant_direction"),
                "average_speed": flow_stats.get("average_speed"),
                "congestion_level": flow_stats.get("congestion_level"),
                "processing_time": processing_time
            }
            
            return results
        except Exception as e:
            logger.error(f"Error analyzing sequence for flow: {str(e)}")
            raise
    
    def _update_tracks(
        self,
        detections: List[Dict[str, Any]],
        frame_width: int,
        frame_height: int,
        frame_index: int,
        max_age: int
    ):
        """
        Update tracks with new detections.
        
        Args:
            detections: List of detections in the current frame
            frame_width: Width of the frame
            frame_height: Height of the frame
            frame_index: Index of the current frame
            max_age: Maximum frames to keep a track alive without detection
        """
        try:
            # In a real implementation, you would use DeepSORT for tracking
            # For example:
            # # Convert detections to DeepSORT format
            # deepsort_detections = []
            # for det in detections:
            #     bbox = det["bounding_box"]
            #     x1 = bbox["x"] * frame_width
            #     y1 = bbox["y"] * frame_height
            #     x2 = (bbox["x"] + bbox["width"]) * frame_width
            #     y2 = (bbox["y"] + bbox["height"]) * frame_height
            #     confidence = det["confidence"]
            #     class_name = det["type"]
            #     
            #     deepsort_detections.append([x1, y1, x2, y2, confidence, class_name])
            # 
            # # Update DeepSORT tracker
            # tracks = self.models["deepsort"].update(deepsort_detections, frame)
            # 
            # # Update our tracks
            # for track in tracks:
            #     track_id, x1, y1, x2, y2, class_name = track
            #     
            #     # Calculate center point
            #     center_x = (x1 + x2) / 2 / frame_width
            #     center_y = (y1 + y2) / 2 / frame_height
            #     
            #     # Add point to track
            #     self.tracks[track_id].append({
            #         "x": center_x,
            #         "y": center_y,
            #         "frame": frame_index,
            #         "timestamp": datetime.now()
            #     })
            
            # For now, we'll just simulate simple tracking
            # This is a very basic tracking algorithm based on IoU
            
            # Convert current detections to format for matching
            current_objects = {}
            for det in detections:
                obj_id = det["id"]
                bbox = det["bounding_box"]
                obj_type = det["type"]
                
                current_objects[obj_id] = {
                    "bbox": bbox,
                    "type": obj_type,
                    "matched": False
                }
            
            # Match with existing tracks
            for last_id, last_obj in self.last_frame_objects.items():
                best_match_id = None
                best_match_iou = 0.3  # Minimum IoU threshold
                
                for curr_id, curr_obj in current_objects.items():
                    if curr_obj["matched"]:
                        continue
                    
                    # Only match objects of the same type
                    if curr_obj["type"] != last_obj["type"]:
                        continue
                    
                    # Calculate IoU
                    iou = self._calculate_iou(last_obj["bbox"], curr_obj["bbox"])
                    
                    if iou > best_match_iou:
                        best_match_id = curr_id
                        best_match_iou = iou
                
                if best_match_id:
                    # Match found, update track
                    track_id = last_obj["track_id"]
                    bbox = current_objects[best_match_id]["bbox"]
                    
                    # Calculate center point
                    center_x = bbox["x"] + bbox["width"] / 2
                    center_y = bbox["y"] + bbox["height"] / 2
                    
                    # Add point to track
                    self.tracks[track_id].append({
                        "x": center_x,
                        "y": center_y,
                        "frame": frame_index,
                        "timestamp": datetime.now()
                    })
                    
                    # Update last frame object
                    current_objects[best_match_id]["track_id"] = track_id
                    current_objects[best_match_id]["matched"] = True
                    
                    # Update last seen
                    last_obj["last_seen"] = frame_index
                else:
                    # No match found, increment age
                    last_obj["age"] += 1
                    
                    # If age exceeds max_age, remove track
                    if last_obj["age"] > max_age:
                        continue
                    
                    # Otherwise, keep track and add to current objects
                    track_id = last_obj["track_id"]
                    bbox = last_obj["bbox"]
                    
                    # Calculate center point
                    center_x = bbox["x"] + bbox["width"] / 2
                    center_y = bbox["y"] + bbox["height"] / 2
                    
                    # Add point to track (with same position as last seen)
                    self.tracks[track_id].append({
                        "x": center_x,
                        "y": center_y,
                        "frame": frame_index,
                        "timestamp": datetime.now()
                    })
                    
                    # Add to current objects
                    obj_id = f"ghost_{track_id}_{frame_index}"
                    current_objects[obj_id] = {
                        "bbox": bbox,
                        "type": last_obj["type"],
                        "track_id": track_id,
                        "matched": True,
                        "age": last_obj["age"],
                        "last_seen": last_obj["last_seen"]
                    }
            
            # Create new tracks for unmatched detections
            for curr_id, curr_obj in current_objects.items():
                if not curr_obj.get("matched", False):
                    # Create new track
                    track_id = f"track_{self.next_track_id}"
                    self.next_track_id += 1
                    
                    bbox = curr_obj["bbox"]
                    
                    # Calculate center point
                    center_x = bbox["x"] + bbox["width"] / 2
                    center_y = bbox["y"] + bbox["height"] / 2
                    
                    # Add point to track
                    self.tracks[track_id].append({
                        "x": center_x,
                        "y": center_y,
                        "frame": frame_index,
                        "timestamp": datetime.now()
                    })
                    
                    # Update current object
                    curr_obj["track_id"] = track_id
                    curr_obj["age"] = 0
                    curr_obj["last_seen"] = frame_index
            
            # Update last frame objects
            self.last_frame_objects = current_objects
        except Exception as e:
            logger.error(f"Error updating tracks: {str(e)}")
            raise
    
    def _calculate_iou(self, bbox1: Dict[str, float], bbox2: Dict[str, float]) -> float:
        """
        Calculate Intersection over Union (IoU) between two bounding boxes.
        
        Args:
            bbox1: First bounding box (x, y, width, height)
            bbox2: Second bounding box (x, y, width, height)
            
        Returns:
            IoU value (0-1)
        """
        # Convert to x1, y1, x2, y2 format
        x1_1, y1_1 = bbox1["x"], bbox1["y"]
        x2_1, y2_1 = x1_1 + bbox1["width"], y1_1 + bbox1["height"]
        
        x1_2, y1_2 = bbox2["x"], bbox2["y"]
        x2_2, y2_2 = x1_2 + bbox2["width"], y1_2 + bbox2["height"]
        
        # Calculate intersection area
        x1_i = max(x1_1, x1_2)
        y1_i = max(y1_1, y1_2)
        x2_i = min(x2_1, x2_2)
        y2_i = min(y2_1, y2_2)
        
        if x2_i < x1_i or y2_i < y1_i:
            return 0.0  # No intersection
        
        intersection_area = (x2_i - x1_i) * (y2_i - y1_i)
        
        # Calculate union area
        bbox1_area = bbox1["width"] * bbox1["height"]
        bbox2_area = bbox2["width"] * bbox2["height"]
        union_area = bbox1_area + bbox2_area - intersection_area
        
        # Calculate IoU
        iou = intersection_area / union_area if union_area > 0 else 0.0
        
        return iou
    
    def _filter_tracks(self, min_track_length: int) -> List[Dict[str, Any]]:
        """
        Filter tracks by length and convert to output format.
        
        Args:
            min_track_length: Minimum number of points in a track
            
        Returns:
            List of valid tracks in output format
        """
        valid_tracks = []
        
        for track_id, points in self.tracks.items():
            if len(points) >= min_track_length:
                # Calculate track properties
                start_time = points[0]["timestamp"]
                end_time = points[-1]["timestamp"]
                
                # Calculate track length (Euclidean distance between points)
                length = 0.0
                for i in range(1, len(points)):
                    dx = points[i]["x"] - points[i-1]["x"]
                    dy = points[i]["y"] - points[i-1]["y"]
                    length += np.sqrt(dx*dx + dy*dy)
                
                # Calculate average speed (length / time)
                time_diff = (end_time - start_time).total_seconds()
                average_speed = length / time_diff if time_diff > 0 else 0.0
                
                # Create track object
                track = {
                    "id": track_id,
                    "object_type": self.last_frame_objects.get(track_id, {}).get("type", "unknown"),
                    "points": points,
                    "start_time": start_time,
                    "end_time": end_time,
                    "length": float(length),
                    "average_speed": float(average_speed),
                    "is_complete": points[-1]["frame"] == max(p["frame"] for p in points)
                }
                
                valid_tracks.append(track)
        
        return valid_tracks
    
    def _calculate_flow_vectors(
        self,
        tracks: List[Dict[str, Any]],
        frame_width: int,
        frame_height: int
    ) -> List[Dict[str, Any]]:
        """
        Calculate flow vectors from tracks.
        
        Args:
            tracks: List of tracks
            frame_width: Width of the frame
            frame_height: Height of the frame
            
        Returns:
            List of flow vectors
        """
        # Create grid for flow vectors
        grid_size = 10  # 10x10 grid
        cell_width = 1.0 / grid_size
        cell_height = 1.0 / grid_size
        
        # Initialize grid
        grid = {}
        for i in range(grid_size):
            for j in range(grid_size):
                cell_key = f"{i}_{j}"
                grid[cell_key] = {
                    "dx_sum": 0.0,
                    "dy_sum": 0.0,
                    "count": 0
                }
        
        # Accumulate flow vectors in grid
        for track in tracks:
            points = track["points"]
            
            # Need at least 2 points to calculate flow
            if len(points) < 2:
                continue
            
            # Calculate flow for each segment of the track
            for i in range(1, len(points)):
                # Get points
                p1 = points[i-1]
                p2 = points[i]
                
                # Calculate displacement
                dx = p2["x"] - p1["x"]
                dy = p2["y"] - p1["y"]
                
                # Skip if displacement is too small
                if abs(dx) < 0.001 and abs(dy) < 0.001:
                    continue
                
                # Calculate grid cell for start point
                grid_x = min(grid_size - 1, int(p1["x"] / cell_width))
                grid_y = min(grid_size - 1, int(p1["y"] / cell_height))
                cell_key = f"{grid_x}_{grid_y}"
                
                # Accumulate flow vector
                grid[cell_key]["dx_sum"] += dx
                grid[cell_key]["dy_sum"] += dy
                grid[cell_key]["count"] += 1
        
        # Create flow vectors from grid
        flow_vectors = []
        for i in range(grid_size):
            for j in range(grid_size):
                cell_key = f"{i}_{j}"
                cell = grid[cell_key]
                
                if cell["count"] > 0:
                    # Calculate average flow vector
                    dx_avg = cell["dx_sum"] / cell["count"]
                    dy_avg = cell["dy_sum"] / cell["count"]
                    
                    # Calculate magnitude and direction
                    magnitude = np.sqrt(dx_avg*dx_avg + dy_avg*dy_avg)
                    direction = np.degrees(np.arctan2(dy_avg, dx_avg))
                    
                    # Calculate center of cell
                    center_x = (i + 0.5) * cell_width
                    center_y = (j + 0.5) * cell_height
                    
                    # Create flow vector
                    flow_vector = {
                        "start_x": center_x,
                        "start_y": center_y,
                        "end_x": center_x + dx_avg * 0.5,  # Scale for visualization
                        "end_y": center_y + dy_avg * 0.5,  # Scale for visualization
                        "magnitude": float(magnitude),
                        "direction": float(direction),
                        "count": cell["count"]
                    }
                    
                    flow_vectors.append(flow_vector)
        
        return flow_vectors
    
    def _calculate_flow_statistics(
        self,
        tracks: List[Dict[str, Any]],
        flow_vectors: List[Dict[str, Any]]
    ) -> Dict[str, Any]:
        """
        Calculate statistics from flow vectors.
        
        Args:
            tracks: List of tracks
            flow_vectors: List of flow vectors
            
        Returns:
            Dictionary containing flow statistics
        """
        # Initialize statistics
        stats = {
            "dominant_direction": None,
            "average_speed": None,
            "congestion_level": None
        }
        
        # Calculate dominant direction
        if flow_vectors:
            # Weight directions by magnitude and count
            weighted_directions = []
            for vector in flow_vectors:
                direction = vector["direction"]
                weight = vector["magnitude"] * vector["count"]
                weighted_directions.extend([direction] * int(weight * 10))
            
            if weighted_directions:
                # Convert to radians for circular mean
                radians = np.radians(weighted_directions)
                
                # Calculate circular mean
                sin_sum = np.sum(np.sin(radians))
                cos_sum = np.sum(np.cos(radians))
                
                # Convert back to degrees
                dominant_direction = np.degrees(np.arctan2(sin_sum, cos_sum))
                
                # Ensure positive angle
                if dominant_direction < 0:
                    dominant_direction += 360
                
                stats["dominant_direction"] = float(dominant_direction)
        
        # Calculate average speed
        if tracks:
            speeds = [track["average_speed"] for track in tracks]
            stats["average_speed"] = float(np.mean(speeds))
        
        # Calculate congestion level
        if tracks:
            # Count tracks per frame
            frame_counts = defaultdict(int)
            for track in tracks:
                for point in track["points"]:
                    frame_counts[point["frame"]] += 1
            
            # Calculate average tracks per frame
            if frame_counts:
                avg_tracks_per_frame = np.mean(list(frame_counts.values()))
                
                # Normalize to 0-1 range (assuming max 50 tracks is congested)
                congestion_level = min(1.0, avg_tracks_per_frame / 50.0)
                
                stats["congestion_level"] = float(congestion_level)
        
        return stats
    
    def _filter_detections_by_roi(
        self,
        detections: List[Dict[str, Any]],
        region_of_interest: Dict[str, Any],
        frame_width: int,
        frame_height: int
    ) -> List[Dict[str, Any]]:
        """
        Filter detections by region of interest.
        
        Args:
            detections: List of detections
            region_of_interest: ROI definition
            frame_width: Width of the frame
            frame_height: Height of the frame
            
        Returns:
            Filtered list of detections
        """
        filtered_detections = []
        
        roi_type = region_of_interest.get("type", "polygon")
        
        if roi_type == "polygon":
            points = region_of_interest.get("points", [])
            if not points:
                return detections
            
            # Convert normalized points to pixel coordinates
            points_px = []
            for point in points:
                x = int(point["x"] * frame_width)
                y = int(point["y"] * frame_height)
                points_px.append([x, y])
            
            # Create polygon
            polygon = np.array(points_px, dtype=np.int32)
            
            # Filter detections
            for det in detections:
                bbox = det["bounding_box"]
                
                # Calculate center point
                center_x = int((bbox["x"] + bbox["width"] / 2) * frame_width)
                center_y = int((bbox["y"] + bbox["height"] / 2) * frame_height)
                
                # Check if center point is inside polygon
                if cv2.pointPolygonTest(polygon, (center_x, center_y), False) >= 0:
                    filtered_detections.append(det)
        
        elif roi_type == "rectangle":
            x = region_of_interest.get("x", 0)
            y = region_of_interest.get("y", 0)
            w = region_of_interest.get("width", 1)
            h = region_of_interest.get("height", 1)
            
            # Filter detections
            for det in detections:
                bbox = det["bounding_box"]
                
                # Calculate center point
                center_x = bbox["x"] + bbox["width"] / 2
                center_y = bbox["y"] + bbox["height"] / 2
                
                # Check if center point is inside rectangle
                if (x <= center_x <= x + w) and (y <= center_y <= y + h):
                    filtered_detections.append(det)
        
        elif roi_type == "circle":
            center_x = region_of_interest.get("center_x", 0.5)
            center_y = region_of_interest.get("center_y", 0.5)
            radius = region_of_interest.get("radius", 0.5)
            
            # Filter detections
            for det in detections:
                bbox = det["bounding_box"]
                
                # Calculate center point
                det_center_x = bbox["x"] + bbox["width"] / 2
                det_center_y = bbox["y"] + bbox["height"] / 2
                
                # Calculate distance from circle center
                dx = det_center_x - center_x
                dy = det_center_y - center_y
                distance = np.sqrt(dx*dx + dy*dy)
                
                # Check if center point is inside circle
                if distance <= radius:
                    filtered_detections.append(det)
        
        else:
            # Unknown ROI type, return all detections
            return detections
        
        return filtered_detections
