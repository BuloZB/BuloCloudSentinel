"""
Path planning module for Bulo.Cloud Sentinel.

This module provides algorithms for generating waypoints for different mission types.
"""

import math
import numpy as np
from typing import List, Dict, Any, Optional, Tuple
from shapely.geometry import Polygon, LineString, Point
from shapely.ops import triangulate
import asyncio

from backend.mission_planning.models import (
    Position, Waypoint, MappingSettings, OrbitSettings, FacadeSettings,
    CameraAction, CameraActionDetails, ActionTrigger
)


class PathPlanner:
    """
    Path planner for generating waypoints for different mission types.
    """
    
    async def generate_mapping_waypoints(self, 
                                        boundary_points: List[Position], 
                                        settings: MappingSettings) -> List[Waypoint]:
        """
        Generate waypoints for a mapping mission.
        
        Args:
            boundary_points: List of positions defining the mapping area
            settings: Mapping mission settings
            
        Returns:
            List of waypoints for the mapping mission
        """
        if settings.area_mode == "polygon":
            return await self._generate_polygon_mapping_waypoints(boundary_points, settings)
        elif settings.area_mode == "rectangle":
            return await self._generate_rectangle_mapping_waypoints(boundary_points, settings)
        elif settings.area_mode == "corridor":
            return await self._generate_corridor_mapping_waypoints(boundary_points, settings)
        else:
            raise ValueError(f"Unsupported area mode: {settings.area_mode}")
    
    async def generate_orbit_waypoints(self, settings: OrbitSettings) -> List[Waypoint]:
        """
        Generate waypoints for an orbit mission.
        
        Args:
            settings: Orbit mission settings
            
        Returns:
            List of waypoints for the orbit mission
        """
        waypoints = []
        
        # Calculate number of waypoints based on radius and desired spacing
        # A good rule of thumb is to have a waypoint every 5-10 degrees
        angle_step = 10  # degrees
        num_waypoints = int(abs(settings.end_angle - settings.start_angle) / angle_step)
        
        # Ensure at least 8 waypoints for a full circle
        num_waypoints = max(num_waypoints, 8)
        
        # Calculate angle step
        angle_step = abs(settings.end_angle - settings.start_angle) / num_waypoints
        
        # Determine direction
        direction = 1 if settings.rotation_direction == "clockwise" else -1
        
        # Generate waypoints
        for i in range(num_waypoints + 1):
            angle_rad = math.radians(settings.start_angle + i * angle_step * direction)
            
            # Calculate position
            x = settings.center.longitude + settings.radius * math.cos(angle_rad) / 111320  # approximate conversion to degrees
            y = settings.center.latitude + settings.radius * math.sin(angle_rad) / 110540   # approximate conversion to degrees
            
            position = Position(
                latitude=y,
                longitude=x,
                altitude=settings.altitude
            )
            
            # Calculate heading based on heading mode
            heading = None
            if settings.heading_mode == "poi":
                # Point towards center
                dx = settings.center.longitude - x
                dy = settings.center.latitude - y
                heading = math.degrees(math.atan2(dy, dx))
            elif settings.heading_mode == "tangent":
                # Tangent to circle
                heading = math.degrees(angle_rad + math.pi/2 * direction)
            
            # Create waypoint
            waypoint = Waypoint(
                position=position,
                heading=heading,
                speed=settings.speed,
                order=i
            )
            
            # Add camera actions if needed
            if settings.capture_mode == "continuous" and i == 0:
                # Start recording at first waypoint
                waypoint.actions.append(CameraActionDetails(
                    action=CameraAction.START_RECORDING,
                    trigger=ActionTrigger.ON_ARRIVAL
                ))
            elif settings.capture_mode == "interval" and settings.photo_interval:
                # Take photo at each waypoint
                waypoint.actions.append(CameraActionDetails(
                    action=CameraAction.TAKE_PHOTO,
                    trigger=ActionTrigger.ON_ARRIVAL
                ))
            
            # Add gimbal control
            if settings.gimbal_mode == "follow":
                # Point camera at center
                dx = settings.center.longitude - x
                dy = settings.center.latitude - y
                distance = math.sqrt(dx**2 + dy**2)
                height_diff = settings.center.altitude - settings.altitude if settings.center.altitude else 0
                pitch = math.degrees(math.atan2(height_diff, distance * 111000))  # approximate conversion to meters
                
                waypoint.actions.append(CameraActionDetails(
                    action=CameraAction.TILT_CAMERA,
                    trigger=ActionTrigger.ON_ARRIVAL,
                    gimbal_pitch=pitch
                ))
            elif settings.gimbal_pitch is not None:
                waypoint.actions.append(CameraActionDetails(
                    action=CameraAction.TILT_CAMERA,
                    trigger=ActionTrigger.ON_ARRIVAL,
                    gimbal_pitch=settings.gimbal_pitch
                ))
            
            waypoints.append(waypoint)
        
        # Add stop recording action to last waypoint if needed
        if settings.capture_mode == "continuous":
            waypoints[-1].actions.append(CameraActionDetails(
                action=CameraAction.STOP_RECORDING,
                trigger=ActionTrigger.ON_ARRIVAL
            ))
        
        return waypoints
    
    async def generate_facade_waypoints(self, settings: FacadeSettings) -> List[Waypoint]:
        """
        Generate waypoints for a facade mission.
        
        Args:
            settings: Facade mission settings
            
        Returns:
            List of waypoints for the facade mission
        """
        waypoints = []
        
        # Calculate direction vector
        dx = settings.end_position.longitude - settings.start_position.longitude
        dy = settings.end_position.latitude - settings.start_position.latitude
        facade_length = math.sqrt(dx**2 + dy**2) * 111000  # approximate conversion to meters
        
        # Normalize direction vector
        length = math.sqrt(dx**2 + dy**2)
        if length > 0:
            dx /= length
            dy /= length
        
        # Calculate perpendicular vector (90 degrees counterclockwise)
        perpx = -dy
        perpy = dx
        
        # Calculate number of photos based on overlap or distance
        num_photos = 0
        if settings.capture_mode == "overlap" and settings.overlap:
            # Calculate field of view
            # This is a simplification - in reality, you'd need camera specs
            fov_h = 60  # degrees
            distance = settings.distance_from_facade
            photo_width = 2 * distance * math.tan(math.radians(fov_h/2))
            
            # Calculate overlap in meters
            overlap_meters = photo_width * settings.overlap / 100
            
            # Calculate step size
            step_size = photo_width - overlap_meters
            
            # Calculate number of photos
            num_photos = max(2, int(facade_length / step_size) + 1)
        elif settings.capture_mode == "distance" and settings.capture_distance:
            # Calculate number of photos based on distance
            num_photos = max(2, int(facade_length / settings.capture_distance) + 1)
        else:
            # Default to at least start and end
            num_photos = 2
        
        # Generate waypoints for each layer
        for layer in range(settings.layers):
            # Calculate layer height
            layer_height = settings.layer_height or (settings.height / settings.layers)
            height = layer_height * layer
            
            # Generate waypoints for this layer
            for i in range(num_photos):
                # Calculate position along facade
                t = i / (num_photos - 1) if num_photos > 1 else 0
                
                # Alternate direction for even layers (zigzag pattern)
                if layer % 2 == 1:
                    t = 1 - t
                
                # Calculate position
                x = settings.start_position.longitude + t * dx
                y = settings.start_position.latitude + t * dy
                
                # Add perpendicular offset for distance from facade
                x += perpx * settings.distance_from_facade / 111320  # approximate conversion to degrees
                y += perpy * settings.distance_from_facade / 110540  # approximate conversion to degrees
                
                position = Position(
                    latitude=y,
                    longitude=x,
                    altitude=settings.start_position.altitude + height if settings.start_position.altitude else height
                )
                
                # Calculate heading (towards facade)
                heading = math.degrees(math.atan2(-perpy, -perpx))
                
                # Create waypoint
                waypoint = Waypoint(
                    position=position,
                    heading=heading,
                    speed=settings.flight_speed,
                    order=layer * num_photos + i
                )
                
                # Add camera actions
                waypoint.actions.append(CameraActionDetails(
                    action=CameraAction.TAKE_PHOTO,
                    trigger=ActionTrigger.ON_ARRIVAL
                ))
                
                if settings.gimbal_pitch is not None:
                    waypoint.actions.append(CameraActionDetails(
                        action=CameraAction.TILT_CAMERA,
                        trigger=ActionTrigger.ON_ARRIVAL,
                        gimbal_pitch=settings.gimbal_pitch
                    ))
                
                waypoints.append(waypoint)
        
        return waypoints
    
    async def _generate_polygon_mapping_waypoints(self, 
                                                boundary_points: List[Position], 
                                                settings: MappingSettings) -> List[Waypoint]:
        """
        Generate waypoints for a polygon mapping mission.
        
        Args:
            boundary_points: List of positions defining the polygon
            settings: Mapping mission settings
            
        Returns:
            List of waypoints for the mapping mission
        """
        # Convert boundary points to shapely polygon
        coords = [(p.longitude, p.latitude) for p in boundary_points]
        polygon = Polygon(coords)
        
        # Get bounding box
        minx, miny, maxx, maxy = polygon.bounds
        
        # Calculate grid spacing based on overlap
        # This is a simplification - in reality, you'd need camera specs
        altitude = settings.flight_altitude
        fov_h = 60  # degrees
        fov_v = 40  # degrees
        
        # Calculate ground sampling distance
        gsd_x = 2 * altitude * math.tan(math.radians(fov_h/2))
        gsd_y = 2 * altitude * math.tan(math.radians(fov_v/2))
        
        # Calculate step size based on overlap
        step_x = gsd_x * (1 - settings.side_overlap / 100)
        step_y = gsd_y * (1 - settings.front_overlap / 100)
        
        # Convert to degrees
        step_x_deg = step_x / 111320  # approximate conversion to degrees
        step_y_deg = step_y / 110540  # approximate conversion to degrees
        
        # Calculate grid angle
        angle_rad = math.radians(settings.grid_angle)
        
        # Generate grid lines
        lines = []
        
        # Calculate rotated bounding box
        width = maxx - minx
        height = maxy - miny
        center_x = (minx + maxx) / 2
        center_y = (miny + maxy) / 2
        
        # Calculate diagonal length of bounding box
        diagonal = math.sqrt(width**2 + height**2)
        
        # Generate parallel lines covering the rotated bounding box
        current_offset = -diagonal / 2
        while current_offset <= diagonal / 2:
            # Calculate line endpoints
            x1 = center_x - diagonal/2 * math.cos(angle_rad + math.pi/2)
            y1 = center_y - diagonal/2 * math.sin(angle_rad + math.pi/2)
            x2 = center_x + diagonal/2 * math.cos(angle_rad + math.pi/2)
            y2 = center_y + diagonal/2 * math.sin(angle_rad + math.pi/2)
            
            # Offset line
            offset_x = current_offset * math.cos(angle_rad)
            offset_y = current_offset * math.sin(angle_rad)
            
            x1 += offset_x
            y1 += offset_y
            x2 += offset_x
            y2 += offset_y
            
            # Create line
            line = LineString([(x1, y1), (x2, y2)])
            
            # Clip line to polygon
            if line.intersects(polygon):
                clipped_line = line.intersection(polygon)
                if not clipped_line.is_empty:
                    lines.append(clipped_line)
            
            # Increment offset
            current_offset += step_x_deg
        
        # Generate crosshatch if needed
        if settings.use_crosshatch:
            crosshatch_angle_rad = math.radians(settings.grid_angle + settings.crosshatch_angle)
            
            # Generate perpendicular lines
            current_offset = -diagonal / 2
            while current_offset <= diagonal / 2:
                # Calculate line endpoints
                x1 = center_x - diagonal/2 * math.cos(crosshatch_angle_rad + math.pi/2)
                y1 = center_y - diagonal/2 * math.sin(crosshatch_angle_rad + math.pi/2)
                x2 = center_x + diagonal/2 * math.cos(crosshatch_angle_rad + math.pi/2)
                y2 = center_y + diagonal/2 * math.sin(crosshatch_angle_rad + math.pi/2)
                
                # Offset line
                offset_x = current_offset * math.cos(crosshatch_angle_rad)
                offset_y = current_offset * math.sin(crosshatch_angle_rad)
                
                x1 += offset_x
                y1 += offset_y
                x2 += offset_x
                y2 += offset_y
                
                # Create line
                line = LineString([(x1, y1), (x2, y2)])
                
                # Clip line to polygon
                if line.intersects(polygon):
                    clipped_line = line.intersection(polygon)
                    if not clipped_line.is_empty:
                        lines.append(clipped_line)
                
                # Increment offset
                current_offset += step_y_deg
        
        # Convert lines to waypoints
        waypoints = []
        order = 0
        
        for i, line in enumerate(lines):
            # Extract coordinates
            if isinstance(line, LineString):
                coords = list(line.coords)
            else:
                # Handle MultiLineString or other geometries
                coords = []
                for geom in line.geoms:
                    coords.extend(list(geom.coords))
            
            # Reverse direction for every other line (zigzag pattern)
            if i % 2 == 1:
                coords = coords[::-1]
            
            # Create waypoints
            for j, (x, y) in enumerate(coords):
                position = Position(
                    latitude=y,
                    longitude=x,
                    altitude=settings.flight_altitude
                )
                
                waypoint = Waypoint(
                    position=position,
                    speed=settings.flight_speed,
                    order=order
                )
                
                # Add camera actions
                if settings.camera_trigger == "auto":
                    waypoint.actions.append(CameraActionDetails(
                        action=CameraAction.TAKE_PHOTO,
                        trigger=ActionTrigger.ON_ARRIVAL
                    ))
                
                if settings.gimbal_pitch is not None:
                    waypoint.actions.append(CameraActionDetails(
                        action=CameraAction.TILT_CAMERA,
                        trigger=ActionTrigger.ON_ARRIVAL,
                        gimbal_pitch=settings.gimbal_pitch
                    ))
                
                waypoints.append(waypoint)
                order += 1
        
        return waypoints
    
    async def _generate_rectangle_mapping_waypoints(self, 
                                                  boundary_points: List[Position], 
                                                  settings: MappingSettings) -> List[Waypoint]:
        """
        Generate waypoints for a rectangle mapping mission.
        
        Args:
            boundary_points: List of positions defining the rectangle (should be 4 points)
            settings: Mapping mission settings
            
        Returns:
            List of waypoints for the mapping mission
        """
        # Ensure we have exactly 4 points
        if len(boundary_points) != 4:
            raise ValueError("Rectangle mapping requires exactly 4 boundary points")
        
        # Convert to polygon and use polygon mapping
        return await self._generate_polygon_mapping_waypoints(boundary_points, settings)
    
    async def _generate_corridor_mapping_waypoints(self, 
                                                 boundary_points: List[Position], 
                                                 settings: MappingSettings) -> List[Waypoint]:
        """
        Generate waypoints for a corridor mapping mission.
        
        Args:
            boundary_points: List of positions defining the corridor centerline
            settings: Mapping mission settings
            
        Returns:
            List of waypoints for the mapping mission
        """
        waypoints = []
        
        # Create centerline
        centerline = LineString([(p.longitude, p.latitude) for p in boundary_points])
        
        # Calculate corridor width based on altitude and camera specs
        altitude = settings.flight_altitude
        fov_h = 60  # degrees
        
        # Calculate ground sampling distance
        gsd_x = 2 * altitude * math.tan(math.radians(fov_h/2))
        
        # Calculate number of parallel lines based on corridor width and overlap
        corridor_width = settings.margin * 2 if settings.margin else gsd_x
        step_size = gsd_x * (1 - settings.side_overlap / 100)
        
        num_lines = max(1, int(corridor_width / step_size))
        
        # Generate parallel lines
        for i in range(num_lines):
            # Calculate offset
            offset = (i - (num_lines - 1) / 2) * step_size
            
            # Create offset line
            if offset == 0:
                line = centerline
            else:
                line = centerline.parallel_offset(offset / 111320, 'right')  # approximate conversion to degrees
            
            # Extract coordinates
            if isinstance(line, LineString):
                coords = list(line.coords)
            else:
                # Handle MultiLineString or other geometries
                coords = []
                for geom in line.geoms:
                    coords.extend(list(geom.coords))
            
            # Reverse direction for every other line (zigzag pattern)
            if i % 2 == 1:
                coords = coords[::-1]
            
            # Create waypoints
            for j, (x, y) in enumerate(coords):
                position = Position(
                    latitude=y,
                    longitude=x,
                    altitude=settings.flight_altitude
                )
                
                waypoint = Waypoint(
                    position=position,
                    speed=settings.flight_speed,
                    order=i * len(coords) + j
                )
                
                # Add camera actions
                if settings.camera_trigger == "auto":
                    waypoint.actions.append(CameraActionDetails(
                        action=CameraAction.TAKE_PHOTO,
                        trigger=ActionTrigger.ON_ARRIVAL
                    ))
                
                if settings.gimbal_pitch is not None:
                    waypoint.actions.append(CameraActionDetails(
                        action=CameraAction.TILT_CAMERA,
                        trigger=ActionTrigger.ON_ARRIVAL,
                        gimbal_pitch=settings.gimbal_pitch
                    ))
                
                waypoints.append(waypoint)
        
        return waypoints
