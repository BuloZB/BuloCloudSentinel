"""
Blender animation converter for the Drone Show microservice.

This module provides utilities for converting Blender animations to drone show choreographies.
"""

import json
import math
import uuid
from typing import Dict, Any, List, Optional
from datetime import datetime

from drone_show_service.models.choreography import (
    Choreography, ChoreographyMetadata, ChoreographyType, ChoreographyStatus,
    DroneTrajectory, Waypoint, Position, LEDState, LEDColor, LEDEffect
)


def convert_blender_animation(
    animation_data: Dict[str, Any], name: str, description: Optional[str] = None, author: Optional[str] = None
) -> Choreography:
    """
    Convert a Blender animation to a drone show choreography.
    
    Args:
        animation_data: Blender animation data
        name: Choreography name
        description: Choreography description
        author: Choreography author
        
    Returns:
        Choreography
    """
    # Extract objects and animation data
    objects = animation_data.get("objects", [])
    
    # Create trajectories
    trajectories = []
    for obj in objects:
        # Skip non-drone objects
        if not obj.get("is_drone", False):
            continue
        
        # Get object name
        obj_name = obj.get("name", f"drone_{len(trajectories) + 1}")
        
        # Get animation frames
        frames = obj.get("frames", [])
        
        # Create waypoints
        waypoints = []
        led_states = []
        
        for frame in frames:
            # Get frame time
            frame_time = frame.get("time", 0.0)
            
            # Get position
            position = Position(
                lat=frame.get("lat", 0.0),
                lon=frame.get("lon", 0.0),
                alt=frame.get("alt", 0.0),
            )
            
            # Get heading
            heading = frame.get("heading", 0.0)
            
            # Create waypoint
            waypoint = Waypoint(
                time=frame_time,
                position=position,
                heading=heading,
            )
            
            waypoints.append(waypoint)
            
            # Get LED color
            color = frame.get("color", {"r": 0, "g": 0, "b": 0})
            led_color = LEDColor(
                r=color.get("r", 0),
                g=color.get("g", 0),
                b=color.get("b", 0),
            )
            
            # Get LED effect
            effect = frame.get("effect", "solid")
            
            # Create LED state
            led_state = LEDState(
                time=frame_time,
                color=led_color,
                effect=LEDEffect(effect),
                effect_params=frame.get("effect_params"),
            )
            
            led_states.append(led_state)
        
        # Create trajectory
        trajectory = DroneTrajectory(
            drone_id=obj_name,
            waypoints=waypoints,
            led_states=led_states,
        )
        
        trajectories.append(trajectory)
    
    # Calculate duration
    duration = 0.0
    for trajectory in trajectories:
        for waypoint in trajectory.waypoints:
            duration = max(duration, waypoint.time)
    
    # Create metadata
    metadata = ChoreographyMetadata(
        name=name,
        description=description,
        author=author,
        created_at=datetime.utcnow(),
        updated_at=datetime.utcnow(),
        tags=["blender", "imported"],
        duration=duration,
        drone_count=len(trajectories),
        status=ChoreographyStatus.DRAFT,
    )
    
    # Create choreography
    choreography = Choreography(
        id=str(uuid.uuid4()),
        metadata=metadata,
        type=ChoreographyType.BLENDER,
        trajectories=trajectories,
    )
    
    return choreography


def save_choreography_to_file(choreography: Choreography, file_path: str) -> bool:
    """
    Save a choreography to a file.
    
    Args:
        choreography: Choreography to save
        file_path: Path to save file
        
    Returns:
        True if successful, False otherwise
    """
    try:
        # Convert to dictionary
        data = json.loads(choreography.json())
        
        # Save to file
        with open(file_path, "w") as f:
            json.dump(data, f, indent=2)
        
        return True
    except Exception as e:
        print(f"Error saving choreography to file: {str(e)}")
        return False


def load_choreography_from_file(file_path: str) -> Optional[Choreography]:
    """
    Load a choreography from a file.
    
    Args:
        file_path: Path to load file
        
    Returns:
        Choreography if successful, None otherwise
    """
    try:
        # Load from file
        with open(file_path, "r") as f:
            data = json.load(f)
        
        # Convert to Choreography
        return Choreography(**data)
    except Exception as e:
        print(f"Error loading choreography from file: {str(e)}")
        return None
