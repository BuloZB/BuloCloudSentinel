"""
Anduril Lattice Task Model for Bulo.Cloud Sentinel.

This module defines the Task data model compatible with Anduril's Lattice platform.
"""

from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, List, Any, Optional, Union


class TaskState(Enum):
    """Task states in Anduril's Lattice platform."""
    CREATED = "CREATED"
    PENDING = "PENDING"
    ASSIGNED = "ASSIGNED"
    EXECUTING = "EXECUTING"
    SUCCEEDED = "SUCCEEDED"
    FAILED = "FAILED"
    CANCELED = "CANCELED"
    REJECTED = "REJECTED"


@dataclass
class TaskStatus:
    """
    Task status component of the task model.
    
    Represents the current state and progress of a task.
    """
    state: str
    created_time: str  # ISO-8601 format
    last_updated_time: str  # ISO-8601 format
    progress: Optional[float] = None
    message: Optional[str] = None
    error: Optional[Dict[str, Any]] = None
    result: Optional[Dict[str, Any]] = None
    
    def to_dict(self) -> Dict[str, Any]:
        """
        Convert the task status to a dictionary representation.
        
        Returns:
            Dictionary representation of the task status
        """
        result = {
            "state": self.state,
            "created_time": self.created_time,
            "last_updated_time": self.last_updated_time
        }
        
        if self.progress is not None:
            result["progress"] = self.progress
        
        if self.message:
            result["message"] = self.message
        
        if self.error:
            result["error"] = self.error
        
        if self.result:
            result["result"] = self.result
        
        return result
    
    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'TaskStatus':
        """
        Create a task status from a dictionary.
        
        Args:
            data: Dictionary representation of the task status
            
        Returns:
            TaskStatus instance
        """
        return cls(
            state=data.get("state", TaskState.CREATED.value),
            created_time=data.get("created_time", ""),
            last_updated_time=data.get("last_updated_time", ""),
            progress=data.get("progress"),
            message=data.get("message"),
            error=data.get("error"),
            result=data.get("result")
        )


@dataclass
class TaskDefinition:
    """
    Task definition component of the task model.
    
    Represents the definition and parameters of a task.
    """
    task_type: str
    assignee_id: str
    description: str
    parameters: Dict[str, Any] = field(default_factory=dict)
    priority: int = 0
    timeout_seconds: Optional[int] = None
    scheduled_time: Optional[str] = None  # ISO-8601 format
    expiry_time: Optional[str] = None  # ISO-8601 format
    requester_id: Optional[str] = None
    
    def to_dict(self) -> Dict[str, Any]:
        """
        Convert the task definition to a dictionary representation.
        
        Returns:
            Dictionary representation of the task definition
        """
        result = {
            "task_type": self.task_type,
            "assignee_id": self.assignee_id,
            "description": self.description,
            "parameters": self.parameters,
            "priority": self.priority
        }
        
        if self.timeout_seconds is not None:
            result["timeout_seconds"] = self.timeout_seconds
        
        if self.scheduled_time:
            result["scheduled_time"] = self.scheduled_time
        
        if self.expiry_time:
            result["expiry_time"] = self.expiry_time
        
        if self.requester_id:
            result["requester_id"] = self.requester_id
        
        return result
    
    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'TaskDefinition':
        """
        Create a task definition from a dictionary.
        
        Args:
            data: Dictionary representation of the task definition
            
        Returns:
            TaskDefinition instance
        """
        return cls(
            task_type=data.get("task_type", ""),
            assignee_id=data.get("assignee_id", ""),
            description=data.get("description", ""),
            parameters=data.get("parameters", {}),
            priority=data.get("priority", 0),
            timeout_seconds=data.get("timeout_seconds"),
            scheduled_time=data.get("scheduled_time"),
            expiry_time=data.get("expiry_time"),
            requester_id=data.get("requester_id")
        )


@dataclass
class Task:
    """
    Task data model compatible with Anduril's Lattice platform.
    
    A task represents a command to a networked actor to perform some useful action.
    """
    task_id: str
    definition: TaskDefinition
    status: TaskStatus
    
    def to_dict(self) -> Dict[str, Any]:
        """
        Convert the task to a dictionary representation.
        
        Returns:
            Dictionary representation of the task
        """
        return {
            "task_id": self.task_id,
            "definition": self.definition.to_dict(),
            "status": self.status.to_dict()
        }
    
    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'Task':
        """
        Create a task from a dictionary.
        
        Args:
            data: Dictionary representation of the task
            
        Returns:
            Task instance
        """
        return cls(
            task_id=data.get("task_id", ""),
            definition=TaskDefinition.from_dict(data.get("definition", {})),
            status=TaskStatus.from_dict(data.get("status", {}))
        )


def create_surveillance_task(
    assignee_id: str,
    target_latitude: float,
    target_longitude: float,
    target_altitude: Optional[float] = None,
    duration_seconds: int = 300,
    description: str = "Surveillance Task",
    priority: int = 0,
    requester_id: Optional[str] = None
) -> TaskDefinition:
    """
    Create a surveillance task definition.
    
    Args:
        assignee_id: ID of the asset to assign the task to
        target_latitude: Target latitude in degrees
        target_longitude: Target longitude in degrees
        target_altitude: Optional target altitude in meters
        duration_seconds: Duration of the surveillance in seconds
        description: Human-readable description
        priority: Task priority (higher is more important)
        requester_id: Optional ID of the requester
        
    Returns:
        Surveillance task definition
    """
    # Create target position
    target_position = {
        "latitude_degrees": target_latitude,
        "longitude_degrees": target_longitude
    }
    
    if target_altitude is not None:
        target_position["altitude_hae_meters"] = target_altitude
    
    # Create parameters
    parameters = {
        "target_position": target_position,
        "duration_seconds": duration_seconds
    }
    
    # Create task definition
    task_definition = TaskDefinition(
        task_type="SURVEILLANCE",
        assignee_id=assignee_id,
        description=description,
        parameters=parameters,
        priority=priority,
        timeout_seconds=duration_seconds + 60,  # Add 1 minute for setup
        requester_id=requester_id
    )
    
    return task_definition


def create_patrol_task(
    assignee_id: str,
    waypoints: List[Dict[str, float]],
    loop: bool = False,
    speed_mps: float = 5.0,
    description: str = "Patrol Task",
    priority: int = 0,
    requester_id: Optional[str] = None
) -> TaskDefinition:
    """
    Create a patrol task definition.
    
    Args:
        assignee_id: ID of the asset to assign the task to
        waypoints: List of waypoints (each with latitude_degrees, longitude_degrees, and optional altitude_hae_meters)
        loop: Whether to loop through the waypoints continuously
        speed_mps: Speed in meters per second
        description: Human-readable description
        priority: Task priority (higher is more important)
        requester_id: Optional ID of the requester
        
    Returns:
        Patrol task definition
    """
    # Create parameters
    parameters = {
        "waypoints": waypoints,
        "loop": loop,
        "speed_mps": speed_mps
    }
    
    # Estimate duration based on waypoints and speed
    total_distance = 0.0
    for i in range(len(waypoints) - 1):
        # Simple Euclidean distance (not accurate for long distances)
        wp1 = waypoints[i]
        wp2 = waypoints[i + 1]
        
        # Convert to meters (very rough approximation)
        lat_diff = (wp2["latitude_degrees"] - wp1["latitude_degrees"]) * 111000
        lon_diff = (wp2["longitude_degrees"] - wp1["longitude_degrees"]) * 111000 * 0.7  # Rough correction for longitude
        
        distance = (lat_diff ** 2 + lon_diff ** 2) ** 0.5
        total_distance += distance
    
    estimated_duration = int(total_distance / speed_mps) + 60  # Add 1 minute buffer
    
    # Create task definition
    task_definition = TaskDefinition(
        task_type="PATROL",
        assignee_id=assignee_id,
        description=description,
        parameters=parameters,
        priority=priority,
        timeout_seconds=estimated_duration,
        requester_id=requester_id
    )
    
    return task_definition
