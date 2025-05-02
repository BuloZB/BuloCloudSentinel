"""
API endpoints for Anduril Lattice integration.

This module provides FastAPI endpoints for interacting with Anduril's Lattice platform.
"""

from fastapi import APIRouter, HTTPException, Depends, BackgroundTasks

from security.validation.unified_validation import (
    validate_email,
    validate_username,
    validate_name,
    validate_uuid,
    validate_url,
    sanitize_string,
    sanitize_html,
    check_sql_injection,
    input_validator,
    form_validator,
    request_validator,
)
from pydantic import BaseModel, Field
from typing import Dict, List, Any, Optional

from backend.anduril_lattice.adapter import AndurilLatticeAdapter
from backend.anduril_lattice.entity_model import Entity, EntityType, create_track_entity, create_asset_entity, create_geo_entity
from backend.anduril_lattice.task_model import Task, TaskDefinition, TaskStatus, TaskState
from backend.api.dependencies import get_current_user
from backend.sensor_fusion.engine import SensorFusionEngine
from backend.mesh_networking.mesh_network import MeshNetwork

# Create router
router = APIRouter(prefix="/anduril-lattice", tags=["Anduril Lattice"])

# Global adapter instance
adapter = None


# Pydantic models for API requests/responses
class EntityRequest(BaseModel):
    """Request model for entity operations."""
    entity_id: Optional[str] = None
    is_live: bool = True
    aliases: Dict[str, str] = Field(default_factory=dict)
    description: Optional[str] = None
    location: Optional[Dict[str, Any]] = None
    mil_view: Optional[Dict[str, str]] = None
    ontology: Optional[Dict[str, str]] = None
    geo_shape: Optional[Dict[str, Any]] = None
    geo_details: Optional[Dict[str, Any]] = None
    signal: Optional[Dict[str, Any]] = None
    sensors: Optional[List[Dict[str, Any]]] = None
    additional_components: Dict[str, Any] = Field(default_factory=dict)


class EntityResponse(BaseModel):
    """Response model for entity operations."""
    entity_id: str
    is_live: bool
    aliases: Dict[str, str] = Field(default_factory=dict)
    description: Optional[str] = None
    created_time: Optional[str] = None
    expiry_time: Optional[str] = None
    location: Optional[Dict[str, Any]] = None
    mil_view: Optional[Dict[str, str]] = None
    ontology: Optional[Dict[str, str]] = None
    geo_shape: Optional[Dict[str, Any]] = None
    geo_details: Optional[Dict[str, Any]] = None
    signal: Optional[Dict[str, Any]] = None
    sensors: Optional[List[Dict[str, Any]]] = None
    provenance: Dict[str, str] = Field(default_factory=dict)
    additional_components: Dict[str, Any] = Field(default_factory=dict)


class TaskDefinitionRequest(BaseModel):
    """Request model for task definition."""
    task_type: str
    assignee_id: str
    description: str
    parameters: Dict[str, Any] = Field(default_factory=dict)
    priority: int = 0
    timeout_seconds: Optional[int] = None
    scheduled_time: Optional[str] = None
    expiry_time: Optional[str] = None
    requester_id: Optional[str] = None


class TaskStatusRequest(BaseModel):
    """Request model for task status updates."""
    state: str
    progress: Optional[float] = None
    message: Optional[str] = None
    error: Optional[Dict[str, Any]] = None
    result: Optional[Dict[str, Any]] = None


class TaskResponse(BaseModel):
    """Response model for task operations."""
    task_id: str
    definition: Dict[str, Any]
    status: Dict[str, Any]


class TrackEntityRequest(BaseModel):
    """Request model for creating a track entity."""
    entity_id: Optional[str] = None
    name: str
    latitude: float
    longitude: float
    altitude: Optional[float] = None
    disposition: str = "DISPOSITION_UNKNOWN"
    environment: str = "ENVIRONMENT_SURFACE"
    platform_type: str = "Unknown"
    expiry_minutes: int = 5


class AssetEntityRequest(BaseModel):
    """Request model for creating an asset entity."""
    entity_id: Optional[str] = None
    name: str
    latitude: float
    longitude: float
    altitude: Optional[float] = None
    disposition: str = "DISPOSITION_FRIENDLY"
    environment: str = "ENVIRONMENT_SURFACE"
    platform_type: str = "Surveillance"
    expiry_minutes: int = 30


class GeoPointRequest(BaseModel):
    """Request model for creating a geo point entity."""
    entity_id: Optional[str] = None
    name: str
    latitude: float
    longitude: float
    altitude: Optional[float] = None
    geo_type: str = "GEO_TYPE_GENERAL"
    expiry_minutes: int = 60


class GeoPolygonRequest(BaseModel):
    """Request model for creating a geo polygon entity."""
    entity_id: Optional[str] = None
    name: str
    positions: List[Dict[str, float]]
    geo_type: str = "GEO_TYPE_GENERAL"
    expiry_minutes: int = 60


class SurveillanceTaskRequest(BaseModel):
    """Request model for creating a surveillance task."""
    assignee_id: str
    target_latitude: float
    target_longitude: float
    target_altitude: Optional[float] = None
    duration_seconds: int = 300
    description: str = "Surveillance Task"
    priority: int = 0
    requester_id: Optional[str] = None


class PatrolTaskRequest(BaseModel):
    """Request model for creating a patrol task."""
    assignee_id: str
    waypoints: List[Dict[str, float]]
    loop: bool = False
    speed_mps: float = 5.0
    description: str = "Patrol Task"
    priority: int = 0
    requester_id: Optional[str] = None


# Initialization

def validate_request_data(request_data: dict, schema: dict) -> dict:
    """
    Validate request data against a schema.

    Args:
        request_data: Request data to validate
        schema: Validation schema

    Returns:
        Validated request data
    """
    return request_validator.validate_request(request_data, schema)

def initialize_adapter(sensor_fusion_engine: SensorFusionEngine, mesh_network: MeshNetwork):
    """
    Initialize the Anduril Lattice adapter.
    
    Args:
        sensor_fusion_engine: The sensor fusion engine instance
        mesh_network: The mesh network instance
    """
    global adapter
    if adapter is None:
        adapter = AndurilLatticeAdapter(
            sensor_fusion_engine=sensor_fusion_engine,
            mesh_network=mesh_network
        )


# API endpoints
@router.post("/entities", response_model=EntityResponse)
async def create_entity(
    request: EntityRequest,
    current_user: str = Depends(get_current_user)
):
    """Create a new entity."""
    if adapter is None:
        raise HTTPException(status_code=500, detail="Anduril Lattice adapter not initialized")
    
    # Convert request to entity
    entity = Entity(
        entity_id=request.entity_id or "",
        is_live=request.is_live,
        aliases=request.aliases,
        description=request.description,
        location=request.location,
        mil_view=request.mil_view,
        ontology=request.ontology,
        geo_shape=request.geo_shape,
        geo_details=request.geo_details,
        signal=request.signal,
        sensors=request.sensors
    )
    
    # Add additional components
    entity.additional_components = request.additional_components
    
    # Publish entity
    entity_id = await adapter.publish_entity(entity)
    
    # Get the created entity
    created_entity = await adapter.get_entity(entity_id)
    if not created_entity:
        raise HTTPException(status_code=500, detail="Failed to create entity")
    
    # Convert to response model
    response = EntityResponse(
        entity_id=created_entity.entity_id,
        is_live=created_entity.is_live,
        aliases=created_entity.aliases,
        description=created_entity.description,
        created_time=created_entity.created_time,
        expiry_time=created_entity.expiry_time,
        location=created_entity.location,
        mil_view=created_entity.mil_view,
        ontology=created_entity.ontology,
        geo_shape=created_entity.geo_shape,
        geo_details=created_entity.geo_details,
        signal=created_entity.signal,
        sensors=created_entity.sensors,
        provenance=created_entity.provenance,
        additional_components=created_entity.additional_components
    )
    
    return response


@router.get("/entities/{entity_id}", response_model=EntityResponse)
async def get_entity(
    entity_id: str,
    current_user: str = Depends(get_current_user)
):
    """Get an entity by ID."""
    if adapter is None:
        raise HTTPException(status_code=500, detail="Anduril Lattice adapter not initialized")
    
    # Get the entity
    entity = await adapter.get_entity(entity_id)
    if not entity:
        raise HTTPException(status_code=404, detail=f"Entity {entity_id} not found")
    
    # Convert to response model
    response = EntityResponse(
        entity_id=entity.entity_id,
        is_live=entity.is_live,
        aliases=entity.aliases,
        description=entity.description,
        created_time=entity.created_time,
        expiry_time=entity.expiry_time,
        location=entity.location,
        mil_view=entity.mil_view,
        ontology=entity.ontology,
        geo_shape=entity.geo_shape,
        geo_details=entity.geo_details,
        signal=entity.signal,
        sensors=entity.sensors,
        provenance=entity.provenance,
        additional_components=entity.additional_components
    )
    
    return response


@router.put("/entities/{entity_id}", response_model=EntityResponse)
async def update_entity(
    entity_id: str,
    request: EntityRequest,
    current_user: str = Depends(get_current_user)
):
    """Update an entity."""
    if adapter is None:
        raise HTTPException(status_code=500, detail="Anduril Lattice adapter not initialized")
    
    # Create updates dictionary
    updates = {}
    
    if request.is_live is not None:
        updates["is_live"] = request.is_live
    
    if request.aliases:
        updates["aliases"] = request.aliases
    
    if request.description is not None:
        updates["description"] = request.description
    
    if request.location is not None:
        updates["location"] = request.location
    
    if request.mil_view is not None:
        updates["mil_view"] = request.mil_view
    
    if request.ontology is not None:
        updates["ontology"] = request.ontology
    
    if request.geo_shape is not None:
        updates["geo_shape"] = request.geo_shape
    
    if request.geo_details is not None:
        updates["geo_details"] = request.geo_details
    
    if request.signal is not None:
        updates["signal"] = request.signal
    
    if request.sensors is not None:
        updates["sensors"] = request.sensors
    
    # Add additional components
    for key, value in request.additional_components.items():
        updates[key] = value
    
    # Update the entity
    updated_entity = await adapter.update_entity(entity_id, updates)
    if not updated_entity:
        raise HTTPException(status_code=404, detail=f"Entity {entity_id} not found")
    
    # Convert to response model
    response = EntityResponse(
        entity_id=updated_entity.entity_id,
        is_live=updated_entity.is_live,
        aliases=updated_entity.aliases,
        description=updated_entity.description,
        created_time=updated_entity.created_time,
        expiry_time=updated_entity.expiry_time,
        location=updated_entity.location,
        mil_view=updated_entity.mil_view,
        ontology=updated_entity.ontology,
        geo_shape=updated_entity.geo_shape,
        geo_details=updated_entity.geo_details,
        signal=updated_entity.signal,
        sensors=updated_entity.sensors,
        provenance=updated_entity.provenance,
        additional_components=updated_entity.additional_components
    )
    
    return response


@router.delete("/entities/{entity_id}", response_model=Dict[str, bool])
async def delete_entity(
    entity_id: str,
    current_user: str = Depends(get_current_user)
):
    """Delete an entity."""
    if adapter is None:
        raise HTTPException(status_code=500, detail="Anduril Lattice adapter not initialized")
    
    # Delete the entity
    success = await adapter.delete_entity(entity_id)
    if not success:
        raise HTTPException(status_code=404, detail=f"Entity {entity_id} not found")
    
    return {"success": True}


@router.get("/entities", response_model=List[EntityResponse])
async def get_entities(
    entity_type: Optional[str] = None,
    current_user: str = Depends(get_current_user)
):
    """Get entities matching the specified criteria."""
    if adapter is None:
        raise HTTPException(status_code=500, detail="Anduril Lattice adapter not initialized")
    
    # Convert entity type string to enum if provided
    entity_type_enum = None
    if entity_type:
        try:
            entity_type_enum = EntityType(entity_type)
        except ValueError:
            raise HTTPException(status_code=400, detail=f"Invalid entity type: {entity_type}")
    
    # Get entities
    entities = await adapter.get_entities(entity_type=entity_type_enum)
    
    # Convert to response models
    responses = []
    for entity in entities:
        response = EntityResponse(
            entity_id=entity.entity_id,
            is_live=entity.is_live,
            aliases=entity.aliases,
            description=entity.description,
            created_time=entity.created_time,
            expiry_time=entity.expiry_time,
            location=entity.location,
            mil_view=entity.mil_view,
            ontology=entity.ontology,
            geo_shape=entity.geo_shape,
            geo_details=entity.geo_details,
            signal=entity.signal,
            sensors=entity.sensors,
            provenance=entity.provenance,
            additional_components=entity.additional_components
        )
        responses.append(response)
    
    return responses


@router.post("/entities/track", response_model=EntityResponse)
async def create_track(
    request: TrackEntityRequest,
    current_user: str = Depends(get_current_user)
):
    """Create a track entity."""
    if adapter is None:
        raise HTTPException(status_code=500, detail="Anduril Lattice adapter not initialized")
    
    # Create track entity
    entity = create_track_entity(
        entity_id=request.entity_id or str(uuid.uuid4()),
        name=request.name,
        latitude=request.latitude,
        longitude=request.longitude,
        altitude=request.altitude,
        disposition=request.disposition,
        environment=request.environment,
        platform_type=request.platform_type,
        expiry_minutes=request.expiry_minutes
    )
    
    # Publish entity
    entity_id = await adapter.publish_entity(entity)
    
    # Get the created entity
    created_entity = await adapter.get_entity(entity_id)
    if not created_entity:
        raise HTTPException(status_code=500, detail="Failed to create track entity")
    
    # Convert to response model
    response = EntityResponse(
        entity_id=created_entity.entity_id,
        is_live=created_entity.is_live,
        aliases=created_entity.aliases,
        description=created_entity.description,
        created_time=created_entity.created_time,
        expiry_time=created_entity.expiry_time,
        location=created_entity.location,
        mil_view=created_entity.mil_view,
        ontology=created_entity.ontology,
        geo_shape=created_entity.geo_shape,
        geo_details=created_entity.geo_details,
        signal=created_entity.signal,
        sensors=created_entity.sensors,
        provenance=created_entity.provenance,
        additional_components=created_entity.additional_components
    )
    
    return response


@router.post("/entities/asset", response_model=EntityResponse)
async def create_asset(
    request: AssetEntityRequest,
    current_user: str = Depends(get_current_user)
):
    """Create an asset entity."""
    if adapter is None:
        raise HTTPException(status_code=500, detail="Anduril Lattice adapter not initialized")
    
    # Create asset entity
    entity = create_asset_entity(
        entity_id=request.entity_id or str(uuid.uuid4()),
        name=request.name,
        latitude=request.latitude,
        longitude=request.longitude,
        altitude=request.altitude,
        disposition=request.disposition,
        environment=request.environment,
        platform_type=request.platform_type,
        expiry_minutes=request.expiry_minutes
    )
    
    # Publish entity
    entity_id = await adapter.publish_entity(entity)
    
    # Get the created entity
    created_entity = await adapter.get_entity(entity_id)
    if not created_entity:
        raise HTTPException(status_code=500, detail="Failed to create asset entity")
    
    # Convert to response model
    response = EntityResponse(
        entity_id=created_entity.entity_id,
        is_live=created_entity.is_live,
        aliases=created_entity.aliases,
        description=created_entity.description,
        created_time=created_entity.created_time,
        expiry_time=created_entity.expiry_time,
        location=created_entity.location,
        mil_view=created_entity.mil_view,
        ontology=created_entity.ontology,
        geo_shape=created_entity.geo_shape,
        geo_details=created_entity.geo_details,
        signal=created_entity.signal,
        sensors=created_entity.sensors,
        provenance=created_entity.provenance,
        additional_components=created_entity.additional_components
    )
    
    return response


@router.post("/entities/geo/point", response_model=EntityResponse)
async def create_geo_point(
    request: GeoPointRequest,
    current_user: str = Depends(get_current_user)
):
    """Create a geo point entity."""
    if adapter is None:
        raise HTTPException(status_code=500, detail="Anduril Lattice adapter not initialized")
    
    # Create position
    position = {
        "latitude_degrees": request.latitude,
        "longitude_degrees": request.longitude
    }
    
    if request.altitude is not None:
        position["altitude_hae_meters"] = request.altitude
    
    # Create geo point
    point = {
        "position": position
    }
    
    # Create geo entity
    entity = create_geo_entity(
        entity_id=request.entity_id or str(uuid.uuid4()),
        name=request.name,
        geo_type=request.geo_type,
        expiry_minutes=request.expiry_minutes,
        point=point
    )
    
    # Publish entity
    entity_id = await adapter.publish_entity(entity)
    
    # Get the created entity
    created_entity = await adapter.get_entity(entity_id)
    if not created_entity:
        raise HTTPException(status_code=500, detail="Failed to create geo point entity")
    
    # Convert to response model
    response = EntityResponse(
        entity_id=created_entity.entity_id,
        is_live=created_entity.is_live,
        aliases=created_entity.aliases,
        description=created_entity.description,
        created_time=created_entity.created_time,
        expiry_time=created_entity.expiry_time,
        location=created_entity.location,
        mil_view=created_entity.mil_view,
        ontology=created_entity.ontology,
        geo_shape=created_entity.geo_shape,
        geo_details=created_entity.geo_details,
        signal=created_entity.signal,
        sensors=created_entity.sensors,
        provenance=created_entity.provenance,
        additional_components=created_entity.additional_components
    )
    
    return response


@router.post("/entities/geo/polygon", response_model=EntityResponse)
async def create_geo_polygon(
    request: GeoPolygonRequest,
    current_user: str = Depends(get_current_user)
):
    """Create a geo polygon entity."""
    if adapter is None:
        raise HTTPException(status_code=500, detail="Anduril Lattice adapter not initialized")
    
    # Create positions
    positions = []
    for pos in request.positions:
        position = {
            "position": {
                "latitude_degrees": pos["latitude"],
                "longitude_degrees": pos["longitude"]
            }
        }
        
        if "altitude" in pos:
            position["position"]["altitude_hae_meters"] = pos["altitude"]
        
        positions.append(position)
    
    # Create polygon
    polygon = {
        "rings": [
            {
                "positions": positions
            }
        ]
    }
    
    # Create geo entity
    entity = create_geo_entity(
        entity_id=request.entity_id or str(uuid.uuid4()),
        name=request.name,
        geo_type=request.geo_type,
        expiry_minutes=request.expiry_minutes,
        polygon=polygon
    )
    
    # Publish entity
    entity_id = await adapter.publish_entity(entity)
    
    # Get the created entity
    created_entity = await adapter.get_entity(entity_id)
    if not created_entity:
        raise HTTPException(status_code=500, detail="Failed to create geo polygon entity")
    
    # Convert to response model
    response = EntityResponse(
        entity_id=created_entity.entity_id,
        is_live=created_entity.is_live,
        aliases=created_entity.aliases,
        description=created_entity.description,
        created_time=created_entity.created_time,
        expiry_time=created_entity.expiry_time,
        location=created_entity.location,
        mil_view=created_entity.mil_view,
        ontology=created_entity.ontology,
        geo_shape=created_entity.geo_shape,
        geo_details=created_entity.geo_details,
        signal=created_entity.signal,
        sensors=created_entity.sensors,
        provenance=created_entity.provenance,
        additional_components=created_entity.additional_components
    )
    
    return response


@router.post("/tasks", response_model=TaskResponse)
async def create_task(
    request: TaskDefinitionRequest,
    current_user: str = Depends(get_current_user)
):
    """Create a new task."""
    if adapter is None:
        raise HTTPException(status_code=500, detail="Anduril Lattice adapter not initialized")
    
    # Convert request to task definition
    task_definition = TaskDefinition(
        task_type=request.task_type,
        assignee_id=request.assignee_id,
        description=request.description,
        parameters=request.parameters,
        priority=request.priority,
        timeout_seconds=request.timeout_seconds,
        scheduled_time=request.scheduled_time,
        expiry_time=request.expiry_time,
        requester_id=request.requester_id
    )
    
    # Create task
    task_id = await adapter.create_task(task_definition)
    
    # Get the created task
    created_task = await adapter.get_task(task_id)
    if not created_task:
        raise HTTPException(status_code=500, detail="Failed to create task")
    
    # Convert to response model
    response = TaskResponse(
        task_id=created_task.task_id,
        definition=created_task.definition.to_dict(),
        status=created_task.status.to_dict()
    )
    
    return response


@router.get("/tasks/{task_id}", response_model=TaskResponse)
async def get_task(
    task_id: str,
    current_user: str = Depends(get_current_user)
):
    """Get a task by ID."""
    if adapter is None:
        raise HTTPException(status_code=500, detail="Anduril Lattice adapter not initialized")
    
    # Get the task
    task = await adapter.get_task(task_id)
    if not task:
        raise HTTPException(status_code=404, detail=f"Task {task_id} not found")
    
    # Convert to response model
    response = TaskResponse(
        task_id=task.task_id,
        definition=task.definition.to_dict(),
        status=task.status.to_dict()
    )
    
    return response


@router.put("/tasks/{task_id}/status", response_model=TaskResponse)
async def update_task_status(
    task_id: str,
    request: TaskStatusRequest,
    current_user: str = Depends(get_current_user)
):
    """Update a task's status."""
    if adapter is None:
        raise HTTPException(status_code=500, detail="Anduril Lattice adapter not initialized")
    
    # Create updates dictionary
    updates = {}
    
    if request.state:
        updates["state"] = request.state
    
    if request.progress is not None:
        updates["progress"] = request.progress
    
    if request.message is not None:
        updates["message"] = request.message
    
    if request.error is not None:
        updates["error"] = request.error
    
    if request.result is not None:
        updates["result"] = request.result
    
    # Update the task status
    updated_task = await adapter.update_task_status(task_id, updates)
    if not updated_task:
        raise HTTPException(status_code=404, detail=f"Task {task_id} not found")
    
    # Convert to response model
    response = TaskResponse(
        task_id=updated_task.task_id,
        definition=updated_task.definition.to_dict(),
        status=updated_task.status.to_dict()
    )
    
    return response


@router.get("/tasks", response_model=List[TaskResponse])
async def get_tasks(
    assignee_id: Optional[str] = None,
    status: Optional[str] = None,
    current_user: str = Depends(get_current_user)
):
    """Get tasks matching the specified criteria."""
    if adapter is None:
        raise HTTPException(status_code=500, detail="Anduril Lattice adapter not initialized")
    
    # Get tasks
    tasks = await adapter.get_tasks(assignee_id=assignee_id, status=status)
    
    # Convert to response models
    responses = []
    for task in tasks:
        response = TaskResponse(
            task_id=task.task_id,
            definition=task.definition.to_dict(),
            status=task.status.to_dict()
        )
        responses.append(response)
    
    return responses


@router.post("/tasks/surveillance", response_model=TaskResponse)
async def create_surveillance_task(
    request: SurveillanceTaskRequest,
    current_user: str = Depends(get_current_user)
):
    """Create a surveillance task."""
    if adapter is None:
        raise HTTPException(status_code=500, detail="Anduril Lattice adapter not initialized")
    
    # Import task creation function
    from backend.anduril_lattice.task_model import create_surveillance_task
    
    # Create surveillance task definition
    task_definition = create_surveillance_task(
        assignee_id=request.assignee_id,
        target_latitude=request.target_latitude,
        target_longitude=request.target_longitude,
        target_altitude=request.target_altitude,
        duration_seconds=request.duration_seconds,
        description=request.description,
        priority=request.priority,
        requester_id=request.requester_id
    )
    
    # Create task
    task_id = await adapter.create_task(task_definition)
    
    # Get the created task
    created_task = await adapter.get_task(task_id)
    if not created_task:
        raise HTTPException(status_code=500, detail="Failed to create surveillance task")
    
    # Convert to response model
    response = TaskResponse(
        task_id=created_task.task_id,
        definition=created_task.definition.to_dict(),
        status=created_task.status.to_dict()
    )
    
    return response


@router.post("/tasks/patrol", response_model=TaskResponse)
async def create_patrol_task(
    request: PatrolTaskRequest,
    current_user: str = Depends(get_current_user)
):
    """Create a patrol task."""
    if adapter is None:
        raise HTTPException(status_code=500, detail="Anduril Lattice adapter not initialized")
    
    # Import task creation function
    from backend.anduril_lattice.task_model import create_patrol_task
    
    # Create patrol task definition
    task_definition = create_patrol_task(
        assignee_id=request.assignee_id,
        waypoints=request.waypoints,
        loop=request.loop,
        speed_mps=request.speed_mps,
        description=request.description,
        priority=request.priority,
        requester_id=request.requester_id
    )
    
    # Create task
    task_id = await adapter.create_task(task_definition)
    
    # Get the created task
    created_task = await adapter.get_task(task_id)
    if not created_task:
        raise HTTPException(status_code=500, detail="Failed to create patrol task")
    
    # Convert to response model
    response = TaskResponse(
        task_id=created_task.task_id,
        definition=created_task.definition.to_dict(),
        status=created_task.status.to_dict()
    )
    
    return response


@router.post("/start", response_model=Dict[str, bool])
async def start_adapter(
    background_tasks: BackgroundTasks,
    current_user: str = Depends(get_current_user)
):
    """Start the Anduril Lattice adapter."""
    if adapter is None:
        raise HTTPException(status_code=500, detail="Anduril Lattice adapter not initialized")
    
    # Start the adapter in the background
    background_tasks.add_task(adapter.start)
    
    return {"success": True}


@router.post("/stop", response_model=Dict[str, bool])
async def stop_adapter(
    background_tasks: BackgroundTasks,
    current_user: str = Depends(get_current_user)
):
    """Stop the Anduril Lattice adapter."""
    if adapter is None:
        raise HTTPException(status_code=500, detail="Anduril Lattice adapter not initialized")
    
    # Stop the adapter in the background
    background_tasks.add_task(adapter.stop)
    
    return {"success": True}
