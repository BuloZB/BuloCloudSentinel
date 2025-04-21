"""
Anduril Lattice Adapter for Bulo.Cloud Sentinel.

This module provides integration with Anduril's Lattice platform, enabling
interoperability between Bulo.Cloud Sentinel and Anduril's mesh network.
"""

import asyncio
import json
import logging
import uuid
from datetime import datetime, timedelta
from typing import Dict, List, Any, Optional, Callable, Tuple, Set

from backend.anduril_lattice.entity_model import Entity, EntityComponent, EntityType
from backend.anduril_lattice.task_model import Task, TaskStatus, TaskDefinition
from backend.sensor_fusion.engine import SensorFusionEngine
from backend.mesh_networking.mesh_network import MeshNetwork, Message, MessageType

logger = logging.getLogger(__name__)

class AndurilLatticeAdapter:
    """
    Adapter for integrating with Anduril's Lattice platform.
    
    This adapter translates between Bulo.Cloud Sentinel's internal data models
    and Anduril's Lattice data models, enabling seamless interoperability.
    """
    
    def __init__(
        self,
        sensor_fusion_engine: SensorFusionEngine,
        mesh_network: MeshNetwork,
        integration_name: str = "BuloCloudSentinel",
        data_type: str = "SurveillancePlatform"
    ):
        """
        Initialize the Anduril Lattice adapter.
        
        Args:
            sensor_fusion_engine: The sensor fusion engine instance
            mesh_network: The mesh network instance
            integration_name: Name of this integration in Anduril's system
            data_type: Data type identifier for Anduril's system
        """
        self.sensor_fusion_engine = sensor_fusion_engine
        self.mesh_network = mesh_network
        self.integration_name = integration_name
        self.data_type = data_type
        
        # Entity storage
        self.entities: Dict[str, Entity] = {}
        self.entity_expiry_times: Dict[str, datetime] = {}
        
        # Task storage
        self.tasks: Dict[str, Task] = {}
        self.task_handlers: Dict[str, Callable] = {}
        
        # Subscription callbacks
        self.entity_subscribers: List[Callable] = []
        self.task_subscribers: List[Callable] = []
        
        # Background tasks
        self.background_tasks = set()
        self._running = False
    
    async def start(self):
        """Start the adapter and background tasks."""
        if self._running:
            return
        
        self._running = True
        
        # Start background tasks
        task = asyncio.create_task(self._entity_expiry_checker())
        self.background_tasks.add(task)
        task.add_done_callback(self.background_tasks.remove)
        
        task = asyncio.create_task(self._sensor_fusion_to_entity_sync())
        self.background_tasks.add(task)
        task.add_done_callback(self.background_tasks.remove)
        
        task = asyncio.create_task(self._mesh_network_to_entity_sync())
        self.background_tasks.add(task)
        task.add_done_callback(self.background_tasks.remove)
        
        logger.info("Anduril Lattice adapter started")
    
    async def stop(self):
        """Stop the adapter and background tasks."""
        self._running = False
        
        # Cancel all background tasks
        for task in self.background_tasks:
            task.cancel()
        
        # Wait for all tasks to complete
        if self.background_tasks:
            await asyncio.gather(*self.background_tasks, return_exceptions=True)
        
        logger.info("Anduril Lattice adapter stopped")
    
    # Entity Management
    
    async def publish_entity(self, entity: Entity) -> str:
        """
        Publish an entity to the Anduril Lattice system.
        
        Args:
            entity: The entity to publish
            
        Returns:
            The entity ID
        """
        # Ensure entity has required components
        if not entity.entity_id:
            entity.entity_id = str(uuid.uuid4())
        
        if not entity.provenance:
            entity.provenance = {
                "integration_name": self.integration_name,
                "data_type": self.data_type,
                "source_update_time": datetime.utcnow().isoformat()
            }
        
        if not entity.expiry_time:
            # Default expiry time is 5 minutes from now
            expiry_time = datetime.utcnow() + timedelta(minutes=5)
            entity.expiry_time = expiry_time.isoformat()
            self.entity_expiry_times[entity.entity_id] = expiry_time
        
        # Store the entity
        self.entities[entity.entity_id] = entity
        
        # Notify subscribers
        for subscriber in self.entity_subscribers:
            try:
                await subscriber(entity, "CREATE" if entity.is_live else "DELETE")
            except Exception as e:
                logger.error(f"Error notifying entity subscriber: {str(e)}")
        
        return entity.entity_id
    
    async def update_entity(self, entity_id: str, updates: Dict[str, Any]) -> Optional[Entity]:
        """
        Update an existing entity.
        
        Args:
            entity_id: The ID of the entity to update
            updates: Dictionary of component updates
            
        Returns:
            The updated entity or None if not found
        """
        if entity_id not in self.entities:
            return None
        
        entity = self.entities[entity_id]
        
        # Update components
        for component_name, component_value in updates.items():
            setattr(entity, component_name, component_value)
        
        # Update provenance source update time
        if entity.provenance:
            entity.provenance["source_update_time"] = datetime.utcnow().isoformat()
        
        # Notify subscribers
        for subscriber in self.entity_subscribers:
            try:
                await subscriber(entity, "UPDATE")
            except Exception as e:
                logger.error(f"Error notifying entity subscriber: {str(e)}")
        
        return entity
    
    async def delete_entity(self, entity_id: str) -> bool:
        """
        Delete an entity.
        
        Args:
            entity_id: The ID of the entity to delete
            
        Returns:
            True if the entity was deleted, False otherwise
        """
        if entity_id not in self.entities:
            return False
        
        entity = self.entities[entity_id]
        entity.is_live = False
        
        # Notify subscribers
        for subscriber in self.entity_subscribers:
            try:
                await subscriber(entity, "DELETE")
            except Exception as e:
                logger.error(f"Error notifying entity subscriber: {str(e)}")
        
        # Remove from storage
        del self.entities[entity_id]
        if entity_id in self.entity_expiry_times:
            del self.entity_expiry_times[entity_id]
        
        return True
    
    async def get_entity(self, entity_id: str) -> Optional[Entity]:
        """
        Get an entity by ID.
        
        Args:
            entity_id: The ID of the entity to get
            
        Returns:
            The entity or None if not found
        """
        return self.entities.get(entity_id)
    
    async def get_entities(self, 
                          entity_type: Optional[EntityType] = None, 
                          component_filter: Optional[Dict[str, Any]] = None) -> List[Entity]:
        """
        Get entities matching the specified criteria.
        
        Args:
            entity_type: Optional entity type to filter by
            component_filter: Optional component filter
            
        Returns:
            List of matching entities
        """
        result = []
        
        for entity in self.entities.values():
            # Filter by entity type if specified
            if entity_type and entity.ontology and entity.ontology.get("template") != entity_type.value:
                continue
            
            # Filter by components if specified
            if component_filter:
                match = True
                for component_name, component_value in component_filter.items():
                    if not hasattr(entity, component_name) or getattr(entity, component_name) != component_value:
                        match = False
                        break
                
                if not match:
                    continue
            
            result.append(entity)
        
        return result
    
    def subscribe_to_entities(self, callback: Callable[[Entity, str], None]) -> None:
        """
        Subscribe to entity updates.
        
        Args:
            callback: Callback function that takes an entity and event type
                     (CREATE, UPDATE, DELETE) as arguments
        """
        self.entity_subscribers.append(callback)
    
    def unsubscribe_from_entities(self, callback: Callable) -> None:
        """
        Unsubscribe from entity updates.
        
        Args:
            callback: The callback function to remove
        """
        if callback in self.entity_subscribers:
            self.entity_subscribers.remove(callback)
    
    # Task Management
    
    async def create_task(self, task_definition: TaskDefinition) -> str:
        """
        Create a new task.
        
        Args:
            task_definition: The task definition
            
        Returns:
            The task ID
        """
        task_id = str(uuid.uuid4())
        
        task = Task(
            task_id=task_id,
            definition=task_definition,
            status=TaskStatus(
                state="CREATED",
                created_time=datetime.utcnow().isoformat(),
                last_updated_time=datetime.utcnow().isoformat()
            )
        )
        
        self.tasks[task_id] = task
        
        # Notify subscribers
        for subscriber in self.task_subscribers:
            try:
                await subscriber(task, "CREATE")
            except Exception as e:
                logger.error(f"Error notifying task subscriber: {str(e)}")
        
        return task_id
    
    async def update_task_status(self, task_id: str, status_updates: Dict[str, Any]) -> Optional[Task]:
        """
        Update a task's status.
        
        Args:
            task_id: The ID of the task to update
            status_updates: Dictionary of status updates
            
        Returns:
            The updated task or None if not found
        """
        if task_id not in self.tasks:
            return None
        
        task = self.tasks[task_id]
        
        # Update status fields
        for field_name, field_value in status_updates.items():
            setattr(task.status, field_name, field_value)
        
        # Update last updated time
        task.status.last_updated_time = datetime.utcnow().isoformat()
        
        # Notify subscribers
        for subscriber in self.task_subscribers:
            try:
                await subscriber(task, "UPDATE")
            except Exception as e:
                logger.error(f"Error notifying task subscriber: {str(e)}")
        
        return task
    
    async def get_task(self, task_id: str) -> Optional[Task]:
        """
        Get a task by ID.
        
        Args:
            task_id: The ID of the task to get
            
        Returns:
            The task or None if not found
        """
        return self.tasks.get(task_id)
    
    async def get_tasks(self, 
                       assignee_id: Optional[str] = None, 
                       status: Optional[str] = None) -> List[Task]:
        """
        Get tasks matching the specified criteria.
        
        Args:
            assignee_id: Optional assignee ID to filter by
            status: Optional status to filter by
            
        Returns:
            List of matching tasks
        """
        result = []
        
        for task in self.tasks.values():
            # Filter by assignee if specified
            if assignee_id and task.definition.assignee_id != assignee_id:
                continue
            
            # Filter by status if specified
            if status and task.status.state != status:
                continue
            
            result.append(task)
        
        return result
    
    def register_task_handler(self, task_type: str, handler: Callable[[Task], None]) -> None:
        """
        Register a handler for a specific task type.
        
        Args:
            task_type: The type of task to handle
            handler: The handler function
        """
        self.task_handlers[task_type] = handler
    
    def unregister_task_handler(self, task_type: str) -> None:
        """
        Unregister a task handler.
        
        Args:
            task_type: The type of task handler to remove
        """
        if task_type in self.task_handlers:
            del self.task_handlers[task_type]
    
    def subscribe_to_tasks(self, callback: Callable[[Task, str], None]) -> None:
        """
        Subscribe to task updates.
        
        Args:
            callback: Callback function that takes a task and event type
                     (CREATE, UPDATE) as arguments
        """
        self.task_subscribers.append(callback)
    
    def unsubscribe_from_tasks(self, callback: Callable) -> None:
        """
        Unsubscribe from task updates.
        
        Args:
            callback: The callback function to remove
        """
        if callback in self.task_subscribers:
            self.task_subscribers.remove(callback)
    
    # Background Tasks
    
    async def _entity_expiry_checker(self):
        """Background task to check for expired entities."""
        while self._running:
            now = datetime.utcnow()
            expired_entity_ids = []
            
            # Find expired entities
            for entity_id, expiry_time in self.entity_expiry_times.items():
                if expiry_time <= now:
                    expired_entity_ids.append(entity_id)
            
            # Delete expired entities
            for entity_id in expired_entity_ids:
                await self.delete_entity(entity_id)
            
            # Sleep for 10 seconds
            await asyncio.sleep(10)
    
    async def _sensor_fusion_to_entity_sync(self):
        """Background task to sync sensor fusion data to entities."""
        while self._running:
            try:
                # Get fused data from sensor fusion engine
                fused_data = await self.sensor_fusion_engine.fuse_data()
                
                if fused_data:
                    # Convert position data to entities
                    if "position" in fused_data and fused_data["position"]:
                        position = fused_data["position"]
                        entity = Entity(
                            entity_id=f"position_{self.integration_name}",
                            is_live=True,
                            aliases={"name": "Platform Position"},
                            location={
                                "position": {
                                    "latitude_degrees": position.get("x", 0),
                                    "longitude_degrees": position.get("y", 0),
                                    "altitude_hae_meters": position.get("z", 0) if "z" in position else None
                                }
                            },
                            mil_view={
                                "disposition": "DISPOSITION_FRIENDLY",
                                "environment": "ENVIRONMENT_SURFACE"
                            },
                            ontology={
                                "template": "TEMPLATE_ASSET",
                                "platform_type": "Surveillance"
                            }
                        )
                        await self.publish_entity(entity)
                    
                    # Convert detection data to entities
                    if "detections" in fused_data and fused_data["detections"]:
                        for idx, detection in enumerate(fused_data["detections"]):
                            entity = Entity(
                                entity_id=f"detection_{idx}_{self.integration_name}",
                                is_live=True,
                                aliases={"name": f"Detection {detection.get('type', 'Unknown')}"},
                                location={
                                    "position": {
                                        "latitude_degrees": detection.get("x", 0),
                                        "longitude_degrees": detection.get("y", 0),
                                        "altitude_hae_meters": detection.get("z", 0) if "z" in detection else None
                                    }
                                },
                                mil_view={
                                    "disposition": "DISPOSITION_UNKNOWN",
                                    "environment": "ENVIRONMENT_SURFACE"
                                },
                                ontology={
                                    "template": "TEMPLATE_TRACK",
                                    "platform_type": detection.get("type", "Unknown")
                                }
                            )
                            await self.publish_entity(entity)
            except Exception as e:
                logger.error(f"Error in sensor fusion to entity sync: {str(e)}")
            
            # Sleep for 1 second
            await asyncio.sleep(1)
    
    async def _mesh_network_to_entity_sync(self):
        """Background task to sync mesh network data to entities."""
        while self._running:
            try:
                # Get network topology
                topology = self.mesh_network.get_network_topology()
                
                if topology and "nodes" in topology:
                    # Convert nodes to entities
                    for node_id, node_data in topology["nodes"].items():
                        entity = Entity(
                            entity_id=f"mesh_node_{node_id}",
                            is_live=True,
                            aliases={"name": f"Mesh Node {node_id}"},
                            mil_view={
                                "disposition": "DISPOSITION_FRIENDLY",
                                "environment": "ENVIRONMENT_SURFACE"
                            },
                            ontology={
                                "template": "TEMPLATE_ASSET",
                                "platform_type": "Communication"
                            }
                        )
                        await self.publish_entity(entity)
            except Exception as e:
                logger.error(f"Error in mesh network to entity sync: {str(e)}")
            
            # Sleep for 5 seconds
            await asyncio.sleep(5)
