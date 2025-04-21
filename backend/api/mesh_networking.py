from fastapi import APIRouter, HTTPException, Depends, BackgroundTasks
from pydantic import BaseModel, Field
from typing import List, Dict, Any, Optional
from enum import Enum
from datetime import datetime, timezone

from backend.mesh_networking.mesh_network import (
    MeshNetwork, NodeCapability, NodeStatus,
    Message, MessageType, MessagePriority, RoutingProtocol
)
from backend.api.dependencies import get_current_user
import asyncio
import json

router = APIRouter(prefix="/mesh", tags=["Mesh Networking"])
mesh_network = MeshNetwork()

# Pydantic models for API requests/responses
class NodeCapabilityEnum(str, Enum):
    ROUTING = "routing"
    SENSOR = "sensor"
    CONTROL = "control"
    STORAGE = "storage"
    GATEWAY = "gateway"
    AI = "ai"

class NodeModel(BaseModel):
    node_id: str = Field(..., description="Unique identifier for the node")
    capabilities: List[NodeCapabilityEnum] = Field(
        default=[NodeCapabilityEnum.ROUTING],
        description="Capabilities of the node"
    )

class ConnectNodesRequest(BaseModel):
    node1_id: str = Field(..., description="ID of the first node")
    node2_id: str = Field(..., description="ID of the second node")
    metadata: Optional[Dict[str, Any]] = Field(
        default=None,
        description="Optional metadata for the connection"
    )

class MessageContentModel(BaseModel):
    content: Any = Field(..., description="Message content")
    message_type: str = Field(
        default="data",
        description="Type of message (data, control, discovery, heartbeat, routing, ack)"
    )
    priority: str = Field(
        default="normal",
        description="Message priority (low, normal, high, critical)"
    )

class DirectMessageRequest(BaseModel):
    sender_id: str = Field(..., description="ID of the sending node")
    target_id: str = Field(..., description="ID of the target node")
    content: Any = Field(..., description="Message content")
    message_type: str = Field(
        default="data",
        description="Type of message (data, control, discovery, heartbeat, routing, ack)"
    )
    priority: str = Field(
        default="normal",
        description="Message priority (low, normal, high, critical)"
    )

class BroadcastMessageRequest(BaseModel):
    sender_id: str = Field(..., description="ID of the sending node")
    content: Any = Field(..., description="Message content")
    message_type: str = Field(
        default="data",
        description="Type of message (data, control, discovery, heartbeat, routing, ack)"
    )
    priority: str = Field(
        default="normal",
        description="Message priority (low, normal, high, critical)"
    )
    ttl: int = Field(
        default=10,
        description="Time-to-live (hop count) for the message"
    )

class NodeInfoResponse(BaseModel):
    node_id: str
    status: str
    capabilities: List[str]
    peer_count: int
    metrics: Dict[str, Any]

class NetworkTopologyResponse(BaseModel):
    nodes: Dict[str, Dict[str, Any]]
    connections: List[Dict[str, Any]]

class NetworkMetricsResponse(BaseModel):
    node_count: int
    connection_count: int
    messages_total: int
    messages_sent: int
    messages_received: int
    messages_forwarded: int
    messages_dropped: int
    uptime_seconds: float
    start_time: str

# API endpoints
@router.post("/nodes", response_model=Dict[str, str], status_code=201)
async def add_node(node: NodeModel, current_user: str = Depends(get_current_user)):
    """Add a new node to the mesh network."""
    # Convert string capabilities to NodeCapability enum
    capabilities = [NodeCapability(cap.value) for cap in node.capabilities]

    # Add node to network
    mesh_network.add_node(node.node_id, capabilities)
    return {"detail": f"Node {node.node_id} added successfully"}

@router.get("/nodes", response_model=Dict[str, List[str]])
async def list_nodes(current_user: str = Depends(get_current_user)):
    """List all nodes in the mesh network."""
    nodes = mesh_network.get_all_nodes()
    return {"nodes": list(nodes.keys())}

@router.get("/nodes/{node_id}", response_model=NodeInfoResponse)
async def get_node(node_id: str, current_user: str = Depends(get_current_user)):
    """Get information about a specific node."""
    node = mesh_network.get_node(node_id)
    if not node:
        raise HTTPException(status_code=404, detail=f"Node {node_id} not found")

    # Update node status
    node.update_status()

    # Get node metrics
    metrics = node.get_metrics()

    return NodeInfoResponse(
        node_id=node.node_id,
        status=node.status.value,
        capabilities=[cap.value for cap in node.capabilities],
        peer_count=len(node.peers),
        metrics=metrics
    )

@router.delete("/nodes/{node_id}", response_model=Dict[str, str])
async def remove_node(node_id: str, current_user: str = Depends(get_current_user)):
    """Remove a node from the mesh network."""
    node = mesh_network.get_node(node_id)
    if not node:
        raise HTTPException(status_code=404, detail=f"Node {node_id} not found")

    mesh_network.remove_node(node_id)
    return {"detail": f"Node {node_id} removed successfully"}

@router.post("/connect", response_model=Dict[str, str])
async def connect_nodes(request: ConnectNodesRequest, current_user: str = Depends(get_current_user)):
    """Connect two nodes in the mesh network."""
    result = mesh_network.connect_nodes(
        request.node1_id,
        request.node2_id,
        request.metadata
    )

    if not result:
        raise HTTPException(
            status_code=400,
            detail=f"Failed to connect nodes {request.node1_id} and {request.node2_id}"
        )

    return {"detail": f"Nodes {request.node1_id} and {request.node2_id} connected successfully"}

@router.post("/disconnect", response_model=Dict[str, str])
async def disconnect_nodes(request: ConnectNodesRequest, current_user: str = Depends(get_current_user)):
    """Disconnect two nodes in the mesh network."""
    result = mesh_network.disconnect_nodes(request.node1_id, request.node2_id)

    if not result:
        raise HTTPException(
            status_code=400,
            detail=f"Failed to disconnect nodes {request.node1_id} and {request.node2_id}"
        )

    return {"detail": f"Nodes {request.node1_id} and {request.node2_id} disconnected successfully"}

@router.get("/topology", response_model=NetworkTopologyResponse)
async def get_network_topology(current_user: str = Depends(get_current_user)):
    """Get the current network topology."""
    topology = mesh_network.get_network_topology()
    return NetworkTopologyResponse(**topology)

@router.get("/metrics", response_model=NetworkMetricsResponse)
async def get_network_metrics(current_user: str = Depends(get_current_user)):
    """Get network-wide metrics."""
    metrics = mesh_network.get_network_metrics()
    return NetworkMetricsResponse(**metrics)

@router.post("/nodes/{node_id}/messages/direct", response_model=Dict[str, str])
async def send_direct_message(
    node_id: str,
    request: DirectMessageRequest,
    current_user: str = Depends(get_current_user)
):
    """Send a direct message from one node to another."""
    if node_id != request.sender_id:
        raise HTTPException(
            status_code=400,
            detail=f"Sender ID in path ({node_id}) does not match sender ID in request ({request.sender_id})"
        )

    # Convert string message type and priority to enums
    message_type = MessageType(request.message_type)
    priority = MessagePriority(request.priority)

    # Send message
    result = await mesh_network.send_direct_message(
        sender_id=request.sender_id,
        target_id=request.target_id,
        content=request.content,
        message_type=message_type,
        priority=priority
    )

    if not result:
        raise HTTPException(
            status_code=400,
            detail=f"Failed to send message from {request.sender_id} to {request.target_id}"
        )

    return {"detail": "Message sent successfully"}

@router.post("/nodes/{node_id}/messages/broadcast", response_model=Dict[str, str])
async def broadcast_message(
    node_id: str,
    request: BroadcastMessageRequest,
    current_user: str = Depends(get_current_user)
):
    """Broadcast a message from a node to all its peers."""
    if node_id != request.sender_id:
        raise HTTPException(
            status_code=400,
            detail=f"Sender ID in path ({node_id}) does not match sender ID in request ({request.sender_id})"
        )

    # Convert string message type and priority to enums
    message_type = MessageType(request.message_type)
    priority = MessagePriority(request.priority)

    # Create message
    message = Message(
        sender_id=request.sender_id,
        message_type=message_type,
        content=request.content,
        priority=priority,
        ttl=request.ttl,
        routing_protocol=RoutingProtocol.FLOODING
    )

    # Broadcast message
    await mesh_network.broadcast(request.sender_id, message)

    return {"detail": "Message broadcasted successfully"}

@router.get("/nodes/{node_id}/messages", response_model=Dict[str, List[Dict[str, Any]]])
async def get_messages(node_id: str, current_user: str = Depends(get_current_user)):
    """Get messages from a node's queue."""
    node = mesh_network.get_node(node_id)
    if not node:
        raise HTTPException(status_code=404, detail=f"Node {node_id} not found")

    messages = []
    # Get up to 100 messages from the queue
    for _ in range(100):
        if node.message_queue.empty():
            break
        message = await node.message_queue.get()
        messages.append(message.to_dict())

    return {"messages": messages}

@router.post("/nodes/{node_id}/discovery", response_model=Dict[str, str])
async def start_discovery(node_id: str, current_user: str = Depends(get_current_user)):
    """Start the discovery process for a node."""
    result = await mesh_network.start_discovery(node_id)

    if not result:
        raise HTTPException(
            status_code=400,
            detail=f"Failed to start discovery for node {node_id}"
        )

    return {"detail": f"Discovery started for node {node_id}"}

@router.get("/nodes/{node_id}/discovered", response_model=Dict[str, Dict[str, Any]])
async def get_discovered_nodes(node_id: str, current_user: str = Depends(get_current_user)):
    """Get nodes discovered by a specific node."""
    discovered = mesh_network.get_discovered_nodes(node_id)

    if discovered is None:
        raise HTTPException(
            status_code=404,
            detail=f"Node {node_id} not found or discovery not started"
        )

    return {"discovered_nodes": discovered}

@router.get("/nodes/{node_id}/peers", response_model=Dict[str, Dict[str, Any]])
async def get_node_peers(node_id: str, current_user: str = Depends(get_current_user)):
    """Get all peers of a specific node."""
    node = mesh_network.get_node(node_id)
    if not node:
        raise HTTPException(status_code=404, detail=f"Node {node_id} not found")

    peers = node.get_peers()
    return {"peers": peers}
