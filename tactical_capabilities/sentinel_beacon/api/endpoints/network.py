"""
API endpoints for network management.
"""

from typing import List, Optional, Dict, Any
from fastapi import APIRouter, Depends, HTTPException, Query, status, Request
from datetime import datetime

from api.schemas import NetworkStatus
from core.security import get_current_user, has_permission

router = APIRouter()

@router.get("/status", response_model=NetworkStatus)
async def get_network_status(
    request: Request,
    current_user = Depends(get_current_user)
):
    """
    Get the current status of the mesh network.
    
    Args:
        request: FastAPI request
        current_user: Current authenticated user
        
    Returns:
        Network status
    """
    # Get mesh manager
    mesh_manager = request.app.state.mesh_manager
    
    # Check if connected
    if not mesh_manager.connected:
        raise HTTPException(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail="Mesh network not connected"
        )
    
    # Get network info
    network_info = await mesh_manager.get_network_info()
    
    # Create network status
    network_status = NetworkStatus(
        local_node=network_info.get("local_node", {}),
        nodes=network_info.get("nodes", []),
        channels=network_info.get("channels", []),
        node_count=network_info.get("node_count", 0),
        channel_count=network_info.get("channel_count", 0),
        timestamp=datetime.fromisoformat(network_info.get("timestamp", datetime.utcnow().isoformat()))
    )
    
    return network_status

@router.get("/topology")
async def get_network_topology(
    request: Request,
    current_user = Depends(get_current_user)
):
    """
    Get the topology of the mesh network.
    
    Args:
        request: FastAPI request
        current_user: Current authenticated user
        
    Returns:
        Network topology
    """
    # Get mesh manager
    mesh_manager = request.app.state.mesh_manager
    
    # Check if connected
    if not mesh_manager.connected:
        raise HTTPException(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail="Mesh network not connected"
        )
    
    # Get nodes
    nodes = await mesh_manager.get_nodes()
    
    # Create links
    links = []
    for node in nodes:
        for neighbor_id in node.get("neighbors", []):
            links.append({
                "source": node["id"],
                "target": neighbor_id
            })
    
    # Create topology
    topology = {
        "nodes": nodes,
        "links": links,
        "timestamp": datetime.utcnow().isoformat()
    }
    
    return topology

@router.post("/scan")
async def scan_network(
    request: Request,
    current_user = Depends(has_permission("network:scan"))
):
    """
    Scan for nearby mesh network nodes.
    
    Args:
        request: FastAPI request
        current_user: Current authenticated user with required permission
        
    Returns:
        Scan results
    """
    # Get mesh manager
    mesh_manager = request.app.state.mesh_manager
    
    # Check if connected
    if not mesh_manager.connected:
        raise HTTPException(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail="Mesh network not connected"
        )
    
    # In a real implementation, this would trigger a network scan
    # For now, we'll just return a placeholder
    
    return {
        "message": "Network scan initiated",
        "timestamp": datetime.utcnow().isoformat()
    }
