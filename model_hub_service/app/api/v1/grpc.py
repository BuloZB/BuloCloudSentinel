"""
gRPC endpoints for the Model Hub service.

This module provides API endpoints for gRPC communication with edge devices.
"""

import logging
import asyncio
from typing import Dict, List, Any, Optional

from fastapi import APIRouter, Depends, HTTPException, status, WebSocket, WebSocketDisconnect
from fastapi.responses import JSONResponse

# Setup logging
logger = logging.getLogger(__name__)

# Create router
router = APIRouter()

# Store for active WebSocket connections
active_connections: Dict[str, WebSocket] = {}

@router.websocket("/ws/{client_id}")
async def websocket_endpoint(websocket: WebSocket, client_id: str):
    """
    WebSocket endpoint for edge device updates.
    
    Args:
        websocket: WebSocket connection
        client_id: Client ID
    """
    await websocket.accept()
    active_connections[client_id] = websocket
    
    try:
        # Send initial message
        await websocket.send_json({
            "type": "connected",
            "message": f"Connected as {client_id}",
        })
        
        # Keep connection open and handle messages
        while True:
            # Wait for message
            data = await websocket.receive_json()
            
            # Process message
            if data.get("type") == "ping":
                await websocket.send_json({
                    "type": "pong",
                    "timestamp": data.get("timestamp"),
                })
            elif data.get("type") == "status":
                # Process status update from edge device
                logger.info(f"Received status update from {client_id}: {data}")
                
                # Send acknowledgement
                await websocket.send_json({
                    "type": "ack",
                    "message": "Status update received",
                })
            else:
                # Unknown message type
                logger.warning(f"Received unknown message type from {client_id}: {data}")
                
                # Send error
                await websocket.send_json({
                    "type": "error",
                    "message": "Unknown message type",
                })
    except WebSocketDisconnect:
        logger.info(f"Client {client_id} disconnected")
    except Exception as e:
        logger.error(f"Error in WebSocket connection for {client_id}: {e}")
    finally:
        # Remove connection from active connections
        if client_id in active_connections:
            del active_connections[client_id]

@router.post("/broadcast")
async def broadcast_message(message: Dict[str, Any]):
    """
    Broadcast a message to all connected edge devices.
    
    Args:
        message: Message to broadcast
        
    Returns:
        Broadcast status
    """
    try:
        # Check if there are any active connections
        if not active_connections:
            return {
                "status": "warning",
                "message": "No active connections",
                "connections": 0,
            }
        
        # Broadcast message to all connections
        for client_id, websocket in active_connections.items():
            try:
                await websocket.send_json(message)
            except Exception as e:
                logger.error(f"Error sending message to {client_id}: {e}")
        
        return {
            "status": "ok",
            "message": "Message broadcasted",
            "connections": len(active_connections),
        }
    except Exception as e:
        logger.error(f"Error broadcasting message: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error broadcasting message: {str(e)}",
        )

@router.post("/notify/{client_id}")
async def notify_client(client_id: str, message: Dict[str, Any]):
    """
    Send a message to a specific edge device.
    
    Args:
        client_id: Client ID
        message: Message to send
        
    Returns:
        Notification status
    """
    try:
        # Check if client is connected
        if client_id not in active_connections:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail=f"Client {client_id} not connected",
            )
        
        # Send message to client
        await active_connections[client_id].send_json(message)
        
        return {
            "status": "ok",
            "message": f"Message sent to {client_id}",
        }
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error sending message to {client_id}: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error sending message: {str(e)}",
        )

@router.get("/connections")
async def get_connections():
    """
    Get all active connections.
    
    Returns:
        List of active connections
    """
    return {
        "connections": list(active_connections.keys()),
        "count": len(active_connections),
    }

@router.post("/deploy/{client_id}")
async def deploy_model_to_client(client_id: str, model_id: str):
    """
    Deploy a model to a specific edge device.
    
    Args:
        client_id: Client ID
        model_id: Model ID
        
    Returns:
        Deployment status
    """
    try:
        # Check if client is connected
        if client_id not in active_connections:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail=f"Client {client_id} not connected",
            )
        
        # Send deployment message to client
        await active_connections[client_id].send_json({
            "type": "deploy",
            "model_id": model_id,
            "timestamp": asyncio.get_event_loop().time(),
        })
        
        return {
            "status": "ok",
            "message": f"Deployment initiated for {client_id}",
            "model_id": model_id,
        }
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error deploying model to {client_id}: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error deploying model: {str(e)}",
        )
