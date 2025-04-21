"""
SentinelWeb Backend - Telemetry Endpoints

This module provides endpoints for drone telemetry data.
"""

import asyncio
import json
import logging
from typing import Any, Dict, List, Optional

from fastapi import APIRouter, Depends, HTTPException, status, Request, WebSocket, WebSocketDisconnect
from pydantic import BaseModel

from backend.core.auth import get_current_user, User
from backend.services.sentinel_client import SentinelClient

router = APIRouter()
logger = logging.getLogger(__name__)

# Pydantic models
class TelemetryData(BaseModel):
    """Telemetry data model."""
    drone_id: str
    timestamp: str
    location: Dict[str, float]
    altitude: float
    heading: float
    speed: float
    battery_level: float
    status: str
    sensors: Optional[Dict[str, Any]] = None

class TelemetryHistoryRequest(BaseModel):
    """Telemetry history request model."""
    drone_id: str
    start_time: Optional[str] = None
    end_time: Optional[str] = None
    interval: Optional[str] = "1m"
    metrics: Optional[List[str]] = None

class TelemetryHistoryResponse(BaseModel):
    """Telemetry history response model."""
    drone_id: str
    start_time: str
    end_time: str
    interval: str
    metrics: List[str]
    data: List[Dict[str, Any]]

# WebSocket connection manager
class TelemetryConnectionManager:
    """Manager for WebSocket connections."""
    
    def __init__(self):
        """Initialize the connection manager."""
        self.active_connections: Dict[str, List[WebSocket]] = {}
        self.drone_subscribers: Dict[str, List[WebSocket]] = {}
    
    async def connect(self, websocket: WebSocket, client_id: str):
        """
        Connect a WebSocket client.
        
        Args:
            websocket: WebSocket connection
            client_id: Client ID
        """
        await websocket.accept()
        if client_id not in self.active_connections:
            self.active_connections[client_id] = []
        self.active_connections[client_id].append(websocket)
    
    def disconnect(self, websocket: WebSocket, client_id: str):
        """
        Disconnect a WebSocket client.
        
        Args:
            websocket: WebSocket connection
            client_id: Client ID
        """
        if client_id in self.active_connections:
            if websocket in self.active_connections[client_id]:
                self.active_connections[client_id].remove(websocket)
            if not self.active_connections[client_id]:
                del self.active_connections[client_id]
        
        # Remove from drone subscribers
        for drone_id, subscribers in self.drone_subscribers.items():
            if websocket in subscribers:
                subscribers.remove(websocket)
            if not subscribers:
                del self.drone_subscribers[drone_id]
    
    async def subscribe_to_drone(self, websocket: WebSocket, drone_id: str):
        """
        Subscribe a client to a drone's telemetry.
        
        Args:
            websocket: WebSocket connection
            drone_id: Drone ID
        """
        if drone_id not in self.drone_subscribers:
            self.drone_subscribers[drone_id] = []
        self.drone_subscribers[drone_id].append(websocket)
    
    async def unsubscribe_from_drone(self, websocket: WebSocket, drone_id: str):
        """
        Unsubscribe a client from a drone's telemetry.
        
        Args:
            websocket: WebSocket connection
            drone_id: Drone ID
        """
        if drone_id in self.drone_subscribers:
            if websocket in self.drone_subscribers[drone_id]:
                self.drone_subscribers[drone_id].remove(websocket)
            if not self.drone_subscribers[drone_id]:
                del self.drone_subscribers[drone_id]
    
    async def broadcast_to_drone_subscribers(self, drone_id: str, message: Any):
        """
        Broadcast a message to all subscribers of a drone.
        
        Args:
            drone_id: Drone ID
            message: Message to broadcast
        """
        if drone_id in self.drone_subscribers:
            disconnected_websockets = []
            for websocket in self.drone_subscribers[drone_id]:
                try:
                    await websocket.send_json(message)
                except Exception as e:
                    logger.error(f"Error sending message to WebSocket: {str(e)}")
                    disconnected_websockets.append(websocket)
            
            # Remove disconnected WebSockets
            for websocket in disconnected_websockets:
                if websocket in self.drone_subscribers[drone_id]:
                    self.drone_subscribers[drone_id].remove(websocket)
            
            if not self.drone_subscribers[drone_id]:
                del self.drone_subscribers[drone_id]

# Create connection manager
manager = TelemetryConnectionManager()

@router.get("/drones/{drone_id}")
async def get_drone_telemetry(
    drone_id: str,
    request: Request,
    current_user: User = Depends(get_current_user)
) -> Any:
    """
    Get current telemetry for a drone.
    
    Args:
        drone_id: Drone ID
        request: FastAPI request
        current_user: Current authenticated user
        
    Returns:
        Drone telemetry
    """
    # Get sentinel client from app state
    sentinel_client: SentinelClient = request.app.state.sentinel_client
    
    try:
        # Get telemetry from BuloCloudSentinel
        telemetry = await sentinel_client.get_telemetry(drone_id)
        return telemetry
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail=f"Error getting drone telemetry: {str(e)}"
        )

@router.post("/history", response_model=TelemetryHistoryResponse)
async def get_telemetry_history(
    request_data: TelemetryHistoryRequest,
    request: Request,
    current_user: User = Depends(get_current_user)
) -> Any:
    """
    Get historical telemetry for a drone.
    
    Args:
        request_data: Request data
        request: FastAPI request
        current_user: Current authenticated user
        
    Returns:
        Telemetry history
    """
    # This is a placeholder - the actual implementation would depend on BuloCloudSentinel's API
    # In a real implementation, you would query a time-series database or similar
    
    # Create dummy data
    data = []
    for i in range(10):
        data.append({
            "timestamp": f"2023-06-01T12:{i:02d}:00Z",
            "location": {
                "latitude": 47.6062 + (i * 0.001),
                "longitude": -122.3321 + (i * 0.001)
            },
            "altitude": 50 + (i * 5),
            "heading": 90 + (i * 10),
            "speed": 5 + (i * 0.5),
            "battery_level": 90 - (i * 2),
            "status": "flying"
        })
    
    return {
        "drone_id": request_data.drone_id,
        "start_time": request_data.start_time or "2023-06-01T12:00:00Z",
        "end_time": request_data.end_time or "2023-06-01T12:10:00Z",
        "interval": request_data.interval,
        "metrics": request_data.metrics or ["location", "altitude", "heading", "speed", "battery_level", "status"],
        "data": data
    }

@router.websocket("/ws/{client_id}")
async def websocket_endpoint(websocket: WebSocket, client_id: str):
    """
    WebSocket endpoint for real-time telemetry.
    
    Args:
        websocket: WebSocket connection
        client_id: Client ID
    """
    await manager.connect(websocket, client_id)
    
    try:
        while True:
            # Receive message from client
            data = await websocket.receive_text()
            message = json.loads(data)
            
            # Handle message
            if message.get("type") == "subscribe":
                drone_id = message.get("drone_id")
                if drone_id:
                    await manager.subscribe_to_drone(websocket, drone_id)
                    await websocket.send_json({
                        "type": "subscription",
                        "status": "success",
                        "drone_id": drone_id
                    })
            
            elif message.get("type") == "unsubscribe":
                drone_id = message.get("drone_id")
                if drone_id:
                    await manager.unsubscribe_from_drone(websocket, drone_id)
                    await websocket.send_json({
                        "type": "unsubscription",
                        "status": "success",
                        "drone_id": drone_id
                    })
            
            # Send ping to keep connection alive
            await websocket.send_json({"type": "ping"})
            
    except WebSocketDisconnect:
        manager.disconnect(websocket, client_id)
    except Exception as e:
        logger.error(f"WebSocket error: {str(e)}")
        manager.disconnect(websocket, client_id)

# Background task to simulate telemetry updates
async def telemetry_simulator():
    """Simulate telemetry updates for testing."""
    while True:
        # Simulate telemetry for drone1
        drone_id = "drone1"
        telemetry = {
            "type": "telemetry",
            "drone_id": drone_id,
            "timestamp": "2023-06-01T12:00:00Z",
            "location": {
                "latitude": 47.6062,
                "longitude": -122.3321
            },
            "altitude": 50,
            "heading": 90,
            "speed": 5,
            "battery_level": 90,
            "status": "flying"
        }
        
        # Broadcast to subscribers
        await manager.broadcast_to_drone_subscribers(drone_id, telemetry)
        
        # Wait before next update
        await asyncio.sleep(1)

# Start telemetry simulator in background
@router.on_event("startup")
async def startup_telemetry_simulator():
    """Start the telemetry simulator on startup."""
    asyncio.create_task(telemetry_simulator())
