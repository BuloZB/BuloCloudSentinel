"""
Mesh networking protocols for Bulo.Cloud Sentinel Tactical Use Module.

This module provides various protocols for mesh networking communication.
"""

import asyncio
import json
import uuid
import time
import random
from typing import Dict, List, Any, Optional, Set, Tuple, Callable
from enum import Enum
from datetime import datetime


class MessageType(Enum):
    """Types of messages in the mesh network."""
    DATA = "data"
    CONTROL = "control"
    DISCOVERY = "discovery"
    HEARTBEAT = "heartbeat"
    ROUTING = "routing"
    ACKNOWLEDGMENT = "ack"


class MessagePriority(Enum):
    """Priority levels for messages."""
    LOW = 0
    NORMAL = 1
    HIGH = 2
    CRITICAL = 3


class RoutingProtocol(Enum):
    """Supported routing protocols."""
    FLOODING = "flooding"
    DIRECT = "direct"
    SPANNING_TREE = "spanning_tree"
    GOSSIP = "gossip"


class Message:
    """
    Message class for mesh network communication.
    """
    
    def __init__(
        self,
        sender_id: str,
        message_type: MessageType,
        content: Any,
        target_id: Optional[str] = None,
        priority: MessagePriority = MessagePriority.NORMAL,
        ttl: int = 10,
        routing_protocol: RoutingProtocol = RoutingProtocol.FLOODING
    ):
        """
        Initialize a new message.
        
        Args:
            sender_id: ID of the sending node
            message_type: Type of message
            content: Message content
            target_id: Optional target node ID (None for broadcast)
            priority: Message priority
            ttl: Time-to-live (hop count)
            routing_protocol: Routing protocol to use
        """
        self.id = str(uuid.uuid4())
        self.sender_id = sender_id
        self.message_type = message_type
        self.content = content
        self.target_id = target_id
        self.priority = priority
        self.ttl = ttl
        self.timestamp = datetime.utcnow().isoformat()
        self.routing_protocol = routing_protocol
        self.hops = []
        self.acknowledged = False
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert message to dictionary for serialization."""
        return {
            "id": self.id,
            "sender_id": self.sender_id,
            "message_type": self.message_type.value,
            "content": self.content,
            "target_id": self.target_id,
            "priority": self.priority.value,
            "ttl": self.ttl,
            "timestamp": self.timestamp,
            "routing_protocol": self.routing_protocol.value,
            "hops": self.hops,
            "acknowledged": self.acknowledged
        }
    
    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'Message':
        """Create message from dictionary."""
        msg = cls(
            sender_id=data["sender_id"],
            message_type=MessageType(data["message_type"]),
            content=data["content"],
            target_id=data.get("target_id"),
            priority=MessagePriority(data["priority"]),
            ttl=data["ttl"],
            routing_protocol=RoutingProtocol(data["routing_protocol"])
        )
        msg.id = data["id"]
        msg.timestamp = data["timestamp"]
        msg.hops = data["hops"]
        msg.acknowledged = data["acknowledged"]
        return msg
    
    def add_hop(self, node_id: str) -> None:
        """Add a hop to the message path."""
        self.hops.append(node_id)
        self.ttl -= 1
    
    def is_expired(self) -> bool:
        """Check if the message has expired (TTL <= 0)."""
        return self.ttl <= 0
    
    def should_forward(self, node_id: str) -> bool:
        """Check if the message should be forwarded by this node."""
        # Don't forward if expired
        if self.is_expired():
            return False
        
        # Don't forward if already seen (loop prevention)
        if node_id in self.hops:
            return False
        
        # Direct messages only go to target
        if self.routing_protocol == RoutingProtocol.DIRECT:
            return self.target_id == node_id
        
        # Flooding goes to everyone
        if self.routing_protocol == RoutingProtocol.FLOODING:
            return True
        
        # Other protocols would have more complex logic
        return True


class RoutingTable:
    """
    Routing table for mesh network nodes.
    """
    
    def __init__(self):
        """Initialize an empty routing table."""
        self.routes: Dict[str, Dict[str, Any]] = {}
        self.last_updated = datetime.utcnow()
    
    def add_route(self, target_id: str, next_hop: str, metric: int) -> None:
        """
        Add or update a route in the routing table.
        
        Args:
            target_id: Destination node ID
            next_hop: Next hop node ID
            metric: Route metric (e.g., hop count)
        """
        self.routes[target_id] = {
            "next_hop": next_hop,
            "metric": metric,
            "last_updated": datetime.utcnow()
        }
        self.last_updated = datetime.utcnow()
    
    def get_next_hop(self, target_id: str) -> Optional[str]:
        """
        Get the next hop for a target node.
        
        Args:
            target_id: Destination node ID
        
        Returns:
            Next hop node ID or None if no route exists
        """
        route = self.routes.get(target_id)
        if route:
            return route["next_hop"]
        return None
    
    def remove_route(self, target_id: str) -> None:
        """
        Remove a route from the routing table.
        
        Args:
            target_id: Destination node ID
        """
        if target_id in self.routes:
            del self.routes[target_id]
            self.last_updated = datetime.utcnow()
    
    def get_all_routes(self) -> Dict[str, Dict[str, Any]]:
        """Get all routes in the routing table."""
        return self.routes
    
    def clear_stale_routes(self, max_age_seconds: int = 300) -> int:
        """
        Clear stale routes from the routing table.
        
        Args:
            max_age_seconds: Maximum age of routes in seconds
        
        Returns:
            Number of routes cleared
        """
        now = datetime.utcnow()
        stale_routes = []
        
        for target_id, route in self.routes.items():
            last_updated = route["last_updated"]
            age = (now - last_updated).total_seconds()
            if age > max_age_seconds:
                stale_routes.append(target_id)
        
        for target_id in stale_routes:
            self.remove_route(target_id)
        
        return len(stale_routes)


class MessageHandler:
    """
    Handler for processing mesh network messages.
    """
    
    def __init__(self):
        """Initialize a new message handler."""
        self.handlers: Dict[MessageType, List[Callable]] = {
            message_type: [] for message_type in MessageType
        }
    
    def register_handler(self, message_type: MessageType, handler: Callable) -> None:
        """
        Register a handler for a specific message type.
        
        Args:
            message_type: Type of message to handle
            handler: Handler function
        """
        self.handlers[message_type].append(handler)
    
    async def handle_message(self, message: Message) -> None:
        """
        Handle a message by calling all registered handlers.
        
        Args:
            message: Message to handle
        """
        for handler in self.handlers[message.message_type]:
            try:
                await handler(message)
            except Exception as e:
                print(f"Error in message handler: {str(e)}")


class DiscoveryProtocol:
    """
    Protocol for node discovery in the mesh network.
    """
    
    def __init__(self, node_id: str, broadcast_func):
        """
        Initialize the discovery protocol.
        
        Args:
            node_id: ID of the local node
            broadcast_func: Function to broadcast messages
        """
        self.node_id = node_id
        self.broadcast = broadcast_func
        self.discovered_nodes: Dict[str, Dict[str, Any]] = {}
        self.last_discovery = datetime.utcnow()
    
    async def start_discovery(self) -> None:
        """Start the discovery process."""
        discovery_message = Message(
            sender_id=self.node_id,
            message_type=MessageType.DISCOVERY,
            content={"capabilities": ["mesh", "data", "control"]},
            routing_protocol=RoutingProtocol.FLOODING,
            ttl=5
        )
        await self.broadcast(discovery_message)
        self.last_discovery = datetime.utcnow()
    
    def handle_discovery_message(self, message: Message) -> None:
        """
        Handle a discovery message from another node.
        
        Args:
            message: Discovery message
        """
        sender_id = message.sender_id
        if sender_id != self.node_id:
            self.discovered_nodes[sender_id] = {
                "last_seen": datetime.utcnow(),
                "capabilities": message.content.get("capabilities", []),
                "hops": len(message.hops)
            }
    
    def get_discovered_nodes(self) -> Dict[str, Dict[str, Any]]:
        """Get all discovered nodes."""
        return self.discovered_nodes
    
    def clear_stale_nodes(self, max_age_seconds: int = 300) -> int:
        """
        Clear stale nodes from the discovered nodes list.
        
        Args:
            max_age_seconds: Maximum age of nodes in seconds
        
        Returns:
            Number of nodes cleared
        """
        now = datetime.utcnow()
        stale_nodes = []
        
        for node_id, node_info in self.discovered_nodes.items():
            last_seen = node_info["last_seen"]
            age = (now - last_seen).total_seconds()
            if age > max_age_seconds:
                stale_nodes.append(node_id)
        
        for node_id in stale_nodes:
            del self.discovered_nodes[node_id]
        
        return len(stale_nodes)
