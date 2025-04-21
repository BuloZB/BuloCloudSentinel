import asyncio
import json
import time
import uuid
from typing import Dict, Set, Any, List, Optional, Callable, Tuple
from datetime import datetime
from enum import Enum

from backend.mesh_networking.protocols import (
    Message, MessageType, MessagePriority, RoutingProtocol,
    RoutingTable, MessageHandler, DiscoveryProtocol
)
from backend.mesh_networking.security import MeshSecurity, SecureMessageWrapper


class NodeStatus(Enum):
    """Status of a mesh node."""
    INITIALIZING = "initializing"
    ACTIVE = "active"
    DEGRADED = "degraded"
    OFFLINE = "offline"


class NodeCapability(Enum):
    """Capabilities of a mesh node."""
    ROUTING = "routing"
    SENSOR = "sensor"
    CONTROL = "control"
    STORAGE = "storage"
    GATEWAY = "gateway"
    AI = "ai"


class MeshNode:
    """A node in the mesh network."""

    def __init__(self, node_id: str, capabilities: List[NodeCapability] = None):
        """Initialize a new mesh node."""
        self.node_id = node_id
        self.peers: Dict[str, Dict[str, Any]] = {}
        self.message_queue = asyncio.Queue()
        self.status = NodeStatus.INITIALIZING
        self.capabilities = capabilities or [NodeCapability.ROUTING]
        self.routing_table = RoutingTable()
        self.message_handler = MessageHandler()
        self.last_heartbeat = datetime.utcnow()
        self.metrics = {
            "messages_sent": 0,
            "messages_received": 0,
            "messages_forwarded": 0,
            "messages_dropped": 0
        }
        self.message_cache: Dict[str, Message] = {}  # Cache of recently seen messages
        self.max_cache_size = 1000

    async def send_message(self, peer_id: str, message: Message) -> bool:
        """Send a message to a peer."""
        if peer_id not in self.peers:
            return False

        # Add this node to the message path
        message.add_hop(self.node_id)

        # Update metrics
        self.metrics["messages_sent"] += 1

        # In a real implementation, this would send the message over a network
        # For now, we'll just put it in the peer's message queue
        peer = self.peers[peer_id]["node"]
        await peer.receive_message(message)
        return True

    async def receive_message(self, message: Message) -> None:
        """Receive a message from a peer."""
        # Update metrics
        self.metrics["messages_received"] += 1

        # Check if we've seen this message before (loop prevention)
        if message.id in self.message_cache:
            self.metrics["messages_dropped"] += 1
            return

        # Add to message cache
        self.message_cache[message.id] = message
        if len(self.message_cache) > self.max_cache_size:
            # Remove oldest messages if cache is full
            oldest_id = next(iter(self.message_cache))
            del self.message_cache[oldest_id]

        # Handle the message
        await self.message_handler.handle_message(message)

        # Put in queue for application layer
        await self.message_queue.put(message)

    async def forward_message(self, message: Message) -> None:
        """Forward a message to appropriate peers."""
        # Don't forward if message is expired or we've seen it before
        if message.is_expired() or self.node_id in message.hops:
            return

        # Update metrics
        self.metrics["messages_forwarded"] += 1

        # Determine target peers based on routing protocol
        if message.routing_protocol == RoutingProtocol.DIRECT:
            # Direct routing - send only to the target or next hop
            if message.target_id in self.peers:
                await self.send_message(message.target_id, message)
            else:
                next_hop = self.routing_table.get_next_hop(message.target_id)
                if next_hop:
                    await self.send_message(next_hop, message)

        elif message.routing_protocol == RoutingProtocol.FLOODING:
            # Flooding - send to all peers except those already in the path
            for peer_id in self.peers:
                if peer_id not in message.hops:
                    await self.send_message(peer_id, message)

        elif message.routing_protocol == RoutingProtocol.SPANNING_TREE:
            # Spanning tree - send only to parent and children
            # This would require a spanning tree to be built
            pass

        elif message.routing_protocol == RoutingProtocol.GOSSIP:
            # Gossip - send to a random subset of peers
            peers_to_forward = min(3, len(self.peers))  # Forward to at most 3 peers
            if peers_to_forward > 0:
                random_peers = list(self.peers.keys())
                import random
                random.shuffle(random_peers)
                for peer_id in random_peers[:peers_to_forward]:
                    if peer_id not in message.hops:
                        await self.send_message(peer_id, message)

    def add_peer(self, peer_id: str, peer_node: 'MeshNode', metadata: Dict[str, Any] = None) -> None:
        """Add a peer to this node."""
        self.peers[peer_id] = {
            "node": peer_node,
            "connected_since": datetime.utcnow(),
            "metadata": metadata or {},
            "status": "active"
        }

        # Update routing table
        self.routing_table.add_route(peer_id, peer_id, 1)  # Direct connection

    def remove_peer(self, peer_id: str) -> None:
        """Remove a peer from this node."""
        if peer_id in self.peers:
            del self.peers[peer_id]

            # Update routing table
            self.routing_table.remove_route(peer_id)

    def get_peers(self) -> Dict[str, Dict[str, Any]]:
        """Get all peers of this node."""
        # Return a copy without the actual node objects
        result = {}
        for peer_id, peer_info in self.peers.items():
            result[peer_id] = {
                "connected_since": peer_info["connected_since"],
                "metadata": peer_info["metadata"],
                "status": peer_info["status"]
            }
        return result

    def update_status(self) -> None:
        """Update the status of this node based on peer connections."""
        if not self.peers:
            self.status = NodeStatus.DEGRADED
        else:
            self.status = NodeStatus.ACTIVE

    def get_metrics(self) -> Dict[str, Any]:
        """Get metrics for this node."""
        return {
            **self.metrics,
            "peer_count": len(self.peers),
            "status": self.status.value,
            "last_heartbeat": self.last_heartbeat.isoformat()
        }

    def register_message_handler(self, message_type: MessageType, handler: Callable) -> None:
        """Register a handler for a specific message type."""
        self.message_handler.register_handler(message_type, handler)


class MeshNetwork:
    """A mesh network of interconnected nodes."""

    def __init__(self, security_enabled: bool = True):
        """Initialize a new mesh network."""
        self.nodes: Dict[str, MeshNode] = {}
        self.security_enabled = security_enabled
        self.security = MeshSecurity() if security_enabled else None
        self.secure_wrapper = SecureMessageWrapper(self.security) if security_enabled else None
        self.discovery_protocols: Dict[str, DiscoveryProtocol] = {}
        self.network_metrics = {
            "node_count": 0,
            "connection_count": 0,
            "messages_total": 0,
            "start_time": datetime.utcnow().isoformat()
        }

    def add_node(self, node_id: str, capabilities: List[NodeCapability] = None) -> MeshNode:
        """Add a node to the network."""
        if node_id in self.nodes:
            return self.nodes[node_id]

        node = MeshNode(node_id, capabilities)
        self.nodes[node_id] = node

        # Create discovery protocol for this node
        self.discovery_protocols[node_id] = DiscoveryProtocol(
            node_id=node_id,
            broadcast_func=lambda msg: self.broadcast(node_id, msg)
        )

        # Register discovery message handler
        node.register_message_handler(
            MessageType.DISCOVERY,
            lambda msg: self.discovery_protocols[node_id].handle_discovery_message(msg)
        )

        # Update metrics
        self.network_metrics["node_count"] = len(self.nodes)

        return node

    def remove_node(self, node_id: str) -> None:
        """Remove a node from the network."""
        if node_id not in self.nodes:
            return

        # Disconnect from all peers
        node = self.nodes[node_id]
        peers = list(node.peers.keys())
        for peer_id in peers:
            self.disconnect_nodes(node_id, peer_id)

        # Remove discovery protocol
        if node_id in self.discovery_protocols:
            del self.discovery_protocols[node_id]

        # Remove node
        del self.nodes[node_id]

        # Update metrics
        self.network_metrics["node_count"] = len(self.nodes)

    def connect_nodes(self, node1_id: str, node2_id: str, metadata: Dict[str, Any] = None) -> bool:
        """Connect two nodes in the network."""
        if node1_id not in self.nodes or node2_id not in self.nodes:
            return False

        node1 = self.nodes[node1_id]
        node2 = self.nodes[node2_id]

        # Add peers
        node1.add_peer(node2_id, node2, metadata)
        node2.add_peer(node1_id, node1, metadata)

        # Update metrics
        self.network_metrics["connection_count"] += 1

        return True

    def disconnect_nodes(self, node1_id: str, node2_id: str) -> bool:
        """Disconnect two nodes in the network."""
        if node1_id not in self.nodes or node2_id not in self.nodes:
            return False

        node1 = self.nodes[node1_id]
        node2 = self.nodes[node2_id]

        # Remove peers
        node1.remove_peer(node2_id)
        node2.remove_peer(node1_id)

        # Update metrics
        self.network_metrics["connection_count"] -= 1

        return True

    async def broadcast(self, sender_id: str, message: Message) -> None:
        """Broadcast a message from a sender to all its peers."""
        if sender_id not in self.nodes:
            return

        sender = self.nodes[sender_id]

        # Apply security if enabled
        if self.security_enabled and self.secure_wrapper:
            # In a real implementation, we would encrypt the message here
            pass

        # Forward to all peers
        await sender.forward_message(message)

        # Update metrics
        self.network_metrics["messages_total"] += 1

    async def send_direct_message(self, sender_id: str, target_id: str, content: Any,
                                 message_type: MessageType = MessageType.DATA,
                                 priority: MessagePriority = MessagePriority.NORMAL) -> bool:
        """Send a direct message from a sender to a target."""
        if sender_id not in self.nodes:
            return False

        sender = self.nodes[sender_id]

        # Create message
        message = Message(
            sender_id=sender_id,
            message_type=message_type,
            content=content,
            target_id=target_id,
            priority=priority,
            routing_protocol=RoutingProtocol.DIRECT
        )

        # Apply security if enabled
        if self.security_enabled and self.secure_wrapper:
            # In a real implementation, we would encrypt the message here
            pass

        # Send directly if target is a peer
        if target_id in sender.peers:
            return await sender.send_message(target_id, message)

        # Otherwise, try to route through the network
        next_hop = sender.routing_table.get_next_hop(target_id)
        if next_hop:
            return await sender.send_message(next_hop, message)

        # If no route, try flooding as a last resort
        message.routing_protocol = RoutingProtocol.FLOODING
        await sender.forward_message(message)

        # Update metrics
        self.network_metrics["messages_total"] += 1

        return True

    def get_node(self, node_id: str) -> Optional[MeshNode]:
        """Get a node by ID."""
        return self.nodes.get(node_id)

    def get_all_nodes(self) -> Dict[str, MeshNode]:
        """Get all nodes in the network."""
        return self.nodes

    def get_network_topology(self) -> Dict[str, Any]:
        """Get the network topology."""
        nodes = {}
        connections = []

        for node_id, node in self.nodes.items():
            nodes[node_id] = {
                "status": node.status.value,
                "capabilities": [cap.value for cap in node.capabilities]
            }

            for peer_id in node.peers:
                if node_id < peer_id:  # Avoid duplicates
                    connections.append({
                        "source": node_id,
                        "target": peer_id
                    })

        return {
            "nodes": nodes,
            "connections": connections
        }

    def get_network_metrics(self) -> Dict[str, Any]:
        """Get network-wide metrics."""
        # Aggregate node metrics
        messages_sent = 0
        messages_received = 0
        messages_forwarded = 0
        messages_dropped = 0

        for node in self.nodes.values():
            metrics = node.metrics
            messages_sent += metrics["messages_sent"]
            messages_received += metrics["messages_received"]
            messages_forwarded += metrics["messages_forwarded"]
            messages_dropped += metrics["messages_dropped"]

        return {
            **self.network_metrics,
            "messages_sent": messages_sent,
            "messages_received": messages_received,
            "messages_forwarded": messages_forwarded,
            "messages_dropped": messages_dropped,
            "uptime_seconds": (datetime.utcnow() - datetime.fromisoformat(self.network_metrics["start_time"])).total_seconds()
        }

    async def start_discovery(self, node_id: str) -> bool:
        """Start the discovery process for a node."""
        if node_id not in self.discovery_protocols:
            return False

        await self.discovery_protocols[node_id].start_discovery()
        return True

    def get_discovered_nodes(self, node_id: str) -> Dict[str, Dict[str, Any]]:
        """Get nodes discovered by a specific node."""
        if node_id not in self.discovery_protocols:
            return {}

        return self.discovery_protocols[node_id].get_discovered_nodes()
