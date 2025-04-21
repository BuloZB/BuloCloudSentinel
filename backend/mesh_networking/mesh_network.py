import asyncio
from typing import Dict, Set, Any

class MeshNode:
    def __init__(self, node_id: str):
        self.node_id = node_id
        self.peers: Set[str] = set()
        self.message_queue = asyncio.Queue()

    async def send_message(self, peer_id: str, message: Any):
        # Placeholder: send message to peer
        pass

    async def receive_message(self):
        return await self.message_queue.get()

    def add_peer(self, peer_id: str):
        self.peers.add(peer_id)

    def remove_peer(self, peer_id: str):
        self.peers.discard(peer_id)

class MeshNetwork:
    def __init__(self):
        self.nodes: Dict[str, MeshNode] = {}

    def add_node(self, node_id: str):
        if node_id not in self.nodes:
            self.nodes[node_id] = MeshNode(node_id)

    def remove_node(self, node_id: str):
        if node_id in self.nodes:
            del self.nodes[node_id]
            for node in self.nodes.values():
                node.remove_peer(node_id)

    def connect_nodes(self, node1_id: str, node2_id: str):
        if node1_id in self.nodes and node2_id in self.nodes:
            self.nodes[node1_id].add_peer(node2_id)
            self.nodes[node2_id].add_peer(node1_id)

    def disconnect_nodes(self, node1_id: str, node2_id: str):
        if node1_id in self.nodes and node2_id in self.nodes:
            self.nodes[node1_id].remove_peer(node2_id)
            self.nodes[node2_id].remove_peer(node1_id)

    async def broadcast(self, sender_id: str, message: Any):
        if sender_id not in self.nodes:
            return
        sender = self.nodes[sender_id]
        for peer_id in sender.peers:
            peer = self.nodes.get(peer_id)
            if peer:
                await peer.message_queue.put((sender_id, message))
