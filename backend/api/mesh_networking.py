from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from typing import List, Optional
from backend.mesh_networking.mesh_network import MeshNetwork
import asyncio

router = APIRouter()
mesh_network = MeshNetwork()

class NodeModel(BaseModel):
    node_id: str

class ConnectNodesRequest(BaseModel):
    node1_id: str
    node2_id: str

class MessageModel(BaseModel):
    sender_id: str
    message: str

@router.post("/mesh/nodes")
async def add_node(node: NodeModel):
    mesh_network.add_node(node.node_id)
    return {"detail": f"Node {node.node_id} added"}

@router.delete("/mesh/nodes/{node_id}")
async def remove_node(node_id: str):
    mesh_network.remove_node(node_id)
    return {"detail": f"Node {node_id} removed"}

@router.post("/mesh/connect")
async def connect_nodes(request: ConnectNodesRequest):
    mesh_network.connect_nodes(request.node1_id, request.node2_id)
    return {"detail": f"Nodes {request.node1_id} and {request.node2_id} connected"}

@router.post("/mesh/disconnect")
async def disconnect_nodes(request: ConnectNodesRequest):
    mesh_network.disconnect_nodes(request.node1_id, request.node2_id)
    return {"detail": f"Nodes {request.node1_id} and {request.node2_id} disconnected"}

@router.get("/mesh/nodes/{node_id}/messages")
async def get_messages(node_id: str):
    node = mesh_network.nodes.get(node_id)
    if not node:
        raise HTTPException(status_code=404, detail="Node not found")
    messages = []
    while not node.message_queue.empty():
        messages.append(await node.message_queue.get())
    return {"messages": messages}

@router.post("/mesh/nodes/{node_id}/broadcast")
async def broadcast_message(node_id: str, message: MessageModel):
    await mesh_network.broadcast(node_id, message.message)
    return {"detail": "Message broadcasted"}
