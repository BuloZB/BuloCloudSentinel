"""
Mesh manager service for the SentinelBeacon module.

This service is responsible for managing the Meshtastic mesh network, including:
- Connecting to the Meshtastic device
- Configuring the device
- Managing channels
- Sending and receiving messages
- Handling position updates
"""

import asyncio
import logging
import json
import time
from datetime import datetime
from typing import Dict, List, Optional, Any, Callable

import meshtastic
import meshtastic.tcp_interface
import meshtastic.serial_interface
from meshtastic.mesh_pb2 import User, Position, Data
from meshtastic.__init__ import BROADCAST_ADDR

from core.config import settings

logger = logging.getLogger(__name__)

class MeshManager:
    """Mesh manager service."""
    
    def __init__(self):
        """Initialize the mesh manager."""
        self.interface = None
        self.connected = False
        self.node_info = None
        self.message_callbacks = []
        self.position_callbacks = []
        self.node_callbacks = []
        self.running = False
        self.message_queue = asyncio.Queue()
        self.position_task = None
        self.message_task = None
    
    async def connect(self):
        """Connect to the Meshtastic device."""
        logger.info(f"Connecting to Meshtastic device at {settings.MESHTASTIC_DEVICE}")
        
        try:
            # Connect to the device
            if settings.MESHTASTIC_DEVICE.startswith("tcp:"):
                # TCP connection (for remote devices)
                host = settings.MESHTASTIC_DEVICE[4:]
                self.interface = meshtastic.tcp_interface.TCPInterface(host)
            else:
                # Serial connection (for local devices)
                self.interface = meshtastic.serial_interface.SerialInterface(
                    settings.MESHTASTIC_DEVICE,
                    settings.MESHTASTIC_BAUDRATE
                )
            
            # Get node info
            self.node_info = self.interface.getMyNodeInfo()
            
            # Set up message callback
            self.interface.onReceive = self._on_message_received
            
            # Set up node callback
            self.interface.onNodeUpdated = self._on_node_updated
            
            # Configure the device
            await self._configure_device()
            
            # Start position broadcasting task
            self.position_task = asyncio.create_task(self._position_broadcast_task())
            
            # Start message processing task
            self.message_task = asyncio.create_task(self._message_processing_task())
            
            self.connected = True
            self.running = True
            
            logger.info(f"Connected to Meshtastic device: {self.node_info.my_node_num}")
            
            return True
        
        except Exception as e:
            logger.error(f"Failed to connect to Meshtastic device: {e}")
            return False
    
    async def disconnect(self):
        """Disconnect from the Meshtastic device."""
        logger.info("Disconnecting from Meshtastic device")
        
        self.running = False
        
        # Cancel position broadcasting task
        if self.position_task:
            self.position_task.cancel()
            try:
                await self.position_task
            except asyncio.CancelledError:
                pass
        
        # Cancel message processing task
        if self.message_task:
            self.message_task.cancel()
            try:
                await self.message_task
            except asyncio.CancelledError:
                pass
        
        # Close the interface
        if self.interface:
            self.interface.close()
            self.interface = None
        
        self.connected = False
        
        logger.info("Disconnected from Meshtastic device")
    
    async def send_message(
        self,
        text: str,
        to_node_id: Optional[str] = None,
        channel_name: Optional[str] = None,
        priority: str = "normal",
        want_ack: bool = True
    ) -> Optional[str]:
        """
        Send a message to a node or broadcast.
        
        Args:
            text: Message text
            to_node_id: Destination node ID (None for broadcast)
            channel_name: Channel name (None for default channel)
            priority: Message priority (low, normal, high, emergency)
            want_ack: Whether to request acknowledgment
            
        Returns:
            Message ID if successful, None otherwise
        """
        if not self.connected:
            logger.error("Not connected to Meshtastic device")
            return None
        
        try:
            # Convert node ID to integer if provided
            dest_id = int(to_node_id, 16) if to_node_id else BROADCAST_ADDR
            
            # Get channel index
            channel_index = 0  # Default channel
            if channel_name:
                for i, channel in enumerate(self.interface.getChannels()):
                    if channel.settings.name == channel_name:
                        channel_index = i
                        break
            
            # Set message priority
            priority_map = {
                "low": Data.Priority.LOW,
                "normal": Data.Priority.NORMAL,
                "high": Data.Priority.HIGH,
                "emergency": Data.Priority.EMERGENCY
            }
            msg_priority = priority_map.get(priority, Data.Priority.NORMAL)
            
            # Send message
            message_id = self.interface.sendText(
                text=text,
                destinationId=dest_id,
                channelIndex=channel_index,
                wantAck=want_ack,
                wantResponse=False,
                priority=msg_priority
            )
            
            logger.info(f"Sent message to {to_node_id or 'broadcast'} on channel {channel_name or 'default'}: {text}")
            
            return message_id
        
        except Exception as e:
            logger.error(f"Failed to send message: {e}")
            return None
    
    async def get_nodes(self) -> List[Dict[str, Any]]:
        """
        Get all nodes in the mesh network.
        
        Returns:
            List of nodes
        """
        if not self.connected:
            logger.error("Not connected to Meshtastic device")
            return []
        
        try:
            nodes = []
            
            for node_id, node in self.interface.nodes.items():
                node_data = {
                    "id": f"{node_id:08x}",
                    "name": node.user.longName if node.user else f"Node {node_id:08x}",
                    "short_name": node.user.shortName if node.user else f"N{node_id:08x}",
                    "hardware": node.user.hwModel if node.user else "unknown",
                    "last_seen": node.lastHeard,
                    "battery_level": node.deviceMetrics.batteryLevel if node.deviceMetrics else None,
                    "voltage": node.deviceMetrics.voltage if node.deviceMetrics else None,
                    "signal_strength": node.snr,
                    "neighbors": [f"{n:08x}" for n in node.neighbors.keys()] if hasattr(node, "neighbors") else [],
                    "position": None
                }
                
                # Add position if available
                if node.position:
                    node_data["position"] = {
                        "latitude": node.position.latitude,
                        "longitude": node.position.longitude,
                        "altitude": node.position.altitude,
                        "time": node.position.time
                    }
                
                nodes.append(node_data)
            
            return nodes
        
        except Exception as e:
            logger.error(f"Failed to get nodes: {e}")
            return []
    
    async def get_channels(self) -> List[Dict[str, Any]]:
        """
        Get all channels in the mesh network.
        
        Returns:
            List of channels
        """
        if not self.connected:
            logger.error("Not connected to Meshtastic device")
            return []
        
        try:
            channels = []
            
            for i, channel in enumerate(self.interface.getChannels()):
                channel_data = {
                    "index": i,
                    "name": channel.settings.name,
                    "role": str(channel.role),
                    "enabled": channel.role != meshtastic.mesh_pb2.Channel.Role.DISABLED,
                    "encrypted": channel.settings.psk is not None
                }
                
                channels.append(channel_data)
            
            return channels
        
        except Exception as e:
            logger.error(f"Failed to get channels: {e}")
            return []
    
    async def set_position(self, latitude: float, longitude: float, altitude: float) -> bool:
        """
        Set the device's position.
        
        Args:
            latitude: Latitude in degrees
            longitude: Longitude in degrees
            altitude: Altitude in meters
            
        Returns:
            True if successful, False otherwise
        """
        if not self.connected:
            logger.error("Not connected to Meshtastic device")
            return False
        
        try:
            # Create position packet
            position = Position(
                latitude=latitude,
                longitude=longitude,
                altitude=int(altitude),
                time=int(time.time())
            )
            
            # Send position
            self.interface.sendPosition(position)
            
            logger.info(f"Set position: lat={latitude}, lon={longitude}, alt={altitude}")
            
            return True
        
        except Exception as e:
            logger.error(f"Failed to set position: {e}")
            return False
    
    def add_message_callback(self, callback: Callable[[Dict[str, Any]], None]):
        """
        Add a callback for received messages.
        
        Args:
            callback: Callback function
        """
        self.message_callbacks.append(callback)
    
    def add_position_callback(self, callback: Callable[[Dict[str, Any]], None]):
        """
        Add a callback for position updates.
        
        Args:
            callback: Callback function
        """
        self.position_callbacks.append(callback)
    
    def add_node_callback(self, callback: Callable[[Dict[str, Any]], None]):
        """
        Add a callback for node updates.
        
        Args:
            callback: Callback function
        """
        self.node_callbacks.append(callback)
    
    async def _configure_device(self):
        """Configure the Meshtastic device."""
        logger.info("Configuring Meshtastic device")
        
        try:
            # Set user info
            self.interface.setOwner(settings.MESHTASTIC_NODE_NAME)
            
            # Set region
            self.interface.setConfig("lora", {"region": settings.MESHTASTIC_REGION})
            
            # Set modem config
            self.interface.setModemConfig(settings.MESHTASTIC_MODEM_CONFIG)
            
            # Set transmit power
            self.interface.setConfig("lora", {"tx_power": settings.MESHTASTIC_TX_POWER})
            
            # Set position broadcast interval
            self.interface.setConfig("position", {"position_broadcast_secs": settings.MESHTASTIC_POSITION_BROADCAST_SECS})
            
            # Set Bluetooth enabled/disabled
            self.interface.setConfig("device", {"bluetooth_enabled": settings.MESHTASTIC_BLUETOOTH_ENABLED})
            
            # Configure primary channel
            primary_channel = {
                "name": settings.MESHTASTIC_CHANNEL_NAME,
                "psk": settings.MESHTASTIC_CHANNEL_PSK,
                "role": meshtastic.mesh_pb2.Channel.Role.PRIMARY
            }
            self.interface.setChannel(0, primary_channel)
            
            # Save configuration
            self.interface.writeConfig()
            
            logger.info("Meshtastic device configured successfully")
        
        except Exception as e:
            logger.error(f"Failed to configure Meshtastic device: {e}")
            raise
    
    def _on_message_received(self, packet, interface):
        """
        Handle received messages.
        
        Args:
            packet: Received packet
            interface: Meshtastic interface
        """
        try:
            # Check if this is a data packet
            if packet.get("decoded", {}).get("portnum") == "TEXT_MESSAGE_APP":
                message = {
                    "id": packet.get("id"),
                    "from_id": f"{packet.get('fromId'):08x}",
                    "to_id": f"{packet.get('toId'):08x}" if packet.get("toId") != BROADCAST_ADDR else None,
                    "channel_index": packet.get("channel", 0),
                    "text": packet.get("decoded", {}).get("text", ""),
                    "timestamp": datetime.utcnow().isoformat(),
                    "hop_limit": packet.get("hopLimit", 0),
                    "hop_start": packet.get("hopStart", 0),
                    "want_ack": packet.get("wantAck", False),
                    "priority": str(packet.get("priority", "normal")).lower()
                }
                
                # Add to message queue for processing
                asyncio.create_task(self.message_queue.put(message))
        
        except Exception as e:
            logger.error(f"Error handling received message: {e}")
    
    def _on_node_updated(self, node):
        """
        Handle node updates.
        
        Args:
            node: Updated node
        """
        try:
            # Create node data
            node_data = {
                "id": f"{node.nodeId:08x}",
                "name": node.user.longName if node.user else f"Node {node.nodeId:08x}",
                "short_name": node.user.shortName if node.user else f"N{node.nodeId:08x}",
                "hardware": node.user.hwModel if node.user else "unknown",
                "last_seen": node.lastHeard,
                "battery_level": node.deviceMetrics.batteryLevel if node.deviceMetrics else None,
                "voltage": node.deviceMetrics.voltage if node.deviceMetrics else None,
                "signal_strength": node.snr,
                "neighbors": [f"{n:08x}" for n in node.neighbors.keys()] if hasattr(node, "neighbors") else [],
                "position": None
            }
            
            # Add position if available
            if node.position:
                node_data["position"] = {
                    "latitude": node.position.latitude,
                    "longitude": node.position.longitude,
                    "altitude": node.position.altitude,
                    "time": node.position.time
                }
                
                # Call position callbacks
                for callback in self.position_callbacks:
                    try:
                        callback(node_data["position"])
                    except Exception as e:
                        logger.error(f"Error in position callback: {e}")
            
            # Call node callbacks
            for callback in self.node_callbacks:
                try:
                    callback(node_data)
                except Exception as e:
                    logger.error(f"Error in node callback: {e}")
        
        except Exception as e:
            logger.error(f"Error handling node update: {e}")
    
    async def _position_broadcast_task(self):
        """Task for broadcasting position updates."""
        logger.info("Starting position broadcast task")
        
        try:
            while self.running:
                # Wait for the broadcast interval
                await asyncio.sleep(settings.POSITION_BROADCAST_INTERVAL)
                
                # Skip if not connected
                if not self.connected:
                    continue
                
                # Get position from drone interface (to be implemented)
                # For now, we'll just use a fixed position
                latitude = 0.0
                longitude = 0.0
                altitude = 0.0
                
                # Set position
                await self.set_position(latitude, longitude, altitude)
        
        except asyncio.CancelledError:
            logger.info("Position broadcast task cancelled")
            raise
        
        except Exception as e:
            logger.error(f"Error in position broadcast task: {e}")
    
    async def _message_processing_task(self):
        """Task for processing received messages."""
        logger.info("Starting message processing task")
        
        try:
            while self.running:
                # Get message from queue
                message = await self.message_queue.get()
                
                # Call message callbacks
                for callback in self.message_callbacks:
                    try:
                        callback(message)
                    except Exception as e:
                        logger.error(f"Error in message callback: {e}")
                
                # Mark message as processed
                self.message_queue.task_done()
        
        except asyncio.CancelledError:
            logger.info("Message processing task cancelled")
            raise
        
        except Exception as e:
            logger.error(f"Error in message processing task: {e}")
    
    async def get_network_info(self) -> Dict[str, Any]:
        """
        Get information about the mesh network.
        
        Returns:
            Network information
        """
        if not self.connected:
            logger.error("Not connected to Meshtastic device")
            return {}
        
        try:
            # Get nodes
            nodes = await self.get_nodes()
            
            # Get channels
            channels = await self.get_channels()
            
            # Get local node info
            local_node = {
                "id": f"{self.node_info.my_node_num:08x}",
                "name": self.node_info.my_user.longName,
                "short_name": self.node_info.my_user.shortName,
                "hardware": self.node_info.my_user.hwModel,
                "firmware_version": self.node_info.firmware_version,
                "region": self.interface.localConfig.lora.region,
                "modem_config": self.interface.getModemConfig(),
                "tx_power": self.interface.localConfig.lora.tx_power
            }
            
            # Create network info
            network_info = {
                "local_node": local_node,
                "nodes": nodes,
                "channels": channels,
                "node_count": len(nodes),
                "channel_count": len(channels),
                "timestamp": datetime.utcnow().isoformat()
            }
            
            return network_info
        
        except Exception as e:
            logger.error(f"Failed to get network info: {e}")
            return {}
