# SentinelBeacon Module

## Overview
SentinelBeacon is a specialized drone-based module for the Bulo.Cloud Sentinel platform that serves as an aerial relay node for Meshtastic mesh networks. It extends the range and coverage of ground-based Meshtastic devices by providing an elevated position for signal propagation.

## 2. Description & Benefits
The SentinelBeacon module provides a robust, decentralized mesh communication network for drones and ground stations using the Meshtastic protocol. This module enables reliable communication in environments where traditional communication methods may be unreliable or unavailable, such as remote areas, urban canyons, or during electronic warfare operations.

### Key Features:
- **Mesh Networking**: Create a self-healing, decentralized communication network between drones and ground stations
- **Long-Range Communication**: Utilize LoRa radio technology for long-range, low-power communication
- **Secure Messaging**: End-to-end encrypted communication for secure data exchange
- **Position Tracking**: Real-time position sharing between network nodes
- **Telemetry Sharing**: Share critical telemetry data across the mesh network
- **Store-and-Forward**: Reliable message delivery even when direct communication is not possible
- **Channel Management**: Multiple communication channels for different types of data
- **Integration with Existing Systems**: Seamless integration with Bulo.Cloud Sentinel platform
- **Offline Operation**: Continue operations even when disconnected from central infrastructure
- **Low Power Consumption**: Optimized for battery-powered drone operations

### Benefits:
- **Enhanced Resilience**: Maintain communication even when individual nodes fail or are out of range
- **Extended Range**: Significantly extend communication range through multi-hop mesh networking
- **Reduced Dependency**: Operate without reliance on cellular or satellite infrastructure
- **Improved Coordination**: Enable better coordination between drones in a swarm
- **Increased Security**: Secure communication resistant to interception and jamming
- **Operational Flexibility**: Adapt to changing conditions and mission requirements
- **Cost Effectiveness**: Utilize affordable, open-source hardware and software
- **Community Support**: Leverage the active Meshtastic community and ecosystem

## 3. Integration with Existing Architecture

The SentinelBeacon module integrates with the following Bulo.Cloud Sentinel components:

- **Drone Control System**: Provides an alternative communication channel for drone control
- **Tactical Capabilities**: Enhances tactical operations with resilient communication
- **ISR Service**: Enables sharing of intelligence, surveillance, and reconnaissance data
- **SIGINT Service**: Provides a backup communication channel during electronic warfare operations
- **Mission Planning**: Incorporates mesh network planning into mission parameters

### Hardware Integration:
- **Drone Hardware**: Integrates with drone flight controllers and onboard computers
- **Radio Hardware**: Utilizes Meshtastic-compatible LoRa radios (e.g., T-Beam, T-Echo, RAK4631)
- **Ground Stations**: Connects to ground control stations and base stations

### Software Integration:
- **Meshtastic Protocol**: Implements the Meshtastic protocol for mesh networking
- **API Integration**: Provides APIs for other services to send and receive messages
- **Event System**: Publishes events to the message bus for real-time updates
- **Configuration Management**: Integrates with the platform's configuration system

## 4. API Endpoints

The SentinelBeacon module provides the following API endpoints:

### Nodes
```
GET /api/v1/nodes - Get all mesh network nodes
GET /api/v1/nodes/{node_id} - Get a specific node
PUT /api/v1/nodes/{node_id} - Update a node's configuration
DELETE /api/v1/nodes/{node_id} - Remove a node from the network
```

### Messages
```
GET /api/v1/messages - Get all messages
POST /api/v1/messages - Send a new message
GET /api/v1/messages/{message_id} - Get a specific message
DELETE /api/v1/messages/{message_id} - Delete a message
```

### Channels
```
GET /api/v1/channels - Get all channels
POST /api/v1/channels - Create a new channel
GET /api/v1/channels/{channel_id} - Get a specific channel
PUT /api/v1/channels/{channel_id} - Update a channel
DELETE /api/v1/channels/{channel_id} - Delete a channel
```

### Network
```
GET /api/v1/network/status - Get network status
GET /api/v1/network/topology - Get network topology
POST /api/v1/network/scan - Scan for nearby nodes
```

### Configuration
```
GET /api/v1/config - Get module configuration
PUT /api/v1/config - Update module configuration
POST /api/v1/config/reset - Reset to default configuration
```

### Telemetry
```
GET /api/v1/telemetry - Get telemetry data
POST /api/v1/telemetry - Send telemetry data
GET /api/v1/telemetry/{node_id} - Get telemetry for a specific node
```

## 5. Data Models

The SentinelBeacon module uses the following data models:

### Node
```python
class Node:
    id: str  # Meshtastic node ID (8-character hex)
    name: str  # User-friendly name
    type: NodeType  # drone, ground_station, relay, etc.
    hardware: str  # Hardware type (T-Beam, T-Echo, etc.)
    position: Optional[Position]  # Current position
    last_seen: datetime  # Last time node was seen
    battery_level: Optional[float]  # Battery level (0.0-1.0)
    signal_strength: Optional[int]  # RSSI in dBm
    channels: List[str]  # Channel IDs
    neighbors: List[str]  # Neighbor node IDs
    config: Dict[str, Any]  # Node configuration
    metadata: Dict[str, Any]  # Additional metadata
```

### Message
```python
class Message:
    id: str  # Message ID
    from_id: str  # Sender node ID
    to_id: Optional[str]  # Recipient node ID (None for broadcast)
    channel_id: str  # Channel ID
    type: MessageType  # text, position, telemetry, etc.
    content: str  # Message content
    priority: MessagePriority  # low, normal, high, emergency
    sent_time: datetime  # Time message was sent
    received_time: Optional[datetime]  # Time message was received
    hop_count: int  # Number of hops
    acknowledged: bool  # Whether message was acknowledged
    encrypted: bool  # Whether message is encrypted
    metadata: Dict[str, Any]  # Additional metadata
```

### Channel
```python
class Channel:
    id: str  # Channel ID
    name: str  # Channel name
    description: Optional[str]  # Channel description
    psk: Optional[str]  # Pre-shared key for encryption
    enabled: bool  # Whether channel is enabled
    role: ChannelRole  # primary, secondary, emergency, etc.
    settings: Dict[str, Any]  # Channel settings
    metadata: Dict[str, Any]  # Additional metadata
```

### Position
```python
class Position:
    latitude: float  # Latitude in degrees
    longitude: float  # Longitude in degrees
    altitude: Optional[float]  # Altitude in meters
    accuracy: Optional[float]  # Accuracy in meters
    time: datetime  # Time position was recorded
    speed: Optional[float]  # Speed in m/s
    heading: Optional[float]  # Heading in degrees
```

### Telemetry
```python
class Telemetry:
    node_id: str  # Node ID
    time: datetime  # Time telemetry was recorded
    battery_level: Optional[float]  # Battery level (0.0-1.0)
    voltage: Optional[float]  # Battery voltage
    temperature: Optional[float]  # Temperature in Celsius
    humidity: Optional[float]  # Humidity in percent
    pressure: Optional[float]  # Pressure in hPa
    air_quality: Optional[int]  # Air quality index
    signal_strength: Optional[int]  # RSSI in dBm
    snr: Optional[float]  # Signal-to-noise ratio
    tx_power: Optional[int]  # Transmit power in dBm
    rx_bytes: Optional[int]  # Received bytes
    tx_bytes: Optional[int]  # Transmitted bytes
    cpu_usage: Optional[float]  # CPU usage in percent
    memory_usage: Optional[float]  # Memory usage in percent
    storage_usage: Optional[float]  # Storage usage in percent
    metadata: Dict[str, Any]  # Additional metadata
```

### NetworkStatus
```python
class NetworkStatus:
    node_count: int  # Number of nodes in the network
    active_node_count: int  # Number of active nodes
    channel_count: int  # Number of channels
    message_count: int  # Number of messages in the last hour
    uptime: int  # Uptime in seconds
    connected: bool  # Whether connected to the network
    local_node_id: str  # Local node ID
    region: str  # Regulatory region
    modem_config: str  # LoRa modem configuration
    bandwidth: int  # Bandwidth in Hz
    spreading_factor: int  # Spreading factor
    coding_rate: str  # Coding rate
    frequency: int  # Frequency in Hz
    tx_power: int  # Transmit power in dBm
    metadata: Dict[str, Any]  # Additional metadata
```

## 6. Hardware Requirements

The SentinelBeacon module requires the following hardware:

### Supported LoRa Devices:
- **LILYGO TTGO T-Beam**: ESP32 + LoRa + GPS + Battery Management
- **LILYGO TTGO T-Echo**: ESP32 + LoRa + GPS + E-Ink Display
- **RAK Wireless RAK4631**: nRF52840 + LoRa + BLE
- **Heltec WiFi LoRa 32**: ESP32 + LoRa + OLED Display
- **LilyGO T3**: ESP32 + LoRa
- **Meshtastic DIY**: Custom hardware following Meshtastic specifications

### Recommended Specifications:
- **Processor**: ESP32 or nRF52840
- **LoRa Chipset**: SX1276, SX1262, or SX1268
- **Frequency Bands**:
  - 433 MHz (Asia)
  - 868 MHz (Europe)
  - 915 MHz (North America, Australia)
  - 923 MHz (Japan, Korea)
- **Antenna**: External antenna with 2-5 dBi gain
- **GPS**: Optional but recommended for position tracking
- **Power**: LiPo battery with at least 1000 mAh capacity
- **Interfaces**: UART, I2C, or SPI for integration with drone hardware

### Drone Integration:
- **Connection**: UART or I2C connection to flight controller
- **Mounting**: Secure mounting away from interference sources
- **Power**: Connection to drone power system or separate battery
- **Antenna Placement**: Optimal placement for maximum range

## 7. Software Architecture

The SentinelBeacon module is built on the following software architecture:

### Core Components:
- **Meshtastic Protocol**: Implements the Meshtastic protocol for mesh networking
- **Radio Interface**: Manages communication with the LoRa radio hardware
- **Message Router**: Routes messages between nodes and applications
- **Position Tracker**: Tracks and shares node positions
- **Telemetry Manager**: Collects and distributes telemetry data
- **Channel Manager**: Manages communication channels
- **Configuration Manager**: Handles module configuration
- **Security Manager**: Implements encryption and authentication
- **API Server**: Provides RESTful API for integration with other services

### Integration Components:
- **Drone Interface**: Integrates with drone flight controllers
- **Event Publisher**: Publishes events to the message bus
- **Database Connector**: Stores persistent data
- **Web Interface**: Provides web-based management interface
- **CLI Tool**: Command-line interface for configuration and testing

### Communication Flow:
1. Application sends message via API
2. Message Router processes message
3. Security Manager encrypts message if needed
4. Radio Interface transmits message
5. Receiving node's Radio Interface receives message
6. Security Manager decrypts message if needed
7. Message Router delivers message to appropriate application
8. Event Publisher notifies subscribers of new message

## 8. Sample Code

### Node Configuration Example
```python
async def configure_node(node_id: str, config: Dict[str, Any]) -> Dict[str, Any]:
    """
    Configure a Meshtastic node.

    Args:
        node_id: Node ID
        config: Node configuration

    Returns:
        Updated node configuration
    """
    try:
        # Connect to the node
        interface = meshtastic.SerialInterface(node_id)

        # Configure radio settings
        if "radio" in config:
            radio_config = config["radio"]
            interface.setConfig("radio", {
                "freq": radio_config.get("freq", 915000000),
                "tx_power": radio_config.get("tx_power", 20),
                "bandwidth": radio_config.get("bandwidth", 125),
                "spread_factor": radio_config.get("spread_factor", 7),
                "coding_rate": radio_config.get("coding_rate", 5),
                "preamble_length": radio_config.get("preamble_length", 8)
            })

        # Configure device settings
        if "device" in config:
            device_config = config["device"]
            interface.setConfig("device", {
                "role": device_config.get("role", "CLIENT"),
                "serial_enabled": device_config.get("serial_enabled", True),
                "debug_log_enabled": device_config.get("debug_log_enabled", False),
                "node_info_broadcast_secs": device_config.get("node_info_broadcast_secs", 900)
            })

        # Configure position settings
        if "position" in config:
            position_config = config["position"]
            interface.setConfig("position", {
                "position_broadcast_secs": position_config.get("position_broadcast_secs", 300),
                "fixed_position": position_config.get("fixed_position", False),
                "gps_enabled": position_config.get("gps_enabled", True),
                "gps_update_interval": position_config.get("gps_update_interval", 30),
                "smart_position_enabled": position_config.get("smart_position_enabled", True)
            })

        # Configure power settings
        if "power" in config:
            power_config = config["power"]
            interface.setConfig("power", {
                "ls_secs": power_config.get("ls_secs", 300),
                "min_wake_secs": power_config.get("min_wake_secs", 10),
                "on_battery": power_config.get("on_battery", True),
                "wait_bluetooth_secs": power_config.get("wait_bluetooth_secs", 60)
            })

        # Save configuration
        interface.writeConfig()

        # Disconnect from the node
        interface.close()

        # Return updated configuration
        return interface.getConfig()

    except Exception as e:
        logger.error(f"Error configuring node {node_id}: {e}")
        raise
```

### Sending a Message Example
```python
async def send_message(
    content: str,
    channel_id: str,
    to_id: Optional[str] = None,
    priority: str = "normal",
    want_ack: bool = True
) -> Dict[str, Any]:
    """
    Send a message over the mesh network.

    Args:
        content: Message content
        channel_id: Channel ID
        to_id: Recipient node ID (None for broadcast)
        priority: Message priority (low, normal, high, emergency)
        want_ack: Whether to request acknowledgment

    Returns:
        Sent message details
    """
    try:
        # Connect to the local node
        interface = meshtastic.SerialInterface()

        # Get channel by ID
        channel = None
        for ch in interface.getChannels():
            if ch.settings.name == channel_id:
                channel = ch
                break

        if not channel:
            raise ValueError(f"Channel {channel_id} not found")

        # Set message priority
        priority_map = {
            "low": meshtastic.mesh_pb2.Data.Priority.LOW,
            "normal": meshtastic.mesh_pb2.Data.Priority.NORMAL,
            "high": meshtastic.mesh_pb2.Data.Priority.HIGH,
            "emergency": meshtastic.mesh_pb2.Data.Priority.EMERGENCY
        }
        msg_priority = priority_map.get(priority, meshtastic.mesh_pb2.Data.Priority.NORMAL)

        # Send message
        message_id = interface.sendText(
            text=content,
            destinationId=to_id,
            channelIndex=channel.index,
            wantAck=want_ack,
            wantResponse=False,
            priority=msg_priority
        )

        # Create message object
        message = {
            "id": message_id,
            "from_id": interface.myNodeInfo.my_node_num,
            "to_id": to_id,
            "channel_id": channel_id,
            "type": "text",
            "content": content,
            "priority": priority,
            "sent_time": datetime.utcnow().isoformat(),
            "received_time": None,
            "hop_count": 0,
            "acknowledged": False,
            "encrypted": channel.settings.psk is not None,
            "metadata": {
                "want_ack": want_ack,
                "want_response": False
            }
        }

        # Disconnect from the node
        interface.close()

        return message

    except Exception as e:
        logger.error(f"Error sending message: {e}")
        raise
```

### Network Topology Example
```python
async def get_network_topology() -> Dict[str, Any]:
    """
    Get the current network topology.

    Returns:
        Network topology
    """
    try:
        # Connect to the local node
        interface = meshtastic.SerialInterface()

        # Get nodes
        nodes = []
        for node_id, node in interface.nodes.items():
            nodes.append({
                "id": node_id,
                "name": node.user.longName if node.user else f"Node {node_id}",
                "position": {
                    "latitude": node.position.latitude if node.position else None,
                    "longitude": node.position.longitude if node.position else None,
                    "altitude": node.position.altitude if node.position else None,
                    "time": node.position.time if node.position else None
                } if node.position else None,
                "last_seen": node.lastHeard,
                "battery_level": node.deviceMetrics.batteryLevel if node.deviceMetrics else None,
                "signal_strength": node.snr
            })

        # Get links
        links = []
        for node_id, node in interface.nodes.items():
            for neighbor_id in node.neighbors:
                links.append({
                    "from_id": node_id,
                    "to_id": neighbor_id,
                    "snr": node.neighbors[neighbor_id].snr
                })

        # Get channels
        channels = []
        for channel in interface.getChannels():
            channels.append({
                "id": channel.settings.name,
                "name": channel.settings.name,
                "enabled": channel.role != meshtastic.mesh_pb2.Channel.Role.DISABLED,
                "role": str(channel.role),
                "psk": channel.settings.psk is not None
            })

        # Create topology
        topology = {
            "nodes": nodes,
            "links": links,
            "channels": channels,
            "local_node_id": interface.myNodeInfo.my_node_num,
            "time": datetime.utcnow().isoformat()
        }

        # Disconnect from the node
        interface.close()

        return topology

    except Exception as e:
        logger.error(f"Error getting network topology: {e}")
        raise
```

## 9. Deployment Instructions

### Hardware Setup:
1. **Acquire Hardware**: Purchase compatible Meshtastic devices
2. **Flash Firmware**: Flash the latest Meshtastic firmware to the devices
3. **Configure Radio**: Set appropriate frequency band for your region
4. **Connect to Drone**: Connect the device to the drone's flight controller
5. **Mount Antenna**: Mount the antenna in an optimal position
6. **Power Connection**: Connect to drone power or install battery

### Software Setup:
1. **Install Dependencies**: Install Python and required packages
2. **Install Meshtastic Library**: `pip install meshtastic`
3. **Clone Repository**: Clone the SentinelBeacon repository
4. **Configure Module**: Set appropriate configuration for your deployment
5. **Start Service**: Start the SentinelBeacon service
6. **Verify Connection**: Verify connection to the mesh network
7. **Test Communication**: Test sending and receiving messages

### Drone Integration:
1. **Connect to Flight Controller**: Connect the device to the flight controller
2. **Configure MAVLink Bridge**: Set up MAVLink bridge if needed
3. **Test Telemetry**: Verify telemetry data is being transmitted
4. **Test Commands**: Verify command messages are being received
5. **Configure Failsafe**: Set up appropriate failsafe behavior

### Network Setup:
1. **Create Channels**: Create appropriate channels for different types of data
2. **Set Encryption**: Configure encryption for secure channels
3. **Configure Roles**: Assign appropriate roles to nodes
4. **Test Range**: Test communication range between nodes
5. **Verify Mesh**: Verify multi-hop message routing

## 10. Security Considerations

The SentinelBeacon module implements the following security measures:

### Encryption:
- **Channel Encryption**: AES-256 encryption for channel messages
- **PSK Management**: Secure management of pre-shared keys
- **Key Rotation**: Regular rotation of encryption keys

### Authentication:
- **Node Authentication**: Verification of node identity
- **Message Authentication**: Verification of message origin
- **Access Control**: Role-based access to channels and features

### Physical Security:
- **Tamper Detection**: Detection of physical tampering
- **Secure Storage**: Secure storage of sensitive data
- **Remote Wipe**: Ability to remotely wipe sensitive data

### Operational Security:
- **Frequency Hopping**: Optional frequency hopping for enhanced security
- **Low Probability of Detection**: Minimized transmission power and duration
- **Jamming Resistance**: Spread spectrum techniques for jamming resistance

### Data Security:
- **Minimal Data Collection**: Collection of only necessary data
- **Data Retention**: Limited retention of sensitive data
- **Secure Deletion**: Secure deletion of expired data

## 11. Limitations and Considerations

When deploying the SentinelBeacon module, consider the following limitations:

### Technical Limitations:
- **Bandwidth**: Limited bandwidth (typically <10 kbps)
- **Latency**: Higher latency compared to direct communication
- **Range**: Limited range depending on terrain and obstacles
- **Power Consumption**: Increased power consumption during active transmission
- **Payload Size**: Limited message size (typically <240 bytes)

### Regulatory Considerations:
- **Frequency Regulations**: Compliance with local frequency regulations
- **Transmit Power**: Compliance with maximum transmit power regulations
- **Duty Cycle**: Compliance with duty cycle limitations
- **Licensing**: Proper licensing for frequency bands if required

### Operational Considerations:
- **Network Density**: Performance degradation with high node density
- **Interference**: Potential interference from other devices
- **Battery Life**: Impact on drone battery life
- **Weight**: Additional weight affecting drone performance
- **Antenna Placement**: Critical for optimal performance

### Security Considerations:
- **Radio Direction Finding**: Vulnerability to direction finding
- **Signal Jamming**: Vulnerability to deliberate jamming
- **Physical Access**: Risk of physical access to devices
- **Firmware Security**: Importance of keeping firmware updated
