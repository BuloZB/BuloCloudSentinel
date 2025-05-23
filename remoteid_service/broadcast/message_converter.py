"""
Message converter for the Remote ID & Regulatory Compliance Service.

This module provides functionality for converting between different Remote ID
message formats.
"""

import logging
from datetime import datetime
from typing import Dict, List, Optional, Any, Union

from remoteid_service.api.schemas.remoteid import (
    ASTMMessage,
    ASTMLocationMessage,
    ASTMOperatorIDMessage,
    ASTMBasicIDMessage,
    BroadcastMethod,
    Position,
    RemoteIDMode,
    Velocity,
    OperatorID,
    OperatorIDType,
)

# Configure logging
logger = logging.getLogger(__name__)

class MessageConverter:
    """
    Message converter.
    
    This class provides functionality for converting between different Remote ID
    message formats.
    """
    
    def __init__(self):
        """
        Initialize the message converter.
        """
        pass
    
    def create_message(
        self,
        drone_id: str,
        mode: RemoteIDMode,
        position: Optional[Position],
        velocity: Optional[Velocity] = None,
        operator_id: Optional[Dict[str, Any]] = None,
        serial_number: Optional[str] = None,
        timestamp: Optional[datetime] = None,
    ) -> ASTMMessage:
        """
        Create an ASTM F3411-22a message.
        
        Args:
            drone_id: Drone ID
            mode: Remote ID mode
            position: Position
            velocity: Velocity
            operator_id: Operator ID
            serial_number: Serial number
            timestamp: Timestamp
            
        Returns:
            ASTMMessage: ASTM message
        """
        # Set default timestamp
        if not timestamp:
            timestamp = datetime.utcnow()
        
        # Create location message if position is provided
        if position:
            location_message = ASTMLocationMessage(
                uas_id=drone_id,
                operator_id=operator_id.get("id") if operator_id else None,
                position=position,
                velocity=velocity,
                timestamp=timestamp,
                operational_status="Airborne",
            )
            
            return ASTMMessage(message=location_message)
        
        # Create operator ID message if operator ID is provided
        elif operator_id:
            operator_id_message = ASTMOperatorIDMessage(
                uas_id=drone_id,
                operator_id=operator_id.get("id"),
                operator_id_type=operator_id.get("type", OperatorIDType.OTHER),
                timestamp=timestamp,
            )
            
            return ASTMMessage(message=operator_id_message)
        
        # Create basic ID message if serial number is provided
        elif serial_number:
            basic_id_message = ASTMBasicIDMessage(
                uas_id=drone_id,
                uas_id_type="serial_number",
                timestamp=timestamp,
            )
            
            return ASTMMessage(message=basic_id_message)
        
        # Default to basic ID message
        else:
            basic_id_message = ASTMBasicIDMessage(
                uas_id=drone_id,
                uas_id_type="serial_number",
                timestamp=timestamp,
            )
            
            return ASTMMessage(message=basic_id_message)
    
    def create_wifi_nan_message(
        self,
        astm_message: ASTMMessage,
    ) -> bytes:
        """
        Create a Wi-Fi NAN message.
        
        Args:
            astm_message: ASTM message
            
        Returns:
            bytes: Wi-Fi NAN message
        """
        # Convert ASTM message to Wi-Fi NAN format
        # This is a simplified implementation
        message = astm_message.message
        
        if isinstance(message, ASTMLocationMessage):
            # Create location message
            nan_message = {
                "message_type": "location",
                "uas_id": message.uas_id,
                "lat": message.position.latitude,
                "lon": message.position.longitude,
                "alt": message.position.altitude,
                "speed_h": message.velocity.speed_horizontal if message.velocity else 0,
                "speed_v": message.velocity.speed_vertical if message.velocity and message.velocity.speed_vertical else 0,
                "heading": message.velocity.heading if message.velocity and message.velocity.heading else 0,
                "timestamp": int(message.timestamp.timestamp()),
            }
        elif isinstance(message, ASTMOperatorIDMessage):
            # Create operator ID message
            nan_message = {
                "message_type": "operator_id",
                "uas_id": message.uas_id,
                "operator_id": message.operator_id,
                "operator_id_type": message.operator_id_type,
                "timestamp": int(message.timestamp.timestamp()),
            }
        elif isinstance(message, ASTMBasicIDMessage):
            # Create basic ID message
            nan_message = {
                "message_type": "basic_id",
                "uas_id": message.uas_id,
                "uas_id_type": message.uas_id_type,
                "timestamp": int(message.timestamp.timestamp()),
            }
        else:
            # Default message
            nan_message = {
                "message_type": "unknown",
                "uas_id": "unknown",
                "timestamp": int(datetime.utcnow().timestamp()),
            }
        
        # Convert to bytes
        return str(nan_message).encode("utf-8")
    
    def create_bluetooth_le_message(
        self,
        astm_message: ASTMMessage,
    ) -> bytes:
        """
        Create a Bluetooth LE message.
        
        Args:
            astm_message: ASTM message
            
        Returns:
            bytes: Bluetooth LE message
        """
        # Convert ASTM message to Bluetooth LE format
        # This is a simplified implementation
        message = astm_message.message
        
        if isinstance(message, ASTMLocationMessage):
            # Create location message
            ble_message = {
                "message_type": "location",
                "uas_id": message.uas_id,
                "lat": message.position.latitude,
                "lon": message.position.longitude,
                "alt": message.position.altitude,
                "speed_h": message.velocity.speed_horizontal if message.velocity else 0,
                "speed_v": message.velocity.speed_vertical if message.velocity and message.velocity.speed_vertical else 0,
                "heading": message.velocity.heading if message.velocity and message.velocity.heading else 0,
                "timestamp": int(message.timestamp.timestamp()),
            }
        elif isinstance(message, ASTMOperatorIDMessage):
            # Create operator ID message
            ble_message = {
                "message_type": "operator_id",
                "uas_id": message.uas_id,
                "operator_id": message.operator_id,
                "operator_id_type": message.operator_id_type,
                "timestamp": int(message.timestamp.timestamp()),
            }
        elif isinstance(message, ASTMBasicIDMessage):
            # Create basic ID message
            ble_message = {
                "message_type": "basic_id",
                "uas_id": message.uas_id,
                "uas_id_type": message.uas_id_type,
                "timestamp": int(message.timestamp.timestamp()),
            }
        else:
            # Default message
            ble_message = {
                "message_type": "unknown",
                "uas_id": "unknown",
                "timestamp": int(datetime.utcnow().timestamp()),
            }
        
        # Convert to bytes
        return str(ble_message).encode("utf-8")
