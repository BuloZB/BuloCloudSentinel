"""
Remote ID broadcasting service for the Remote ID & Regulatory Compliance Service.

This module provides functionality for broadcasting Remote ID messages.
"""

import asyncio
import logging
import time
import uuid
from datetime import datetime
from typing import Dict, List, Optional, Any, Union

from sqlalchemy.ext.asyncio import AsyncSession

from remoteid_service.api.schemas.remoteid import (
    BroadcastMethod,
    BroadcastStatus,
    Position,
    RemoteIDMode,
    Velocity,
    OperatorID,
    ASTMMessage,
    ASTMLocationMessage,
    ASTMOperatorIDMessage,
    ASTMBasicIDMessage,
)
from remoteid_service.broadcast.message_converter import MessageConverter
from remoteid_service.broadcast.wifi_nan import WiFiNANBroadcaster
from remoteid_service.broadcast.bluetooth_le import BluetoothLEBroadcaster
from remoteid_service.core.settings import Settings
from remoteid_service.db.models import RemoteIDBroadcast
from remoteid_service.services.remoteid_logging import RemoteIDLoggingService

# Configure logging
logger = logging.getLogger(__name__)

class RemoteIDBroadcastService:
    """
    Remote ID broadcasting service.
    
    This service manages Remote ID broadcasting for drones, including
    starting, stopping, and updating broadcasts.
    """
    
    def __init__(self, settings: Settings):
        """
        Initialize the Remote ID broadcasting service.
        
        Args:
            settings: Application settings
        """
        self.settings = settings
        self.active_broadcasts: Dict[str, Dict[str, Any]] = {}
        self.message_converter = MessageConverter()
        
        # Initialize broadcasters
        self.broadcasters = {}
        
        if BroadcastMethod.WIFI_NAN in self.settings.BROADCAST_METHODS:
            if self.settings.ENABLE_HARDWARE:
                self.broadcasters[BroadcastMethod.WIFI_NAN] = WiFiNANBroadcaster(
                    adapter=self.settings.WIFI_ADAPTER
                )
            else:
                # Use simulated broadcaster
                self.broadcasters[BroadcastMethod.WIFI_NAN] = WiFiNANBroadcaster(
                    adapter=None, simulated=True
                )
        
        if BroadcastMethod.BLUETOOTH_LE in self.settings.BROADCAST_METHODS:
            if self.settings.ENABLE_HARDWARE:
                self.broadcasters[BroadcastMethod.BLUETOOTH_LE] = BluetoothLEBroadcaster(
                    adapter=self.settings.BLUETOOTH_ADAPTER
                )
            else:
                # Use simulated broadcaster
                self.broadcasters[BroadcastMethod.BLUETOOTH_LE] = BluetoothLEBroadcaster(
                    adapter=None, simulated=True
                )
        
        # Initialize broadcast task
        self.broadcast_task = None
        self.running = False
    
    async def start(self) -> None:
        """
        Start the Remote ID broadcasting service.
        """
        if self.running:
            return
        
        self.running = True
        
        # Initialize broadcasters
        for method, broadcaster in self.broadcasters.items():
            try:
                await broadcaster.initialize()
                logger.info(f"Initialized {method} broadcaster")
            except Exception as e:
                logger.error(f"Error initializing {method} broadcaster: {str(e)}")
        
        # Start broadcast task
        self.broadcast_task = asyncio.create_task(self._broadcast_loop())
        logger.info("Started Remote ID broadcasting service")
    
    async def stop(self) -> None:
        """
        Stop the Remote ID broadcasting service.
        """
        if not self.running:
            return
        
        self.running = False
        
        # Stop broadcast task
        if self.broadcast_task:
            self.broadcast_task.cancel()
            try:
                await self.broadcast_task
            except asyncio.CancelledError:
                pass
            self.broadcast_task = None
        
        # Stop broadcasters
        for method, broadcaster in self.broadcasters.items():
            try:
                await broadcaster.shutdown()
                logger.info(f"Shut down {method} broadcaster")
            except Exception as e:
                logger.error(f"Error shutting down {method} broadcaster: {str(e)}")
        
        logger.info("Stopped Remote ID broadcasting service")
    
    async def _broadcast_loop(self) -> None:
        """
        Main broadcast loop.
        
        This method runs in a background task and periodically broadcasts
        Remote ID messages for all active drones.
        """
        while self.running:
            try:
                # Broadcast for all active drones
                for drone_id, broadcast_data in self.active_broadcasts.items():
                    await self._broadcast_drone(drone_id, broadcast_data)
                
                # Wait for next broadcast interval
                await asyncio.sleep(self.settings.BROADCAST_INTERVAL)
            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error(f"Error in broadcast loop: {str(e)}")
                await asyncio.sleep(1)  # Avoid tight loop on error
    
    async def _broadcast_drone(self, drone_id: str, broadcast_data: Dict[str, Any]) -> None:
        """
        Broadcast Remote ID messages for a drone.
        
        Args:
            drone_id: Drone ID
            broadcast_data: Broadcast data
        """
        try:
            # Check if broadcasting is enabled
            if not broadcast_data.get("broadcasting", False):
                return
            
            # Get broadcast parameters
            mode = broadcast_data.get("mode", RemoteIDMode.FAA)
            methods = broadcast_data.get("methods", [])
            position = broadcast_data.get("position")
            velocity = broadcast_data.get("velocity")
            operator_id = broadcast_data.get("operator_id")
            serial_number = broadcast_data.get("serial_number")
            session_id = broadcast_data.get("session_id")
            
            # Skip if no position data
            if not position:
                return
            
            # Update last broadcast time
            broadcast_data["last_update"] = datetime.utcnow()
            
            # Create ASTM message
            astm_message = self.message_converter.create_message(
                drone_id=drone_id,
                mode=mode,
                position=position,
                velocity=velocity,
                operator_id=operator_id,
                serial_number=serial_number,
                timestamp=broadcast_data["last_update"],
            )
            
            # Broadcast using each method
            for method in methods:
                if method in self.broadcasters:
                    try:
                        await self.broadcasters[method].broadcast(astm_message)
                        logger.debug(f"Broadcast {method} message for drone {drone_id}")
                    except Exception as e:
                        logger.error(f"Error broadcasting {method} message for drone {drone_id}: {str(e)}")
        except Exception as e:
            logger.error(f"Error broadcasting for drone {drone_id}: {str(e)}")
    
    async def start_broadcast(
        self,
        drone_id: str,
        mode: RemoteIDMode,
        methods: List[BroadcastMethod],
        operator_id: Optional[OperatorID] = None,
        serial_number: Optional[str] = None,
        session_id: Optional[str] = None,
        initial_position: Optional[Position] = None,
        initial_velocity: Optional[Velocity] = None,
        metadata: Optional[Dict[str, Any]] = None,
        db: AsyncSession = None,
    ) -> BroadcastStatus:
        """
        Start Remote ID broadcasting for a drone.
        
        Args:
            drone_id: Drone ID
            mode: Broadcast mode
            methods: Broadcast methods
            operator_id: Operator ID
            serial_number: Serial number
            session_id: Session ID
            initial_position: Initial position
            initial_velocity: Initial velocity
            metadata: Additional metadata
            db: Database session
            
        Returns:
            BroadcastStatus: Broadcast status
        """
        # Generate session ID if not provided
        if not session_id:
            session_id = str(uuid.uuid4())
        
        # Store broadcast data
        self.active_broadcasts[drone_id] = {
            "broadcasting": True,
            "mode": mode,
            "methods": methods,
            "operator_id": operator_id.dict() if operator_id else None,
            "serial_number": serial_number,
            "session_id": session_id,
            "position": initial_position.dict() if initial_position else None,
            "velocity": initial_velocity.dict() if initial_velocity else None,
            "metadata": metadata,
            "start_time": datetime.utcnow(),
            "last_update": datetime.utcnow(),
        }
        
        logger.info(f"Started Remote ID broadcasting for drone {drone_id}")
        
        # Return broadcast status
        return BroadcastStatus(
            drone_id=drone_id,
            broadcasting=True,
            mode=mode,
            methods=methods,
            session_id=session_id,
            last_update=self.active_broadcasts[drone_id]["last_update"],
            position=initial_position,
            velocity=initial_velocity,
        )
    
    async def stop_broadcast(
        self,
        drone_id: str,
        session_id: Optional[str] = None,
        db: AsyncSession = None,
    ) -> BroadcastStatus:
        """
        Stop Remote ID broadcasting for a drone.
        
        Args:
            drone_id: Drone ID
            session_id: Session ID
            db: Database session
            
        Returns:
            BroadcastStatus: Broadcast status
        """
        # Check if drone is broadcasting
        if drone_id not in self.active_broadcasts:
            return BroadcastStatus(
                drone_id=drone_id,
                broadcasting=False,
            )
        
        # Check session ID if provided
        if session_id and self.active_broadcasts[drone_id].get("session_id") != session_id:
            raise ValueError(f"Invalid session ID for drone {drone_id}")
        
        # Get broadcast data
        broadcast_data = self.active_broadcasts[drone_id].copy()
        
        # Update broadcasting status
        broadcast_data["broadcasting"] = False
        
        # Remove from active broadcasts
        del self.active_broadcasts[drone_id]
        
        logger.info(f"Stopped Remote ID broadcasting for drone {drone_id}")
        
        # Return broadcast status
        return BroadcastStatus(
            drone_id=drone_id,
            broadcasting=False,
            mode=broadcast_data.get("mode"),
            methods=broadcast_data.get("methods"),
            session_id=broadcast_data.get("session_id"),
            last_update=datetime.utcnow(),
            position=Position(**broadcast_data["position"]) if broadcast_data.get("position") else None,
            velocity=Velocity(**broadcast_data["velocity"]) if broadcast_data.get("velocity") else None,
        )
    
    async def update_broadcast(
        self,
        drone_id: str,
        position: Position,
        velocity: Optional[Velocity] = None,
        session_id: Optional[str] = None,
        timestamp: Optional[datetime] = None,
        db: AsyncSession = None,
    ) -> BroadcastStatus:
        """
        Update Remote ID broadcast data for a drone.
        
        Args:
            drone_id: Drone ID
            position: Position
            velocity: Velocity
            session_id: Session ID
            timestamp: Timestamp
            db: Database session
            
        Returns:
            BroadcastStatus: Broadcast status
        """
        # Check if drone is broadcasting
        if drone_id not in self.active_broadcasts:
            raise ValueError(f"Drone {drone_id} is not broadcasting")
        
        # Check session ID if provided
        if session_id and self.active_broadcasts[drone_id].get("session_id") != session_id:
            raise ValueError(f"Invalid session ID for drone {drone_id}")
        
        # Update broadcast data
        self.active_broadcasts[drone_id]["position"] = position.dict()
        if velocity:
            self.active_broadcasts[drone_id]["velocity"] = velocity.dict()
        
        # Update timestamp
        if timestamp:
            self.active_broadcasts[drone_id]["last_update"] = timestamp
        else:
            self.active_broadcasts[drone_id]["last_update"] = datetime.utcnow()
        
        logger.debug(f"Updated Remote ID broadcast data for drone {drone_id}")
        
        # Return broadcast status
        return BroadcastStatus(
            drone_id=drone_id,
            broadcasting=self.active_broadcasts[drone_id].get("broadcasting", False),
            mode=self.active_broadcasts[drone_id].get("mode"),
            methods=self.active_broadcasts[drone_id].get("methods"),
            session_id=self.active_broadcasts[drone_id].get("session_id"),
            last_update=self.active_broadcasts[drone_id]["last_update"],
            position=position,
            velocity=velocity,
        )
    
    async def get_broadcast_status(self, drone_id: str) -> BroadcastStatus:
        """
        Get Remote ID broadcast status for a drone.
        
        Args:
            drone_id: Drone ID
            
        Returns:
            BroadcastStatus: Broadcast status
        """
        # Check if drone is broadcasting
        if drone_id not in self.active_broadcasts:
            return BroadcastStatus(
                drone_id=drone_id,
                broadcasting=False,
            )
        
        # Get broadcast data
        broadcast_data = self.active_broadcasts[drone_id]
        
        # Return broadcast status
        return BroadcastStatus(
            drone_id=drone_id,
            broadcasting=broadcast_data.get("broadcasting", False),
            mode=broadcast_data.get("mode"),
            methods=broadcast_data.get("methods"),
            session_id=broadcast_data.get("session_id"),
            last_update=broadcast_data.get("last_update"),
            position=Position(**broadcast_data["position"]) if broadcast_data.get("position") else None,
            velocity=Velocity(**broadcast_data["velocity"]) if broadcast_data.get("velocity") else None,
        )
    
    async def get_astm_message(self, drone_id: str) -> ASTMMessage:
        """
        Get ASTM F3411-22a message for a drone.
        
        Args:
            drone_id: Drone ID
            
        Returns:
            ASTMMessage: ASTM message
        """
        # Check if drone is broadcasting
        if drone_id not in self.active_broadcasts:
            raise ValueError(f"Drone {drone_id} is not broadcasting")
        
        # Get broadcast data
        broadcast_data = self.active_broadcasts[drone_id]
        
        # Create ASTM message
        astm_message = self.message_converter.create_message(
            drone_id=drone_id,
            mode=broadcast_data.get("mode", RemoteIDMode.FAA),
            position=Position(**broadcast_data["position"]) if broadcast_data.get("position") else None,
            velocity=Velocity(**broadcast_data["velocity"]) if broadcast_data.get("velocity") else None,
            operator_id=broadcast_data.get("operator_id"),
            serial_number=broadcast_data.get("serial_number"),
            timestamp=broadcast_data.get("last_update", datetime.utcnow()),
        )
        
        return astm_message
