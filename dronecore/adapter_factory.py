"""
Flight Controller Adapter Factory

This module provides a factory for creating flight controller adapters based on the connection type.
"""

import logging
from typing import Dict, Any, Optional

from dronecore.flight_controller_adapter import FlightControllerAdapter
from dronecore.ardupilot_adapter import ArduPilotAdapter
from dronecore.px4_adapter import PX4Adapter
from dronecore.betaflight_adapter import BetaflightAdapter
from dronecore.dji_adapter import DJIAdapter

logger = logging.getLogger(__name__)

class AdapterFactory:
    """Factory for creating flight controller adapters."""
    
    @staticmethod
    def create_adapter(connection_type: str, connection_params: Dict[str, Any]) -> Optional[FlightControllerAdapter]:
        """
        Create a flight controller adapter based on the connection type.
        
        Args:
            connection_type: Type of the connection (ardupilot, px4, betaflight, dji).
            connection_params: Connection parameters for the adapter.
            
        Returns:
            FlightControllerAdapter: An instance of the appropriate adapter, or None if the type is not supported.
        """
        connection_string = connection_params.get("connection_string", "")
        
        if connection_type.lower() == "ardupilot":
            logger.info(f"Creating ArduPilot adapter with connection string: {connection_string}")
            return ArduPilotAdapter(connection_string)
        elif connection_type.lower() == "px4":
            logger.info(f"Creating PX4 adapter with connection string: {connection_string}")
            return PX4Adapter(connection_string)
        elif connection_type.lower() == "betaflight":
            logger.info(f"Creating Betaflight adapter with connection string: {connection_string}")
            return BetaflightAdapter(connection_string)
        elif connection_type.lower() == "dji":
            logger.info(f"Creating DJI adapter with connection parameters")
            return DJIAdapter(connection_params)
        else:
            logger.error(f"Unsupported connection type: {connection_type}")
            return None
