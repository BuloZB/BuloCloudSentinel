"""
Remote ID CLI for the Remote ID & Regulatory Compliance Service.

This module provides a command-line interface for Remote ID functionality.
"""

import argparse
import asyncio
import json
import logging
import sys
from datetime import datetime
from typing import Dict, List, Optional, Any

import httpx

from remoteid_service.api.schemas.remoteid import (
    RemoteIDMode,
    BroadcastMethod,
    Position,
    Velocity,
    OperatorID,
    OperatorIDType,
)

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
)
logger = logging.getLogger("remoteid_cli")

class RemoteIDCLI:
    """
    Remote ID CLI.
    
    This class provides a command-line interface for Remote ID functionality.
    """
    
    def __init__(self):
        """
        Initialize the Remote ID CLI.
        """
        self.parser = self._create_parser()
    
    def _create_parser(self) -> argparse.ArgumentParser:
        """
        Create the argument parser.
        
        Returns:
            argparse.ArgumentParser: Argument parser
        """
        parser = argparse.ArgumentParser(
            description="Remote ID CLI for the Remote ID & Regulatory Compliance Service",
        )
        
        # Add subparsers
        subparsers = parser.add_subparsers(dest="command", help="Command")
        
        # Start command
        start_parser = subparsers.add_parser("start", help="Start Remote ID broadcasting")
        start_parser.add_argument("--drone-id", required=True, help="Drone ID")
        start_parser.add_argument("--mode", choices=["faa", "eu", "custom"], default="faa", help="Broadcast mode")
        start_parser.add_argument("--methods", nargs="+", choices=["wifi_nan", "bluetooth_le", "network", "astm_network"], default=["wifi_nan", "bluetooth_le"], help="Broadcast methods")
        start_parser.add_argument("--operator-id", help="Operator ID")
        start_parser.add_argument("--operator-id-type", choices=["faa_registration", "easa_registration", "utm_registration", "other"], default="other", help="Operator ID type")
        start_parser.add_argument("--serial-number", help="Serial number")
        start_parser.add_argument("--latitude", type=float, help="Initial latitude")
        start_parser.add_argument("--longitude", type=float, help="Initial longitude")
        start_parser.add_argument("--altitude", type=float, help="Initial altitude")
        start_parser.add_argument("--speed", type=float, help="Initial speed")
        start_parser.add_argument("--heading", type=float, help="Initial heading")
        start_parser.add_argument("--server", default="http://localhost:8080", help="Server URL")
        
        # Stop command
        stop_parser = subparsers.add_parser("stop", help="Stop Remote ID broadcasting")
        stop_parser.add_argument("--drone-id", required=True, help="Drone ID")
        stop_parser.add_argument("--session-id", help="Session ID")
        stop_parser.add_argument("--server", default="http://localhost:8080", help="Server URL")
        
        # Update command
        update_parser = subparsers.add_parser("update", help="Update Remote ID broadcast data")
        update_parser.add_argument("--drone-id", required=True, help="Drone ID")
        update_parser.add_argument("--latitude", type=float, required=True, help="Latitude")
        update_parser.add_argument("--longitude", type=float, required=True, help="Longitude")
        update_parser.add_argument("--altitude", type=float, required=True, help="Altitude")
        update_parser.add_argument("--speed", type=float, help="Speed")
        update_parser.add_argument("--heading", type=float, help="Heading")
        update_parser.add_argument("--session-id", help="Session ID")
        update_parser.add_argument("--server", default="http://localhost:8080", help="Server URL")
        
        # Status command
        status_parser = subparsers.add_parser("status", help="Get Remote ID broadcast status")
        status_parser.add_argument("--drone-id", required=True, help="Drone ID")
        status_parser.add_argument("--server", default="http://localhost:8080", help="Server URL")
        
        # Logs command
        logs_parser = subparsers.add_parser("logs", help="Get Remote ID broadcast logs")
        logs_parser.add_argument("--drone-id", help="Drone ID")
        logs_parser.add_argument("--start-time", help="Start time (ISO format)")
        logs_parser.add_argument("--end-time", help="End time (ISO format)")
        logs_parser.add_argument("--limit", type=int, default=100, help="Limit")
        logs_parser.add_argument("--offset", type=int, default=0, help="Offset")
        logs_parser.add_argument("--server", default="http://localhost:8080", help="Server URL")
        
        return parser
    
    async def run(self, args: Optional[List[str]] = None) -> None:
        """
        Run the CLI.
        
        Args:
            args: Command-line arguments
        """
        # Parse arguments
        args = self.parser.parse_args(args)
        
        # Check command
        if not args.command:
            self.parser.print_help()
            return
        
        # Execute command
        if args.command == "start":
            await self._start_broadcast(args)
        elif args.command == "stop":
            await self._stop_broadcast(args)
        elif args.command == "update":
            await self._update_broadcast(args)
        elif args.command == "status":
            await self._get_status(args)
        elif args.command == "logs":
            await self._get_logs(args)
        else:
            self.parser.print_help()
    
    async def _start_broadcast(self, args: argparse.Namespace) -> None:
        """
        Start Remote ID broadcasting.
        
        Args:
            args: Command-line arguments
        """
        try:
            # Create request data
            request_data = {
                "drone_id": args.drone_id,
                "mode": args.mode,
                "methods": args.methods,
            }
            
            # Add operator ID if provided
            if args.operator_id:
                request_data["operator_id"] = {
                    "id": args.operator_id,
                    "type": args.operator_id_type,
                }
            
            # Add serial number if provided
            if args.serial_number:
                request_data["serial_number"] = args.serial_number
            
            # Add initial position if provided
            if args.latitude and args.longitude and args.altitude:
                request_data["initial_position"] = {
                    "latitude": args.latitude,
                    "longitude": args.longitude,
                    "altitude": args.altitude,
                }
            
            # Add initial velocity if provided
            if args.speed:
                request_data["initial_velocity"] = {
                    "speed_horizontal": args.speed,
                    "heading": args.heading,
                }
            
            # Send request
            async with httpx.AsyncClient() as client:
                response = await client.post(
                    f"{args.server}/api/v1/remoteid/broadcast/start",
                    json=request_data,
                    timeout=30,
                )
                
                # Check response
                response.raise_for_status()
                
                # Print response
                print(json.dumps(response.json(), indent=2))
        except Exception as e:
            logger.error(f"Error starting Remote ID broadcast: {str(e)}")
            sys.exit(1)
    
    async def _stop_broadcast(self, args: argparse.Namespace) -> None:
        """
        Stop Remote ID broadcasting.
        
        Args:
            args: Command-line arguments
        """
        try:
            # Create request data
            request_data = {
                "drone_id": args.drone_id,
            }
            
            # Add session ID if provided
            if args.session_id:
                request_data["session_id"] = args.session_id
            
            # Send request
            async with httpx.AsyncClient() as client:
                response = await client.post(
                    f"{args.server}/api/v1/remoteid/broadcast/stop",
                    json=request_data,
                    timeout=30,
                )
                
                # Check response
                response.raise_for_status()
                
                # Print response
                print(json.dumps(response.json(), indent=2))
        except Exception as e:
            logger.error(f"Error stopping Remote ID broadcast: {str(e)}")
            sys.exit(1)
    
    async def _update_broadcast(self, args: argparse.Namespace) -> None:
        """
        Update Remote ID broadcast data.
        
        Args:
            args: Command-line arguments
        """
        try:
            # Create request data
            request_data = {
                "drone_id": args.drone_id,
                "position": {
                    "latitude": args.latitude,
                    "longitude": args.longitude,
                    "altitude": args.altitude,
                },
            }
            
            # Add velocity if provided
            if args.speed:
                request_data["velocity"] = {
                    "speed_horizontal": args.speed,
                    "heading": args.heading,
                }
            
            # Add session ID if provided
            if args.session_id:
                request_data["session_id"] = args.session_id
            
            # Send request
            async with httpx.AsyncClient() as client:
                response = await client.post(
                    f"{args.server}/api/v1/remoteid/broadcast/update",
                    json=request_data,
                    timeout=30,
                )
                
                # Check response
                response.raise_for_status()
                
                # Print response
                print(json.dumps(response.json(), indent=2))
        except Exception as e:
            logger.error(f"Error updating Remote ID broadcast: {str(e)}")
            sys.exit(1)
    
    async def _get_status(self, args: argparse.Namespace) -> None:
        """
        Get Remote ID broadcast status.
        
        Args:
            args: Command-line arguments
        """
        try:
            # Send request
            async with httpx.AsyncClient() as client:
                response = await client.get(
                    f"{args.server}/api/v1/remoteid/broadcast/status/{args.drone_id}",
                    timeout=30,
                )
                
                # Check response
                response.raise_for_status()
                
                # Print response
                print(json.dumps(response.json(), indent=2))
        except Exception as e:
            logger.error(f"Error getting Remote ID broadcast status: {str(e)}")
            sys.exit(1)
    
    async def _get_logs(self, args: argparse.Namespace) -> None:
        """
        Get Remote ID broadcast logs.
        
        Args:
            args: Command-line arguments
        """
        try:
            # Create request data
            request_data = {
                "limit": args.limit,
                "offset": args.offset,
            }
            
            # Add drone ID if provided
            if args.drone_id:
                request_data["drone_id"] = args.drone_id
            
            # Add start time if provided
            if args.start_time:
                request_data["start_time"] = args.start_time
            
            # Add end time if provided
            if args.end_time:
                request_data["end_time"] = args.end_time
            
            # Send request
            async with httpx.AsyncClient() as client:
                response = await client.post(
                    f"{args.server}/api/v1/remoteid/logs",
                    json=request_data,
                    timeout=30,
                )
                
                # Check response
                response.raise_for_status()
                
                # Print response
                print(json.dumps(response.json(), indent=2))
        except Exception as e:
            logger.error(f"Error getting Remote ID broadcast logs: {str(e)}")
            sys.exit(1)

def main():
    """
    Main entry point.
    """
    cli = RemoteIDCLI()
    asyncio.run(cli.run())

if __name__ == "__main__":
    main()
