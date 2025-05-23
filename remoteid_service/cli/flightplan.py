"""
Flight plan CLI for the Remote ID & Regulatory Compliance Service.

This module provides a command-line interface for flight plan functionality.
"""

import argparse
import asyncio
import json
import logging
import sys
import yaml
from datetime import datetime
from typing import Dict, List, Optional, Any

import httpx

from remoteid_service.api.schemas.flightplans import (
    FlightPlanType,
    FlightPlanStatus,
)

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
)
logger = logging.getLogger("flightplan_cli")

class FlightPlanCLI:
    """
    Flight plan CLI.
    
    This class provides a command-line interface for flight plan functionality.
    """
    
    def __init__(self):
        """
        Initialize the flight plan CLI.
        """
        self.parser = self._create_parser()
    
    def _create_parser(self) -> argparse.ArgumentParser:
        """
        Create the argument parser.
        
        Returns:
            argparse.ArgumentParser: Argument parser
        """
        parser = argparse.ArgumentParser(
            description="Flight plan CLI for the Remote ID & Regulatory Compliance Service",
        )
        
        # Add subparsers
        subparsers = parser.add_subparsers(dest="command", help="Command")
        
        # Create command
        create_parser = subparsers.add_parser("create", help="Create a flight plan")
        create_parser.add_argument("--file", required=True, help="YAML file containing flight plan data")
        create_parser.add_argument("--server", default="http://localhost:8080", help="Server URL")
        
        # Get command
        get_parser = subparsers.add_parser("get", help="Get a flight plan")
        get_parser.add_argument("--id", required=True, help="Flight plan ID")
        get_parser.add_argument("--server", default="http://localhost:8080", help="Server URL")
        
        # Update command
        update_parser = subparsers.add_parser("update", help="Update a flight plan")
        update_parser.add_argument("--id", required=True, help="Flight plan ID")
        update_parser.add_argument("--file", required=True, help="YAML file containing flight plan data")
        update_parser.add_argument("--server", default="http://localhost:8080", help="Server URL")
        
        # Delete command
        delete_parser = subparsers.add_parser("delete", help="Delete a flight plan")
        delete_parser.add_argument("--id", required=True, help="Flight plan ID")
        delete_parser.add_argument("--server", default="http://localhost:8080", help="Server URL")
        
        # Search command
        search_parser = subparsers.add_parser("search", help="Search for flight plans")
        search_parser.add_argument("--operator-id", help="Operator ID")
        search_parser.add_argument("--drone-id", help="Drone ID")
        search_parser.add_argument("--status", choices=["draft", "submitted", "approved", "rejected", "cancelled", "completed"], help="Status")
        search_parser.add_argument("--start-time-from", help="Start time from (ISO format)")
        search_parser.add_argument("--start-time-to", help="Start time to (ISO format)")
        search_parser.add_argument("--limit", type=int, default=100, help="Limit")
        search_parser.add_argument("--offset", type=int, default=0, help="Offset")
        search_parser.add_argument("--server", default="http://localhost:8080", help="Server URL")
        
        # Submit command
        submit_parser = subparsers.add_parser("submit", help="Submit a flight plan")
        submit_parser.add_argument("--id", required=True, help="Flight plan ID")
        submit_parser.add_argument("--type", choices=["easa_sora", "faa_laanc", "custom"], required=True, help="Submission type")
        submit_parser.add_argument("--file", help="YAML file containing additional submission data")
        submit_parser.add_argument("--server", default="http://localhost:8080", help="Server URL")
        
        # Cancel command
        cancel_parser = subparsers.add_parser("cancel", help="Cancel a flight plan")
        cancel_parser.add_argument("--id", required=True, help="Flight plan ID")
        cancel_parser.add_argument("--reason", help="Cancellation reason")
        cancel_parser.add_argument("--server", default="http://localhost:8080", help="Server URL")
        
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
        if args.command == "create":
            await self._create_flight_plan(args)
        elif args.command == "get":
            await self._get_flight_plan(args)
        elif args.command == "update":
            await self._update_flight_plan(args)
        elif args.command == "delete":
            await self._delete_flight_plan(args)
        elif args.command == "search":
            await self._search_flight_plans(args)
        elif args.command == "submit":
            await self._submit_flight_plan(args)
        elif args.command == "cancel":
            await self._cancel_flight_plan(args)
        else:
            self.parser.print_help()
    
    async def _create_flight_plan(self, args: argparse.Namespace) -> None:
        """
        Create a flight plan.
        
        Args:
            args: Command-line arguments
        """
        try:
            # Load flight plan data from YAML file
            with open(args.file, "r") as f:
                flight_plan_data = yaml.safe_load(f)
            
            # Send request
            async with httpx.AsyncClient() as client:
                response = await client.post(
                    f"{args.server}/api/v1/flightplans",
                    json=flight_plan_data,
                    timeout=30,
                )
                
                # Check response
                response.raise_for_status()
                
                # Print response
                print(json.dumps(response.json(), indent=2))
        except Exception as e:
            logger.error(f"Error creating flight plan: {str(e)}")
            sys.exit(1)
    
    async def _get_flight_plan(self, args: argparse.Namespace) -> None:
        """
        Get a flight plan.
        
        Args:
            args: Command-line arguments
        """
        try:
            # Send request
            async with httpx.AsyncClient() as client:
                response = await client.get(
                    f"{args.server}/api/v1/flightplans/{args.id}",
                    timeout=30,
                )
                
                # Check response
                response.raise_for_status()
                
                # Print response
                print(json.dumps(response.json(), indent=2))
        except Exception as e:
            logger.error(f"Error getting flight plan: {str(e)}")
            sys.exit(1)
    
    async def _update_flight_plan(self, args: argparse.Namespace) -> None:
        """
        Update a flight plan.
        
        Args:
            args: Command-line arguments
        """
        try:
            # Load flight plan data from YAML file
            with open(args.file, "r") as f:
                flight_plan_data = yaml.safe_load(f)
            
            # Send request
            async with httpx.AsyncClient() as client:
                response = await client.put(
                    f"{args.server}/api/v1/flightplans/{args.id}",
                    json=flight_plan_data,
                    timeout=30,
                )
                
                # Check response
                response.raise_for_status()
                
                # Print response
                print(json.dumps(response.json(), indent=2))
        except Exception as e:
            logger.error(f"Error updating flight plan: {str(e)}")
            sys.exit(1)
    
    async def _delete_flight_plan(self, args: argparse.Namespace) -> None:
        """
        Delete a flight plan.
        
        Args:
            args: Command-line arguments
        """
        try:
            # Send request
            async with httpx.AsyncClient() as client:
                response = await client.delete(
                    f"{args.server}/api/v1/flightplans/{args.id}",
                    timeout=30,
                )
                
                # Check response
                response.raise_for_status()
                
                # Print response
                print(json.dumps(response.json(), indent=2))
        except Exception as e:
            logger.error(f"Error deleting flight plan: {str(e)}")
            sys.exit(1)
    
    async def _search_flight_plans(self, args: argparse.Namespace) -> None:
        """
        Search for flight plans.
        
        Args:
            args: Command-line arguments
        """
        try:
            # Create request data
            request_data = {
                "limit": args.limit,
                "offset": args.offset,
            }
            
            # Add operator ID if provided
            if args.operator_id:
                request_data["operator_id"] = args.operator_id
            
            # Add drone ID if provided
            if args.drone_id:
                request_data["drone_id"] = args.drone_id
            
            # Add status if provided
            if args.status:
                request_data["status"] = args.status
            
            # Add start time from if provided
            if args.start_time_from:
                request_data["start_time_from"] = args.start_time_from
            
            # Add start time to if provided
            if args.start_time_to:
                request_data["start_time_to"] = args.start_time_to
            
            # Send request
            async with httpx.AsyncClient() as client:
                response = await client.post(
                    f"{args.server}/api/v1/flightplans/search",
                    json=request_data,
                    timeout=30,
                )
                
                # Check response
                response.raise_for_status()
                
                # Print response
                print(json.dumps(response.json(), indent=2))
        except Exception as e:
            logger.error(f"Error searching flight plans: {str(e)}")
            sys.exit(1)
    
    async def _submit_flight_plan(self, args: argparse.Namespace) -> None:
        """
        Submit a flight plan.
        
        Args:
            args: Command-line arguments
        """
        try:
            # Create request data
            request_data = {
                "flight_plan_id": args.id,
                "submission_type": args.type,
            }
            
            # Add additional data if provided
            if args.file:
                with open(args.file, "r") as f:
                    additional_data = yaml.safe_load(f)
                
                request_data["additional_data"] = additional_data
            
            # Send request
            async with httpx.AsyncClient() as client:
                response = await client.post(
                    f"{args.server}/api/v1/flightplans/submit",
                    json=request_data,
                    timeout=30,
                )
                
                # Check response
                response.raise_for_status()
                
                # Print response
                print(json.dumps(response.json(), indent=2))
        except Exception as e:
            logger.error(f"Error submitting flight plan: {str(e)}")
            sys.exit(1)
    
    async def _cancel_flight_plan(self, args: argparse.Namespace) -> None:
        """
        Cancel a flight plan.
        
        Args:
            args: Command-line arguments
        """
        try:
            # Create request data
            request_data = {
                "flight_plan_id": args.id,
            }
            
            # Add reason if provided
            if args.reason:
                request_data["reason"] = args.reason
            
            # Send request
            async with httpx.AsyncClient() as client:
                response = await client.post(
                    f"{args.server}/api/v1/flightplans/cancel",
                    json=request_data,
                    timeout=30,
                )
                
                # Check response
                response.raise_for_status()
                
                # Print response
                print(json.dumps(response.json(), indent=2))
        except Exception as e:
            logger.error(f"Error cancelling flight plan: {str(e)}")
            sys.exit(1)

def main():
    """
    Main entry point.
    """
    cli = FlightPlanCLI()
    asyncio.run(cli.run())

if __name__ == "__main__":
    main()
