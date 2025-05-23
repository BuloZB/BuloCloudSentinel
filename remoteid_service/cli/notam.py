"""
NOTAM CLI for the Remote ID & Regulatory Compliance Service.

This module provides a command-line interface for NOTAM functionality.
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

from remoteid_service.api.schemas.notams import (
    NOTAMSource,
    NOTAMType,
    NOTAMStatus,
)

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
)
logger = logging.getLogger("notam_cli")

class NOTAMCLI:
    """
    NOTAM CLI.
    
    This class provides a command-line interface for NOTAM functionality.
    """
    
    def __init__(self):
        """
        Initialize the NOTAM CLI.
        """
        self.parser = self._create_parser()
    
    def _create_parser(self) -> argparse.ArgumentParser:
        """
        Create the argument parser.
        
        Returns:
            argparse.ArgumentParser: Argument parser
        """
        parser = argparse.ArgumentParser(
            description="NOTAM CLI for the Remote ID & Regulatory Compliance Service",
        )
        
        # Add subparsers
        subparsers = parser.add_subparsers(dest="command", help="Command")
        
        # Import command
        import_parser = subparsers.add_parser("import", help="Import NOTAMs")
        import_parser.add_argument("--source", choices=["faa", "easa", "eurocontrol", "icao", "other"], required=True, help="NOTAM source")
        import_parser.add_argument("--region", help="Region")
        import_parser.add_argument("--start-time", help="Start time (ISO format)")
        import_parser.add_argument("--end-time", help="End time (ISO format)")
        import_parser.add_argument("--location", help="Location code")
        import_parser.add_argument("--force", action="store_true", help="Force update")
        import_parser.add_argument("--server", default="http://localhost:8080", help="Server URL")
        
        # Get command
        get_parser = subparsers.add_parser("get", help="Get a NOTAM")
        get_parser.add_argument("--id", required=True, help="NOTAM ID")
        get_parser.add_argument("--server", default="http://localhost:8080", help="Server URL")
        
        # Search command
        search_parser = subparsers.add_parser("search", help="Search for NOTAMs")
        search_parser.add_argument("--source", choices=["faa", "easa", "eurocontrol", "icao", "other"], help="NOTAM source")
        search_parser.add_argument("--type", choices=["airspace", "obstacle", "airport", "procedure", "other"], help="NOTAM type")
        search_parser.add_argument("--location", help="Location code")
        search_parser.add_argument("--status", choices=["active", "inactive", "upcoming", "expired"], help="Status")
        search_parser.add_argument("--start-from", help="Effective start from (ISO format)")
        search_parser.add_argument("--start-to", help="Effective start to (ISO format)")
        search_parser.add_argument("--end-from", help="Effective end from (ISO format)")
        search_parser.add_argument("--end-to", help="Effective end to (ISO format)")
        search_parser.add_argument("--area", help="YAML file containing area polygon")
        search_parser.add_argument("--point", help="YAML file containing point")
        search_parser.add_argument("--radius", type=float, help="Radius (meters)")
        search_parser.add_argument("--limit", type=int, default=100, help="Limit")
        search_parser.add_argument("--offset", type=int, default=0, help="Offset")
        search_parser.add_argument("--server", default="http://localhost:8080", help="Server URL")
        
        # Check flight plan command
        check_parser = subparsers.add_parser("check", help="Check a flight plan for NOTAM conflicts")
        check_parser.add_argument("--flight-plan-id", required=True, help="Flight plan ID")
        check_parser.add_argument("--server", default="http://localhost:8080", help="Server URL")
        
        # Sources command
        sources_parser = subparsers.add_parser("sources", help="Get available NOTAM sources")
        sources_parser.add_argument("--server", default="http://localhost:8080", help="Server URL")
        
        # Regions command
        regions_parser = subparsers.add_parser("regions", help="Get available NOTAM regions for a source")
        regions_parser.add_argument("--source", choices=["faa", "easa", "eurocontrol", "icao", "other"], required=True, help="NOTAM source")
        regions_parser.add_argument("--server", default="http://localhost:8080", help="Server URL")
        
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
        if args.command == "import":
            await self._import_notams(args)
        elif args.command == "get":
            await self._get_notam(args)
        elif args.command == "search":
            await self._search_notams(args)
        elif args.command == "check":
            await self._check_flight_plan(args)
        elif args.command == "sources":
            await self._get_sources(args)
        elif args.command == "regions":
            await self._get_regions(args)
        else:
            self.parser.print_help()
    
    async def _import_notams(self, args: argparse.Namespace) -> None:
        """
        Import NOTAMs.
        
        Args:
            args: Command-line arguments
        """
        try:
            # Create request data
            request_data = {
                "source": args.source,
                "force_update": args.force,
            }
            
            # Add region if provided
            if args.region:
                request_data["region"] = args.region
            
            # Add start time if provided
            if args.start_time:
                request_data["start_time"] = args.start_time
            
            # Add end time if provided
            if args.end_time:
                request_data["end_time"] = args.end_time
            
            # Add location if provided
            if args.location:
                request_data["location_code"] = args.location
            
            # Send request
            async with httpx.AsyncClient() as client:
                response = await client.post(
                    f"{args.server}/api/v1/notams/import",
                    json=request_data,
                    timeout=30,
                )
                
                # Check response
                response.raise_for_status()
                
                # Print response
                print(json.dumps(response.json(), indent=2))
        except Exception as e:
            logger.error(f"Error importing NOTAMs: {str(e)}")
            sys.exit(1)
    
    async def _get_notam(self, args: argparse.Namespace) -> None:
        """
        Get a NOTAM.
        
        Args:
            args: Command-line arguments
        """
        try:
            # Send request
            async with httpx.AsyncClient() as client:
                response = await client.get(
                    f"{args.server}/api/v1/notams/{args.id}",
                    timeout=30,
                )
                
                # Check response
                response.raise_for_status()
                
                # Print response
                print(json.dumps(response.json(), indent=2))
        except Exception as e:
            logger.error(f"Error getting NOTAM: {str(e)}")
            sys.exit(1)
    
    async def _search_notams(self, args: argparse.Namespace) -> None:
        """
        Search for NOTAMs.
        
        Args:
            args: Command-line arguments
        """
        try:
            # Create request data
            request_data = {
                "limit": args.limit,
                "offset": args.offset,
            }
            
            # Add source if provided
            if args.source:
                request_data["source"] = args.source
            
            # Add type if provided
            if args.type:
                request_data["notam_type"] = args.type
            
            # Add location if provided
            if args.location:
                request_data["location_code"] = args.location
            
            # Add status if provided
            if args.status:
                request_data["status"] = args.status
            
            # Add effective start from if provided
            if args.start_from:
                request_data["effective_start_from"] = args.start_from
            
            # Add effective start to if provided
            if args.start_to:
                request_data["effective_start_to"] = args.start_to
            
            # Add effective end from if provided
            if args.end_from:
                request_data["effective_end_from"] = args.end_from
            
            # Add effective end to if provided
            if args.end_to:
                request_data["effective_end_to"] = args.end_to
            
            # Add area if provided
            if args.area:
                with open(args.area, "r") as f:
                    area_data = yaml.safe_load(f)
                
                request_data["area"] = area_data
            
            # Add point if provided
            if args.point:
                with open(args.point, "r") as f:
                    point_data = yaml.safe_load(f)
                
                request_data["point"] = point_data
            
            # Add radius if provided
            if args.radius:
                request_data["radius"] = args.radius
            
            # Send request
            async with httpx.AsyncClient() as client:
                response = await client.post(
                    f"{args.server}/api/v1/notams/search",
                    json=request_data,
                    timeout=30,
                )
                
                # Check response
                response.raise_for_status()
                
                # Print response
                print(json.dumps(response.json(), indent=2))
        except Exception as e:
            logger.error(f"Error searching NOTAMs: {str(e)}")
            sys.exit(1)
    
    async def _check_flight_plan(self, args: argparse.Namespace) -> None:
        """
        Check a flight plan for NOTAM conflicts.
        
        Args:
            args: Command-line arguments
        """
        try:
            # Create request data
            request_data = {
                "flight_plan_id": args.flight_plan_id,
            }
            
            # Send request
            async with httpx.AsyncClient() as client:
                response = await client.post(
                    f"{args.server}/api/v1/notams/check-flight-plan",
                    json=request_data,
                    timeout=30,
                )
                
                # Check response
                response.raise_for_status()
                
                # Print response
                print(json.dumps(response.json(), indent=2))
        except Exception as e:
            logger.error(f"Error checking flight plan: {str(e)}")
            sys.exit(1)
    
    async def _get_sources(self, args: argparse.Namespace) -> None:
        """
        Get available NOTAM sources.
        
        Args:
            args: Command-line arguments
        """
        try:
            # Send request
            async with httpx.AsyncClient() as client:
                response = await client.get(
                    f"{args.server}/api/v1/notams/sources",
                    timeout=30,
                )
                
                # Check response
                response.raise_for_status()
                
                # Print response
                print(json.dumps(response.json(), indent=2))
        except Exception as e:
            logger.error(f"Error getting NOTAM sources: {str(e)}")
            sys.exit(1)
    
    async def _get_regions(self, args: argparse.Namespace) -> None:
        """
        Get available NOTAM regions for a source.
        
        Args:
            args: Command-line arguments
        """
        try:
            # Send request
            async with httpx.AsyncClient() as client:
                response = await client.get(
                    f"{args.server}/api/v1/notams/regions/{args.source}",
                    timeout=30,
                )
                
                # Check response
                response.raise_for_status()
                
                # Print response
                print(json.dumps(response.json(), indent=2))
        except Exception as e:
            logger.error(f"Error getting NOTAM regions: {str(e)}")
            sys.exit(1)

def main():
    """
    Main entry point.
    """
    cli = NOTAMCLI()
    asyncio.run(cli.run())

if __name__ == "__main__":
    main()
