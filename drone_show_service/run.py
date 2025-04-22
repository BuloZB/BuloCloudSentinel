"""
Run script for the Drone Show microservice.

This script provides a convenient way to run the Drone Show microservice.
"""

import uvicorn
import os
import argparse


def main():
    """Run the Drone Show microservice."""
    parser = argparse.ArgumentParser(description="Run the Drone Show microservice")
    parser.add_argument("--host", default="0.0.0.0", help="Host to bind to")
    parser.add_argument("--port", type=int, default=8000, help="Port to bind to")
    parser.add_argument("--reload", action="store_true", help="Enable auto-reload")
    parser.add_argument("--debug", action="store_true", help="Enable debug mode")
    args = parser.parse_args()
    
    # Set environment variables
    if args.debug:
        os.environ["LOG_LEVEL"] = "DEBUG"
    
    # Run the application
    uvicorn.run(
        "drone_show_service.api.main:app",
        host=args.host,
        port=args.port,
        reload=args.reload,
    )


if __name__ == "__main__":
    main()
