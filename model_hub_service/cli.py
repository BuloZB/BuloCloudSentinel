#!/usr/bin/env python3
"""
Command-line interface for the Model Hub service.

This module provides a CLI for interacting with the Model Hub service.
"""

import os
import sys
import json
import argparse
import logging
from typing import Dict, List, Any, Optional
from pathlib import Path
import hashlib
import requests

# Setup logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
)
logger = logging.getLogger(__name__)

# Default API URL
DEFAULT_API_URL = "http://localhost:8070"

def get_api_url() -> str:
    """
    Get the API URL from environment variable or use default.
    
    Returns:
        API URL
    """
    return os.environ.get("MODEL_HUB_API_URL", DEFAULT_API_URL)

def calculate_file_hash(file_path: str) -> str:
    """
    Calculate SHA-256 hash of a file.
    
    Args:
        file_path: Path to the file
        
    Returns:
        SHA-256 hash of the file
    """
    sha256_hash = hashlib.sha256()
    
    with open(file_path, "rb") as f:
        # Read and update hash in chunks
        for byte_block in iter(lambda: f.read(4096), b""):
            sha256_hash.update(byte_block)
    
    return sha256_hash.hexdigest()

def push_model(args):
    """
    Push a model to the Model Hub.
    
    Args:
        args: Command-line arguments
    """
    # Check if file exists
    if not os.path.isfile(args.path):
        logger.error(f"File not found: {args.path}")
        sys.exit(1)
    
    # Calculate file hash
    file_hash = calculate_file_hash(args.path)
    logger.info(f"File hash: {file_hash}")
    
    # Prepare request
    url = f"{get_api_url()}/api/v1/models"
    
    # Prepare form data
    form_data = {
        "name": args.name,
        "version": args.version,
        "description": args.description,
        "model_type": args.model_type,
        "framework": args.framework,
        "stage": args.stage,
    }
    
    # Prepare tags
    if args.tags:
        tags = {}
        for tag in args.tags:
            key, value = tag.split("=")
            tags[key] = value
        form_data["tags"] = json.dumps(tags)
    
    # Prepare file
    files = {
        "file": (os.path.basename(args.path), open(args.path, "rb")),
    }
    
    # Send request
    try:
        response = requests.post(url, data=form_data, files=files)
        response.raise_for_status()
        
        # Parse response
        model = response.json()
        
        logger.info(f"Model pushed successfully: {model['name']} version {model['version']}")
        logger.info(f"Model ID: {model['id']}")
        
        return model
    except requests.exceptions.RequestException as e:
        logger.error(f"Error pushing model: {e}")
        if hasattr(e, "response") and e.response is not None:
            logger.error(f"Response: {e.response.text}")
        sys.exit(1)

def list_models(args):
    """
    List models in the Model Hub.
    
    Args:
        args: Command-line arguments
    """
    # Prepare request
    url = f"{get_api_url()}/api/v1/models"
    
    # Prepare query parameters
    params = {}
    if args.name:
        params["name"] = args.name
    if args.model_type:
        params["model_type"] = args.model_type
    if args.stage:
        params["stage"] = args.stage
    if args.active is not None:
        params["is_active"] = args.active
    
    # Send request
    try:
        response = requests.get(url, params=params)
        response.raise_for_status()
        
        # Parse response
        models = response.json()
        
        # Print models
        if not models:
            logger.info("No models found")
            return
        
        logger.info(f"Found {len(models)} models:")
        for model in models:
            active_str = "active" if model["is_active"] else "inactive"
            logger.info(f"- {model['name']} version {model['version']} ({model['stage']}, {active_str})")
            logger.info(f"  ID: {model['id']}")
            logger.info(f"  Type: {model['model_type']}")
            logger.info(f"  Framework: {model['framework']}")
            if model["description"]:
                logger.info(f"  Description: {model['description']}")
            logger.info("")
        
        return models
    except requests.exceptions.RequestException as e:
        logger.error(f"Error listing models: {e}")
        if hasattr(e, "response") and e.response is not None:
            logger.error(f"Response: {e.response.text}")
        sys.exit(1)

def deploy_model(args):
    """
    Deploy a model.
    
    Args:
        args: Command-line arguments
    """
    # Prepare request
    url = f"{get_api_url()}/api/v1/deployments"
    
    # Prepare request data
    data = {
        "model_id": args.model_id,
        "environment": args.environment,
        "deployment_type": args.deployment_type,
        "target": args.target,
        "auto_rollback_enabled": not args.disable_auto_rollback,
    }
    
    if args.rollback_threshold:
        data["rollback_threshold"] = args.rollback_threshold
    
    # Send request
    try:
        response = requests.post(url, json=data)
        response.raise_for_status()
        
        # Parse response
        deployment = response.json()
        
        logger.info(f"Deployment initiated: {deployment['id']}")
        logger.info(f"Model ID: {deployment['model_id']}")
        logger.info(f"Environment: {deployment['environment']}")
        logger.info(f"Type: {deployment['deployment_type']}")
        logger.info(f"Target: {deployment['target']}")
        logger.info(f"Status: {deployment['status']}")
        
        return deployment
    except requests.exceptions.RequestException as e:
        logger.error(f"Error deploying model: {e}")
        if hasattr(e, "response") and e.response is not None:
            logger.error(f"Response: {e.response.text}")
        sys.exit(1)

def rollback_deployment(args):
    """
    Rollback a deployment.
    
    Args:
        args: Command-line arguments
    """
    # Prepare request
    url = f"{get_api_url()}/api/v1/deployments/{args.deployment_id}/rollback"
    
    # Send request
    try:
        response = requests.post(url)
        response.raise_for_status()
        
        # Parse response
        deployment = response.json()
        
        logger.info(f"Deployment rolled back: {deployment['id']}")
        logger.info(f"Model ID: {deployment['model_id']}")
        logger.info(f"Environment: {deployment['environment']}")
        logger.info(f"Status: {deployment['status']}")
        
        return deployment
    except requests.exceptions.RequestException as e:
        logger.error(f"Error rolling back deployment: {e}")
        if hasattr(e, "response") and e.response is not None:
            logger.error(f"Response: {e.response.text}")
        sys.exit(1)

def main():
    """Main entry point for the CLI."""
    # Create parser
    parser = argparse.ArgumentParser(description="Model Hub CLI")
    subparsers = parser.add_subparsers(dest="command", help="Command to run")
    
    # Push command
    push_parser = subparsers.add_parser("push", help="Push a model to the Model Hub")
    push_parser.add_argument("path", help="Path to the model file")
    push_parser.add_argument("--name", required=True, help="Name of the model")
    push_parser.add_argument("--version", required=True, help="Version of the model")
    push_parser.add_argument("--description", help="Description of the model")
    push_parser.add_argument("--model-type", required=True, help="Type of the model")
    push_parser.add_argument("--framework", required=True, help="Framework of the model")
    push_parser.add_argument("--stage", default="development", choices=["development", "staging", "production", "archived"], help="Stage of the model")
    push_parser.add_argument("--tags", nargs="+", help="Tags for the model (key=value)")
    
    # List command
    list_parser = subparsers.add_parser("list", help="List models in the Model Hub")
    list_parser.add_argument("--name", help="Filter by model name")
    list_parser.add_argument("--model-type", help="Filter by model type")
    list_parser.add_argument("--stage", choices=["development", "staging", "production", "archived"], help="Filter by model stage")
    list_parser.add_argument("--active", action="store_true", help="Filter by active status")
    list_parser.add_argument("--inactive", action="store_false", dest="active", help="Filter by inactive status")
    list_parser.set_defaults(active=None)
    
    # Deploy command
    deploy_parser = subparsers.add_parser("deploy", help="Deploy a model")
    deploy_parser.add_argument("model_id", help="ID of the model to deploy")
    deploy_parser.add_argument("--environment", default="production", help="Environment to deploy to")
    deploy_parser.add_argument("--deployment-type", default="blue-green", choices=["blue-green", "canary", "rolling"], help="Type of deployment")
    deploy_parser.add_argument("--target", default="all", choices=["edge", "cloud", "all"], help="Target for deployment")
    deploy_parser.add_argument("--disable-auto-rollback", action="store_true", help="Disable automatic rollback")
    deploy_parser.add_argument("--rollback-threshold", type=float, help="Threshold for automatic rollback")
    
    # Rollback command
    rollback_parser = subparsers.add_parser("rollback", help="Rollback a deployment")
    rollback_parser.add_argument("deployment_id", help="ID of the deployment to roll back")
    
    # Parse arguments
    args = parser.parse_args()
    
    # Run command
    if args.command == "push":
        push_model(args)
    elif args.command == "list":
        list_models(args)
    elif args.command == "deploy":
        deploy_model(args)
    elif args.command == "rollback":
        rollback_deployment(args)
    else:
        parser.print_help()
        sys.exit(1)

if __name__ == "__main__":
    main()
