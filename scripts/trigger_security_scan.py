#!/usr/bin/env python3
"""
Trigger Security Scan Script for Bulo.Cloud Sentinel.

This script triggers the security-scan.yml workflow on GitHub.
It requires a GitHub personal access token with the 'workflow' scope.

Usage:
    python trigger_security_scan.py --token GITHUB_TOKEN
"""

import argparse
import json
import logging
import os
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Tuple
import urllib.request
import urllib.error

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(levelname)s - %(message)s",
    handlers=[
        logging.FileHandler("trigger_security_scan.log"),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger(__name__)

# Define the repository information
REPO_OWNER = "BuloZB"
REPO_NAME = "BuloCloudSentinel"
WORKFLOW_ID = "security-scan.yml"

def trigger_workflow(token: str) -> bool:
    """
    Trigger the security-scan.yml workflow on GitHub.
    
    Args:
        token: GitHub personal access token
        
    Returns:
        True if the workflow was triggered successfully, False otherwise
    """
    logger.info(f"Triggering workflow {WORKFLOW_ID}...")
    
    url = f"https://api.github.com/repos/{REPO_OWNER}/{REPO_NAME}/actions/workflows/{WORKFLOW_ID}/dispatches"
    
    headers = {
        "Accept": "application/vnd.github.v3+json",
        "Authorization": f"token {token}",
        "Content-Type": "application/json"
    }
    
    data = {
        "ref": "main"
    }
    
    try:
        request = urllib.request.Request(
            url,
            data=json.dumps(data).encode("utf-8"),
            headers=headers,
            method="POST"
        )
        
        with urllib.request.urlopen(request) as response:
            if response.getcode() == 204:
                logger.info("Workflow triggered successfully")
                return True
            else:
                logger.error(f"Failed to trigger workflow: {response.getcode()}")
                return False
    except urllib.error.HTTPError as e:
        logger.error(f"Failed to trigger workflow: {e.code} {e.reason}")
        return False
    except Exception as e:
        logger.error(f"Failed to trigger workflow: {str(e)}")
        return False

def check_workflow_status(token: str) -> Dict:
    """
    Check the status of the security-scan.yml workflow on GitHub.
    
    Args:
        token: GitHub personal access token
        
    Returns:
        Workflow status information
    """
    logger.info(f"Checking workflow status...")
    
    url = f"https://api.github.com/repos/{REPO_OWNER}/{REPO_NAME}/actions/workflows/{WORKFLOW_ID}/runs"
    
    headers = {
        "Accept": "application/vnd.github.v3+json",
        "Authorization": f"token {token}"
    }
    
    try:
        request = urllib.request.Request(
            url,
            headers=headers,
            method="GET"
        )
        
        with urllib.request.urlopen(request) as response:
            data = json.loads(response.read().decode("utf-8"))
            
            if "workflow_runs" in data and len(data["workflow_runs"]) > 0:
                latest_run = data["workflow_runs"][0]
                
                return {
                    "id": latest_run["id"],
                    "status": latest_run["status"],
                    "conclusion": latest_run.get("conclusion"),
                    "created_at": latest_run["created_at"],
                    "updated_at": latest_run["updated_at"],
                    "html_url": latest_run["html_url"]
                }
            else:
                logger.error("No workflow runs found")
                return {}
    except urllib.error.HTTPError as e:
        logger.error(f"Failed to check workflow status: {e.code} {e.reason}")
        return {}
    except Exception as e:
        logger.error(f"Failed to check workflow status: {str(e)}")
        return {}

def wait_for_workflow_completion(token: str, timeout: int = 600) -> Dict:
    """
    Wait for the security-scan.yml workflow to complete.
    
    Args:
        token: GitHub personal access token
        timeout: Timeout in seconds
        
    Returns:
        Workflow status information
    """
    logger.info(f"Waiting for workflow to complete (timeout: {timeout} seconds)...")
    
    start_time = time.time()
    
    while time.time() - start_time < timeout:
        status = check_workflow_status(token)
        
        if not status:
            logger.error("Failed to get workflow status")
            return {}
        
        if status["status"] == "completed":
            logger.info(f"Workflow completed with conclusion: {status['conclusion']}")
            return status
        
        logger.info(f"Workflow status: {status['status']}")
        
        # Wait for 30 seconds before checking again
        time.sleep(30)
    
    logger.error(f"Workflow did not complete within {timeout} seconds")
    return status

def main():
    """
    Main function.
    """
    parser = argparse.ArgumentParser(description="Trigger the security-scan.yml workflow on GitHub")
    parser.add_argument("--token", help="GitHub personal access token")
    parser.add_argument("--wait", action="store_true", help="Wait for the workflow to complete")
    parser.add_argument("--timeout", type=int, default=600, help="Timeout in seconds when waiting for workflow completion")
    
    args = parser.parse_args()
    
    # Get the GitHub token
    token = args.token or os.environ.get("GITHUB_TOKEN")
    
    if not token:
        logger.error("GitHub token not provided")
        logger.info("Please provide a GitHub token using the --token option or the GITHUB_TOKEN environment variable")
        return 1
    
    # Trigger the workflow
    if not trigger_workflow(token):
        return 1
    
    # Wait for the workflow to complete if requested
    if args.wait:
        status = wait_for_workflow_completion(token, args.timeout)
        
        if not status:
            return 1
        
        if status["conclusion"] != "success":
            logger.error(f"Workflow failed with conclusion: {status['conclusion']}")
            logger.info(f"See details at: {status['html_url']}")
            return 1
    else:
        # Just check the status once
        status = check_workflow_status(token)
        
        if status:
            logger.info(f"Workflow status: {status['status']}")
            logger.info(f"See details at: {status['html_url']}")
    
    return 0

if __name__ == "__main__":
    sys.exit(main())
