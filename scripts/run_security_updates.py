#!/usr/bin/env python3
"""
Script to run all security updates.

This script runs all the security update scripts in the correct order.
"""

import os
import sys
import logging
import subprocess
from pathlib import Path

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger(__name__)

# Define the root directory
ROOT_DIR = Path(__file__).parent.parent
SCRIPTS_DIR = ROOT_DIR / "scripts"

# Define the scripts to run
SCRIPTS = [
    "update_dependencies.py",
    "update_auth.py",
    "update_validation.py",
]

def run_script(script_name: str) -> bool:
    """
    Run a script.
    
    Args:
        script_name: Name of the script to run
        
    Returns:
        True if the script ran successfully, False otherwise
    """
    script_path = SCRIPTS_DIR / script_name
    
    if not script_path.exists():
        logger.error(f"Script not found: {script_path}")
        return False
    
    logger.info(f"Running script: {script_name}")
    
    try:
        # Make the script executable
        os.chmod(script_path, 0o755)
        
        # Run the script
        result = subprocess.run(
            [sys.executable, str(script_path)],
            check=True,
            capture_output=True,
            text=True
        )
        
        # Log the output
        for line in result.stdout.splitlines():
            logger.info(f"  {line}")
        
        return True
    except subprocess.CalledProcessError as e:
        logger.error(f"Script failed: {script_name}")
        logger.error(f"  Exit code: {e.returncode}")
        logger.error(f"  Output: {e.stdout}")
        logger.error(f"  Error: {e.stderr}")
        return False

def main():
    """
    Main function.
    """
    logger.info("Starting security updates")
    
    # Run each script
    success = True
    for script_name in SCRIPTS:
        if not run_script(script_name):
            success = False
            logger.error(f"Failed to run script: {script_name}")
    
    if success:
        logger.info("All security updates completed successfully")
    else:
        logger.error("Some security updates failed")
        sys.exit(1)

if __name__ == "__main__":
    main()
