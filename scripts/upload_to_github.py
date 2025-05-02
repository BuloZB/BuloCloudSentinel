#!/usr/bin/env python3
"""
Script to upload the repository to GitHub.

This script:
1. Runs the prepare_github_upload.py script to prepare the repository
2. Adds all files to git
3. Commits the changes
4. Pushes to GitHub
"""

import os
import sys
import logging
import subprocess
import argparse
from pathlib import Path

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger(__name__)

# Define the root directory
ROOT_DIR = Path(__file__).parent.parent

def run_prepare_script() -> bool:
    """
    Run the prepare_github_upload.py script.
    
    Returns:
        True if successful, False otherwise
    """
    logger.info("Running prepare_github_upload.py")
    
    prepare_script = ROOT_DIR / "scripts" / "prepare_github_upload.py"
    
    try:
        result = subprocess.run(
            [sys.executable, str(prepare_script)],
            check=True,
            capture_output=True,
            text=True
        )
        
        # Log the output
        for line in result.stdout.splitlines():
            logger.info(f"  {line}")
        
        return True
    except subprocess.CalledProcessError as e:
        logger.error(f"Error running prepare_github_upload.py: {e.returncode}")
        logger.error(f"Output: {e.stdout}")
        logger.error(f"Error: {e.stderr}")
        return False

def git_add_all() -> bool:
    """
    Add all files to git.
    
    Returns:
        True if successful, False otherwise
    """
    logger.info("Adding all files to git")
    
    try:
        subprocess.run(
            ["git", "add", "."],
            check=True,
            capture_output=True,
            text=True
        )
        
        return True
    except subprocess.CalledProcessError as e:
        logger.error(f"Error adding files to git: {e.returncode}")
        logger.error(f"Output: {e.stdout}")
        logger.error(f"Error: {e.stderr}")
        return False

def git_commit(message: str) -> bool:
    """
    Commit the changes.
    
    Args:
        message: Commit message
        
    Returns:
        True if successful, False otherwise
    """
    logger.info(f"Committing changes with message: {message}")
    
    try:
        subprocess.run(
            ["git", "commit", "-m", message],
            check=True,
            capture_output=True,
            text=True
        )
        
        return True
    except subprocess.CalledProcessError as e:
        logger.error(f"Error committing changes: {e.returncode}")
        logger.error(f"Output: {e.stdout}")
        logger.error(f"Error: {e.stderr}")
        return False

def git_push(branch: str) -> bool:
    """
    Push to GitHub.
    
    Args:
        branch: Branch to push to
        
    Returns:
        True if successful, False otherwise
    """
    logger.info(f"Pushing to GitHub branch: {branch}")
    
    try:
        subprocess.run(
            ["git", "push", "origin", branch],
            check=True,
            capture_output=True,
            text=True
        )
        
        return True
    except subprocess.CalledProcessError as e:
        logger.error(f"Error pushing to GitHub: {e.returncode}")
        logger.error(f"Output: {e.stdout}")
        logger.error(f"Error: {e.stderr}")
        return False

def main() -> int:
    """
    Main function.
    
    Returns:
        Exit code (0 for success, non-zero for failure)
    """
    parser = argparse.ArgumentParser(description="Upload repository to GitHub")
    parser.add_argument("--message", "-m", default="Update repository with security improvements", help="Commit message")
    parser.add_argument("--branch", "-b", default="main", help="Branch to push to")
    parser.add_argument("--skip-prepare", action="store_true", help="Skip running prepare_github_upload.py")
    
    args = parser.parse_args()
    
    logger.info("Uploading repository to GitHub")
    
    # Run prepare script
    if not args.skip_prepare:
        if not run_prepare_script():
            logger.error("Failed to prepare repository")
            return 1
    
    # Add all files to git
    if not git_add_all():
        logger.error("Failed to add files to git")
        return 1
    
    # Commit the changes
    if not git_commit(args.message):
        logger.error("Failed to commit changes")
        return 1
    
    # Push to GitHub
    if not git_push(args.branch):
        logger.error("Failed to push to GitHub")
        return 1
    
    logger.info("Successfully uploaded repository to GitHub")
    
    return 0

if __name__ == "__main__":
    sys.exit(main())
