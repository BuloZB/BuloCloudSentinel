#!/usr/bin/env python3
"""
Fix Dependabot Vulnerabilities Script for Bulo.Cloud Sentinel.

This script updates vulnerable dependencies identified by Dependabot.

Usage:
    python fix_dependabot_vulnerabilities.py [--dry-run]
"""

import argparse
import logging
import os
import re
import subprocess
import sys
from pathlib import Path
from typing import Dict, List, Optional, Tuple

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(levelname)s - %(message)s",
    handlers=[
        logging.FileHandler("fix_dependabot_vulnerabilities.log"),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger(__name__)

# Define the root directory
ROOT_DIR = Path(__file__).parent.parent

# Define the vulnerable packages and their safe versions
VULNERABLE_PACKAGES = {
    # Critical
    "torch": {
        "safe_version": ">=2.1.0",
        "files": [
            "requirements.txt"
        ],
        "severity": "Critical",
        "description": "PyTorch: 'torch.load' with 'weights_only=True' leads to remote code execution"
    },
    # High
    "aiohttp": {
        "safe_version": ">=3.9.0",
        "files": [
            "dock_driver/requirements.txt"
        ],
        "severity": "High",
        "description": "aiohttp vulnerable to Denial of Service when trying to parse malformed POST requests"
    },
    # Moderate
    "postcss": {
        "safe_version": ">=8.4.31",
        "files": [
            "frontend/package-lock.json"
        ],
        "severity": "Moderate",
        "description": "PostCSS line return parsing error"
    },
    # Low
    "cookie": {
        "safe_version": ">=0.5.0",
        "files": [
            "temp-openwebui/package-lock.json",
            "sentinelweb/package-lock.json"
        ],
        "severity": "Low",
        "description": "cookie accepts cookie name, path, and domain with out of bounds characters"
    },
    "nth-check": {
        "safe_version": ">=2.0.1",
        "files": [
            "frontend/package-lock.json"
        ],
        "severity": "High",
        "description": "Inefficient Regular Expression Complexity in nth-check"
    }
}

def find_requirements_files() -> List[Path]:
    """
    Find all requirements.txt files in the repository.
    
    Returns:
        List of paths to requirements.txt files
    """
    logger.info("Finding requirements.txt files...")
    
    requirements_files = []
    
    for root, dirs, files in os.walk(ROOT_DIR):
        for file in files:
            if file == "requirements.txt":
                requirements_files.append(Path(root) / file)
    
    logger.info(f"Found {len(requirements_files)} requirements.txt files")
    return requirements_files

def find_package_lock_files() -> List[Path]:
    """
    Find all package-lock.json files in the repository.
    
    Returns:
        List of paths to package-lock.json files
    """
    logger.info("Finding package-lock.json files...")
    
    package_lock_files = []
    
    for root, dirs, files in os.walk(ROOT_DIR):
        for file in files:
            if file == "package-lock.json":
                package_lock_files.append(Path(root) / file)
    
    logger.info(f"Found {len(package_lock_files)} package-lock.json files")
    return package_lock_files

def update_python_package(package: str, version: str, file_path: Path, dry_run: bool = False) -> bool:
    """
    Update a Python package in a requirements.txt file.
    
    Args:
        package: Package name
        version: Safe version
        file_path: Path to the requirements.txt file
        dry_run: Whether to perform a dry run
        
    Returns:
        True if the package was updated, False otherwise
    """
    logger.info(f"Updating {package} to {version} in {file_path}...")
    
    if not file_path.exists():
        logger.error(f"File {file_path} does not exist")
        return False
    
    # Read the file
    with open(file_path, "r") as f:
        content = f.read()
    
    # Check if the package is in the file
    package_regex = re.compile(rf"^{package}(==|>=|<=|~=|!=|>|<|@).*$", re.MULTILINE)
    if not package_regex.search(content):
        logger.warning(f"Package {package} not found in {file_path}")
        return False
    
    # Update the package
    updated_content = package_regex.sub(f"{package}{version}", content)
    
    # Write the updated content
    if not dry_run:
        with open(file_path, "w") as f:
            f.write(updated_content)
        
        logger.info(f"Updated {package} to {version} in {file_path}")
    else:
        logger.info(f"Would update {package} to {version} in {file_path}")
    
    return True

def update_npm_package(package: str, version: str, file_path: Path, dry_run: bool = False) -> bool:
    """
    Update an npm package in a package-lock.json file.
    
    Args:
        package: Package name
        version: Safe version
        file_path: Path to the package-lock.json file
        dry_run: Whether to perform a dry run
        
    Returns:
        True if the package was updated, False otherwise
    """
    logger.info(f"Updating {package} to {version} in {file_path}...")
    
    if not file_path.exists():
        logger.error(f"File {file_path} does not exist")
        return False
    
    # Get the directory containing the package-lock.json file
    package_dir = file_path.parent
    
    # Run npm update
    if not dry_run:
        try:
            subprocess.run(
                ["npm", "update", package, "--save"],
                cwd=package_dir,
                check=True
            )
            
            logger.info(f"Updated {package} in {file_path}")
            return True
        except subprocess.CalledProcessError as e:
            logger.error(f"Failed to update {package} in {file_path}: {e}")
            return False
    else:
        logger.info(f"Would update {package} in {file_path}")
        return True

def fix_vulnerabilities(dry_run: bool = False) -> Dict[str, List[str]]:
    """
    Fix vulnerabilities identified by Dependabot.
    
    Args:
        dry_run: Whether to perform a dry run
        
    Returns:
        Dictionary of updated packages and their files
    """
    logger.info("Fixing vulnerabilities...")
    
    # Find requirements.txt files
    requirements_files = find_requirements_files()
    
    # Find package-lock.json files
    package_lock_files = find_package_lock_files()
    
    # Track updated packages
    updated_packages = {}
    
    # Update Python packages
    for package, info in VULNERABLE_PACKAGES.items():
        updated_packages[package] = []
        
        # Check if the package is a Python package
        for file_pattern in info["files"]:
            if file_pattern.endswith("requirements.txt"):
                # Find matching requirements.txt files
                for file_path in requirements_files:
                    if str(file_path).endswith(file_pattern):
                        if update_python_package(package, info["safe_version"], file_path, dry_run):
                            updated_packages[package].append(str(file_path))
    
    # Update npm packages
    for package, info in VULNERABLE_PACKAGES.items():
        if package not in updated_packages:
            updated_packages[package] = []
        
        # Check if the package is an npm package
        for file_pattern in info["files"]:
            if file_pattern.endswith("package-lock.json"):
                # Find matching package-lock.json files
                for file_path in package_lock_files:
                    if str(file_path).endswith(file_pattern):
                        if update_npm_package(package, info["safe_version"], file_path, dry_run):
                            updated_packages[package].append(str(file_path))
    
    return updated_packages

def main():
    """
    Main function.
    """
    parser = argparse.ArgumentParser(description="Fix vulnerabilities identified by Dependabot")
    parser.add_argument("--dry-run", action="store_true", help="Perform a dry run without making changes")
    
    args = parser.parse_args()
    
    # Fix vulnerabilities
    updated_packages = fix_vulnerabilities(args.dry_run)
    
    # Print summary
    print("\nSummary of updates:")
    for package, files in updated_packages.items():
        if files:
            print(f"- {package}: Updated in {len(files)} files")
            for file in files:
                print(f"  - {file}")
        else:
            print(f"- {package}: Not updated in any files")
    
    return 0

if __name__ == "__main__":
    sys.exit(main())
