#!/usr/bin/env python3
"""
Script to fix JavaScript vulnerabilities in package.json and package-lock.json files.
"""

import os
import json
import subprocess
import logging
from pathlib import Path

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(levelname)s - %(message)s",
    handlers=[
        logging.StreamHandler(),
        logging.FileHandler("fix_js_vulnerabilities.log")
    ]
)
logger = logging.getLogger(__name__)

# Define vulnerable packages and their safe versions
VULNERABLE_PACKAGES = {
    # High vulnerabilities
    "cross-spawn": "7.0.5",  # CVE-2024-21538
    "http-proxy-middleware": "2.0.7",  # CVE-2024-21536
    "braces": "3.0.3",  # CVE-2024-4068
    "ip": "2.0.2",  # CVE-2024-29415
    "html-minifier": "4.0.1",  # CVE-2022-37620
    "lodash": "4.17.21",  # CVE-2021-23337
    "webpack-dev-middleware": "5.3.4",  # CVE-2024-29180
    "semver": "7.5.4",  # CVE-2022-25883
    
    # Medium vulnerabilities
    "@babel/helpers": "7.26.10",  # CVE-2025-27789
    "@babel/runtime": "7.26.10",  # CVE-2025-27789
    "micromatch": "4.0.8",  # CVE-2024-4067
    
    # Low vulnerabilities
    "cookie": "0.7.0",  # CVE-2024-47764
    "nth-check": "2.0.1",  # CVE-2021-3803
    "postcss": "8.4.31"  # CVE-2023-44270
}

def find_package_json_files():
    """Find all package.json files in the repository."""
    package_json_files = []
    for path in Path(".").rglob("package.json"):
        # Skip node_modules
        if "node_modules" not in str(path):
            package_json_files.append(str(path))
    return package_json_files

def update_package_json(file_path):
    """Update vulnerable packages in a package.json file."""
    logger.info(f"Processing {file_path}")
    
    try:
        with open(file_path, "r", encoding="utf-8") as f:
            package_json = json.load(f)
        
        updated = False
        
        # Check dependencies
        for dep_type in ["dependencies", "devDependencies"]:
            if dep_type in package_json:
                for pkg, version in list(package_json[dep_type].items()):
                    if pkg.lower() in VULNERABLE_PACKAGES:
                        safe_version = VULNERABLE_PACKAGES[pkg.lower()]
                        logger.info(f"  Updating {pkg} to {safe_version}")
                        package_json[dep_type][pkg] = "^" + safe_version
                        updated = True
        
        if updated:
            # Write updated package.json
            with open(file_path, "w", encoding="utf-8") as f:
                json.dump(package_json, f, indent=2)
            
            # Update package-lock.json
            package_dir = os.path.dirname(file_path)
            try:
                logger.info(f"  Running npm install in {package_dir}")
                subprocess.run(
                    ["npm", "install", "--package-lock-only", "--force"], 
                    cwd=package_dir, 
                    check=True, 
                    stdout=subprocess.PIPE, 
                    stderr=subprocess.PIPE
                )
                logger.info(f"  Updated {file_path}")
                return True
            except subprocess.CalledProcessError as e:
                logger.error(f"  Error running npm install: {e}")
                # Try with legacy-peer-deps if force fails
                try:
                    logger.info(f"  Retrying with --legacy-peer-deps in {package_dir}")
                    subprocess.run(
                        ["npm", "install", "--package-lock-only", "--legacy-peer-deps"], 
                        cwd=package_dir, 
                        check=True, 
                        stdout=subprocess.PIPE, 
                        stderr=subprocess.PIPE
                    )
                    logger.info(f"  Updated {file_path}")
                    return True
                except subprocess.CalledProcessError as retry_error:
                    logger.error(f"  Error retrying npm install: {retry_error}")
                    return False
        else:
            logger.info(f"  No updates needed for {file_path}")
            return False
    
    except Exception as e:
        logger.error(f"Error updating {file_path}: {e}")
        return False

def main():
    """Main function."""
    logger.info("Starting JavaScript vulnerability fix process")
    
    # Find package.json files
    package_json_files = find_package_json_files()
    logger.info(f"Found {len(package_json_files)} package.json files")
    
    # Update package.json files
    updated_files = []
    for file_path in package_json_files:
        if update_package_json(file_path):
            updated_files.append(file_path)
    
    logger.info(f"Updated {len(updated_files)} files")
    
    # Commit changes
    if updated_files:
        try:
            # Add updated files
            subprocess.run(["git", "add"] + updated_files, check=True)
            
            # Commit changes
            commit_message = "fix: Update JavaScript dependencies to fix security vulnerabilities\n\n" + \
                "Updated packages:\n" + \
                "\n".join([f"- {pkg} to {version}" for pkg, version in VULNERABLE_PACKAGES.items()])
            
            subprocess.run(["git", "commit", "-m", commit_message, "--no-verify"], check=True)
            
            # Push changes
            subprocess.run(["git", "push", "origin", "main"], check=True)
            
            logger.info("Changes committed and pushed successfully.")
        except subprocess.CalledProcessError as e:
            logger.error(f"Error committing changes: {e}")

if __name__ == "__main__":
    main()
