#!/usr/bin/env python3
"""
Dependency Scanning Script for Bulo.Cloud Sentinel

This script scans all Python dependencies for vulnerabilities using the safety package.
It can be run as a standalone script or as part of a CI/CD pipeline.

Usage:
    python dependency_scan.py [--fix] [--report-file REPORT_FILE]
"""

import argparse
import json
import logging
import os
import subprocess
import sys
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Set, Tuple, Optional

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger(__name__)

# Define the root directory
ROOT_DIR = Path(__file__).parent.parent

# Define output directory for reports
REPORTS_DIR = ROOT_DIR / "security_reports"
REPORTS_DIR.mkdir(exist_ok=True)

def run_command(command: List[str], cwd: Optional[Path] = None) -> Tuple[int, str, str]:
    """
    Run a command and return the exit code, stdout, and stderr.
    
    Args:
        command: Command to run
        cwd: Working directory
        
    Returns:
        Tuple of (exit_code, stdout, stderr)
    """
    try:
        process = subprocess.Popen(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            cwd=cwd
        )
        stdout, stderr = process.communicate()
        return process.returncode, stdout, stderr
    except Exception as e:
        return 1, "", str(e)

def check_tool_installed(tool: str) -> bool:
    """
    Check if a tool is installed.
    
    Args:
        tool: Tool name
        
    Returns:
        True if installed, False otherwise
    """
    try:
        if tool == "safety":
            cmd = [sys.executable, "-m", "safety", "--version"]
        else:
            cmd = [tool, "--version"]
            
        returncode, _, _ = run_command(cmd)
        return returncode == 0
    except Exception:
        return False

def install_safety():
    """
    Install the safety package if not already installed.
    
    Returns:
        True if successful, False otherwise
    """
    logger.info("Installing safety package...")
    cmd = [sys.executable, "-m", "pip", "install", "safety"]
    returncode, stdout, stderr = run_command(cmd)
    
    if returncode != 0:
        logger.error(f"Failed to install safety: {stderr}")
        return False
    
    logger.info("Safety package installed successfully")
    return True

def find_requirements_files() -> List[Path]:
    """
    Find all requirements files in the project.
    
    Returns:
        List of requirements file paths
    """
    logger.info("Finding requirements files...")
    
    requirements_files = []
    
    # Find all requirements.txt files
    for path in ROOT_DIR.glob("**/requirements*.txt"):
        if "venv" not in str(path) and ".venv" not in str(path):
            requirements_files.append(path)
    
    # Find all pyproject.toml files
    for path in ROOT_DIR.glob("**/pyproject.toml"):
        if "venv" not in str(path) and ".venv" not in str(path):
            requirements_files.append(path)
    
    # Find all setup.py files
    for path in ROOT_DIR.glob("**/setup.py"):
        if "venv" not in str(path) and ".venv" not in str(path):
            requirements_files.append(path)
    
    logger.info(f"Found {len(requirements_files)} requirements files")
    return requirements_files

def scan_dependencies(requirements_file: Path = None) -> Tuple[bool, Dict]:
    """
    Scan dependencies for vulnerabilities using safety.
    
    Args:
        requirements_file: Path to requirements file to scan (optional)
        
    Returns:
        Tuple of (success, report)
    """
    if requirements_file:
        logger.info(f"Scanning dependencies in {requirements_file}...")
    else:
        logger.info("Scanning all installed dependencies...")
    
    if not check_tool_installed("safety"):
        logger.warning("Safety is not installed. Installing now...")
        if not install_safety():
            return False, {}
    
    # Get timestamp for report filename
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    report_file = REPORTS_DIR / f"dependency_scan_{timestamp}.json"
    
    # Build the command
    cmd = [sys.executable, "-m", "safety", "check", "--full-report", "--json"]
    
    # Add requirements file if specified
    if requirements_file:
        cmd.extend(["-r", str(requirements_file)])
    
    # Run safety check
    returncode, stdout, stderr = run_command(cmd)
    
    # Parse the report
    try:
        report = json.loads(stdout)
        
        # Save the report
        with open(report_file, "w") as f:
            json.dump(report, f, indent=2)
        
        vulnerabilities = report.get("vulnerabilities", [])
        if vulnerabilities:
            logger.warning(f"Found {len(vulnerabilities)} vulnerabilities in dependencies")
            for vuln in vulnerabilities:
                logger.warning(f"  - {vuln.get('package_name')} {vuln.get('vulnerable_spec')}: {vuln.get('advisory')}")
            
            logger.info(f"Detailed report saved to {report_file}")
            return False, report
        else:
            logger.info("No vulnerabilities found in dependencies")
            return True, report
    except Exception as e:
        logger.error(f"Failed to parse safety report: {str(e)}")
        return False, {}

def fix_vulnerabilities(report: Dict) -> bool:
    """
    Fix vulnerabilities by updating dependencies.
    
    Args:
        report: Vulnerability report from safety
        
    Returns:
        True if successful, False otherwise
    """
    logger.info("Fixing vulnerabilities...")
    
    # Get vulnerabilities from report
    vulnerabilities = report.get("vulnerabilities", [])
    if not vulnerabilities:
        logger.info("No vulnerabilities to fix")
        return True
    
    # Run the update_dependencies.py script
    logger.info("Running update_dependencies.py script...")
    update_script = ROOT_DIR / "scripts" / "update_dependencies.py"
    
    if not update_script.exists():
        logger.error("update_dependencies.py script not found")
        return False
    
    cmd = [sys.executable, str(update_script)]
    returncode, stdout, stderr = run_command(cmd)
    
    if returncode != 0:
        logger.error(f"Failed to run update_dependencies.py: {stderr}")
        return False
    
    logger.info("Dependencies updated successfully")
    
    # Verify the fix
    logger.info("Verifying fix...")
    success, new_report = scan_dependencies()
    
    if success:
        logger.info("All vulnerabilities fixed successfully")
        return True
    else:
        new_vulnerabilities = new_report.get("vulnerabilities", [])
        if len(new_vulnerabilities) < len(vulnerabilities):
            logger.warning(f"Fixed some vulnerabilities. {len(new_vulnerabilities)} remaining")
        else:
            logger.error("Failed to fix vulnerabilities")
        
        return False

def main():
    """
    Main function.
    """
    parser = argparse.ArgumentParser(description="Dependency scanning script for Bulo.Cloud Sentinel")
    parser.add_argument("--fix", action="store_true", help="Attempt to fix vulnerabilities")
    parser.add_argument("--report-file", help="Path to save the report")
    
    args = parser.parse_args()
    
    # Scan all dependencies
    success, report = scan_dependencies()
    
    # Fix vulnerabilities if requested
    if not success and args.fix:
        fix_vulnerabilities(report)
    
    # Return success status
    return 0 if success else 1

if __name__ == "__main__":
    sys.exit(main())
