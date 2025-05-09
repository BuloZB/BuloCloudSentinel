#!/usr/bin/env python3
"""
Security Review Script for Bulo.Cloud Sentinel.

This script helps implement regular security reviews as outlined in the schedule.
It generates review templates, tracks review status, and generates reports.

Usage:
    python security_review.py [--daily] [--weekly] [--monthly] [--quarterly]
"""

import argparse
import json
import logging
import os
import subprocess
import sys
from datetime import datetime, timedelta
from pathlib import Path
from typing import Dict, List, Optional, Tuple

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(levelname)s - %(message)s",
    handlers=[
        logging.FileHandler("security_review.log"),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger(__name__)

# Define the root directory
ROOT_DIR = Path(__file__).parent.parent

# Define output directory for reviews
REVIEWS_DIR = ROOT_DIR / "security_reviews"
REVIEWS_DIR.mkdir(exist_ok=True)

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

def get_recent_changes(days: int = 7) -> List[str]:
    """
    Get recent changes to the codebase.
    
    Args:
        days: Number of days to look back
        
    Returns:
        List of changed files
    """
    logger.info(f"Getting changes from the last {days} days...")
    
    # Get the date range
    end_date = datetime.now()
    start_date = end_date - timedelta(days=days)
    
    # Format dates for git
    start_date_str = start_date.strftime("%Y-%m-%d")
    end_date_str = end_date.strftime("%Y-%m-%d")
    
    # Run git command
    cmd = [
        "git", "log", "--name-only", "--pretty=format:", 
        f"--since={start_date_str}", 
        f"--until={end_date_str}"
    ]
    
    returncode, stdout, stderr = run_command(cmd, ROOT_DIR)
    
    if returncode != 0:
        logger.error(f"Failed to get recent changes: {stderr}")
        return []
    
    # Parse the output
    files = set()
    for line in stdout.splitlines():
        if line.strip():
            files.add(line.strip())
    
    return sorted(list(files))

def generate_daily_review_template() -> str:
    """
    Generate a template for daily security reviews.
    
    Returns:
        Path to the generated template
    """
    logger.info("Generating daily security review template...")
    
    # Get timestamp for filename
    timestamp = datetime.now().strftime("%Y%m%d")
    template_file = REVIEWS_DIR / f"daily_review_{timestamp}.md"
    
    # Get recent changes
    changes = get_recent_changes(days=1)
    
    # Generate template
    with open(template_file, "w") as f:
        f.write("# Daily Security Review\n\n")
        f.write(f"## Date: {datetime.now().strftime('%Y-%m-%d')}\n\n")
        f.write("## Reviewer: [Your Name]\n\n")
        
        f.write("## Automated Scan Results\n\n")
        f.write("### Dependency Scan\n\n")
        f.write("- [ ] Reviewed dependency scan results\n")
        f.write("- [ ] No vulnerable dependencies found\n")
        f.write("- [ ] Vulnerabilities have been addressed\n\n")
        
        f.write("### Static Code Analysis\n\n")
        f.write("- [ ] Reviewed static code analysis results\n")
        f.write("- [ ] No security issues found\n")
        f.write("- [ ] Security issues have been addressed\n\n")
        
        f.write("### Secret Detection\n\n")
        f.write("- [ ] Reviewed secret detection results\n")
        f.write("- [ ] No secrets found in code\n")
        f.write("- [ ] Secrets have been removed or secured\n\n")
        
        f.write("## Recent Changes\n\n")
        if changes:
            f.write("The following files have been changed in the last day:\n\n")
            for file in changes:
                f.write(f"- [ ] {file}\n")
        else:
            f.write("No changes in the last day.\n\n")
        
        f.write("## Issues Identified\n\n")
        f.write("1. \n\n")
        
        f.write("## Recommendations\n\n")
        f.write("1. \n\n")
        
        f.write("## Next Steps\n\n")
        f.write("1. \n\n")
    
    logger.info(f"Daily review template generated: {template_file}")
    return str(template_file)

def generate_weekly_review_template() -> str:
    """
    Generate a template for weekly security reviews.
    
    Returns:
        Path to the generated template
    """
    logger.info("Generating weekly security review template...")
    
    # Get timestamp for filename
    timestamp = datetime.now().strftime("%Y%m%d")
    template_file = REVIEWS_DIR / f"weekly_review_{timestamp}.md"
    
    # Get recent changes
    changes = get_recent_changes(days=7)
    
    # Generate template
    with open(template_file, "w") as f:
        f.write("# Weekly Security Review\n\n")
        f.write(f"## Week: {datetime.now().strftime('%Y-%m-%d')}\n\n")
        f.write("## Reviewer: [Your Name]\n\n")
        
        f.write("## Code Review\n\n")
        f.write("### Recent Changes\n\n")
        if changes:
            f.write("The following files have been changed in the last week:\n\n")
            for file in changes:
                f.write(f"- [ ] {file}\n")
        else:
            f.write("No changes in the last week.\n\n")
        
        f.write("### Security Issues\n\n")
        f.write("- [ ] No security issues found in code changes\n")
        f.write("- [ ] Security issues have been addressed\n\n")
        
        f.write("## Configuration Review\n\n")
        f.write("- [ ] No security issues found in configuration changes\n")
        f.write("- [ ] Security issues have been addressed\n\n")
        
        f.write("## Vulnerability Triage\n\n")
        f.write("- [ ] No new vulnerabilities identified\n")
        f.write("- [ ] Vulnerabilities have been prioritized\n")
        f.write("- [ ] Vulnerabilities have been assigned for remediation\n\n")
        
        f.write("## Issues Identified\n\n")
        f.write("1. \n\n")
        
        f.write("## Recommendations\n\n")
        f.write("1. \n\n")
        
        f.write("## Next Steps\n\n")
        f.write("1. \n\n")
    
    logger.info(f"Weekly review template generated: {template_file}")
    return str(template_file)

def generate_monthly_review_template() -> str:
    """
    Generate a template for monthly security reviews.
    
    Returns:
        Path to the generated template
    """
    logger.info("Generating monthly security review template...")
    
    # Get timestamp for filename
    timestamp = datetime.now().strftime("%Y%m")
    template_file = REVIEWS_DIR / f"monthly_review_{timestamp}.md"
    
    # Generate template
    with open(template_file, "w") as f:
        f.write("# Monthly Security Review\n\n")
        f.write(f"## Month: {datetime.now().strftime('%Y-%m')}\n\n")
        f.write("## Reviewer: [Your Name]\n\n")
        
        f.write("## Architecture Review\n\n")
        f.write("- [ ] No security issues found in architecture\n")
        f.write("- [ ] Security issues have been addressed\n\n")
        
        f.write("## Threat Modeling\n\n")
        f.write("- [ ] Threat model has been updated\n")
        f.write("- [ ] New threats have been identified\n")
        f.write("- [ ] Mitigations have been implemented\n\n")
        
        f.write("## Security Control Review\n\n")
        f.write("- [ ] Authentication controls are effective\n")
        f.write("- [ ] Authorization controls are effective\n")
        f.write("- [ ] Data protection controls are effective\n")
        f.write("- [ ] Logging and monitoring controls are effective\n\n")
        
        f.write("## Security Metrics Review\n\n")
        f.write("- [ ] Security metrics have been reviewed\n")
        f.write("- [ ] Trends have been identified\n")
        f.write("- [ ] Improvements have been implemented\n\n")
        
        f.write("## Issues Identified\n\n")
        f.write("1. \n\n")
        
        f.write("## Recommendations\n\n")
        f.write("1. \n\n")
        
        f.write("## Next Steps\n\n")
        f.write("1. \n\n")
    
    logger.info(f"Monthly review template generated: {template_file}")
    return str(template_file)

def generate_quarterly_review_template() -> str:
    """
    Generate a template for quarterly security reviews.
    
    Returns:
        Path to the generated template
    """
    logger.info("Generating quarterly security review template...")
    
    # Get timestamp for filename
    quarter = (datetime.now().month - 1) // 3 + 1
    year = datetime.now().year
    timestamp = f"{year}Q{quarter}"
    template_file = REVIEWS_DIR / f"quarterly_review_{timestamp}.md"
    
    # Generate template
    with open(template_file, "w") as f:
        f.write("# Quarterly Security Review\n\n")
        f.write(f"## Quarter: {timestamp}\n\n")
        f.write("## Reviewer: [Your Name]\n\n")
        
        f.write("## Penetration Testing\n\n")
        f.write("- [ ] Penetration testing has been performed\n")
        f.write("- [ ] Vulnerabilities have been identified\n")
        f.write("- [ ] Vulnerabilities have been addressed\n\n")
        
        f.write("## Security Policy Review\n\n")
        f.write("- [ ] Security policies have been reviewed\n")
        f.write("- [ ] Security policies have been updated\n")
        f.write("- [ ] Security policies are effective\n\n")
        
        f.write("## Security Training Review\n\n")
        f.write("- [ ] Security training materials have been reviewed\n")
        f.write("- [ ] Security training materials have been updated\n")
        f.write("- [ ] Security training has been effective\n\n")
        
        f.write("## Security Incident Review\n\n")
        f.write("- [ ] Security incidents have been reviewed\n")
        f.write("- [ ] Root causes have been identified\n")
        f.write("- [ ] Improvements have been implemented\n\n")
        
        f.write("## Issues Identified\n\n")
        f.write("1. \n\n")
        
        f.write("## Recommendations\n\n")
        f.write("1. \n\n")
        
        f.write("## Next Steps\n\n")
        f.write("1. \n\n")
    
    logger.info(f"Quarterly review template generated: {template_file}")
    return str(template_file)

def main():
    """
    Main function.
    """
    parser = argparse.ArgumentParser(description="Security review script for Bulo.Cloud Sentinel")
    parser.add_argument("--daily", action="store_true", help="Generate daily review template")
    parser.add_argument("--weekly", action="store_true", help="Generate weekly review template")
    parser.add_argument("--monthly", action="store_true", help="Generate monthly review template")
    parser.add_argument("--quarterly", action="store_true", help="Generate quarterly review template")
    
    args = parser.parse_args()
    
    # If no specific review is requested, generate daily review
    if not (args.daily or args.weekly or args.monthly or args.quarterly):
        args.daily = True
    
    # Generate requested review templates
    if args.daily:
        generate_daily_review_template()
    
    if args.weekly:
        generate_weekly_review_template()
    
    if args.monthly:
        generate_monthly_review_template()
    
    if args.quarterly:
        generate_quarterly_review_template()
    
    return 0

if __name__ == "__main__":
    sys.exit(main())
