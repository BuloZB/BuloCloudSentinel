#!/usr/bin/env python3
"""
Monitor Security Reports Script for Bulo.Cloud Sentinel.

This script monitors security reports and generates a summary.
It can be used to track the progress of security improvements over time.

Usage:
    python monitor_security_reports.py [--report-dir REPORT_DIR] [--output OUTPUT]
"""

import argparse
import json
import logging
import os
import sys
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Tuple

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(levelname)s - %(message)s",
    handlers=[
        logging.FileHandler("monitor_security_reports.log"),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger(__name__)

# Define the root directory
ROOT_DIR = Path(__file__).parent.parent

# Define output directory for reports
REPORTS_DIR = ROOT_DIR / "security_reports"

def get_security_reports() -> Dict[str, List[Path]]:
    """
    Get all security reports.
    
    Returns:
        Dictionary of report types and their paths
    """
    logger.info("Getting security reports...")
    
    # Check if the reports directory exists
    if not REPORTS_DIR.exists():
        logger.error(f"Reports directory {REPORTS_DIR} does not exist")
        return {}
    
    # Get all report files
    report_files = list(REPORTS_DIR.glob("*.json")) + list(REPORTS_DIR.glob("*.md"))
    
    # Group reports by type
    reports = {
        "dependency": [],
        "code": [],
        "secret": [],
        "security": []
    }
    
    for file in report_files:
        if "dependency" in file.name:
            reports["dependency"].append(file)
        elif "code" in file.name:
            reports["code"].append(file)
        elif "secret" in file.name:
            reports["secret"].append(file)
        elif "security" in file.name:
            reports["security"].append(file)
    
    # Sort reports by date (newest first)
    for report_type in reports:
        reports[report_type] = sorted(reports[report_type], key=lambda x: x.stat().st_mtime, reverse=True)
    
    return reports

def parse_dependency_report(report_path: Path) -> Dict:
    """
    Parse a dependency scan report.
    
    Args:
        report_path: Path to the report file
        
    Returns:
        Parsed report data
    """
    logger.info(f"Parsing dependency report {report_path}...")
    
    try:
        with open(report_path, "r") as f:
            return json.load(f)
    except Exception as e:
        logger.error(f"Failed to parse dependency report {report_path}: {str(e)}")
        return {}

def parse_code_report(report_path: Path) -> Dict:
    """
    Parse a code scan report.
    
    Args:
        report_path: Path to the report file
        
    Returns:
        Parsed report data
    """
    logger.info(f"Parsing code report {report_path}...")
    
    try:
        with open(report_path, "r") as f:
            return json.load(f)
    except Exception as e:
        logger.error(f"Failed to parse code report {report_path}: {str(e)}")
        return {}

def parse_secret_report(report_path: Path) -> Dict:
    """
    Parse a secret scan report.
    
    Args:
        report_path: Path to the report file
        
    Returns:
        Parsed report data
    """
    logger.info(f"Parsing [REDACTED])
    
    try:
        with open(report_path, "r") as f:
            return json.load(f)
    except Exception as e:
        logger.error(f"Failed to parse [REDACTED])}")
        return {}

def parse_security_report(report_path: Path) -> Dict:
    """
    Parse a security report.
    
    Args:
        report_path: Path to the report file
        
    Returns:
        Parsed report data
    """
    logger.info(f"Parsing security report {report_path}...")
    
    try:
        with open(report_path, "r") as f:
            content = f.read()
        
        # Parse the report content
        report_data = {
            "date": "",
            "status": "",
            "dependency": "",
            "code": "",
            "secret": ""
        }
        
        # Extract the date
        date_match = content.split("## Scan Date: ")[1].split("\n")[0] if "## Scan Date: " in content else ""
        report_data["date"] = date_match
        
        # Extract the status
        status_match = content.split("## Overall Status: ")[1].split("\n")[0] if "## Overall Status: " in content else ""
        report_data["status"] = status_match
        
        # Extract the dependency status
        dependency_match = content.split("## Dependency Scan\n\nStatus: ")[1].split("\n")[0] if "## Dependency Scan\n\nStatus: " in content else ""
        report_data["dependency"] = dependency_match
        
        # Extract the code status
        code_match = content.split("## Code Security Scan\n\nStatus: ")[1].split("\n")[0] if "## Code Security Scan\n\nStatus: " in content else ""
        report_data["code"] = code_match
        
        # Extract the secret status
        secret_match = content.split("## Secret Detection Scan\n\nStatus: ")[1].split("\n")[0] if "## Secret Detection Scan\n\nStatus: " in content else ""
        report_data["secret"] = secret_match
        
        return report_data
    except Exception as e:
        logger.error(f"Failed to parse security report {report_path}: {str(e)}")
        return {}

def generate_summary(reports: Dict[str, List[Path]]) -> str:
    """
    Generate a summary of security reports.
    
    Args:
        reports: Dictionary of report types and their paths
        
    Returns:
        Summary of security reports
    """
    logger.info("Generating summary...")
    
    # Get the latest reports
    latest_reports = {
        "dependency": reports["dependency"][0] if reports["dependency"] else None,
        "code": reports["code"][0] if reports["code"] else None,
        "secret": reports["secret"][0] if reports["secret"] else None,
        "security": reports["security"][0] if reports["security"] else None
    }
    
    # Parse the latest reports
    parsed_reports = {
        "dependency": parse_dependency_report(latest_reports["dependency"]) if latest_reports["dependency"] else {},
        "code": parse_code_report(latest_reports["code"]) if latest_reports["code"] else {},
        "secret": parse_secret_report(latest_reports["secret"]) if latest_reports["secret"] else {},
        "security": parse_security_report(latest_reports["security"]) if latest_reports["security"] else {}
    }
    
    # Generate the summary
    summary = "# Security Reports Summary\n\n"
    
    # Add the date
    summary += f"## Date: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n\n"
    
    # Add the latest security report
    if parsed_reports["security"]:
        summary += "## Latest Security Report\n\n"
        summary += f"Date: {parsed_reports['security']['date']}\n\n"
        summary += f"Status: {parsed_reports['security']['status']}\n\n"
        summary += f"Dependency Scan: {parsed_reports['security']['dependency']}\n\n"
        summary += f"Code Security Scan: {parsed_reports['security']['code']}\n\n"
        summary += f"Secret Detection Scan: {parsed_reports['security']['secret']}\n\n"
    
    # Add the latest dependency report
    if latest_reports["dependency"]:
        summary += "## Latest Dependency Scan\n\n"
        summary += f"Date: {datetime.fromtimestamp(latest_reports['dependency'].stat().st_mtime).strftime('%Y-%m-%d %H:%M:%S')}\n\n"
        summary += f"File: {latest_reports['dependency']}\n\n"
    
    # Add the latest code report
    if latest_reports["code"]:
        summary += "## Latest Code Security Scan\n\n"
        summary += f"Date: {datetime.fromtimestamp(latest_reports['code'].stat().st_mtime).strftime('%Y-%m-%d %H:%M:%S')}\n\n"
        summary += f"File: {latest_reports['code']}\n\n"
        
        # Add code scan metrics
        if "metrics" in parsed_reports["code"]:
            metrics = parsed_reports["code"]["metrics"]
            summary += "### Metrics\n\n"
            
            # Count issues by severity
            high_severity = 0
            medium_severity = 0
            low_severity = 0
            
            for file_metrics in metrics.values():
                high_severity += file_metrics.get("SEVERITY.HIGH", 0)
                medium_severity += file_metrics.get("SEVERITY.MEDIUM", 0)
                low_severity += file_metrics.get("SEVERITY.LOW", 0)
            
            summary += f"High Severity Issues: {high_severity}\n\n"
            summary += f"Medium Severity Issues: {medium_severity}\n\n"
            summary += f"Low Severity Issues: {low_severity}\n\n"
    
    # Add the latest secret report
    if latest_reports["secret"]:
        summary += "## Latest Secret Detection Scan\n\n"
        summary += f"Date: {datetime.fromtimestamp(latest_reports['secret'].stat().st_mtime).strftime('%Y-%m-%d %H:%M:%S')}\n\n"
        summary += f"File: {latest_reports['secret']}\n\n"
        
        # Add secret scan metrics
        if "results" in parsed_reports["secret"]:
            results = parsed_reports["secret"]["results"]
            summary += "### Metrics\n\n"
            summary += f"Files with Potential Secrets: {len(results)}\n\n"
            
            # Count secrets by type
            secret_types = {}
            for file_results in results.values():
                for result in file_results:
                    secret_type = result.get("type", "Unknown")
                    secret_types[secret_type] = secret_types.get(secret_type, 0) + 1
            
            summary += "Secret Types:\n\n"
            for secret_type, count in secret_types.items():
                summary += f"- {secret_type}: {count}\n"
            
            summary += "\n"
    
    # Add recommendations
    summary += "## Recommendations\n\n"
    summary += "1. Review the latest security reports\n"
    summary += "2. Address the identified vulnerabilities\n"
    summary += "3. Run the security scans again to verify the fixes\n"
    
    return summary

def save_summary(summary: str, output_path: Optional[Path] = None) -> str:
    """
    Save the summary to a file.
    
    Args:
        summary: Summary content
        output_path: Path to save the summary
        
    Returns:
        Path to the saved file
    """
    logger.info("Saving summary...")
    
    # Get timestamp for filename
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    
    # Use the provided output path or generate a default one
    if output_path:
        file_path = output_path
    else:
        file_path = ROOT_DIR / f"security_reports_summary_{timestamp}.md"
    
    # Save the summary
    try:
        with open(file_path, "w") as f:
            f.write(summary)
        
        logger.info(f"Summary saved to {file_path}")
        return str(file_path)
    except Exception as e:
        logger.error(f"Failed to save summary: {str(e)}")
        return ""

def main():
    """
    Main function.
    """
    parser = argparse.ArgumentParser(description="Monitor security reports and generate a summary")
    parser.add_argument("--report-dir", help="Directory containing security reports")
    parser.add_argument("--output", help="Path to save the summary")
    
    args = parser.parse_args()
    
    # Set report directory
    if args.report_dir:
        global REPORTS_DIR
        REPORTS_DIR = Path(args.report_dir)
    
    # Get security reports
    reports = get_security_reports()
    
    # Generate summary
    summary = generate_summary(reports)
    
    # Save summary
    output_path = Path(args.output) if args.output else None
    file_path = save_summary(summary, output_path)
    
    if file_path:
        logger.info(f"Summary saved to {file_path}")
    
    return 0

if __name__ == "__main__":
    sys.exit(main())
