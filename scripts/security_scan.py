#!/usr/bin/env python3
"""
Security scanning script for Bulo.Cloud Sentinel.

This script runs automated security scans using OWASP ZAP to identify security
vulnerabilities in the application.
"""

import os
import sys
import logging
import subprocess
import json
import argparse
import time
from pathlib import Path
from datetime import datetime

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger(__name__)

# Define the root directory
ROOT_DIR = Path(__file__).parent.parent
REPORTS_DIR = ROOT_DIR / "reports" / "security"

# Define ZAP Docker image
ZAP_DOCKER_IMAGE = "owasp/zap2docker-stable"

# Define default target URL
DEFAULT_TARGET_URL = "http://localhost:8000"

def ensure_reports_directory():
    """
    Ensure the reports directory exists.
    """
    REPORTS_DIR.mkdir(parents=True, exist_ok=True)
    logger.info(f"Reports directory: {REPORTS_DIR}")

def check_docker_installed():
    """
    Check if Docker is installed.
    
    Returns:
        bool: True if Docker is installed, False otherwise
    """
    try:
        subprocess.run(
            ["docker", "--version"],
            check=True,
            capture_output=True,
            text=True
        )
        return True
    except (subprocess.CalledProcessError, FileNotFoundError):
        logger.error("Docker is not installed or not in PATH")
        return False

def pull_zap_docker_image():
    """
    Pull the OWASP ZAP Docker image.
    
    Returns:
        bool: True if successful, False otherwise
    """
    logger.info(f"Pulling OWASP ZAP Docker image: {ZAP_DOCKER_IMAGE}")
    try:
        subprocess.run(
            ["docker", "pull", ZAP_DOCKER_IMAGE],
            check=True,
            capture_output=True,
            text=True
        )
        return True
    except subprocess.CalledProcessError as e:
        logger.error(f"Failed to pull OWASP ZAP Docker image: {e.stderr}")
        return False

def run_zap_baseline_scan(target_url, report_file, additional_args=None):
    """
    Run a ZAP baseline scan.
    
    Args:
        target_url: URL to scan
        report_file: Path to save the report
        additional_args: Additional arguments for ZAP
        
    Returns:
        bool: True if successful, False otherwise
    """
    logger.info(f"Running ZAP baseline scan against {target_url}")
    
    # Build the command
    cmd = [
        "docker", "run", "--rm", "-v", f"{REPORTS_DIR}:/zap/wrk",
        ZAP_DOCKER_IMAGE, "zap-baseline.py",
        "-t", target_url,
        "-g", "gen.conf",
        "-r", str(Path("/zap/wrk") / Path(report_file).name)
    ]
    
    # Add additional arguments
    if additional_args:
        cmd.extend(additional_args)
    
    # Run the command
    try:
        logger.info(f"Running command: {' '.join(cmd)}")
        result = subprocess.run(
            cmd,
            check=False,  # Don't raise an exception if ZAP finds vulnerabilities
            capture_output=True,
            text=True
        )
        
        # Log the output
        logger.info(result.stdout)
        if result.stderr:
            logger.warning(result.stderr)
        
        # Check if the report was generated
        if not Path(report_file).exists():
            logger.error(f"Report file not generated: {report_file}")
            return False
        
        logger.info(f"ZAP baseline scan completed. Report saved to {report_file}")
        return True
    except subprocess.CalledProcessError as e:
        logger.error(f"Failed to run ZAP baseline scan: {e.stderr}")
        return False

def run_zap_full_scan(target_url, report_file, additional_args=None):
    """
    Run a ZAP full scan.
    
    Args:
        target_url: URL to scan
        report_file: Path to save the report
        additional_args: Additional arguments for ZAP
        
    Returns:
        bool: True if successful, False otherwise
    """
    logger.info(f"Running ZAP full scan against {target_url}")
    
    # Build the command
    cmd = [
        "docker", "run", "--rm", "-v", f"{REPORTS_DIR}:/zap/wrk",
        ZAP_DOCKER_IMAGE, "zap-full-scan.py",
        "-t", target_url,
        "-g", "gen.conf",
        "-r", str(Path("/zap/wrk") / Path(report_file).name)
    ]
    
    # Add additional arguments
    if additional_args:
        cmd.extend(additional_args)
    
    # Run the command
    try:
        logger.info(f"Running command: {' '.join(cmd)}")
        result = subprocess.run(
            cmd,
            check=False,  # Don't raise an exception if ZAP finds vulnerabilities
            capture_output=True,
            text=True
        )
        
        # Log the output
        logger.info(result.stdout)
        if result.stderr:
            logger.warning(result.stderr)
        
        # Check if the report was generated
        if not Path(report_file).exists():
            logger.error(f"Report file not generated: {report_file}")
            return False
        
        logger.info(f"ZAP full scan completed. Report saved to {report_file}")
        return True
    except subprocess.CalledProcessError as e:
        logger.error(f"Failed to run ZAP full scan: {e.stderr}")
        return False

def run_zap_api_scan(target_url, api_definition, report_file, additional_args=None):
    """
    Run a ZAP API scan.
    
    Args:
        target_url: URL to scan
        api_definition: Path to API definition file (OpenAPI/Swagger)
        report_file: Path to save the report
        additional_args: Additional arguments for ZAP
        
    Returns:
        bool: True if successful, False otherwise
    """
    logger.info(f"Running ZAP API scan against {target_url}")
    
    # Build the command
    cmd = [
        "docker", "run", "--rm", "-v", f"{REPORTS_DIR}:/zap/wrk",
        "-v", f"{api_definition}:/zap/wrk/api-definition.yaml",
        ZAP_DOCKER_IMAGE, "zap-api-scan.py",
        "-t", target_url,
        "-f", "openapi",
        "-g", "gen.conf",
        "-r", str(Path("/zap/wrk") / Path(report_file).name),
        "-x", "report.xml",
        "/zap/wrk/api-definition.yaml"
    ]
    
    # Add additional arguments
    if additional_args:
        cmd.extend(additional_args)
    
    # Run the command
    try:
        logger.info(f"Running command: {' '.join(cmd)}")
        result = subprocess.run(
            cmd,
            check=False,  # Don't raise an exception if ZAP finds vulnerabilities
            capture_output=True,
            text=True
        )
        
        # Log the output
        logger.info(result.stdout)
        if result.stderr:
            logger.warning(result.stderr)
        
        # Check if the report was generated
        if not Path(report_file).exists():
            logger.error(f"Report file not generated: {report_file}")
            return False
        
        logger.info(f"ZAP API scan completed. Report saved to {report_file}")
        return True
    except subprocess.CalledProcessError as e:
        logger.error(f"Failed to run ZAP API scan: {e.stderr}")
        return False

def parse_zap_report(report_file):
    """
    Parse a ZAP report and extract vulnerabilities.
    
    Args:
        report_file: Path to the report file
        
    Returns:
        dict: Parsed report data
    """
    logger.info(f"Parsing ZAP report: {report_file}")
    
    try:
        with open(report_file, "r") as f:
            report_data = json.load(f)
        
        # Extract vulnerabilities
        vulnerabilities = []
        for site in report_data.get("site", []):
            for alert in site.get("alerts", []):
                vulnerabilities.append({
                    "name": alert.get("name"),
                    "risk": alert.get("riskdesc"),
                    "confidence": alert.get("confidence"),
                    "description": alert.get("desc"),
                    "solution": alert.get("solution"),
                    "instances": len(alert.get("instances", [])),
                    "reference": alert.get("reference")
                })
        
        # Sort vulnerabilities by risk
        risk_levels = {
            "High": 3,
            "Medium": 2,
            "Low": 1,
            "Informational": 0
        }
        
        vulnerabilities.sort(
            key=lambda x: risk_levels.get(x["risk"].split(" ")[0], 0),
            reverse=True
        )
        
        return {
            "total_vulnerabilities": len(vulnerabilities),
            "vulnerabilities": vulnerabilities
        }
    except Exception as e:
        logger.error(f"Failed to parse ZAP report: {str(e)}")
        return {
            "total_vulnerabilities": 0,
            "vulnerabilities": []
        }

def generate_summary_report(parsed_reports, summary_file):
    """
    Generate a summary report from multiple parsed reports.
    
    Args:
        parsed_reports: Dictionary of parsed reports
        summary_file: Path to save the summary report
        
    Returns:
        bool: True if successful, False otherwise
    """
    logger.info(f"Generating summary report: {summary_file}")
    
    try:
        # Count vulnerabilities by risk level
        risk_counts = {
            "High": 0,
            "Medium": 0,
            "Low": 0,
            "Informational": 0
        }
        
        all_vulnerabilities = []
        
        for scan_type, report_data in parsed_reports.items():
            for vuln in report_data.get("vulnerabilities", []):
                risk_level = vuln["risk"].split(" ")[0]
                if risk_level in risk_counts:
                    risk_counts[risk_level] += 1
                
                all_vulnerabilities.append({
                    "scan_type": scan_type,
                    **vuln
                })
        
        # Sort vulnerabilities by risk
        risk_levels = {
            "High": 3,
            "Medium": 2,
            "Low": 1,
            "Informational": 0
        }
        
        all_vulnerabilities.sort(
            key=lambda x: risk_levels.get(x["risk"].split(" ")[0], 0),
            reverse=True
        )
        
        # Generate summary report
        summary = {
            "scan_date": datetime.now().isoformat(),
            "risk_summary": risk_counts,
            "total_vulnerabilities": sum(risk_counts.values()),
            "vulnerabilities": all_vulnerabilities
        }
        
        # Write summary report
        with open(summary_file, "w") as f:
            json.dump(summary, f, indent=2)
        
        logger.info(f"Summary report generated: {summary_file}")
        
        # Print summary
        logger.info("Vulnerability Summary:")
        logger.info(f"  High: {risk_counts['High']}")
        logger.info(f"  Medium: {risk_counts['Medium']}")
        logger.info(f"  Low: {risk_counts['Low']}")
        logger.info(f"  Informational: {risk_counts['Informational']}")
        logger.info(f"  Total: {sum(risk_counts.values())}")
        
        return True
    except Exception as e:
        logger.error(f"Failed to generate summary report: {str(e)}")
        return False

def main():
    """
    Main function.
    """
    parser = argparse.ArgumentParser(description="Run security scans using OWASP ZAP")
    parser.add_argument("--target", default=DEFAULT_TARGET_URL, help="Target URL to scan")
    parser.add_argument("--api-definition", help="Path to API definition file (OpenAPI/Swagger)")
    parser.add_argument("--scan-type", choices=["baseline", "full", "api", "all"], default="baseline", help="Type of scan to run")
    parser.add_argument("--output-dir", default=str(REPORTS_DIR), help="Directory to save reports")
    
    args = parser.parse_args()
    
    # Ensure reports directory exists
    ensure_reports_directory()
    
    # Check if Docker is installed
    if not check_docker_installed():
        logger.error("Docker is required to run OWASP ZAP scans")
        return 1
    
    # Pull ZAP Docker image
    if not pull_zap_docker_image():
        logger.error("Failed to pull OWASP ZAP Docker image")
        return 1
    
    # Generate timestamp for report files
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    
    # Run scans based on scan type
    parsed_reports = {}
    
    if args.scan_type in ["baseline", "all"]:
        baseline_report_file = REPORTS_DIR / f"zap_baseline_{timestamp}.json"
        if run_zap_baseline_scan(args.target, baseline_report_file):
            parsed_reports["baseline"] = parse_zap_report(baseline_report_file)
    
    if args.scan_type in ["full", "all"]:
        full_report_file = REPORTS_DIR / f"zap_full_{timestamp}.json"
        if run_zap_full_scan(args.target, full_report_file):
            parsed_reports["full"] = parse_zap_report(full_report_file)
    
    if args.scan_type in ["api", "all"] and args.api_definition:
        api_report_file = REPORTS_DIR / f"zap_api_{timestamp}.json"
        if run_zap_api_scan(args.target, args.api_definition, api_report_file):
            parsed_reports["api"] = parse_zap_report(api_report_file)
    
    # Generate summary report
    if parsed_reports:
        summary_file = REPORTS_DIR / f"zap_summary_{timestamp}.json"
        generate_summary_report(parsed_reports, summary_file)
    
    logger.info("Security scanning completed")
    return 0

if __name__ == "__main__":
    sys.exit(main())
