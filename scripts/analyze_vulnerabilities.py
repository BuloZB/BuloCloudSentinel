#!/usr/bin/env python3
"""
Analyze Vulnerabilities Script for Bulo.Cloud Sentinel.

This script analyzes the vulnerabilities identified by GitHub Dependabot
and provides recommendations for addressing them.

Usage:
    python analyze_vulnerabilities.py [--input INPUT_FILE] [--output OUTPUT_FILE]
"""

import argparse
import csv
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
        logging.FileHandler("analyze_vulnerabilities.log"),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger(__name__)

# Define the root directory
ROOT_DIR = Path(__file__).parent.parent

def parse_csv_vulnerabilities(input_file: Path) -> List[Dict]:
    """
    Parse vulnerabilities from a CSV file.
    
    Args:
        input_file: Path to the CSV file
        
    Returns:
        List of vulnerabilities
    """
    logger.info(f"Parsing vulnerabilities from {input_file}...")
    
    vulnerabilities = []
    
    try:
        with open(input_file, "r", newline="") as f:
            reader = csv.DictReader(f)
            for row in reader:
                vulnerabilities.append(row)
    except Exception as e:
        logger.error(f"Failed to parse vulnerabilities from {input_file}: {str(e)}")
    
    return vulnerabilities

def parse_json_vulnerabilities(input_file: Path) -> List[Dict]:
    """
    Parse vulnerabilities from a JSON file.
    
    Args:
        input_file: Path to the JSON file
        
    Returns:
        List of vulnerabilities
    """
    logger.info(f"Parsing vulnerabilities from {input_file}...")
    
    try:
        with open(input_file, "r") as f:
            return json.load(f)
    except Exception as e:
        logger.error(f"Failed to parse vulnerabilities from {input_file}: {str(e)}")
        return []

def analyze_vulnerabilities(vulnerabilities: List[Dict]) -> Dict:
    """
    Analyze vulnerabilities and provide recommendations.
    
    Args:
        vulnerabilities: List of vulnerabilities
        
    Returns:
        Analysis results
    """
    logger.info("Analyzing vulnerabilities...")
    
    # Count vulnerabilities by severity
    severity_counts = {
        "critical": 0,
        "high": 0,
        "moderate": 0,
        "low": 0
    }
    
    # Count vulnerabilities by package
    package_counts = {}
    
    # Count vulnerabilities by ecosystem
    ecosystem_counts = {}
    
    # Analyze each vulnerability
    for vuln in vulnerabilities:
        # Extract severity
        severity = vuln.get("severity", "").lower()
        if severity in severity_counts:
            severity_counts[severity] += 1
        
        # Extract package
        package = vuln.get("package", "")
        if package:
            package_counts[package] = package_counts.get(package, 0) + 1
        
        # Extract ecosystem
        ecosystem = vuln.get("ecosystem", "")
        if ecosystem:
            ecosystem_counts[ecosystem] = ecosystem_counts.get(ecosystem, 0) + 1
    
    # Generate recommendations
    recommendations = []
    
    # Recommend updating critical and high severity vulnerabilities first
    if severity_counts["critical"] > 0 or severity_counts["high"] > 0:
        recommendations.append(
            "Update critical and high severity vulnerabilities first using the dependency-update.yml workflow."
        )
    
    # Recommend enabling Dependabot security updates
    recommendations.append(
        "Enable Dependabot security updates in the repository settings to automatically create pull requests for vulnerable dependencies."
    )
    
    # Recommend running security scans
    recommendations.append(
        "Run the security-scan.yml workflow to perform a comprehensive security scan of the repository."
    )
    
    # Recommend implementing regular security reviews
    recommendations.append(
        "Use the security-review.yml workflow to generate security review templates and perform regular security reviews."
    )
    
    # Return analysis results
    return {
        "severity_counts": severity_counts,
        "package_counts": package_counts,
        "ecosystem_counts": ecosystem_counts,
        "recommendations": recommendations,
        "total_vulnerabilities": sum(severity_counts.values())
    }

def generate_report(analysis: Dict, vulnerabilities: List[Dict]) -> str:
    """
    Generate a report of the vulnerability analysis.
    
    Args:
        analysis: Analysis results
        vulnerabilities: List of vulnerabilities
        
    Returns:
        Report content
    """
    logger.info("Generating report...")
    
    report = "# Vulnerability Analysis Report\n\n"
    
    # Add the date
    report += f"## Date: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n\n"
    
    # Add summary
    report += "## Summary\n\n"
    report += f"Total vulnerabilities: {analysis['total_vulnerabilities']}\n\n"
    
    # Add severity counts
    report += "### Severity Counts\n\n"
    for severity, count in analysis["severity_counts"].items():
        report += f"- {severity.capitalize()}: {count}\n"
    report += "\n"
    
    # Add package counts
    report += "### Package Counts\n\n"
    for package, count in sorted(analysis["package_counts"].items(), key=lambda x: x[1], reverse=True):
        report += f"- {package}: {count}\n"
    report += "\n"
    
    # Add ecosystem counts
    report += "### Ecosystem Counts\n\n"
    for ecosystem, count in sorted(analysis["ecosystem_counts"].items(), key=lambda x: x[1], reverse=True):
        report += f"- {ecosystem}: {count}\n"
    report += "\n"
    
    # Add recommendations
    report += "## Recommendations\n\n"
    for i, recommendation in enumerate(analysis["recommendations"], 1):
        report += f"{i}. {recommendation}\n"
    report += "\n"
    
    # Add vulnerabilities
    report += "## Vulnerabilities\n\n"
    for i, vuln in enumerate(vulnerabilities, 1):
        report += f"### {i}. {vuln.get('package', 'Unknown Package')}\n\n"
        report += f"- **Severity**: {vuln.get('severity', 'Unknown')}\n"
        report += f"- **Ecosystem**: {vuln.get('ecosystem', 'Unknown')}\n"
        report += f"- **Vulnerable Version**: {vuln.get('vulnerable_version', 'Unknown')}\n"
        report += f"- **Patched Version**: {vuln.get('patched_version', 'Unknown')}\n"
        report += f"- **Description**: {vuln.get('description', 'No description available')}\n"
        report += "\n"
    
    return report

def save_report(report: str, output_file: Optional[Path] = None) -> str:
    """
    Save the report to a file.
    
    Args:
        report: Report content
        output_file: Path to save the report
        
    Returns:
        Path to the saved file
    """
    logger.info("Saving report...")
    
    # Get timestamp for filename
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    
    # Use the provided output path or generate a default one
    if output_file:
        file_path = output_file
    else:
        file_path = ROOT_DIR / f"vulnerability_analysis_{timestamp}.md"
    
    # Save the report
    try:
        with open(file_path, "w") as f:
            f.write(report)
        
        logger.info(f"Report saved to {file_path}")
        return str(file_path)
    except Exception as e:
        logger.error(f"Failed to save report: {str(e)}")
        return ""

def main():
    """
    Main function.
    """
    parser = argparse.ArgumentParser(description="Analyze vulnerabilities identified by GitHub Dependabot")
    parser.add_argument("--input", help="Path to the input file (CSV or JSON)")
    parser.add_argument("--output", help="Path to save the report")
    
    args = parser.parse_args()
    
    # Check if input file is provided
    if args.input:
        input_file = Path(args.input)
        
        # Check if the input file exists
        if not input_file.exists():
            logger.error(f"Input file {input_file} does not exist")
            return 1
        
        # Parse vulnerabilities based on file extension
        if input_file.suffix.lower() == ".csv":
            vulnerabilities = parse_csv_vulnerabilities(input_file)
        elif input_file.suffix.lower() == ".json":
            vulnerabilities = parse_json_vulnerabilities(input_file)
        else:
            logger.error(f"Unsupported file format: {input_file.suffix}")
            return 1
    else:
        # Use sample vulnerabilities for demonstration
        vulnerabilities = [
            {
                "package": "cryptography",
                "severity": "Critical",
                "ecosystem": "pip",
                "vulnerable_version": "<46.0.0",
                "patched_version": "46.0.0",
                "description": "Multiple vulnerabilities in the cryptography package could allow an attacker to execute arbitrary code, cause a denial of service, or bypass security controls."
            },
            {
                "package": "python-jose",
                "severity": "High",
                "ecosystem": "pip",
                "vulnerable_version": "*",
                "patched_version": "Replace with PyJWT",
                "description": "The python-jose package had a vulnerability in the JWT token validation that could allow an attacker to forge tokens."
            },
            {
                "package": "python-multipart",
                "severity": "High",
                "ecosystem": "pip",
                "vulnerable_version": "<0.0.21",
                "patched_version": "0.0.21",
                "description": "The python-multipart package had a vulnerability that could allow an attacker to cause a denial of service."
            },
            {
                "package": "pillow",
                "severity": "Moderate",
                "ecosystem": "pip",
                "vulnerable_version": "<11.2.1",
                "patched_version": "11.2.1",
                "description": "The pillow package had a vulnerability that could allow an attacker to execute arbitrary code."
            },
            {
                "package": "fastapi",
                "severity": "Moderate",
                "ecosystem": "pip",
                "vulnerable_version": "<0.115.12",
                "patched_version": "0.115.12",
                "description": "The fastapi package had a vulnerability that could allow an attacker to bypass security controls."
            }
        ]
    
    # Analyze vulnerabilities
    analysis = analyze_vulnerabilities(vulnerabilities)
    
    # Generate report
    report = generate_report(analysis, vulnerabilities)
    
    # Save report
    output_path = Path(args.output) if args.output else None
    file_path = save_report(report, output_path)
    
    if file_path:
        logger.info(f"Report saved to {file_path}")
        
        # Print report to console
        print("\n" + report)
    
    return 0

if __name__ == "__main__":
    sys.exit(main())
