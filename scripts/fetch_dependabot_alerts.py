#!/usr/bin/env python3
"""
Fetch Dependabot Alerts Script for Bulo.Cloud Sentinel.

This script fetches Dependabot alerts from GitHub and generates a report.
It requires a GitHub personal access token with the 'security_events' scope.

Usage:
    python fetch_dependabot_alerts.py --token GITHUB_TOKEN [--output OUTPUT_FILE]
"""

import argparse
import json
import logging
import os
import sys
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional
import urllib.request
import urllib.error

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(levelname)s - %(message)s",
    handlers=[
        logging.FileHandler("fetch_dependabot_alerts.log"),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger(__name__)

# Define the repository information
REPO_OWNER = "BuloZB"
REPO_NAME = "BuloCloudSentinel"

def fetch_dependabot_alerts(token: str) -> List[Dict]:
    """
    Fetch Dependabot alerts from GitHub.
    
    Args:
        token: GitHub personal access token
        
    Returns:
        List of Dependabot alerts
    """
    logger.info("Fetching Dependabot alerts...")
    
    url = f"https://api.github.com/repos/{REPO_OWNER}/{REPO_NAME}/dependabot/alerts"
    
    headers = {
        "Accept": "application/vnd.github.v3+json",
        "Authorization": f"token {token}",
        "X-GitHub-Api-Version": "2022-11-28"
    }
    
    try:
        request = urllib.request.Request(
            url,
            headers=headers,
            method="GET"
        )
        
        with urllib.request.urlopen(request) as response:
            data = json.loads(response.read().decode("utf-8"))
            
            logger.info(f"Fetched {len(data)} Dependabot alerts")
            return data
    except urllib.error.HTTPError as e:
        logger.error(f"Failed to fetch Dependabot alerts: {e.code} {e.reason}")
        if e.code == 401:
            logger.error("Authentication failed. Make sure your token has the 'security_events' scope.")
        elif e.code == 403:
            logger.error("Permission denied. Make sure your token has the 'security_events' scope.")
        elif e.code == 404:
            logger.error("Repository not found or Dependabot alerts not enabled.")
        return []
    except Exception as e:
        logger.error(f"Failed to fetch Dependabot alerts: {str(e)}")
        return []

def analyze_alerts(alerts: List[Dict]) -> Dict:
    """
    Analyze Dependabot alerts.
    
    Args:
        alerts: List of Dependabot alerts
        
    Returns:
        Analysis results
    """
    logger.info("Analyzing Dependabot alerts...")
    
    # Count alerts by severity
    severity_counts = {
        "critical": 0,
        "high": 0,
        "moderate": 0,
        "low": 0
    }
    
    # Count alerts by package
    package_counts = {}
    
    # Count alerts by ecosystem
    ecosystem_counts = {}
    
    # Count alerts by state
    state_counts = {
        "open": 0,
        "dismissed": 0,
        "fixed": 0
    }
    
    # Analyze each alert
    for alert in alerts:
        # Extract severity
        severity = alert.get("security_advisory", {}).get("severity", "").lower()
        if severity in severity_counts:
            severity_counts[severity] += 1
        
        # Extract package
        package = alert.get("security_advisory", {}).get("package", {}).get("name", "")
        if package:
            package_counts[package] = package_counts.get(package, 0) + 1
        
        # Extract ecosystem
        ecosystem = alert.get("security_advisory", {}).get("package", {}).get("ecosystem", "")
        if ecosystem:
            ecosystem_counts[ecosystem] = ecosystem_counts.get(ecosystem, 0) + 1
        
        # Extract state
        state = alert.get("state", "").lower()
        if state in state_counts:
            state_counts[state] += 1
    
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
        "state_counts": state_counts,
        "recommendations": recommendations,
        "total_alerts": len(alerts)
    }

def generate_report(alerts: List[Dict], analysis: Dict) -> str:
    """
    Generate a report of the Dependabot alerts.
    
    Args:
        alerts: List of Dependabot alerts
        analysis: Analysis results
        
    Returns:
        Report content
    """
    logger.info("Generating report...")
    
    report = "# Dependabot Alerts Report\n\n"
    
    # Add the date
    report += f"## Date: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n\n"
    
    # Add summary
    report += "## Summary\n\n"
    report += f"Total alerts: {analysis['total_alerts']}\n\n"
    
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
    
    # Add state counts
    report += "### State Counts\n\n"
    for state, count in analysis["state_counts"].items():
        report += f"- {state.capitalize()}: {count}\n"
    report += "\n"
    
    # Add recommendations
    report += "## Recommendations\n\n"
    for i, recommendation in enumerate(analysis["recommendations"], 1):
        report += f"{i}. {recommendation}\n"
    report += "\n"
    
    # Add alerts
    report += "## Alerts\n\n"
    for i, alert in enumerate(alerts, 1):
        security_advisory = alert.get("security_advisory", {})
        package = security_advisory.get("package", {})
        
        report += f"### {i}. {package.get('name', 'Unknown Package')}\n\n"
        report += f"- **Severity**: {security_advisory.get('severity', 'Unknown')}\n"
        report += f"- **Ecosystem**: {package.get('ecosystem', 'Unknown')}\n"
        report += f"- **Vulnerable Version**: {alert.get('vulnerable_version_range', 'Unknown')}\n"
        report += f"- **State**: {alert.get('state', 'Unknown')}\n"
        report += f"- **CVSS Score**: {security_advisory.get('cvss', {}).get('score', 'Unknown')}\n"
        report += f"- **CWE IDs**: {', '.join(security_advisory.get('cwes', ['Unknown']))}\n"
        report += f"- **Description**: {security_advisory.get('description', 'No description available')}\n"
        
        # Add references
        references = security_advisory.get("references", [])
        if references:
            report += "- **References**:\n"
            for reference in references:
                report += f"  - [{reference.get('title', 'Reference')}]({reference.get('url', '#')})\n"
        
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
        file_path = Path(f"dependabot_alerts_{timestamp}.md")
    
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
    parser = argparse.ArgumentParser(description="Fetch Dependabot alerts from GitHub")
    parser.add_argument("--token", help="GitHub personal access token")
    parser.add_argument("--output", help="Path to save the report")
    
    args = parser.parse_args()
    
    # Get the GitHub token
    token = args.token or os.environ.get("GITHUB_TOKEN")
    
    if not token:
        logger.error("GitHub token not provided")
        logger.info("Please provide a GitHub token using the --token option or the GITHUB_TOKEN environment variable")
        return 1
    
    # Fetch Dependabot alerts
    alerts = fetch_dependabot_alerts(token)
    
    if not alerts:
        logger.error("No Dependabot alerts found")
        return 1
    
    # Analyze alerts
    analysis = analyze_alerts(alerts)
    
    # Generate report
    report = generate_report(alerts, analysis)
    
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
