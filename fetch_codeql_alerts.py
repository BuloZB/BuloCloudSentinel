#!/usr/bin/env python3
"""
Script to fetch CodeQL alerts from GitHub repository and save them to a JSON file.
This will help analyze and fix the issues systematically.
"""

import requests
import json
import os
import sys
from datetime import datetime

# Configuration
# Token should be set as an environment variable
GITHUB_TOKEN = os.environ.get("GITHUB_TOKEN", "")  # Get token from environment variable
REPO_OWNER = "BuloZB"  # GitHub username
REPO_NAME = "BuloCloudSentinel"  # Repository name

# Check if token is provided
if not GITHUB_TOKEN:
    print("Error: GitHub token not provided")
    print("Please set the GITHUB_TOKEN environment variable")
    print("Example: export GITHUB_TOKEN=your_token")
    sys.exit(1)

# GitHub API endpoints
API_BASE = "https://api.github.com"
CODE_SCANNING_ALERTS_ENDPOINT = f"/repos/{REPO_OWNER}/{REPO_NAME}/code-scanning/alerts"

def fetch_codeql_alerts(state="open", per_page=100):
    """
    Fetch CodeQL alerts from GitHub repository.

    Args:
        state: Filter by state (open, closed, fixed, dismissed)
        per_page: Number of results per page

    Returns:
        List of CodeQL alerts
    """
    all_alerts = []
    page = 1

    while True:
        # Set up request parameters
        params = {
            "state": state,
            "per_page": per_page,
            "page": page,
            "tool_name": "CodeQL"
        }

        # Set up request headers
        headers = {
            "Authorization": f"token {GITHUB_TOKEN}",
            "Accept": "application/vnd.github.v3+json"
        }

        # Make request to GitHub API
        url = f"{API_BASE}{CODE_SCANNING_ALERTS_ENDPOINT}"
        response = requests.get(url, params=params, headers=headers)

        # Check if request was successful
        if response.status_code != 200:
            print(f"Error fetching alerts: {response.status_code}")
            print(response.text)
            sys.exit(1)

        # Parse response
        alerts = response.json()

        # Add alerts to list
        all_alerts.extend(alerts)

        # Check if there are more pages
        if len(alerts) < per_page:
            break

        # Increment page number
        page += 1

    return all_alerts

def categorize_alerts(alerts):
    """
    Categorize alerts by rule ID and severity.

    Args:
        alerts: List of CodeQL alerts

    Returns:
        Dictionary of categorized alerts
    """
    categories = {}

    for alert in alerts:
        rule_id = alert.get("rule", {}).get("id", "unknown")
        severity = alert.get("rule", {}).get("severity", "unknown")

        if rule_id not in categories:
            categories[rule_id] = {
                "count": 0,
                "severity": severity,
                "description": alert.get("rule", {}).get("description", ""),
                "alerts": []
            }

        categories[rule_id]["count"] += 1
        categories[rule_id]["alerts"].append({
            "number": alert.get("number"),
            "state": alert.get("state"),
            "created_at": alert.get("created_at"),
            "url": alert.get("html_url"),
            "location": {
                "path": alert.get("most_recent_instance", {}).get("location", {}).get("path", ""),
                "start_line": alert.get("most_recent_instance", {}).get("location", {}).get("start_line", 0),
                "end_line": alert.get("most_recent_instance", {}).get("location", {}).get("end_line", 0)
            }
        })

    return categories

def save_alerts_to_file(alerts, categories):
    """
    Save alerts and categories to JSON files.

    Args:
        alerts: List of CodeQL alerts
        categories: Dictionary of categorized alerts
    """
    # Create output directory if it doesn't exist
    os.makedirs("codeql_analysis", exist_ok=True)

    # Get current timestamp
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

    # Save raw alerts
    with open(f"codeql_analysis/codeql_alerts_{timestamp}.json", "w") as f:
        json.dump(alerts, f, indent=2)

    # Save categorized alerts
    with open(f"codeql_analysis/codeql_categories_{timestamp}.json", "w") as f:
        json.dump(categories, f, indent=2)

    # Create summary file
    with open(f"codeql_analysis/codeql_summary_{timestamp}.md", "w") as f:
        f.write("# CodeQL Analysis Summary\n\n")
        f.write(f"Analysis date: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n\n")
        f.write(f"Total alerts: {len(alerts)}\n\n")

        f.write("## Alert Categories\n\n")
        f.write("| Rule ID | Severity | Count | Description |\n")
        f.write("|---------|----------|-------|-------------|\n")

        # Sort categories by count (descending)
        sorted_categories = sorted(categories.items(), key=lambda x: x[1]["count"], reverse=True)

        for rule_id, category in sorted_categories:
            f.write(f"| {rule_id} | {category['severity']} | {category['count']} | {category['description']} |\n")

    print(f"Alerts saved to codeql_analysis/codeql_alerts_{timestamp}.json")
    print(f"Categories saved to codeql_analysis/codeql_categories_{timestamp}.json")
    print(f"Summary saved to codeql_analysis/codeql_summary_{timestamp}.md")

def main():
    """Main function."""
    print("Fetching CodeQL alerts...")
    alerts = fetch_codeql_alerts()
    print(f"Found {len(alerts)} alerts")

    print("Categorizing alerts...")
    categories = categorize_alerts(alerts)
    print(f"Found {len(categories)} categories")

    print("Saving alerts to file...")
    save_alerts_to_file(alerts, categories)

    print("Done!")

if __name__ == "__main__":
    main()
