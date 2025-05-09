#!/usr/bin/env python3
"""
Scheduled Security Scan for Bulo.Cloud Sentinel.

This script runs comprehensive security scans and generates reports.
It can be scheduled to run regularly using cron or Windows Task Scheduler.

Usage:
    python scheduled_security_scan.py [--email] [--slack] [--report-dir REPORT_DIR]
"""

import argparse
import json
import logging
import os
import subprocess
import sys
import time
from datetime import datetime
from email.mime.multipart import MIMEMultipart
from email.mime.text import MIMEText
from pathlib import Path
from typing import Dict, List, Optional, Tuple

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(levelname)s - %(message)s",
    handlers=[
        logging.FileHandler("security_scan.log"),
        logging.StreamHandler()
    ]
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

def run_dependency_scan() -> Tuple[bool, str]:
    """
    Run dependency scan.

    Returns:
        Tuple of (success, report_path)
    """
    logger.info("Running dependency scan...")

    # Get timestamp for report filename
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    report_file = REPORTS_DIR / f"dependency_scan_{timestamp}.json"

    # Run dependency scan
    cmd = [sys.executable, "scripts/dependency_scan.py", "--report-file", str(report_file)]
    returncode, stdout, stderr = run_command(cmd)

    if returncode != 0:
        logger.warning("Dependency scan found vulnerabilities")
    else:
        logger.info("Dependency scan completed successfully")

    return returncode == 0, str(report_file)

def run_code_scan() -> Tuple[bool, str]:
    """
    Run code security scan.

    Returns:
        Tuple of (success, report_path)
    """
    logger.info("Running code security scan...")

    # Get timestamp for report filename
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    report_file = REPORTS_DIR / f"code_scan_{timestamp}.json"

    # Run bandit
    cmd = [
        sys.executable, "-m", "bandit",
        "-r", str(ROOT_DIR),
        "-f", "json",
        "-o", str(report_file),
        "--exclude", ".venv,venv,node_modules,tests,__pycache__"
    ]

    returncode, stdout, stderr = run_command(cmd)

    if returncode == 1:  # 1 means issues found
        logger.warning("Code scan found security issues")
    elif returncode > 1:  # >1 means error
        logger.error(f"Code scan failed: {stderr}")
        return False, ""
    else:
        logger.info("Code scan completed successfully")

    return returncode == 0, str(report_file)

def run_secret_scan() -> Tuple[bool, str]:
    """
    Run secret detection scan.

    Returns:
        Tuple of (success, report_path)
    """
    logger.info("Running secret detection scan...")

    # Get timestamp for report filename
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    report_file = REPORTS_DIR / f"secret_scan_{timestamp}.json"

    # Run detect-secrets
    cmd = [
        sys.executable, "-m", "detect_secrets", "scan",
        "--all-files",
        "--exclude-files", ".*\\.ipynb$|.*\\.pyc$|.*\\.git/.*|.*\\.venv/.*|.*venv/.*|.*node_modules/.*|.*__pycache__/.*",
        str(ROOT_DIR)
    ]

    returncode, stdout, stderr = run_command(cmd)

    if returncode != 0:
        logger.error(f"Secret scan failed: {stderr}")
        return False, ""

    # Save the report
    try:
        with open(report_file, "w") as f:
            f.write(stdout)

        # Parse the report
        report = json.loads(stdout)
        results = report.get("results", {})

        if results:
            logger.warning(f"Secret scan found potential secrets in {len(results)} files")
            return False, str(report_file)
        else:
            logger.info("Secret scan completed successfully")
            return True, str(report_file)
    except Exception as e:
        logger.error(f"Failed to parse secret scan report: {str(e)}")
        return False, ""

def generate_report(scan_results: Dict[str, Tuple[bool, str]]) -> str:
    """
    Generate a comprehensive security report.

    Args:
        scan_results: Dictionary of scan results

    Returns:
        Path to the report file
    """
    logger.info("Generating security report...")

    # Get timestamp for report filename
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    report_file = REPORTS_DIR / f"security_report_{timestamp}.md"

    # Generate report
    with open(report_file, "w") as f:
        f.write("# Security Scan Report\n\n")
        f.write(f"## Scan Date: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n\n")

        # Overall status
        all_passed = all(success for success, _ in scan_results.values())
        status = "PASSED" if all_passed else "FAILED"
        status_icon = "[PASSED]" if all_passed else "[FAILED]"
        f.write(f"## Overall Status: {status_icon} {status}\n\n")

        # Dependency scan
        f.write("## Dependency Scan\n\n")
        if "dependency" in scan_results:
            success, report_path = scan_results["dependency"]
            status = "[PASSED]" if success else "[FAILED]"
            f.write(f"Status: {status}\n\n")
            if report_path:
                f.write(f"Report: {report_path}\n\n")
        else:
            f.write("Not run\n\n")

        # Code scan
        f.write("## Code Security Scan\n\n")
        if "code" in scan_results:
            success, report_path = scan_results["code"]
            status = "[PASSED]" if success else "[FAILED]"
            f.write(f"Status: {status}\n\n")
            if report_path:
                f.write(f"Report: {report_path}\n\n")
        else:
            f.write("Not run\n\n")

        # Secret scan
        f.write("## Secret Detection Scan\n\n")
        if "secret" in scan_results:
            success, report_path = scan_results["secret"]
            status = "[PASSED]" if success else "[FAILED]"
            f.write(f"Status: {status}\n\n")
            if report_path:
                f.write(f"Report: {report_path}\n\n")
        else:
            f.write("Not run\n\n")

        # Recommendations
        f.write("## Recommendations\n\n")
        if not all_passed:
            f.write("1. Review the scan reports and fix the identified issues\n")
            f.write("2. Run the scans again to verify the fixes\n")
            f.write("3. Update dependencies to fix vulnerabilities\n")
        else:
            f.write("All security scans passed. Continue to monitor for new vulnerabilities.\n")

    logger.info(f"Security report generated: {report_file}")
    return str(report_file)

def send_email_report(report_file: str, recipients: List[str]):
    """
    Send the security report via email.

    Args:
        report_file: Path to the report file
        recipients: List of email recipients
    """
    logger.info("Sending email report...")

    # Check if email configuration is available
    smtp_server = os.environ.get("SMTP_SERVER")
    smtp_port = os.environ.get("SMTP_PORT")
    smtp_username = os.environ.get("SMTP_USERNAME")
    smtp_password = os.environ.get("SMTP_PASSWORD")
    sender = os.environ.get("EMAIL_SENDER")

    if not all([smtp_server, smtp_port, smtp_username, smtp_password, sender]):
        logger.error("Email configuration not available")
        return

    try:
        import smtplib

        # Read the report
        with open(report_file, "r") as f:
            report_content = f.read()

        # Create the email
        msg = MIMEMultipart()
        msg["From"] = sender
        msg["To"] = ", ".join(recipients)
        msg["Subject"] = "Security Scan Report"

        # Add the report content
        msg.attach(MIMEText(report_content, "plain"))

        # Send the email
        with smtplib.SMTP(smtp_server, int(smtp_port)) as server:
            server.starttls()
            server.login(smtp_username, smtp_password)
            server.send_message(msg)

        logger.info("Email report sent successfully")
    except Exception as e:
        logger.error(f"Failed to send email report: {str(e)}")

def send_slack_report(report_file: str, webhook_url: str):
    """
    Send the security report to Slack.

    Args:
        report_file: Path to the report file
        webhook_url: Slack webhook URL
    """
    logger.info("Sending Slack report...")

    try:
        import requests

        # Read the report
        with open(report_file, "r") as f:
            report_content = f.read()

        # Create the payload
        payload = {
            "text": "Security Scan Report",
            "blocks": [
                {
                    "type": "section",
                    "text": {
                        "type": "mrkdwn",
                        "text": "Security Scan Report"
                    }
                },
                {
                    "type": "section",
                    "text": {
                        "type": "mrkdwn",
                        "text": report_content[:3000] + "..." if len(report_content) > 3000 else report_content
                    }
                }
            ]
        }

        # Send the report
        response = requests.post(webhook_url, json=payload)
        response.raise_for_status()

        logger.info("Slack report sent successfully")
    except Exception as e:
        logger.error(f"Failed to send Slack report: {str(e)}")

def main():
    """
    Main function.
    """
    parser = argparse.ArgumentParser(description="Run scheduled security scans")
    parser.add_argument("--email", action="store_true", help="Send report via email")
    parser.add_argument("--slack", action="store_true", help="Send report to Slack")
    parser.add_argument("--report-dir", help="Directory to save reports")
    parser.add_argument("--recipients", help="Comma-separated list of email recipients")
    parser.add_argument("--slack-webhook", help="Slack webhook URL")

    args = parser.parse_args()

    # Set report directory
    if args.report_dir:
        global REPORTS_DIR
        REPORTS_DIR = Path(args.report_dir)
        REPORTS_DIR.mkdir(exist_ok=True)

    # Run scans
    scan_results = {}

    # Run dependency scan
    dependency_success, dependency_report = run_dependency_scan()
    scan_results["dependency"] = (dependency_success, dependency_report)

    # Run code scan
    code_success, code_report = run_code_scan()
    scan_results["code"] = (code_success, code_report)

    # Run secret scan
    secret_success, secret_report = run_secret_scan()
    scan_results["secret"] = (secret_success, secret_report)

    # Generate report
    report_file = generate_report(scan_results)

    # Send email report
    if args.email:
        recipients = args.recipients.split(",") if args.recipients else []
        send_email_report(report_file, recipients)

    # Send Slack report
    if args.slack:
        webhook_url = args.slack_webhook or os.environ.get("SLACK_WEBHOOK")
        if webhook_url:
            send_slack_report(report_file, webhook_url)
        else:
            logger.error("Slack webhook URL not provided")

    # Return success only if all scans passed
    return 0 if all(success for success, _ in scan_results.values()) else 1

if __name__ == "__main__":
    sys.exit(main())
