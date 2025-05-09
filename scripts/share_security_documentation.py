#!/usr/bin/env python3
"""
Share Security Documentation Script for Bulo.Cloud Sentinel.

This script generates an email with the security documentation for the Bulo.Cloud Sentinel platform.
It can be used to share the security documentation with the development team.

Usage:
    python share_security_documentation.py [--email] [--recipients RECIPIENTS]
"""

import argparse
import logging
import os
import smtplib
import sys
from datetime import datetime
from email.mime.multipart import MIMEMultipart
from email.mime.text import MIMEText
from pathlib import Path
from typing import List, Optional

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(levelname)s - %(message)s",
    handlers=[
        logging.FileHandler("share_security_documentation.log"),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger(__name__)

# Define the root directory
ROOT_DIR = Path(__file__).parent.parent

def read_documentation_file(file_path: Path) -> str:
    """
    Read a documentation file.
    
    Args:
        file_path: Path to the documentation file
        
    Returns:
        Content of the documentation file
    """
    try:
        with open(file_path, "r") as f:
            return f.read()
    except Exception as e:
        logger.error(f"Failed to read documentation file {file_path}: {str(e)}")
        return ""

def generate_email_content() -> str:
    """
    Generate the email content.
    
    Returns:
        Email content
    """
    logger.info("Generating email content...")
    
    # Read the email template
    email_template_path = ROOT_DIR / "docs" / "security_documentation_email.md"
    email_content = read_documentation_file(email_template_path)
    
    # If the email template doesn't exist, generate a basic email
    if not email_content:
        email_content = f"""# Bulo.Cloud Sentinel Security Documentation

Dear Team,

As part of our ongoing efforts to enhance the security of the Bulo.Cloud Sentinel platform, we have created comprehensive security documentation and implemented security scanning processes. This email provides an overview of the security resources now available to the team.

## Security Documentation

We have created the following security documentation:

1. **Security Guide**: A comprehensive guide to the security features, best practices, and processes implemented in the Bulo.Cloud Sentinel platform.
   - Location: `docs/security_guide.md`

2. **Security Code Review Checklist**: A detailed checklist for security-focused code reviews.
   - Location: `docs/security_code_review_checklist.md`

3. **Security Training**: A training document covering essential security concepts and best practices.
   - Location: `docs/security_training.md`

4. **Security Review Schedule**: A schedule and process for regular security reviews.
   - Location: `docs/security_review_schedule.md`

5. **Vulnerability Remediation Plan**: A plan to address the vulnerabilities identified in the security scans.
   - Location: `docs/vulnerability_remediation_plan.md`

## Security Scanning Tools

We have implemented the following security scanning tools:

1. **Scheduled Security Scan**: A comprehensive security scan that runs daily.
   - Script: `scripts/scheduled_security_scan.py`
   - Manual Trigger: `run_security_scan.ps1` (Windows) or run the script directly

2. **Dependency Scan**: A scan for vulnerable dependencies.
   - Script: `scripts/dependency_scan.py`
   - Usage: `python scripts/dependency_scan.py [--fix]`

3. **Security Review**: A script to generate security review templates.
   - Script: `scripts/security_review.py`
   - Usage: `python scripts/security_review.py [--daily] [--weekly] [--monthly] [--quarterly]`

## Next Steps

1. Review the security documentation.
2. Run security scans on your code.
3. Participate in security reviews.
4. Follow the vulnerability remediation plan.

If you have any questions or concerns about security, please don't hesitate to reach out to the security team.

Thank you for your commitment to security!

Best regards,
The Bulo.Cloud Sentinel Security Team
"""
    
    return email_content

def send_email(recipients: List[str], content: str):
    """
    Send an email with the security documentation.
    
    Args:
        recipients: List of email recipients
        content: Email content
    """
    logger.info("Sending email...")
    
    # Check if email configuration is available
    smtp_server = os.environ.get("SMTP_SERVER")
    smtp_port = os.environ.get("SMTP_PORT")
    smtp_username = os.environ.get("SMTP_USERNAME")
    smtp_password = os.environ.get("SMTP_PASSWORD")
    sender = os.environ.get("EMAIL_SENDER")
    
    if not all([smtp_server, smtp_port, smtp_username, smtp_password, sender]):
        logger.error("Email configuration not available")
        logger.info("Please set the following environment variables:")
        logger.info("  SMTP_SERVER: SMTP server address")
        logger.info("  SMTP_PORT: SMTP server port")
        logger.info("  SMTP_USERNAME: SMTP username")
        logger.info("  SMTP_PASSWORD: SMTP password")
        logger.info("  EMAIL_SENDER: Email sender address")
        return
    
    try:
        # Create the email
        msg = MIMEMultipart()
        msg["From"] = sender
        msg["To"] = ", ".join(recipients)
        msg["Subject"] = "Bulo.Cloud Sentinel Security Documentation"
        
        # Add the content
        msg.attach(MIMEText(content, "plain"))
        
        # Send the email
        with smtplib.SMTP(smtp_server, int(smtp_port)) as server:
            server.starttls()
            server.login(smtp_username, smtp_password)
            server.send_message(msg)
        
        logger.info("Email sent successfully")
    except Exception as e:
        logger.error(f"Failed to send email: {str(e)}")

def save_email_to_file(content: str) -> str:
    """
    Save the email content to a file.
    
    Args:
        content: Email content
        
    Returns:
        Path to the saved file
    """
    logger.info("Saving email to file...")
    
    # Get timestamp for filename
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    file_path = ROOT_DIR / f"security_documentation_email_{timestamp}.md"
    
    # Save the email content
    try:
        with open(file_path, "w") as f:
            f.write(content)
        
        logger.info(f"Email saved to {file_path}")
        return str(file_path)
    except Exception as e:
        logger.error(f"Failed to save email to file: {str(e)}")
        return ""

def main():
    """
    Main function.
    """
    parser = argparse.ArgumentParser(description="Share security documentation with the development team")
    parser.add_argument("--email", action="store_true", help="Send email with security documentation")
    parser.add_argument("--recipients", help="Comma-separated list of email recipients")
    
    args = parser.parse_args()
    
    # Generate email content
    content = generate_email_content()
    
    # Send email if requested
    if args.email:
        if args.recipients:
            recipients = args.recipients.split(",")
            send_email(recipients, content)
        else:
            logger.error("No recipients specified")
            logger.info("Please specify recipients with --recipients")
    else:
        # Save email to file
        file_path = save_email_to_file(content)
        if file_path:
            logger.info(f"Email content saved to {file_path}")
            logger.info("You can send this email manually or use the --email option to send it automatically")
    
    return 0

if __name__ == "__main__":
    sys.exit(main())
