#!/usr/bin/env python3
"""
Script to prepare the repository for upload to GitHub.

This script:
1. Creates/updates .gitignore to exclude test reports and sensitive files
2. Checks for any sensitive information in the codebase
3. Prepares a summary of changes to be uploaded
"""

import os
import re
import sys
import logging
import subprocess
from pathlib import Path
from typing import List, Set

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger(__name__)

# Define the root directory
ROOT_DIR = Path(__file__).parent.parent

# Define patterns for files to exclude
EXCLUDE_PATTERNS = [
    # Test reports
    "reports/",
    "reports/security/",
    "**/*.sarif",
    "**/*-report.json",
    "**/*-report.xml",
    "**/*-report.html",
    "**/*-results.json",
    "**/*-results.xml",
    "**/*-results.html",
    
    # Sensitive files
    "**/.env",
    "**/.env.*",
    "**/secrets.yaml",
    "**/secrets.yml",
    "**/secrets.json",
    "**/credentials.yaml",
    "**/credentials.yml",
    "**/credentials.json",
    
    # Temporary files
    "**/.DS_Store",
    "**/__pycache__/",
    "**/*.pyc",
    "**/*.pyo",
    "**/*.pyd",
    "**/.Python",
    "**/env/",
    "**/venv/",
    "**/.venv/",
    "**/ENV/",
    "**/env.bak/",
    "**/venv.bak/",
    
    # IDE files
    "**/.idea/",
    "**/.vscode/",
    "**/*.swp",
    "**/*.swo",
    
    # Build files
    "**/build/",
    "**/dist/",
    "**/*.egg-info/",
]

# Define patterns for sensitive information
SENSITIVE_PATTERNS = [
    # API keys
    r"api[_-]?key[_-]?=\s*['\"]([\w\d]{20,})['\"]",
    r"api[_-]?secret[_-]?=\s*['\"]([\w\d]{20,})['\"]",
    
    # AWS credentials
    r"aws[_-]?access[_-]?key[_-]?id[_-]?=\s*['\"]([\w\d]{20,})['\"]",
    r"aws[_-]?secret[_-]?access[_-]?key[_-]?=\s*['\"]([\w\d]{20,})['\"]",
    
    # Database credentials
    r"(?:password|passwd|pwd)[_-]?=\s*['\"]((?!\bplaceholderpassword\b)[\w\d@$!%*#?&]{8,})['\"]",
    r"(?:username|user|uid)[_-]?=\s*['\"]((?!\bplaceholderusername\b)[\w\d@$!%*#?&]{3,})['\"]",
    
    # Private keys
    r"-----BEGIN (?:RSA|DSA|EC|OPENSSH) PRIVATE KEY-----",
    
    # JWT tokens
    r"eyJ[a-zA-Z0-9_-]{10,}\.[a-zA-Z0-9_-]{10,}\.[a-zA-Z0-9_-]{10,}",
]

def update_gitignore() -> None:
    """
    Update .gitignore to exclude test reports and sensitive files.
    """
    gitignore_path = ROOT_DIR / ".gitignore"
    
    # Read existing .gitignore if it exists
    existing_patterns = set()
    if gitignore_path.exists():
        with open(gitignore_path, "r", encoding="utf-8") as f:
            for line in f:
                line = line.strip()
                if line and not line.startswith("#"):
                    existing_patterns.add(line)
    
    # Add new patterns
    new_patterns = set(EXCLUDE_PATTERNS) - existing_patterns
    
    if new_patterns:
        logger.info(f"Adding {len(new_patterns)} new patterns to .gitignore")
        
        with open(gitignore_path, "a", encoding="utf-8") as f:
            f.write("\n# Added by prepare_github_upload.py\n")
            for pattern in sorted(new_patterns):
                f.write(f"{pattern}\n")
    else:
        logger.info("No new patterns to add to .gitignore")

def check_sensitive_information() -> List[str]:
    """
    Check for sensitive information in the codebase.
    
    Returns:
        List of files containing sensitive information
    """
    logger.info("Checking for sensitive information")
    
    sensitive_files = []
    
    # Get all text files
    text_extensions = {".py", ".js", ".ts", ".jsx", ".tsx", ".html", ".css", ".md", ".yml", ".yaml", ".json", ".xml", ".txt", ".sh", ".bat", ".ps1"}
    
    for root, _, files in os.walk(ROOT_DIR):
        for file in files:
            file_path = Path(root) / file
            
            # Skip excluded directories
            if any(part.startswith(".") for part in file_path.parts):
                continue
                
            # Skip non-text files
            if file_path.suffix.lower() not in text_extensions:
                continue
            
            # Check file content
            try:
                with open(file_path, "r", encoding="utf-8") as f:
                    content = f.read()
                    
                    for pattern in SENSITIVE_PATTERNS:
                        matches = re.findall(pattern, content)
                        if matches:
                            logger.warning(f"Found sensitive information in {file_path}")
                            sensitive_files.append(str(file_path))
                            break
            except Exception as e:
                logger.error(f"Error reading {file_path}: {str(e)}")
    
    return sensitive_files

def prepare_summary() -> None:
    """
    Prepare a summary of changes to be uploaded.
    """
    logger.info("Preparing summary of changes")
    
    # Get git status
    try:
        result = subprocess.run(
            ["git", "status", "--porcelain"],
            check=True,
            capture_output=True,
            text=True
        )
        
        status_output = result.stdout.strip()
        
        if status_output:
            logger.info("Changes to be uploaded:")
            for line in status_output.split("\n"):
                logger.info(f"  {line}")
        else:
            logger.info("No changes to be uploaded")
    except Exception as e:
        logger.error(f"Error getting git status: {str(e)}")

def main() -> int:
    """
    Main function.
    
    Returns:
        Exit code (0 for success, non-zero for failure)
    """
    logger.info("Preparing repository for upload to GitHub")
    
    # Update .gitignore
    update_gitignore()
    
    # Check for sensitive information
    sensitive_files = check_sensitive_information()
    
    if sensitive_files:
        logger.error(f"Found sensitive information in {len(sensitive_files)} files:")
        for file in sensitive_files:
            logger.error(f"  {file}")
        logger.error("Please remove sensitive information before uploading to GitHub")
        return 1
    
    # Prepare summary
    prepare_summary()
    
    logger.info("Repository is ready for upload to GitHub")
    logger.info("To upload, run the following commands:")
    logger.info("  git add .")
    logger.info("  git commit -m \"Update repository with security improvements\"")
    logger.info("  git push origin main")
    
    return 0

if __name__ == "__main__":
    sys.exit(main())
