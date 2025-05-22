#!/usr/bin/env python3
"""
Script to update dependencies across all requirements files in the codebase.

This script finds all requirements.txt files and updates vulnerable dependencies
to secure versions.
"""

import re
import logging
from pathlib import Path
from typing import List, Tuple

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger(__name__)

# Define the root directory
ROOT_DIR = Path(__file__).parent.parent

# Define patterns for finding requirements files
REQUIREMENTS_PATTERNS = [
    "requirements*.txt",
    "**/requirements*.txt",
    "setup.py",
    "**/setup.py",
    "pyproject.toml",
    "**/pyproject.toml"
]

# Define vulnerable dependencies and their secure replacements
VULNERABLE_DEPENDENCIES = {
    # Format: "dependency_name": ("vulnerable_pattern", "replacement")
    # Authentication and JWT
    "python-jose": (
        r"(python-jose(?:\[cryptography\])?==[\d\.]+)",
        "# \\1  # Removed due to vulnerabilities CVE-2024-33664 and CVE-2024-33663"
    ),
    "pyjwt": (
        r"(pyjwt(?:\[crypto\])?)==2\.10\.1",
        "\\1==2.8.0  # Using a secure version that doesn't have CVE-2024-53861"
    ),
    "authlib": (
        r"(authlib==(?:0|1\.(?:0|1[0-5]))\.[\d\.]+)",
        "authlib==1.2.1  # Updated to fix CVE-2023-30008"
    ),

    # Image processing
    "pillow": (
        r"(pillow==(?:9|10)\.[\d\.]+)",
        "pillow==11.2.2  # Updated to fix CVE-2024-28219, CVE-2023-50447, CVE-2024-4863"
    ),

    # Cryptography and security
    "cryptography": (
        r"(cryptography==(?:4[0-5]|3)\.[\d\.]+)",
        "cryptography==46.0.0  # Updated to latest version to fix CVE-2024-26130, CVE-2024-12797, CVE-2024-6119"
    ),
    "pyopenssl": (
        r"(pyopenssl==(?:2[0-3]|1[0-9]|[0-9])\.[\d\.]+)",
        "pyopenssl==25.0.0  # Updated to latest version"
    ),
    "certifi": (
        r"(certifi==(?:202[0-3]|201[0-9])\.[\d\.]+)",
        "certifi==2024.10.5  # Updated to latest version"
    ),

    # Web frameworks and APIs
    "fastapi": (
        r"(fastapi==(?:0\.[0-9][0-9]|0\.1[0-9][0-9])\.[\d\.]+)",
        "fastapi==0.115.12  # Updated to latest version for security improvements"
    ),
    "django": (
        r"(django==(?:3\.[0-2]|2|1)\.[\d\.]+)",
        "django==4.2.10  # Updated to fix CVE-2024-24680, CVE-2023-43665"
    ),
    "flask": (
        r"(flask==(?:2\.[0-2]|1|0)\.[\d\.]+)",
        "flask==2.3.3  # Updated to fix CVE-2023-30861"
    ),
    "werkzeug": (
        r"(werkzeug==(?:2\.[0-1]|1|0)\.[\d\.]+)",
        "werkzeug==2.3.8  # Updated to fix CVE-2023-46136"
    ),
    "jinja2": (
        r"(jinja2==(?:2\.[0-9]|1|0)\.[\d\.]+)",
        "jinja2==3.1.3  # Updated to fix CVE-2024-22195"
    ),

    # HTTP and networking
    "python-multipart": (
        r"(python-multipart==0\.0\.(?:[0-9]|1[0-9]|20))",
        "python-multipart==0.0.21  # Updated to latest version to fix CVE-2024-53981"
    ),
    "requests": (
        r"(requests==2\.(?:2[0-7]|1[0-9]|[0-9])\.[\d\.]+)",
        "requests==2.31.0  # Updated to fix CVE-2023-32681"
    ),
    "urllib3": (
        r"(urllib3==1\.(?:2[0-5]|1[0-9]|[0-9])\.[\d\.]+)",
        "urllib3==2.0.7  # Updated to fix CVE-2023-45803, CVE-2023-43804"
    ),
    "aiohttp": (
        r"(aiohttp==(?:3\.(?:0|1[0-7])|[0-2])\.[\d\.]+)",
        "aiohttp==3.11.18  # Updated to fix CVE-2024-52303, CVE-2024-52304, CVE-2024-42367"
    ),
    "websockets": (
        r"(websockets==(?:1[0-4]|[0-9])\.[\d\.]+)",
        "websockets==15.0.1  # Updated to latest version"
    ),

    # Databases
    "sqlalchemy": (
        r"(sqlalchemy==(?:1\.[3-4]|1\.[0-2]|0)\.[\d\.]+)",
        "sqlalchemy==2.0.40  # Updated to latest version"
    ),
    "psycopg2-binary": (
        r"(psycopg2-binary==(?:2\.(?:0|1|2|3|4|5|6|7|8))\.[\d\.]+)",
        "psycopg2-binary==2.9.10  # Updated to latest version"
    ),
    "pymongo": (
        r"(pymongo==(?:[0-3]|4\.(?:0|1|2|3|4|5|6|7|8|9|10|11))\.[\d\.]+)",
        "pymongo==4.12.1  # Updated to latest version"
    ),

    # AI and ML
    "langchain-community": (
        r"(langchain-community==0\.3\.(?:1[0-7]|[0-9]))",
        "langchain-community==0.4.0  # Updated to fix CVE-2024-8309, CVE-2024-2965, CVE-2024-46946"
    ),
    "transformers": (
        r"(transformers==(?:4\.(?:0|1|2|3|4|5|6|7|8|9|10|11|12|13|14|15|16|17|18|19|20|21|22|23|24|25|26|27|28|29|30|31|32|33|34|35|36|37|38|39|40|41|42|43|44|45|46|47|48|49|50))\.[\d\.]+)",
        "transformers==4.51.3  # Updated to latest version"
    ),
    "tensorflow": (
        r"(tensorflow==(?:2\.(?:0|1|2|3|4|5|6|7|8|9|10|11|12|13|14|15|16|17|18))\.[\d\.]+)",
        "tensorflow==2.19.0  # Updated to latest version"
    ),
    "torch": (
        r"(torch==(?:1|2\.(?:0|1|2|3|4|5|6))\.[\d\.]+)",
        "torch==2.7.0  # Pinned to specific version for better security"
    ),
    "onnx": (
        r"(onnx==(?:1\.(?:0|1|2|3|4|5|6|7|8|9|10|11|12|13|14|15|16))\.[\d\.]+)",
        "onnx==1.17.0  # Updated to fix CVE-2024-7776"
    ),

    # Utilities
    "pyyaml": (
        r"(pyyaml==(?:5|4|3)\.[\d\.]+)",
        "pyyaml==6.0.1  # Updated to fix CVE-2022-31163, CVE-2022-31097"
    ),
    "numpy": (
        r"(numpy==(?:1\.(?:1[0-9]|20|21|22|23|24|25)|2\.(?:0|1))\.[\d\.]+)",
        "numpy==2.2.5  # Updated to latest version"
    ),
    "scipy": (
        r"(scipy==(?:1\.(?:0|1|2|3|4|5|6|7|8|9|10|11|12|13|14)|2\.(?:0|1))\.[\d\.]+)",
        "scipy==1.15.3  # Updated to latest version"
    ),
}

# Define dependencies to add if they're missing
DEPENDENCIES_TO_ADD = {
    # Format: "dependency_name": "dependency_line"
    # Authentication and security
    "pyjwt": "pyjwt==2.8.0  # Using a secure version that doesn't have CVE-2024-53861",
    "argon2-cffi": "argon2-cffi==23.1.0  # For secure password hashing",
    "bcrypt": "bcrypt==4.3.0  # For secure password hashing",
    "passlib": "passlib[bcrypt]==1.7.4  # For password hashing and verification",

    # Validation and security
    "pydantic": "pydantic==2.11.4  # For secure data validation",
    "python-magic": "python-magic==0.4.27  # For secure file type detection",
    "email-validator": "email-validator==2.2.0  # For secure email validation",

    # Cryptography
    "cryptography": "cryptography==46.0.0  # Updated to latest version to fix CVE-2024-26130, CVE-2024-12797, CVE-2024-6119",
    "pyopenssl": "pyopenssl==25.0.0  # Updated to latest version",
    "certifi": "certifi==2024.10.5  # Updated to latest version",

    # Security tools
    "safety": "safety==3.5.0  # For dependency vulnerability scanning",
    "bandit": "bandit==1.7.7  # For security static analysis",
    "semgrep": "semgrep==1.71.0  # For code security scanning",
    "mypy": "mypy==1.9.0  # For type checking",
    "ruff": "ruff==0.4.7  # For linting",

    # Secure logging
    "loguru": "loguru==0.7.3  # For secure logging",
}

def find_requirements_files() -> List[Path]:
    """
    Find all requirements files in the codebase.

    Returns:
        List of paths to requirements files
    """
    requirements_files = []

    for pattern in REQUIREMENTS_PATTERNS:
        requirements_files.extend(ROOT_DIR.glob(pattern))

    return requirements_files

def update_requirements_file(file_path: Path) -> Tuple[int, int]:
    """
    Update a requirements file with secure dependencies.

    Args:
        file_path: Path to the requirements file

    Returns:
        Tuple of (number of updates, number of additions)
    """
    logger.info(f"Updating {file_path}")

    # Read the file
    with open(file_path, "r", encoding="utf-8") as f:
        content = f.read()

    # Track updates and additions
    updates = 0
    additions = 0

    # Update vulnerable dependencies
    for dep_name, (pattern, replacement) in VULNERABLE_DEPENDENCIES.items():
        # Check if the dependency is present
        if re.search(pattern, content, re.MULTILINE):
            # Update the dependency
            new_content = re.sub(pattern, replacement, content, flags=re.MULTILINE)
            if new_content != content:
                content = new_content
                updates += 1
                logger.info(f"  Updated {dep_name}")

    # Check for python-jose comments and add PyJWT if needed
    if "python-jose" in content and "pyjwt" not in content.lower():
        # Add PyJWT after the python-jose line
        content = re.sub(
            r"(#.*python-jose.*)",
            f"\\1\n{DEPENDENCIES_TO_ADD['pyjwt']}",
            content,
            flags=re.MULTILINE
        )
        additions += 1
        logger.info(f"  Added PyJWT")

    # Add security-related dependencies if they're missing
    for dep_name, dep_line in DEPENDENCIES_TO_ADD.items():
        # Skip PyJWT as it's handled separately
        if dep_name == "pyjwt":
            continue

        # Check if the dependency is already present
        if not re.search(rf"{dep_name}(?:\[[^\]]*\])?==", content, re.MULTILINE | re.IGNORECASE):
            # Add the dependency at the end of the file
            if not content.endswith("\n"):
                content += "\n"
            content += f"{dep_line}\n"
            additions += 1
            logger.info(f"  Added {dep_name}")

    # Write the updated content
    with open(file_path, "w", encoding="utf-8") as f:
        f.write(content)

    return updates, additions

def main():
    """
    Main function.
    """
    logger.info("Starting dependency update")

    # Find requirements files
    requirements_files = find_requirements_files()
    logger.info(f"Found {len(requirements_files)} requirements files")

    # Update each file
    total_updates = 0
    total_additions = 0

    for file_path in requirements_files:
        updates, additions = update_requirements_file(file_path)
        total_updates += updates
        total_additions += additions

    logger.info(f"Completed dependency update")
    logger.info(f"  Total updates: {total_updates}")
    logger.info(f"  Total additions: {total_additions}")

if __name__ == "__main__":
    main()
