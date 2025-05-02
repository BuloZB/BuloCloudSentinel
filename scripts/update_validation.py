#!/usr/bin/env python3
"""
Script to update input validation implementations across the codebase.

This script finds all input validation implementations and updates them to use
the unified validation module.
"""

import os
import re
import logging
from pathlib import Path
from typing import Dict, List, Set, Tuple

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger(__name__)

# Define the root directory
ROOT_DIR = Path(__file__).parent.parent

# Define patterns for finding validation files
VALIDATION_FILE_PATTERNS = [
    "**/validation.py",
    "**/input_validation.py",
    "**/validators.py",
    "**/form_validation.py",
    "**/request_validation.py",
    "**/sanitization.py",
    "**/validation_*.py",
]

# Define patterns for finding API endpoint files
API_FILE_PATTERNS = [
    "**/api/*.py",
    "**/routers/*.py",
    "**/endpoints/*.py",
    "**/controllers/*.py",
]

# Define import patterns to replace
IMPORT_PATTERNS = {
    # Format: "pattern": "replacement"
    r"from .*\.validation import .*": "from security.validation.unified_validation import (\n    validate_email,\n    validate_username,\n    validate_name,\n    validate_uuid,\n    validate_url,\n    sanitize_string,\n    sanitize_html,\n    check_sql_injection,\n    input_validator,\n    form_validator,\n    request_validator,\n)",
    r"import re\nimport html": "from security.validation.unified_validation import (\n    validate_email,\n    validate_username,\n    validate_name,\n    validate_uuid,\n    validate_url,\n    sanitize_string,\n    sanitize_html,\n    check_sql_injection,\n    input_validator,\n    form_validator,\n    request_validator,\n)",
}

# Define the unified validation import to add
UNIFIED_VALIDATION_IMPORT = "from security.validation.unified_validation import (\n    validate_email,\n    validate_username,\n    validate_name,\n    validate_uuid,\n    validate_url,\n    sanitize_string,\n    sanitize_html,\n    check_sql_injection,\n    input_validator,\n    form_validator,\n    request_validator,\n)"

# Define validation function to add to API endpoints
VALIDATION_FUNCTION = '''
def validate_request_data(request_data: dict, schema: dict) -> dict:
    """
    Validate request data against a schema.

    Args:
        request_data: Request data to validate
        schema: Validation schema

    Returns:
        Validated request data
    """
    return request_validator.validate_request(request_data, schema)
'''

def find_validation_files() -> List[Path]:
    """
    Find all validation files in the codebase.

    Returns:
        List of paths to validation files
    """
    validation_files = []

    for pattern in VALIDATION_FILE_PATTERNS:
        validation_files.extend(ROOT_DIR.glob(pattern))

    return validation_files

def find_api_files() -> List[Path]:
    """
    Find all API endpoint files in the codebase.

    Returns:
        List of paths to API endpoint files
    """
    api_files = []

    for pattern in API_FILE_PATTERNS:
        api_files.extend(ROOT_DIR.glob(pattern))

    return api_files

def update_validation_file(file_path: Path) -> int:
    """
    Update a validation file to use the unified validation module.

    Args:
        file_path: Path to the validation file

    Returns:
        Number of updates made
    """
    logger.info(f"Updating validation file: {file_path}")

    # Read the file
    with open(file_path, "r", encoding="utf-8") as f:
        content = f.read()

    # Track updates
    updates = 0

    # Check if the file already imports the unified validation module
    if "from security.validation.unified_validation import" in content:
        logger.info(f"  Already using unified validation")
        return updates

    # Update imports
    for pattern, replacement in IMPORT_PATTERNS.items():
        if re.search(pattern, content):
            new_content = re.sub(pattern, replacement, content)
            if new_content != content:
                content = new_content
                updates += 1
                logger.info(f"  Updated import: {pattern}")

    # Write the updated content
    with open(file_path, "w", encoding="utf-8") as f:
        f.write(content)

    return updates

def update_api_file(file_path: Path) -> int:
    """
    Update an API endpoint file to use the unified validation module.

    Args:
        file_path: Path to the API endpoint file

    Returns:
        Number of updates made
    """
    logger.info(f"Updating API file: {file_path}")

    # Read the file
    with open(file_path, "r", encoding="utf-8") as f:
        content = f.read()

    # Track updates
    updates = 0

    # Check if the file already imports the unified validation module
    if "from security.validation.unified_validation import" in content:
        logger.info(f"  Already using unified validation")
        return updates

    # Check if the file has API endpoints
    if "@router" in content or "@app" in content:
        # Add the unified validation import
        import_match = re.search(r"^import .*$|^from .* import .*$", content, re.MULTILINE)
        if import_match:
            last_import_pos = content.rfind(import_match.group(0))
            last_import_end = last_import_pos + len(import_match.group(0))

            # Insert the unified validation import after the last import
            content = content[:last_import_end] + "\n\n" + UNIFIED_VALIDATION_IMPORT + content[last_import_end:]
            updates += 1
            logger.info(f"  Added unified validation import")

        # Add the validation function if it doesn't exist
        if "def validate_request_data" not in content:
            # Find a good place to add the function
            # Look for the first function definition
            func_match = re.search(r"^def ", content, re.MULTILINE)
            if func_match:
                func_pos = func_match.start()

                # Insert the validation function before the first function
                content = content[:func_pos] + VALIDATION_FUNCTION + "\n" + content[func_pos:]
                updates += 1
                logger.info(f"  Added validation function")

    # Write the updated content
    with open(file_path, "w", encoding="utf-8") as f:
        f.write(content)

    return updates

def main():
    """
    Main function.
    """
    logger.info("Starting validation update")

    # Find validation files
    validation_files = find_validation_files()
    logger.info(f"Found {len(validation_files)} validation files")

    # Update each validation file
    validation_updates = 0
    for file_path in validation_files:
        updates = update_validation_file(file_path)
        validation_updates += updates

    # Find API endpoint files
    api_files = find_api_files()
    logger.info(f"Found {len(api_files)} API endpoint files")

    # Update each API endpoint file
    api_updates = 0
    for file_path in api_files:
        updates = update_api_file(file_path)
        api_updates += updates

    logger.info(f"Completed validation update")
    logger.info(f"  Validation file updates: {validation_updates}")
    logger.info(f"  API endpoint file updates: {api_updates}")
    logger.info(f"  Total updates: {validation_updates + api_updates}")

if __name__ == "__main__":
    main()
