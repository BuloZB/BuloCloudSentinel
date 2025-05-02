#!/usr/bin/env python3
"""
Script to update authentication implementations across the codebase.

This script finds all authentication implementations and updates them to use
the unified authentication module.
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

# Define patterns for finding authentication files
AUTH_FILE_PATTERNS = [
    "**/auth.py",
    "**/authentication.py",
    "**/jwt_handler.py",
    "**/password.py",
    "**/security.py",
    "**/login.py",
    "**/auth_*.py",
]

# Define import patterns to replace
IMPORT_PATTERNS = {
    # Format: "pattern": "replacement"
    r"from jose import jwt": "import jwt",
    r"from jose import JWTError": "from jwt.exceptions import PyJWTError as JWTError",
    r"from jose import JWTError, jwt": "import jwt\nfrom jwt.exceptions import PyJWTError as JWTError",
    r"import jwt\nfrom jose import JWTError": "import jwt\nfrom jwt.exceptions import PyJWTError as JWTError",
    r"from passlib\.context import CryptContext": "from security.auth.unified_auth import hash_password, verify_password",
}

# Define function patterns to replace
FUNCTION_PATTERNS = {
    # Format: "pattern": "replacement"
    r"def get_password_hash\(password: str\)(.*?)return pwd_context\.hash\(password\)": 
        "def get_password_hash(password: str) -> str:\n    return hash_password(password)",
    
    r"def verify_password\(plain_password: str, hashed_password: str\)(.*?)return pwd_context\.verify\(plain_password, hashed_password\)": 
        "def verify_password(plain_password: str, hashed_password: str) -> bool:\n    return verify_password(plain_password, hashed_password)",
    
    r"def create_access_token\((.*?)\)(.*?)jwt\.encode\((.*?), (.*?), algorithm=(.*?)\)": 
        "def create_access_token(\\1)\\2from security.auth.unified_auth import create_access_token as _create_access_token\n    return _create_access_token(subject=\\3)",
}

# Define the unified auth import to add
UNIFIED_AUTH_IMPORT = "from security.auth.unified_auth import (\n    hash_password,\n    verify_password,\n    create_access_token,\n    create_refresh_token,\n    decode_token,\n    get_current_token_data,\n    get_current_user_id,\n    has_role,\n    has_permission,\n    require_role,\n    require_permission,\n)"

def find_auth_files() -> List[Path]:
    """
    Find all authentication files in the codebase.
    
    Returns:
        List of paths to authentication files
    """
    auth_files = []
    
    for pattern in AUTH_FILE_PATTERNS:
        auth_files.extend(ROOT_DIR.glob(pattern))
    
    return auth_files

def update_auth_file(file_path: Path) -> int:
    """
    Update an authentication file to use the unified authentication module.
    
    Args:
        file_path: Path to the authentication file
        
    Returns:
        Number of updates made
    """
    logger.info(f"Updating {file_path}")
    
    # Read the file
    with open(file_path, "r", encoding="utf-8") as f:
        content = f.read()
    
    # Track updates
    updates = 0
    
    # Check if the file already imports the unified auth module
    if "from security.auth.unified_auth import" in content:
        logger.info(f"  Already using unified auth")
        return updates
    
    # Update imports
    for pattern, replacement in IMPORT_PATTERNS.items():
        if re.search(pattern, content):
            new_content = re.sub(pattern, replacement, content)
            if new_content != content:
                content = new_content
                updates += 1
                logger.info(f"  Updated import: {pattern}")
    
    # Update functions
    for pattern, replacement in FUNCTION_PATTERNS.items():
        if re.search(pattern, content, re.DOTALL):
            new_content = re.sub(pattern, replacement, content, flags=re.DOTALL)
            if new_content != content:
                content = new_content
                updates += 1
                logger.info(f"  Updated function: {pattern[:30]}...")
    
    # Add unified auth import if needed
    if updates > 0 and "from security.auth.unified_auth import" not in content:
        # Find the last import
        import_match = re.search(r"^import .*$|^from .* import .*$", content, re.MULTILINE)
        if import_match:
            last_import_pos = content.rfind(import_match.group(0))
            last_import_end = last_import_pos + len(import_match.group(0))
            
            # Insert the unified auth import after the last import
            content = content[:last_import_end] + "\n\n" + UNIFIED_AUTH_IMPORT + content[last_import_end:]
            updates += 1
            logger.info(f"  Added unified auth import")
    
    # Write the updated content
    with open(file_path, "w", encoding="utf-8") as f:
        f.write(content)
    
    return updates

def main():
    """
    Main function.
    """
    logger.info("Starting authentication update")
    
    # Find authentication files
    auth_files = find_auth_files()
    logger.info(f"Found {len(auth_files)} authentication files")
    
    # Update each file
    total_updates = 0
    
    for file_path in auth_files:
        updates = update_auth_file(file_path)
        total_updates += updates
    
    logger.info(f"Completed authentication update")
    logger.info(f"  Total updates: {total_updates}")

if __name__ == "__main__":
    main()
