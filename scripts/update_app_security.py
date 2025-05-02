#!/usr/bin/env python3
"""
Script to update the main application to use the new security features.

This script updates the main FastAPI application to use the unified security
middleware and error handling.
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

# Define patterns for finding main application files
APP_FILE_PATTERNS = [
    "backend/app/main.py",
    "backend/main.py",
    "**/main.py",
    "**/app.py",
]

# Define the security import to add
SECURITY_IMPORT = """
from security.api.unified_security import configure_security
from security.error_handling.secure_error_handler import configure_error_handlers, configure_custom_exception_handlers
"""

# Define the security configuration to add
SECURITY_CONFIG = """
# Configure security middleware
configure_security(
    app,
    # Security headers settings
    include_csp=True,
    
    # CSRF protection settings
    enable_csrf=True,
    csrf_secret_key=os.getenv("SECRET_KEY", ""),
    
    # Rate limiting settings
    enable_rate_limiting=True,
    rate_limit=100,
    rate_window=60,
    redis_url=os.getenv("REDIS_URL", "redis://localhost:6379/0"),
    
    # Common settings
    exclude_paths=["/docs", "/redoc", "/openapi.json"],
    
    # Error handling settings
    configure_errors=True
)
"""

def find_app_files() -> List[Path]:
    """
    Find all main application files in the codebase.
    
    Returns:
        List of paths to main application files
    """
    app_files = []
    
    for pattern in APP_FILE_PATTERNS:
        app_files.extend(ROOT_DIR.glob(pattern))
    
    return app_files

def update_app_file(file_path: Path) -> int:
    """
    Update a main application file to use the new security features.
    
    Args:
        file_path: Path to the main application file
        
    Returns:
        Number of updates made
    """
    logger.info(f"Updating {file_path}")
    
    # Read the file
    with open(file_path, "r", encoding="utf-8") as f:
        content = f.read()
    
    # Track updates
    updates = 0
    
    # Check if the file contains a FastAPI application
    if "from fastapi import FastAPI" in content and "app = FastAPI" in content:
        # Check if the file already uses the unified security middleware
        if "from security.api.unified_security import configure_security" in content:
            logger.info(f"  Already using unified security")
            return updates
        
        # Add the security import
        import_match = re.search(r"^import .*$|^from .* import .*$", content, re.MULTILINE)
        if import_match:
            last_import_pos = content.rfind(import_match.group(0))
            last_import_end = last_import_pos + len(import_match.group(0))
            
            # Insert the security import after the last import
            content = content[:last_import_end] + "\n" + SECURITY_IMPORT + content[last_import_end:]
            updates += 1
            logger.info(f"  Added security import")
        
        # Add the security configuration
        app_creation_match = re.search(r"app = FastAPI\(.*?\)", content, re.DOTALL)
        if app_creation_match:
            app_creation_end = app_creation_match.end()
            
            # Insert the security configuration after the app creation
            content = content[:app_creation_end] + "\n" + SECURITY_CONFIG + content[app_creation_end:]
            updates += 1
            logger.info(f"  Added security configuration")
        
        # Remove existing security middleware
        existing_middleware_patterns = [
            r"app\.add_middleware\(SecurityHeadersMiddleware,.*?\)",
            r"app\.add_middleware\(CSRFMiddleware,.*?\)",
            r"app\.add_middleware\(RateLimitMiddleware,.*?\)",
            r"configure_error_handlers\(app\)",
            r"configure_custom_exception_handlers\(app\)",
        ]
        
        for pattern in existing_middleware_patterns:
            if re.search(pattern, content, re.DOTALL):
                content = re.sub(pattern, "", content, flags=re.DOTALL)
                updates += 1
                logger.info(f"  Removed existing middleware: {pattern}")
    
    # Write the updated content
    with open(file_path, "w", encoding="utf-8") as f:
        f.write(content)
    
    return updates

def main():
    """
    Main function.
    """
    logger.info("Starting application security update")
    
    # Find main application files
    app_files = find_app_files()
    logger.info(f"Found {len(app_files)} main application files")
    
    # Update each file
    total_updates = 0
    
    for file_path in app_files:
        updates = update_app_file(file_path)
        total_updates += updates
    
    logger.info(f"Completed application security update")
    logger.info(f"  Total updates: {total_updates}")

if __name__ == "__main__":
    main()
