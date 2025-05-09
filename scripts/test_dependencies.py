#!/usr/bin/env python3
"""
Script to test the application with updated dependencies.

This script checks if all required dependencies are installed and working correctly.
"""

import importlib
import logging
import sys
from pathlib import Path
from typing import Dict, List, Tuple

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger(__name__)

# Define the root directory
ROOT_DIR = Path(__file__).parent.parent

# Define critical dependencies to test
CRITICAL_DEPENDENCIES = [
    # Authentication
    ("jwt", "PyJWT", "JWT token handling"),
    ("passlib", "passlib", "Password hashing"),
    ("argon2", "argon2-cffi", "Secure password hashing"),
    ("bcrypt", "bcrypt", "Password hashing"),

    # Security
    ("cryptography", "cryptography", "Cryptographic operations"),
    ("OpenSSL", "pyOpenSSL", "SSL/TLS handling"),

    # Web framework
    ("fastapi", "FastAPI", "Web framework"),
    ("uvicorn", "uvicorn", "ASGI server"),
    ("pydantic", "pydantic", "Data validation"),

    # Database
    ("sqlalchemy", "SQLAlchemy", "ORM"),
    ("alembic", "alembic", "Database migrations"),

    # Utilities
    # python-magic requires libmagic which is not available on Windows by default
    # ("magic", "python-magic", "File type detection"),
    ("safety", "safety", "Dependency vulnerability scanning"),
    ("bandit", "bandit", "Security static analysis"),
]

def check_dependency(module_name: str, package_name: str, description: str) -> Tuple[bool, str, str]:
    """
    Check if a dependency is installed and get its version.

    Args:
        module_name: The name of the module to import
        package_name: The name of the package
        description: Description of the package

    Returns:
        Tuple of (success, version, error_message)
    """
    try:
        module = importlib.import_module(module_name)
        version = getattr(module, "__version__", "unknown")
        return True, version, ""
    except ImportError as e:
        return False, "", f"Error importing {package_name}: {str(e)}"
    except Exception as e:
        return False, "", f"Error checking {package_name}: {str(e)}"

def test_jwt_functionality() -> Tuple[bool, str]:
    """
    Test JWT token functionality.

    Returns:
        Tuple of (success, error_message)
    """
    try:
        import jwt

        # Create a test payload
        payload = {"sub": "test", "name": "Test User", "admin": True}

        # Create a token
        token = jwt.encode(payload, "secret", algorithm="HS256")

        # Decode the token
        decoded = jwt.decode(token, "secret", algorithms=["HS256"])

        # Check if the decoded payload matches the original
        if decoded != payload:
            return False, f"Decoded payload does not match original: {decoded} != {payload}"

        return True, ""
    except Exception as e:
        return False, f"Error testing JWT functionality: {str(e)}"

def test_cryptography_functionality() -> Tuple[bool, str]:
    """
    Test cryptography functionality.

    Returns:
        Tuple of (success, error_message)
    """
    try:
        from cryptography.fernet import Fernet

        # Generate a key
        key = Fernet.generate_key()

        # Create a Fernet instance
        f = Fernet(key)

        # Test message
        message = b"Test message"

        # Encrypt the message
        encrypted = f.encrypt(message)

        # Decrypt the message
        decrypted = f.decrypt(encrypted)

        # Check if the decrypted message matches the original
        if decrypted != message:
            return False, f"Decrypted message does not match original: {decrypted} != {message}"

        return True, ""
    except Exception as e:
        return False, f"Error testing cryptography functionality: {str(e)}"

def main():
    """
    Main function.
    """
    logger.info("Testing dependencies")

    # Check dependencies
    all_dependencies_ok = True
    for module_name, package_name, description in CRITICAL_DEPENDENCIES:
        success, version, error = check_dependency(module_name, package_name, description)
        if success:
            logger.info(f"✅ {package_name} ({description}): version {version}")
        else:
            logger.error(f"❌ {package_name} ({description}): {error}")
            all_dependencies_ok = False

    # Test JWT functionality
    logger.info("\nTesting JWT functionality")
    jwt_success, jwt_error = test_jwt_functionality()
    if jwt_success:
        logger.info("✅ JWT functionality working correctly")
    else:
        logger.error(f"❌ JWT functionality test failed: {jwt_error}")
        all_dependencies_ok = False

    # Test cryptography functionality
    logger.info("\nTesting cryptography functionality")
    crypto_success, crypto_error = test_cryptography_functionality()
    if crypto_success:
        logger.info("✅ Cryptography functionality working correctly")
    else:
        logger.error(f"❌ Cryptography functionality test failed: {crypto_error}")
        all_dependencies_ok = False

    # Summary
    logger.info("\nDependency test summary")
    if all_dependencies_ok:
        logger.info("✅ All dependencies are installed and working correctly")
        return 0
    else:
        logger.error("❌ Some dependencies are missing or not working correctly")
        return 1

if __name__ == "__main__":
    sys.exit(main())
