"""
Password handling for Bulo.Cloud Sentinel Security Module.

This module provides secure password hashing, verification, and policy enforcement
using the Argon2id algorithm, which is recommended by OWASP for password hashing.
"""

import re
import secrets
import string
import logging
from typing import Dict, List, Tuple, Optional, Set, Union

from argon2 import PasswordHasher
from argon2.exceptions import VerifyMismatchError, InvalidHash
from pydantic import BaseModel, Field, validator

# Set up logging
log = logging.getLogger(__name__)

# Initialize Argon2 password hasher with secure parameters
# These parameters are based on OWASP recommendations
ph = PasswordHasher(
    time_cost=3,  # Number of iterations
    memory_cost=65536,  # 64 MB
    parallelism=4,  # Number of parallel threads
    hash_len=32,  # Length of the hash in bytes
    salt_len=16,  # Length of the salt in bytes
)

# Password policy configuration
MIN_PASSWORD_LENGTH = 12
REQUIRE_UPPERCASE = True
REQUIRE_LOWERCASE = True
REQUIRE_DIGITS = True
REQUIRE_SPECIAL_CHARS = True
MAX_PASSWORD_LENGTH = 128
PASSWORD_HISTORY_SIZE = 5

# Common passwords (top 20 most common)
COMMON_PASSWORDS = {
    "123456", "123456789", "qwerty", "password", "12345", "qwerty123", "1q2w3e",
    "12345678", "111111", "1234567890", "1234567", "abc123", "password1",
    "admin", "welcome", "monkey", "login", "sunshine", "bailey", "princess"
}

# Models
class PasswordPolicy(BaseModel):
    """Password policy configuration."""
    min_length: int = Field(default=MIN_PASSWORD_LENGTH, ge=8)
    max_length: int = Field(default=MAX_PASSWORD_LENGTH, le=256)
    require_uppercase: bool = Field(default=REQUIRE_UPPERCASE)
    require_lowercase: bool = Field(default=REQUIRE_LOWERCASE)
    require_digits: bool = Field(default=REQUIRE_DIGITS)
    require_special_chars: bool = Field(default=REQUIRE_SPECIAL_CHARS)
    history_size: int = Field(default=PASSWORD_HISTORY_SIZE, ge=0)
    check_common_passwords: bool = Field(default=True)
    check_username_in_password: bool = Field(default=True)
    check_personal_info: bool = Field(default=True)
    check_sequential_chars: bool = Field(default=True)
    check_repeated_chars: bool = Field(default=True)

class PasswordValidationError(Exception):
    """Exception raised for password validation errors."""
    pass

def hash_password(password: str) -> str:
    """
    Hash a password using Argon2id.

    Args:
        password: Plain text password

    Returns:
        Hashed password
    """
    return ph.hash(password)

def verify_password(hashed_password: str, plain_password: str) -> bool:
    """
    Verify a password against a hash.

    Args:
        hashed_password: Hashed password
        plain_password: Plain text password to verify

    Returns:
        True if password matches, False otherwise
    """
    try:
        ph.verify(hashed_password, plain_password)

        # Check if the password hash needs to be rehashed
        if ph.check_needs_rehash(hashed_password):
            log.info("Password hash needs rehashing due to parameter changes")

        return True
    except VerifyMismatchError:
        return False
    except InvalidHash as e:
        log.error(f"Invalid password hash format: {str(e)}")
        return False

def check_password_needs_rehash(hashed_password: str) -> bool:
    """
    Check if a password hash needs to be rehashed.

    Args:
        hashed_password: Hashed password

    Returns:
        True if password needs rehashing, False otherwise
    """
    try:
        return ph.check_needs_rehash(hashed_password)
    except InvalidHash as e:
        log.error(f"Invalid password hash format: {str(e)}")
        return True  # If the hash is invalid, it definitely needs rehashing

def validate_password(
    password: str,
    policy: Optional[PasswordPolicy] = None,
    username: Optional[str] = None,
    personal_info: Optional[List[str]] = None
) -> Tuple[bool, Optional[str]]:
    """
    Validate a password against the password policy.

    Args:
        password: Password to validate
        policy: Optional custom password policy
        username: Optional username to check against password
        personal_info: Optional list of personal information to check against password

    Returns:
        Tuple of (is_valid, error_message)
    """
    if policy is None:
        policy = PasswordPolicy()

    # Check length
    if len(password) < policy.min_length:
        return False, f"Password must be at least {policy.min_length} characters long"

    if len(password) > policy.max_length:
        return False, f"Password must be at most {policy.max_length} characters long"

    # Check character requirements
    if policy.require_uppercase and not any(c.isupper() for c in password):
        return False, "Password must contain at least one uppercase letter"

    if policy.require_lowercase and not any(c.islower() for c in password):
        return False, "Password must contain at least one lowercase letter"

    if policy.require_digits and not any(c.isdigit() for c in password):
        return False, "Password must contain at least one digit"

    if policy.require_special_chars and not any(c in string.punctuation for c in password):
        return False, "Password must contain at least one special character"

    # Check for common patterns
    if policy.check_repeated_chars and re.search(r'(.)\1{2,}', password):  # Three or more repeated characters
        return False, "Password contains too many repeated characters"

    if policy.check_sequential_chars:
        if re.search(r'(123|234|345|456|567|678|789|987|876|765|654|543|432|321)', password):
            return False, "Password contains sequential numbers"

        if re.search(r'(abc|bcd|cde|def|efg|fgh|ghi|hij|ijk|jkl|klm|lmn|mno|nop|opq|pqr|qrs|rst|stu|tuv|uvw|vwx|wxy|xyz)', password.lower()):
            return False, "Password contains sequential letters"

    # Check for common passwords
    if policy.check_common_passwords and password.lower() in COMMON_PASSWORDS:
        return False, "Password is too common and easily guessable"

    # Check if password contains username
    if policy.check_username_in_password and username and len(username) > 3:
        if username.lower() in password.lower():
            return False, "Password must not contain your username"

    # Check if password contains personal information
    if policy.check_personal_info and personal_info:
        for info in personal_info:
            if info and len(info) > 3 and info.lower() in password.lower():
                return False, "Password must not contain personal information"

    log.debug("Password validation successful")
    return True, None

def generate_password(
    length: int = MIN_PASSWORD_LENGTH,
    include_uppercase: bool = True,
    include_lowercase: bool = True,
    include_digits: bool = True,
    include_special_chars: bool = True
) -> str:
    """
    Generate a secure random password.

    Args:
        length: Length of the password
        include_uppercase: Include uppercase letters
        include_lowercase: Include lowercase letters
        include_digits: Include digits
        include_special_chars: Include special characters

    Returns:
        Secure random password
    """
    # Define character sets
    chars = ""

    if include_uppercase:
        chars += string.ascii_uppercase

    if include_lowercase:
        chars += string.ascii_lowercase

    if include_digits:
        chars += string.digits

    if include_special_chars:
        chars += string.punctuation

    if not chars:
        raise ValueError("At least one character set must be included")

    # Generate password
    password = ""

    # Ensure at least one character from each included set
    if include_uppercase:
        password += secrets.choice(string.ascii_uppercase)

    if include_lowercase:
        password += secrets.choice(string.ascii_lowercase)

    if include_digits:
        password += secrets.choice(string.digits)

    if include_special_chars:
        password += secrets.choice(string.punctuation)

    # Fill the rest with random characters
    remaining_length = length - len(password)

    if remaining_length > 0:
        password += "".join(secrets.choice(chars) for _ in range(remaining_length))

    # Shuffle the password to avoid predictable patterns
    password_list = list(password)
    secrets.SystemRandom().shuffle(password_list)

    return "".join(password_list)
