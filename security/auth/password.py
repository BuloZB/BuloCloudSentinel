"""
Password handling for Bulo.Cloud Sentinel Security Module.

This module provides secure password hashing, verification, and policy enforcement.
"""

import re
import secrets
import string
from typing import Tuple, Optional

from argon2 import PasswordHasher
from argon2.exceptions import VerifyMismatchError
from pydantic import BaseModel, Field, validator

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
        return True
    except VerifyMismatchError:
        return False

def check_password_needs_rehash(hashed_password: str) -> bool:
    """
    Check if a password hash needs to be rehashed.
    
    Args:
        hashed_password: Hashed password
        
    Returns:
        True if password needs rehashing, False otherwise
    """
    return ph.check_needs_rehash(hashed_password)

def validate_password(
    password: str,
    policy: Optional[PasswordPolicy] = None
) -> Tuple[bool, Optional[str]]:
    """
    Validate a password against the password policy.
    
    Args:
        password: Password to validate
        policy: Optional custom password policy
        
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
    if re.search(r'(.)\1{2,}', password):  # Three or more repeated characters
        return False, "Password contains too many repeated characters"
    
    if re.search(r'(123|234|345|456|567|678|789|987|876|765|654|543|432|321)', password):
        return False, "Password contains sequential numbers"
    
    if re.search(r'(abc|bcd|cde|def|efg|fgh|ghi|hij|ijk|jkl|klm|lmn|mno|nop|opq|pqr|qrs|rst|stu|tuv|uvw|vwx|wxy|xyz)', password.lower()):
        return False, "Password contains sequential letters"
    
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
