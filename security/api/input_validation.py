"""
Input validation for Bulo.Cloud Sentinel Security Module.

This module provides input validation utilities to prevent injection attacks.
"""

from security.validation.unified_validation import (
    validate_email,
    validate_username,
    validate_name,
    validate_uuid,
    validate_url,
    sanitize_string,
    sanitize_html,
    check_sql_injection,
    input_validator,
    form_validator,
    request_validator,
)
from typing import Any, Dict, List, Optional, Union, Callable

from fastapi import HTTPException, status
from pydantic import BaseModel, Field, validator

# Regular expressions for validation
EMAIL_REGEX = r'^[a-zA-Z0-9._%+-]+@[a-zA-Z0-9.-]+\.[a-zA-Z]{2,}$'
USERNAME_REGEX = r'^[a-zA-Z0-9_-]{3,32}$'
NAME_REGEX = r'^[a-zA-Z0-9\s\-\'\.]{1,64}$'
URL_REGEX = r'^(https?:\/\/)?([\da-z\.-]+)\.([a-z\.]{2,6})([\/\w \.-]*)*\/?$'
PHONE_REGEX = r'^\+?[0-9]{10,15}$'
ALPHANUMERIC_REGEX = r'^[a-zA-Z0-9]+$'
NUMERIC_REGEX = r'^[0-9]+$'
ALPHA_REGEX = r'^[a-zA-Z]+$'

class ValidationError(HTTPException):
    """Exception raised for validation errors."""
    def __init__(self, detail: str):
        super().__init__(
            status_code=status.HTTP_422_UNPROCESSABLE_ENTITY,
            detail=detail
        )

def validate_email(email: str) -> bool:
    """
    Validate an email address.
    
    Args:
        email: Email address to validate
        
    Returns:
        True if valid, False otherwise
    """
    return bool(re.match(EMAIL_REGEX, email))

def validate_username(username: str) -> bool:
    """
    Validate a username.
    
    Args:
        username: Username to validate
        
    Returns:
        True if valid, False otherwise
    """
    return bool(re.match(USERNAME_REGEX, username))

def validate_name(name: str) -> bool:
    """
    Validate a name.
    
    Args:
        name: Name to validate
        
    Returns:
        True if valid, False otherwise
    """
    return bool(re.match(NAME_REGEX, name))

def validate_url(url: str) -> bool:
    """
    Validate a URL.
    
    Args:
        url: URL to validate
        
    Returns:
        True if valid, False otherwise
    """
    return bool(re.match(URL_REGEX, url))

def validate_phone(phone: str) -> bool:
    """
    Validate a phone number.
    
    Args:
        phone: Phone number to validate
        
    Returns:
        True if valid, False otherwise
    """
    return bool(re.match(PHONE_REGEX, phone))

def validate_alphanumeric(text: str) -> bool:
    """
    Validate alphanumeric text.
    
    Args:
        text: Text to validate
        
    Returns:
        True if valid, False otherwise
    """
    return bool(re.match(ALPHANUMERIC_REGEX, text))

def validate_numeric(text: str) -> bool:
    """
    Validate numeric text.
    
    Args:
        text: Text to validate
        
    Returns:
        True if valid, False otherwise
    """
    return bool(re.match(NUMERIC_REGEX, text))

def validate_alpha(text: str) -> bool:
    """
    Validate alphabetic text.
    
    Args:
        text: Text to validate
        
    Returns:
        True if valid, False otherwise
    """
    return bool(re.match(ALPHA_REGEX, text))

def sanitize_html(text: str) -> str:
    """
    Sanitize HTML to prevent XSS attacks.
    
    Args:
        text: Text to sanitize
        
    Returns:
        Sanitized text
    """
    return html.escape(text)

def validate_and_sanitize(
    text: str,
    validator_func: Optional[Callable[[str], bool]] = None,
    sanitize: bool = True,
    error_message: Optional[str] = None
) -> str:
    """
    Validate and sanitize text.
    
    Args:
        text: Text to validate and sanitize
        validator_func: Optional validator function
        sanitize: Whether to sanitize the text
        error_message: Optional error message
        
    Returns:
        Validated and sanitized text
        
    Raises:
        ValidationError: If validation fails
    """
    # Validate
    if validator_func and not validator_func(text):
        raise ValidationError(error_message or "Invalid input")
    
    # Sanitize
    if sanitize:
        return sanitize_html(text)
    
    return text

# SQL injection prevention
def is_sql_injection(text: str) -> bool:
    """
    Check if text contains SQL injection attempts.
    
    Args:
        text: Text to check
        
    Returns:
        True if SQL injection detected, False otherwise
    """
    # Common SQL injection patterns
    sql_patterns = [
        r'(\s|^)(SELECT|INSERT|UPDATE|DELETE|DROP|ALTER|CREATE|TRUNCATE)(\s|$)',
        r'(\s|^)(UNION|JOIN|AND|OR)(\s|$)',
        r'--',
        r';',
        r'\/\*.*\*\/',
        r'1=1',
        r'1\s*=\s*1',
        r'\'.*\'',
        r'".*"'
    ]
    
    # Check each pattern
    for pattern in sql_patterns:
        if re.search(pattern, text, re.IGNORECASE):
            return True
    
    return False

def prevent_sql_injection(text: str, error_message: Optional[str] = None) -> str:
    """
    Prevent SQL injection.
    
    Args:
        text: Text to check
        error_message: Optional error message
        
    Returns:
        Original text if no SQL injection detected
        
    Raises:
        ValidationError: If SQL injection detected
    """
    if is_sql_injection(text):
        raise ValidationError(error_message or "Potential SQL injection detected")
    
    return text

# NoSQL injection prevention
def is_nosql_injection(text: str) -> bool:
    """
    Check if text contains NoSQL injection attempts.
    
    Args:
        text: Text to check
        
    Returns:
        True if NoSQL injection detected, False otherwise
    """
    # Common NoSQL injection patterns
    nosql_patterns = [
        r'\$where',
        r'\$ne',
        r'\$gt',
        r'\$lt',
        r'\$exists',
        r'\$regex',
        r'{\s*\$',
        r'\$or',
        r'\$and'
    ]
    
    # Check each pattern
    for pattern in nosql_patterns:
        if re.search(pattern, text):
            return True
    
    return False

def prevent_nosql_injection(text: str, error_message: Optional[str] = None) -> str:
    """
    Prevent NoSQL injection.
    
    Args:
        text: Text to check
        error_message: Optional error message
        
    Returns:
        Original text if no NoSQL injection detected
        
    Raises:
        ValidationError: If NoSQL injection detected
    """
    if is_nosql_injection(text):
        raise ValidationError(error_message or "Potential NoSQL injection detected")
    
    return text

# Command injection prevention
def is_command_injection(text: str) -> bool:
    """
    Check if text contains command injection attempts.
    
    Args:
        text: Text to check
        
    Returns:
        True if command injection detected, False otherwise
    """
    # Common command injection patterns
    command_patterns = [
        r'(\s|^)(cat|ls|dir|rm|cp|mv|chmod|chown|touch|echo|bash|sh|python|perl|ruby|php)(\s|$)',
        r';',
        r'\|',
        r'`',
        r'\$\(',
        r'&',
        r'>',
        r'<'
    ]
    
    # Check each pattern
    for pattern in command_patterns:
        if re.search(pattern, text):
            return True
    
    return False

def prevent_command_injection(text: str, error_message: Optional[str] = None) -> str:
    """
    Prevent command injection.
    
    Args:
        text: Text to check
        error_message: Optional error message
        
    Returns:
        Original text if no command injection detected
        
    Raises:
        ValidationError: If command injection detected
    """
    if is_command_injection(text):
        raise ValidationError(error_message or "Potential command injection detected")
    
    return text
