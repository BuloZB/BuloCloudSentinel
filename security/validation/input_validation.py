"""
Input validation utilities for Bulo.Cloud Sentinel.

This module provides functions for validating and sanitizing user inputs
to prevent injection attacks and other security issues.
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
import unicodedata
from typing import Any, Dict, List, Optional, Union, Pattern
from fastapi import HTTPException, status


# Regular expressions for validation
EMAIL_REGEX = re.compile(r"^[a-zA-Z0-9._%+-]+@[a-zA-Z0-9.-]+\.[a-zA-Z]{2,}$")
USERNAME_REGEX = re.compile(r"^[a-zA-Z0-9_-]{3,32}$")
NAME_REGEX = re.compile(r"^[a-zA-Z0-9\s_-]{1,64}$")
UUID_REGEX = re.compile(r"^[0-9a-f]{8}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{12}$")
LATITUDE_REGEX = re.compile(r"^-?([0-8]?[0-9]|90)(\.[0-9]{1,15})?$")
LONGITUDE_REGEX = re.compile(r"^-?((1[0-7][0-9])|([0-9]?[0-9]))(\.[0-9]{1,15})?$")
ALTITUDE_REGEX = re.compile(r"^-?[0-9]+(\.[0-9]+)?$")
PHONE_REGEX = re.compile(r"^\+?[0-9]{7,15}$")
URL_REGEX = re.compile(r"^https?://(?:[-\w.]|(?:%[\da-fA-F]{2}))+(/[-\w%!$&'()*+,;=:~]+)*(?:\?[-\w%!$&'()*+,;=:~./?]*)?(?:#[-\w%!$&'()*+,;=:~./?]*)?$")
FILENAME_REGEX = re.compile(r"^[a-zA-Z0-9_\-. ]+$")
PATH_REGEX = re.compile(r"^[a-zA-Z0-9_\-./]+$")
SQL_INJECTION_REGEX = re.compile(r"(?i)(SELECT|INSERT|UPDATE|DELETE|DROP|ALTER|UNION|INTO|EXEC|EXECUTE)")

# Dangerous characters that should be escaped or removed
DANGEROUS_CHARS = re.compile(r'[<>"\';]')


def validate_email(email: str) -> bool:
    """
    Validate an email address.
    
    Args:
        email: The email address to validate
        
    Returns:
        True if the email is valid, False otherwise
    """
    if not email or not isinstance(email, str):
        return False
    
    # Check length
    if len(email) > 320:  # RFC 5321 limit
        return False
    
    # Check format
    return bool(EMAIL_REGEX.match(email))


def validate_username(username: str) -> bool:
    """
    Validate a username.
    
    Args:
        username: The username to validate
        
    Returns:
        True if the username is valid, False otherwise
    """
    if not username or not isinstance(username, str):
        return False
    
    return bool(USERNAME_REGEX.match(username))


def validate_name(name: str) -> bool:
    """
    Validate a name (person, organization, etc.).
    
    Args:
        name: The name to validate
        
    Returns:
        True if the name is valid, False otherwise
    """
    if not name or not isinstance(name, str):
        return False
    
    return bool(NAME_REGEX.match(name))


def validate_uuid(uuid_str: str) -> bool:
    """
    Validate a UUID string.
    
    Args:
        uuid_str: The UUID string to validate
        
    Returns:
        True if the UUID is valid, False otherwise
    """
    if not uuid_str or not isinstance(uuid_str, str):
        return False
    
    return bool(UUID_REGEX.match(uuid_str))


def validate_coordinates(latitude: float, longitude: float, altitude: Optional[float] = None) -> bool:
    """
    Validate geographic coordinates.
    
    Args:
        latitude: The latitude to validate (-90 to 90)
        longitude: The longitude to validate (-180 to 180)
        altitude: The altitude to validate (optional)
        
    Returns:
        True if the coordinates are valid, False otherwise
    """
    if not isinstance(latitude, (int, float)) or not isinstance(longitude, (int, float)):
        return False
    
    if latitude < -90 or latitude > 90:
        return False
    
    if longitude < -180 or longitude > 180:
        return False
    
    if altitude is not None and not isinstance(altitude, (int, float)):
        return False
    
    return True


def validate_phone(phone: str) -> bool:
    """
    Validate a phone number.
    
    Args:
        phone: The phone number to validate
        
    Returns:
        True if the phone number is valid, False otherwise
    """
    if not phone or not isinstance(phone, str):
        return False
    
    return bool(PHONE_REGEX.match(phone))


def validate_url(url: str) -> bool:
    """
    Validate a URL.
    
    Args:
        url: The URL to validate
        
    Returns:
        True if the URL is valid, False otherwise
    """
    if not url or not isinstance(url, str):
        return False
    
    # Check length
    if len(url) > 2048:  # Common browser limit
        return False
    
    # Check format
    return bool(URL_REGEX.match(url))


def validate_filename(filename: str) -> bool:
    """
    Validate a filename.
    
    Args:
        filename: The filename to validate
        
    Returns:
        True if the filename is valid, False otherwise
    """
    if not filename or not isinstance(filename, str):
        return False
    
    # Check length
    if len(filename) > 255:  # Common filesystem limit
        return False
    
    # Check format
    return bool(FILENAME_REGEX.match(filename))


def validate_path(path: str) -> bool:
    """
    Validate a file path.
    
    Args:
        path: The file path to validate
        
    Returns:
        True if the path is valid, False otherwise
    """
    if not path or not isinstance(path, str):
        return False
    
    # Check length
    if len(path) > 4096:  # Common filesystem limit
        return False
    
    # Check for path traversal attempts
    if ".." in path:
        return False
    
    # Check format
    return bool(PATH_REGEX.match(path))


def sanitize_string(input_str: str) -> str:
    """
    Sanitize a string by removing dangerous characters.
    
    Args:
        input_str: The string to sanitize
        
    Returns:
        The sanitized string
    """
    if not input_str or not isinstance(input_str, str):
        return ""
    
    # Normalize Unicode to avoid evasion techniques
    normalized = unicodedata.normalize("NFKC", input_str)
    
    # HTML escape special characters
    return html.escape(normalized)


def sanitize_html(html_str: str) -> str:
    """
    Sanitize HTML content.
    
    Args:
        html_str: The HTML string to sanitize
        
    Returns:
        The sanitized HTML string
    """
    if not html_str or not isinstance(html_str, str):
        return ""
    
    # This is a simple implementation - in production, use a proper HTML sanitizer library
    # like bleach or html-sanitizer
    return html.escape(html_str)


def check_sql_injection(input_str: str) -> bool:
    """
    Check if a string contains potential SQL injection patterns.
    
    Args:
        input_str: The string to check
        
    Returns:
        True if SQL injection is detected, False otherwise
    """
    if not input_str or not isinstance(input_str, str):
        return False
    
    return bool(SQL_INJECTION_REGEX.search(input_str))


def validate_input(
    input_value: Any, 
    input_type: str, 
    required: bool = True, 
    min_length: Optional[int] = None,
    max_length: Optional[int] = None,
    pattern: Optional[Pattern] = None,
    custom_validator: Optional[callable] = None
) -> Any:
    """
    Validate an input value based on specified criteria.
    
    Args:
        input_value: The value to validate
        input_type: The expected type ('string', 'int', 'float', 'bool', 'email', etc.)
        required: Whether the input is required
        min_length: Minimum length for strings or lists
        max_length: Maximum length for strings or lists
        pattern: Regular expression pattern for string validation
        custom_validator: Custom validation function
        
    Returns:
        The validated input value
        
    Raises:
        HTTPException: If validation fails
    """
    # Check if required
    if required and (input_value is None or (isinstance(input_value, str) and input_value.strip() == "")):
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=f"Input is required"
        )
    
    # If not required and empty, return None
    if not required and (input_value is None or (isinstance(input_value, str) and input_value.strip() == "")):
        return None
    
    # Type validation
    if input_type == "string":
        if not isinstance(input_value, str):
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail=f"Input must be a string"
            )
        
        # Length validation
        if min_length is not None and len(input_value) < min_length:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail=f"Input must be at least {min_length} characters long"
            )
        
        if max_length is not None and len(input_value) > max_length:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail=f"Input must be at most {max_length} characters long"
            )
        
        # Pattern validation
        if pattern and not pattern.match(input_value):
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail=f"Input format is invalid"
            )
    
    elif input_type == "int":
        try:
            input_value = int(input_value)
        except (ValueError, TypeError):
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail=f"Input must be an integer"
            )
    
    elif input_type == "float":
        try:
            input_value = float(input_value)
        except (ValueError, TypeError):
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail=f"Input must be a number"
            )
    
    elif input_type == "bool":
        if isinstance(input_value, str):
            input_value = input_value.lower()
            if input_value in ("true", "1", "yes", "y"):
                input_value = True
            elif input_value in ("false", "0", "no", "n"):
                input_value = False
            else:
                raise HTTPException(
                    status_code=status.HTTP_400_BAD_REQUEST,
                    detail=f"Input must be a boolean"
                )
        elif not isinstance(input_value, bool):
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail=f"Input must be a boolean"
            )
    
    elif input_type == "email":
        if not validate_email(input_value):
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail=f"Input must be a valid email address"
            )
    
    elif input_type == "uuid":
        if not validate_uuid(input_value):
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail=f"Input must be a valid UUID"
            )
    
    elif input_type == "url":
        if not validate_url(input_value):
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail=f"Input must be a valid URL"
            )
    
    elif input_type == "list":
        if not isinstance(input_value, list):
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail=f"Input must be a list"
            )
        
        # Length validation
        if min_length is not None and len(input_value) < min_length:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail=f"List must have at least {min_length} items"
            )
        
        if max_length is not None and len(input_value) > max_length:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail=f"List must have at most {max_length} items"
            )
    
    # Custom validation
    if custom_validator and callable(custom_validator):
        try:
            result = custom_validator(input_value)
            if result is not True:
                error_message = result if isinstance(result, str) else "Custom validation failed"
                raise HTTPException(
                    status_code=status.HTTP_400_BAD_REQUEST,
                    detail=error_message
                )
        except Exception as e:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail=str(e)
            )
    
    return input_value
