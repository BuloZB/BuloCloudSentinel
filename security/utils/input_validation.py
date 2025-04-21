"""
Input validation utilities.

This module provides functions for validating user input to prevent security vulnerabilities.
"""

import re
from typing import Any, Dict, List, Optional, Pattern, Union

# Regular expressions for common input validation
EMAIL_REGEX = re.compile(r'^[a-zA-Z0-9._%+-]+@[a-zA-Z0-9.-]+\.[a-zA-Z]{2,}$')
USERNAME_REGEX = re.compile(r'^[a-zA-Z0-9_-]{3,32}$')
PASSWORD_REGEX = re.compile(r'^(?=.*[a-z])(?=.*[A-Z])(?=.*\d)(?=.*[@$!%*?&])[A-Za-z\d@$!%*?&]{8,}$')
URL_REGEX = re.compile(r'^(https?|rtsp|rtmp)://[^\s/$.?#].[^\s]*$')
FILENAME_REGEX = re.compile(r'^[a-zA-Z0-9_.-]{1,255}$')
UUID_REGEX = re.compile(r'^[0-9a-f]{8}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{12}$')
LATITUDE_REGEX = re.compile(r'^[-+]?([1-8]?\d(\.\d+)?|90(\.0+)?)$')
LONGITUDE_REGEX = re.compile(r'^[-+]?(180(\.0+)?|((1[0-7]\d)|([1-9]?\d))(\.\d+)?)$')

def is_valid_email(email: str) -> bool:
    """
    Validate an email address.
    
    Args:
        email: The email address to validate
        
    Returns:
        True if the email is valid, False otherwise
    """
    if not email or len(email) > 320:  # Max email length
        return False
    return bool(EMAIL_REGEX.match(email))

def is_valid_username(username: str) -> bool:
    """
    Validate a username.
    
    Args:
        username: The username to validate
        
    Returns:
        True if the username is valid, False otherwise
    """
    if not username:
        return False
    return bool(USERNAME_REGEX.match(username))

def is_valid_password(password: str) -> bool:
    """
    Validate a password.
    
    Requires at least:
    - 8 characters
    - 1 uppercase letter
    - 1 lowercase letter
    - 1 digit
    - 1 special character
    
    Args:
        password: The password to validate
        
    Returns:
        True if the password is valid, False otherwise
    """
    if not password:
        return False
    return bool(PASSWORD_REGEX.match(password))

def is_valid_url(url: str, allowed_schemes: Optional[List[str]] = None) -> bool:
    """
    Validate a URL.
    
    Args:
        url: The URL to validate
        allowed_schemes: List of allowed URL schemes (e.g., ['http', 'https'])
        
    Returns:
        True if the URL is valid, False otherwise
    """
    if not url:
        return False
        
    if not URL_REGEX.match(url):
        return False
        
    if allowed_schemes:
        scheme = url.split('://')[0].lower()
        if scheme not in allowed_schemes:
            return False
            
    return True

def is_valid_filename(filename: str) -> bool:
    """
    Validate a filename.
    
    Args:
        filename: The filename to validate
        
    Returns:
        True if the filename is valid, False otherwise
    """
    if not filename:
        return False
    return bool(FILENAME_REGEX.match(filename))

def is_valid_uuid(uuid: str) -> bool:
    """
    Validate a UUID.
    
    Args:
        uuid: The UUID to validate
        
    Returns:
        True if the UUID is valid, False otherwise
    """
    if not uuid:
        return False
    return bool(UUID_REGEX.match(uuid.lower()))

def is_valid_latitude(latitude: Union[str, float]) -> bool:
    """
    Validate a latitude value.
    
    Args:
        latitude: The latitude to validate
        
    Returns:
        True if the latitude is valid, False otherwise
    """
    if isinstance(latitude, float):
        return -90 <= latitude <= 90
    elif isinstance(latitude, str):
        return bool(LATITUDE_REGEX.match(latitude))
    return False

def is_valid_longitude(longitude: Union[str, float]) -> bool:
    """
    Validate a longitude value.
    
    Args:
        longitude: The longitude to validate
        
    Returns:
        True if the longitude is valid, False otherwise
    """
    if isinstance(longitude, float):
        return -180 <= longitude <= 180
    elif isinstance(longitude, str):
        return bool(LONGITUDE_REGEX.match(longitude))
    return False

def sanitize_html(html: str) -> str:
    """
    Sanitize HTML to prevent XSS attacks.
    
    Args:
        html: The HTML to sanitize
        
    Returns:
        Sanitized HTML
    """
    import html as html_module
    return html_module.escape(html)

def validate_json_structure(
    json_data: Dict[str, Any],
    required_fields: List[str],
    field_types: Dict[str, type],
    max_depth: int = 5,
    current_depth: int = 0
) -> List[str]:
    """
    Validate the structure of a JSON object.
    
    Args:
        json_data: The JSON data to validate
        required_fields: List of required field names
        field_types: Dictionary mapping field names to expected types
        max_depth: Maximum allowed depth for nested objects
        current_depth: Current depth in the object hierarchy
        
    Returns:
        List of validation errors, empty if valid
    """
    errors = []
    
    # Check depth
    if current_depth > max_depth:
        errors.append("JSON structure exceeds maximum allowed depth")
        return errors
        
    # Check required fields
    for field in required_fields:
        if field not in json_data:
            errors.append(f"Required field '{field}' is missing")
            
    # Check field types
    for field, expected_type in field_types.items():
        if field in json_data:
            value = json_data[field]
            
            # Handle None values
            if value is None and expected_type is not type(None):
                errors.append(f"Field '{field}' is None but should be {expected_type.__name__}")
                continue
                
            # Check type
            if not isinstance(value, expected_type):
                errors.append(f"Field '{field}' has type {type(value).__name__} but should be {expected_type.__name__}")
                
            # Recursively validate nested dictionaries
            if expected_type is dict and isinstance(value, dict):
                nested_errors = validate_json_structure(
                    value,
                    [],  # No required fields for nested objects
                    {},  # No type checking for nested objects
                    max_depth,
                    current_depth + 1
                )
                errors.extend(nested_errors)
                
    return errors
