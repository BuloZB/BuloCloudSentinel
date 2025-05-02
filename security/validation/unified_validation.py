"""
Unified input validation module for Bulo.Cloud Sentinel.

This module provides a standardized approach to input validation across all components
of the Bulo.Cloud Sentinel platform, using secure validation and sanitization techniques.
"""

import re
import html
import unicodedata
import uuid
import logging
from typing import Any, Dict, List, Optional, Pattern, Set, Union, Callable
from datetime import datetime, date
from fastapi import HTTPException, status, Request
from pydantic import BaseModel, Field, validator

# Set up logging
logger = logging.getLogger(__name__)

# Regular expressions for validation
EMAIL_REGEX = re.compile(r'^[a-zA-Z0-9._%+-]+@[a-zA-Z0-9.-]+\.[a-zA-Z]{2,}$')
USERNAME_REGEX = re.compile(r'^[a-zA-Z0-9_-]{3,32}$')
NAME_REGEX = re.compile(r'^[a-zA-Z0-9\s\-\'\.]{1,64}$')
UUID_REGEX = re.compile(r'^[0-9a-f]{8}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{12}$')
URL_REGEX = re.compile(r'^(https?:\/\/)?([\da-z\.-]+)\.([a-z\.]{2,6})([\/\w \.-]*)*\/?$')
PHONE_REGEX = re.compile(r'^\+?[0-9]{10,15}$')
COORDINATES_REGEX = re.compile(r'^-?\d+(\.\d+)?,-?\d+(\.\d+)?$')
LATITUDE_REGEX = re.compile(r'^-?([0-8]?[0-9]|90)(\.[0-9]{1,10})?$')
LONGITUDE_REGEX = re.compile(r'^-?((1[0-7][0-9])|([0-9]?[0-9]))(\.[0-9]{1,10})?$')
FILENAME_REGEX = re.compile(r'^[a-zA-Z0-9_\-. ]+$')
PATH_REGEX = re.compile(r'^[a-zA-Z0-9_\-./]+$')
SQL_INJECTION_REGEX = re.compile(r'(?i)(SELECT|INSERT|UPDATE|DELETE|DROP|ALTER|UNION|INTO|EXEC|EXECUTE)')

# HTML validation patterns
TAG_REGEX = re.compile(r'<(/?)([a-zA-Z][a-zA-Z0-9]*)((?:\s+[a-zA-Z][a-zA-Z0-9]*(?:\s*=\s*(?:"[^"]*"|\'[^\']*\'|[^\s>]*))?)*)\s*(/?)>')
ATTR_REGEX = re.compile(r'([a-zA-Z][a-zA-Z0-9]*)\s*(?:=\s*(?:"([^"]*)"|\'([^\']*)\'|([^\s>]*)))?')
SCRIPT_REGEX = re.compile(r'<script\b[^>]*>(.*?)</script>', re.IGNORECASE | re.DOTALL)
STYLE_REGEX = re.compile(r'<style\b[^>]*>(.*?)</style>', re.IGNORECASE | re.DOTALL)
COMMENT_REGEX = re.compile(r'<!--(.*?)-->', re.DOTALL)
EVENT_HANDLER_REGEX = re.compile(r'\bon[a-z]+\s*=', re.IGNORECASE)
JAVASCRIPT_URL_REGEX = re.compile(r'(?i)javascript:')
DATA_ATTR_REGEX = re.compile(r'\bdata-[a-z0-9_-]+\s*=', re.IGNORECASE)

# Dangerous characters that should be escaped or removed
DANGEROUS_CHARS = re.compile(r'[<>"\';]')

class ValidationError(Exception):
    """Exception raised for validation errors."""
    def __init__(self, message: str, field: Optional[str] = None):
        self.message = message
        self.field = field
        super().__init__(message)

# Basic validation functions
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
    Validate a name.
    
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
    
    try:
        uuid_obj = uuid.UUID(uuid_str)
        return str(uuid_obj) == uuid_str
    except ValueError:
        return False

def validate_coordinates(coordinates: str) -> bool:
    """
    Validate coordinates in the format "latitude,longitude".
    
    Args:
        coordinates: The coordinates to validate
        
    Returns:
        True if the coordinates are valid, False otherwise
    """
    if not coordinates or not isinstance(coordinates, str):
        return False
    
    return bool(COORDINATES_REGEX.match(coordinates))

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
    
    return bool(PATH_REGEX.match(path))

# Sanitization functions
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
    
    # Remove control characters
    input_str = ''.join(ch for ch in input_str if unicodedata.category(ch)[0] != 'C')
    
    # Replace dangerous characters
    return DANGEROUS_CHARS.sub('', input_str)

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
    
    return html.escape(html_str)

def strip_all_tags(html_content: str) -> str:
    """
    Strip all HTML tags from content.
    
    Args:
        html_content: The HTML content to strip
        
    Returns:
        The content with all HTML tags removed
    """
    if not html_content or not isinstance(html_content, str):
        return ""
    
    # Remove comments
    content = COMMENT_REGEX.sub('', html_content)
    
    # Remove script and style tags with their content
    content = SCRIPT_REGEX.sub('', content)
    content = STYLE_REGEX.sub('', content)
    
    # Remove all tags
    content = TAG_REGEX.sub('', content)
    
    return content

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

def detect_xss(content: str) -> bool:
    """
    Detect potential XSS attacks in content.
    
    Args:
        content: The content to check
        
    Returns:
        True if potential XSS is detected, False otherwise
    """
    if not content or not isinstance(content, str):
        return False
    
    # Check for script tags
    if SCRIPT_REGEX.search(content):
        return True
    
    # Check for event handlers
    if EVENT_HANDLER_REGEX.search(content):
        return True
    
    # Check for javascript: URLs
    if JAVASCRIPT_URL_REGEX.search(content):
        return True
    
    # Check for data attributes (potential for XSS in some contexts)
    if DATA_ATTR_REGEX.search(content):
        return True
    
    return False

# Advanced validation
class InputValidator:
    """
    Validator for input values.
    """
    
    def validate_input(
        self,
        value: Any,
        validation_type: str,
        required: bool = True,
        min_length: Optional[int] = None,
        max_length: Optional[int] = None,
        min_value: Optional[Union[int, float]] = None,
        max_value: Optional[Union[int, float]] = None,
        pattern: Optional[Union[str, Pattern]] = None,
        allowed_values: Optional[List[Any]] = None,
        custom_validator: Optional[Callable] = None,
        field_name: Optional[str] = None
    ) -> Any:
        """
        Validate an input value based on specified criteria.
        
        Args:
            value: The value to validate
            validation_type: Type of validation to perform
            required: Whether the value is required
            min_length: Minimum length for strings or lists
            max_length: Maximum length for strings or lists
            min_value: Minimum value for numbers
            max_value: Maximum value for numbers
            pattern: Regular expression pattern for string validation
            allowed_values: List of allowed values
            custom_validator: Custom validation function
            field_name: Name of the field being validated
            
        Returns:
            The validated value
            
        Raises:
            ValidationError: If validation fails
        """
        # Check if required
        if value is None or (isinstance(value, str) and value.strip() == ""):
            if required:
                raise ValidationError("This field is required", field_name)
            return value
        
        # Type-specific validation
        if validation_type == "string":
            if not isinstance(value, str):
                raise ValidationError("Value must be a string", field_name)
            
            # Check length
            if min_length is not None and len(value) < min_length:
                raise ValidationError(f"Value must be at least {min_length} characters", field_name)
            if max_length is not None and len(value) > max_length:
                raise ValidationError(f"Value must be at most {max_length} characters", field_name)
            
            # Check pattern
            if pattern:
                if isinstance(pattern, str):
                    pattern = re.compile(pattern)
                if not pattern.match(value):
                    raise ValidationError("Value does not match the required pattern", field_name)
        
        elif validation_type == "integer":
            try:
                value = int(value)
            except (ValueError, TypeError):
                raise ValidationError("Value must be an integer", field_name)
            
            # Check range
            if min_value is not None and value < min_value:
                raise ValidationError(f"Value must be at least {min_value}", field_name)
            if max_value is not None and value > max_value:
                raise ValidationError(f"Value must be at most {max_value}", field_name)
        
        elif validation_type == "float":
            try:
                value = float(value)
            except (ValueError, TypeError):
                raise ValidationError("Value must be a number", field_name)
            
            # Check range
            if min_value is not None and value < min_value:
                raise ValidationError(f"Value must be at least {min_value}", field_name)
            if max_value is not None and value > max_value:
                raise ValidationError(f"Value must be at most {max_value}", field_name)
        
        elif validation_type == "boolean":
            if isinstance(value, str):
                value = value.lower()
                if value in ("true", "1", "yes", "y"):
                    value = True
                elif value in ("false", "0", "no", "n"):
                    value = False
                else:
                    raise ValidationError("Value must be a boolean", field_name)
            elif not isinstance(value, bool):
                raise ValidationError("Value must be a boolean", field_name)
        
        elif validation_type == "email":
            if not validate_email(value):
                raise ValidationError("Value must be a valid email address", field_name)
        
        elif validation_type == "url":
            if not validate_url(value):
                raise ValidationError("Value must be a valid URL", field_name)
        
        elif validation_type == "uuid":
            if not validate_uuid(value):
                raise ValidationError("Value must be a valid UUID", field_name)
        
        elif validation_type == "date":
            try:
                if isinstance(value, str):
                    value = datetime.fromisoformat(value).date()
                elif not isinstance(value, date):
                    raise ValidationError("Value must be a valid date", field_name)
            except ValueError:
                raise ValidationError("Value must be a valid date in ISO format", field_name)
        
        elif validation_type == "datetime":
            try:
                if isinstance(value, str):
                    value = datetime.fromisoformat(value)
                elif not isinstance(value, datetime):
                    raise ValidationError("Value must be a valid datetime", field_name)
            except ValueError:
                raise ValidationError("Value must be a valid datetime in ISO format", field_name)
        
        elif validation_type == "list":
            if not isinstance(value, list):
                raise ValidationError("Value must be a list", field_name)
            
            # Check length
            if min_length is not None and len(value) < min_length:
                raise ValidationError(f"List must have at least {min_length} items", field_name)
            if max_length is not None and len(value) > max_length:
                raise ValidationError(f"List must have at most {max_length} items", field_name)
        
        # Check allowed values
        if allowed_values is not None and value not in allowed_values:
            raise ValidationError(f"Value must be one of: {', '.join(map(str, allowed_values))}", field_name)
        
        # Custom validation
        if custom_validator and callable(custom_validator):
            try:
                result = custom_validator(value)
                if result is not True:
                    error_message = result if isinstance(result, str) else "Custom validation failed"
                    raise ValidationError(error_message, field_name)
            except Exception as e:
                if isinstance(e, ValidationError):
                    raise
                raise ValidationError(str(e), field_name)
        
        return value

class FormValidator:
    """
    Validator for form data.
    """
    
    def __init__(self):
        """
        Initialize the validator.
        """
        self.validator = InputValidator()
    
    def validate_form(
        self,
        form_data: Dict[str, Any],
        validation_rules: Dict[str, Dict[str, Any]]
    ) -> Dict[str, Any]:
        """
        Validate form data based on validation rules.
        
        Args:
            form_data: Form data to validate
            validation_rules: Validation rules for each field
            
        Returns:
            Validated form data
            
        Raises:
            HTTPException: If validation fails
        """
        validated_data = {}
        errors = []
        
        for field_name, rules in validation_rules.items():
            value = form_data.get(field_name)
            
            try:
                validated_value = self.validator.validate_input(
                    value=value,
                    validation_type=rules.get("type", "string"),
                    required=rules.get("required", True),
                    min_length=rules.get("min_length"),
                    max_length=rules.get("max_length"),
                    min_value=rules.get("min_value"),
                    max_value=rules.get("max_value"),
                    pattern=rules.get("pattern"),
                    allowed_values=rules.get("allowed_values"),
                    custom_validator=rules.get("custom_validator"),
                    field_name=field_name
                )
                
                validated_data[field_name] = validated_value
            
            except ValidationError as e:
                errors.append({
                    "field": e.field or field_name,
                    "message": e.message
                })
        
        if errors:
            raise HTTPException(
                status_code=status.HTTP_422_UNPROCESSABLE_ENTITY,
                detail={
                    "message": "Validation error",
                    "errors": errors
                }
            )
        
        return validated_data

class RequestValidator:
    """
    Validator for API requests.
    """
    
    def __init__(self):
        """
        Initialize the validator.
        """
        self.form_validator = FormValidator()
    
    def validate_request(
        self,
        request_data: Dict[str, Any],
        validation_schema: Dict[str, Dict[str, Any]]
    ) -> Dict[str, Any]:
        """
        Validate request data based on a validation schema.
        
        Args:
            request_data: Request data to validate
            validation_schema: Validation schema for the request
            
        Returns:
            Validated request data
            
        Raises:
            HTTPException: If validation fails
        """
        return self.form_validator.validate_form(request_data, validation_schema)
    
    def validate_query_params(
        self,
        query_params: Dict[str, Any],
        validation_schema: Dict[str, Dict[str, Any]]
    ) -> Dict[str, Any]:
        """
        Validate query parameters based on a validation schema.
        
        Args:
            query_params: Query parameters to validate
            validation_schema: Validation schema for the query parameters
            
        Returns:
            Validated query parameters
            
        Raises:
            HTTPException: If validation fails
        """
        return self.form_validator.validate_form(query_params, validation_schema)
    
    async def validate_request_body(
        self,
        request: Request,
        validation_schema: Dict[str, Dict[str, Any]]
    ) -> Dict[str, Any]:
        """
        Validate request body based on a validation schema.
        
        Args:
            request: FastAPI request object
            validation_schema: Validation schema for the request body
            
        Returns:
            Validated request body
            
        Raises:
            HTTPException: If validation fails
        """
        try:
            request_data = await request.json()
        except Exception:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="Invalid JSON"
            )
        
        return self.validate_request(request_data, validation_schema)

# Create default instances
input_validator = InputValidator()
form_validator = FormValidator()
request_validator = RequestValidator()
