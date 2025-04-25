"""
Advanced input validation utilities for Bulo.Cloud Sentinel.

This module provides advanced functions for validating and sanitizing user inputs
to prevent injection attacks and other security issues.
"""

import re
import html
import unicodedata
import ipaddress
from typing import Any, Dict, List, Optional, Pattern, Set, Union
from datetime import datetime, date
from fastapi import HTTPException, status
from pydantic import BaseModel, Field, validator

from .input_validation import (
    validate_email,
    validate_username,
    validate_name,
    validate_uuid,
    validate_coordinates,
    validate_phone,
    validate_url,
    validate_filename,
    validate_path,
    sanitize_string,
    check_sql_injection,
)


# Regular expressions for validation
CREDIT_CARD_REGEX = re.compile(r"^(?:4[0-9]{12}(?:[0-9]{3})?|5[1-5][0-9]{14}|3[47][0-9]{13}|3(?:0[0-5]|[68][0-9])[0-9]{11}|6(?:011|5[0-9]{2})[0-9]{12}|(?:2131|1800|35\d{3})\d{11})$")
SSN_REGEX = re.compile(r"^(?!000|666|9\d{2})([0-8]\d{2})(?!00)(\d{2})(?!0000)(\d{4})$")
IP_ADDRESS_REGEX = re.compile(r"^(?:(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.){3}(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)$")
MAC_ADDRESS_REGEX = re.compile(r"^([0-9A-Fa-f]{2}[:-]){5}([0-9A-Fa-f]{2})$")
DATE_REGEX = re.compile(r"^(\d{4})-(\d{2})-(\d{2})$")
TIME_REGEX = re.compile(r"^([01]\d|2[0-3]):([0-5]\d):([0-5]\d)$")
DATETIME_REGEX = re.compile(r"^(\d{4})-(\d{2})-(\d{2})T([01]\d|2[0-3]):([0-5]\d):([0-5]\d)(?:\.\d+)?(?:Z|[+-]\d{2}:\d{2})?$")
COLOR_HEX_REGEX = re.compile(r"^#([A-Fa-f0-9]{6}|[A-Fa-f0-9]{3})$")
DOMAIN_REGEX = re.compile(r"^(?:[a-zA-Z0-9](?:[a-zA-Z0-9-]{0,61}[a-zA-Z0-9])?\.)+[a-zA-Z]{2,}$")
JWT_REGEX = re.compile(r"^[A-Za-z0-9-_]+\.[A-Za-z0-9-_]+\.[A-Za-z0-9-_]*$")


class ValidationError(Exception):
    """
    Exception raised for validation errors.
    """
    
    def __init__(self, message: str, field: Optional[str] = None):
        """
        Initialize the exception.
        
        Args:
            message: Error message
            field: Field that failed validation
        """
        self.message = message
        self.field = field
        super().__init__(message)


class InputValidator:
    """
    Validator for input values.
    """
    
    def __init__(self):
        """
        Initialize the validator.
        """
        pass
    
    def validate_credit_card(self, value: str) -> bool:
        """
        Validate a credit card number using the Luhn algorithm.
        
        Args:
            value: Credit card number to validate
            
        Returns:
            True if the credit card number is valid, False otherwise
        """
        if not value or not isinstance(value, str):
            return False
        
        # Remove spaces and dashes
        value = value.replace(" ", "").replace("-", "")
        
        # Check format
        if not CREDIT_CARD_REGEX.match(value):
            return False
        
        # Luhn algorithm
        digits = [int(d) for d in value]
        odd_digits = digits[-1::-2]
        even_digits = digits[-2::-2]
        checksum = sum(odd_digits)
        for d in even_digits:
            checksum += sum(divmod(d * 2, 10))
        
        return checksum % 10 == 0
    
    def validate_ssn(self, value: str) -> bool:
        """
        Validate a Social Security Number (SSN).
        
        Args:
            value: SSN to validate
            
        Returns:
            True if the SSN is valid, False otherwise
        """
        if not value or not isinstance(value, str):
            return False
        
        # Remove dashes
        value = value.replace("-", "")
        
        # Check format
        return bool(SSN_REGEX.match(value))
    
    def validate_ip_address(self, value: str) -> bool:
        """
        Validate an IP address.
        
        Args:
            value: IP address to validate
            
        Returns:
            True if the IP address is valid, False otherwise
        """
        if not value or not isinstance(value, str):
            return False
        
        # Check format
        if not IP_ADDRESS_REGEX.match(value):
            return False
        
        # Validate using ipaddress module
        try:
            ipaddress.ip_address(value)
            return True
        except ValueError:
            return False
    
    def validate_mac_address(self, value: str) -> bool:
        """
        Validate a MAC address.
        
        Args:
            value: MAC address to validate
            
        Returns:
            True if the MAC address is valid, False otherwise
        """
        if not value or not isinstance(value, str):
            return False
        
        # Check format
        return bool(MAC_ADDRESS_REGEX.match(value))
    
    def validate_date(self, value: str) -> bool:
        """
        Validate a date string (YYYY-MM-DD).
        
        Args:
            value: Date string to validate
            
        Returns:
            True if the date string is valid, False otherwise
        """
        if not value or not isinstance(value, str):
            return False
        
        # Check format
        if not DATE_REGEX.match(value):
            return False
        
        # Validate date
        try:
            year, month, day = map(int, value.split("-"))
            date(year, month, day)
            return True
        except ValueError:
            return False
    
    def validate_time(self, value: str) -> bool:
        """
        Validate a time string (HH:MM:SS).
        
        Args:
            value: Time string to validate
            
        Returns:
            True if the time string is valid, False otherwise
        """
        if not value or not isinstance(value, str):
            return False
        
        # Check format
        return bool(TIME_REGEX.match(value))
    
    def validate_datetime(self, value: str) -> bool:
        """
        Validate a datetime string (ISO 8601).
        
        Args:
            value: Datetime string to validate
            
        Returns:
            True if the datetime string is valid, False otherwise
        """
        if not value or not isinstance(value, str):
            return False
        
        # Check format
        if not DATETIME_REGEX.match(value):
            return False
        
        # Validate datetime
        try:
            datetime.fromisoformat(value.replace("Z", "+00:00"))
            return True
        except ValueError:
            return False
    
    def validate_color_hex(self, value: str) -> bool:
        """
        Validate a color hexadecimal code.
        
        Args:
            value: Color hex code to validate
            
        Returns:
            True if the color hex code is valid, False otherwise
        """
        if not value or not isinstance(value, str):
            return False
        
        # Check format
        return bool(COLOR_HEX_REGEX.match(value))
    
    def validate_domain(self, value: str) -> bool:
        """
        Validate a domain name.
        
        Args:
            value: Domain name to validate
            
        Returns:
            True if the domain name is valid, False otherwise
        """
        if not value or not isinstance(value, str):
            return False
        
        # Check format
        return bool(DOMAIN_REGEX.match(value))
    
    def validate_jwt(self, value: str) -> bool:
        """
        Validate a JWT token format.
        
        Args:
            value: JWT token to validate
            
        Returns:
            True if the JWT token format is valid, False otherwise
        """
        if not value or not isinstance(value, str):
            return False
        
        # Check format
        return bool(JWT_REGEX.match(value))
    
    def validate_input(
        self,
        value: Any,
        validation_type: str,
        required: bool = True,
        min_length: Optional[int] = None,
        max_length: Optional[int] = None,
        min_value: Optional[Union[int, float]] = None,
        max_value: Optional[Union[int, float]] = None,
        pattern: Optional[Pattern] = None,
        allowed_values: Optional[List[Any]] = None,
        custom_validator: Optional[callable] = None,
        field_name: Optional[str] = None
    ) -> Any:
        """
        Validate an input value based on specified criteria.
        
        Args:
            value: Value to validate
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
        if required and (value is None or (isinstance(value, str) and value.strip() == "")):
            raise ValidationError(f"Value is required", field_name)
        
        # If not required and empty, return None
        if not required and (value is None or (isinstance(value, str) and value.strip() == "")):
            return None
        
        # Validate based on type
        if validation_type == "string":
            if not isinstance(value, str):
                raise ValidationError(f"Value must be a string", field_name)
            
            # Length validation
            if min_length is not None and len(value) < min_length:
                raise ValidationError(f"Value must be at least {min_length} characters long", field_name)
            
            if max_length is not None and len(value) > max_length:
                raise ValidationError(f"Value must be at most {max_length} characters long", field_name)
            
            # Pattern validation
            if pattern and not pattern.match(value):
                raise ValidationError(f"Value format is invalid", field_name)
        
        elif validation_type == "integer":
            try:
                value = int(value)
            except (ValueError, TypeError):
                raise ValidationError(f"Value must be an integer", field_name)
            
            # Range validation
            if min_value is not None and value < min_value:
                raise ValidationError(f"Value must be at least {min_value}", field_name)
            
            if max_value is not None and value > max_value:
                raise ValidationError(f"Value must be at most {max_value}", field_name)
        
        elif validation_type == "float":
            try:
                value = float(value)
            except (ValueError, TypeError):
                raise ValidationError(f"Value must be a number", field_name)
            
            # Range validation
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
                    raise ValidationError(f"Value must be a boolean", field_name)
            elif not isinstance(value, bool):
                raise ValidationError(f"Value must be a boolean", field_name)
        
        elif validation_type == "email":
            if not validate_email(value):
                raise ValidationError(f"Value must be a valid email address", field_name)
        
        elif validation_type == "username":
            if not validate_username(value):
                raise ValidationError(f"Value must be a valid username", field_name)
        
        elif validation_type == "name":
            if not validate_name(value):
                raise ValidationError(f"Value must be a valid name", field_name)
        
        elif validation_type == "uuid":
            if not validate_uuid(value):
                raise ValidationError(f"Value must be a valid UUID", field_name)
        
        elif validation_type == "url":
            if not validate_url(value):
                raise ValidationError(f"Value must be a valid URL", field_name)
        
        elif validation_type == "phone":
            if not validate_phone(value):
                raise ValidationError(f"Value must be a valid phone number", field_name)
        
        elif validation_type == "credit_card":
            if not self.validate_credit_card(value):
                raise ValidationError(f"Value must be a valid credit card number", field_name)
        
        elif validation_type == "ssn":
            if not self.validate_ssn(value):
                raise ValidationError(f"Value must be a valid SSN", field_name)
        
        elif validation_type == "ip_address":
            if not self.validate_ip_address(value):
                raise ValidationError(f"Value must be a valid IP address", field_name)
        
        elif validation_type == "mac_address":
            if not self.validate_mac_address(value):
                raise ValidationError(f"Value must be a valid MAC address", field_name)
        
        elif validation_type == "date":
            if not self.validate_date(value):
                raise ValidationError(f"Value must be a valid date (YYYY-MM-DD)", field_name)
        
        elif validation_type == "time":
            if not self.validate_time(value):
                raise ValidationError(f"Value must be a valid time (HH:MM:SS)", field_name)
        
        elif validation_type == "datetime":
            if not self.validate_datetime(value):
                raise ValidationError(f"Value must be a valid datetime (ISO 8601)", field_name)
        
        elif validation_type == "color_hex":
            if not self.validate_color_hex(value):
                raise ValidationError(f"Value must be a valid color hex code", field_name)
        
        elif validation_type == "domain":
            if not self.validate_domain(value):
                raise ValidationError(f"Value must be a valid domain name", field_name)
        
        elif validation_type == "jwt":
            if not self.validate_jwt(value):
                raise ValidationError(f"Value must be a valid JWT token", field_name)
        
        elif validation_type == "list":
            if not isinstance(value, list):
                raise ValidationError(f"Value must be a list", field_name)
            
            # Length validation
            if min_length is not None and len(value) < min_length:
                raise ValidationError(f"List must have at least {min_length} items", field_name)
            
            if max_length is not None and len(value) > max_length:
                raise ValidationError(f"List must have at most {max_length} items", field_name)
        
        elif validation_type == "dict":
            if not isinstance(value, dict):
                raise ValidationError(f"Value must be a dictionary", field_name)
        
        else:
            raise ValidationError(f"Unknown validation type: {validation_type}", field_name)
        
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
    
    def validate_form(self, form_data: Dict[str, Any], validation_rules: Dict[str, Dict[str, Any]]) -> Dict[str, Any]:
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


# Create default instances
input_validator = InputValidator()
form_validator = FormValidator()
request_validator = RequestValidator()
