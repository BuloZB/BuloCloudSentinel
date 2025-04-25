"""
Validation utilities for Bulo.Cloud Sentinel.

This package provides functions for validating and sanitizing inputs
to prevent injection attacks and other security issues.
"""

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
    sanitize_html,
    check_sql_injection,
    validate_input,
)

from .sql_validation import (
    validate_sql_identifier,
    safe_table_name,
    safe_column_name,
    safe_order_direction,
    build_safe_select_query,
    execute_safe_query,
)

from .html_validation import (
    strip_all_tags,
    sanitize_html,
    detect_xss,
    validate_html_content,
)

from .advanced_validation import (
    ValidationError,
    InputValidator,
    FormValidator,
    RequestValidator,
    input_validator,
    form_validator,
    request_validator,
)

__all__ = [
    # Input validation
    "validate_email",
    "validate_username",
    "validate_name",
    "validate_uuid",
    "validate_coordinates",
    "validate_phone",
    "validate_url",
    "validate_filename",
    "validate_path",
    "sanitize_string",
    "sanitize_html",
    "check_sql_injection",
    "validate_input",

    # SQL validation
    "validate_sql_identifier",
    "safe_table_name",
    "safe_column_name",
    "safe_order_direction",
    "build_safe_select_query",
    "execute_safe_query",

    # HTML validation
    "strip_all_tags",
    "sanitize_html",
    "detect_xss",
    "validate_html_content",

    # Advanced validation
    "ValidationError",
    "InputValidator",
    "FormValidator",
    "RequestValidator",
    "input_validator",
    "form_validator",
    "request_validator",
]
