"""
Input sanitization utilities for Bulo.Cloud Sentinel.

This module provides utilities for sanitizing user input to prevent XSS, command injection,
path traversal, and other injection attacks.
"""

import os
import re
import html
from typing import Any, Dict, List, Optional, Union

# XSS prevention
def sanitize_html_input(value: str) -> str:
    """
    Sanitize HTML input to prevent XSS attacks.
    
    Args:
        value: The input string to sanitize
        
    Returns:
        Sanitized string
    """
    if not isinstance(value, str):
        return str(value)
    
    # Escape HTML special characters
    return html.escape(value)

def sanitize_js_input(value: str) -> str:
    """
    Sanitize JavaScript input to prevent XSS attacks.
    
    Args:
        value: The input string to sanitize
        
    Returns:
        Sanitized string
    """
    if not isinstance(value, str):
        return str(value)
    
    # Escape JavaScript special characters
    value = value.replace('\\', '\\\\')
    value = value.replace('\'', '\\\'')
    value = value.replace('"', '\\"')
    value = value.replace('\n', '\\n')
    value = value.replace('\r', '\\r')
    value = value.replace('<', '&lt;')
    value = value.replace('>', '&gt;')
    
    return value

# Command injection prevention
def is_command_injection(value: str) -> bool:
    """
    Check if a string contains potential command injection.
    
    Args:
        value: The input string to check
        
    Returns:
        True if command injection is detected, False otherwise
    """
    if not isinstance(value, str):
        return False
    
    # Common command injection patterns
    patterns = [
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
    for pattern in patterns:
        if re.search(pattern, value):
            return True
    
    return False

def sanitize_command_input(value: str) -> str:
    """
    Sanitize command input to prevent command injection.
    
    Args:
        value: The input string to sanitize
        
    Returns:
        Sanitized string
        
    Raises:
        ValueError: If command injection is detected
    """
    if is_command_injection(value):
        raise ValueError("Potential command injection detected")
    
    # Escape shell special characters
    if isinstance(value, str):
        value = re.sub(r'([;&|<>])', r'\\\1', value)
    
    return value

# Path traversal prevention
def is_path_traversal(value: str) -> bool:
    """
    Check if a string contains potential path traversal.
    
    Args:
        value: The input string to check
        
    Returns:
        True if path traversal is detected, False otherwise
    """
    if not isinstance(value, str):
        return False
    
    # Common path traversal patterns
    patterns = [
        r'\.\.',
        r'\./',
        r'/\.',
        r'~/',
        r'%2e%2e',  # URL encoded ..
        r'%2f%2e%2e'  # URL encoded /..
    ]
    
    # Check each pattern
    for pattern in patterns:
        if re.search(pattern, value):
            return True
    
    return False

def sanitize_path_input(value: str, base_path: Optional[str] = None) -> str:
    """
    Sanitize path input to prevent path traversal.
    
    Args:
        value: The input string to sanitize
        base_path: Optional base path to check against
        
    Returns:
        Sanitized path
        
    Raises:
        ValueError: If path traversal is detected
    """
    if is_path_traversal(value):
        raise ValueError("Potential path traversal detected")
    
    # Normalize path
    normalized_path = os.path.normpath(value)
    
    # If base_path is provided, ensure the path is within the base_path
    if base_path:
        base_path = os.path.abspath(base_path)
        full_path = os.path.abspath(os.path.join(base_path, normalized_path))
        
        if not full_path.startswith(base_path):
            raise ValueError("Path is outside of allowed directory")
        
        # Return path relative to base_path
        return os.path.relpath(full_path, base_path)
    
    return normalized_path

# SQL injection prevention
def is_sql_injection(value: str) -> bool:
    """
    Check if a string contains potential SQL injection.
    
    This function uses more precise patterns to reduce false positives
    while still detecting common SQL injection attempts.
    
    Args:
        value: The input string to check
        
    Returns:
        True if SQL injection is detected, False otherwise
    """
    if not isinstance(value, str):
        return False
    
    # Skip empty strings
    if not value.strip():
        return False
    
    # More precise SQL injection patterns
    patterns = [
        # SQL comments that might be used to truncate queries
        r'--\s+',  # SQL comment with space after
        r'#\s*$',   # MySQL comment at end of line
        
        # Typical SQL injection with quotes and comments
        r'[\'"]\s*(\)|;)\s*(--|#|/\*)',
        
        # UNION-based SQL injection
        r'(\s|^)UNION(\s+ALL)?\s+SELECT\b',
        
        # Batched SQL statements with semicolons
        r';\s*(SELECT|INSERT|UPDATE|DELETE|DROP|ALTER|CREATE|TRUNCATE)\b',
        
        # Common SQL injection test patterns
        r'[\'"]?\s*(OR|AND)\s+[\'"]?\d+[\'"]?\s*=\s*[\'"]?\d+[\'"]?',  # OR 1=1
        r'[\'"]?\s*(OR|AND)\s+[\'"]?[a-zA-Z0-9_]+[\'"]?\s*=\s*[\'"]?[a-zA-Z0-9_]+[\'"]?', # OR a=a
        
        # Typical function-based SQL injection
        r'(SLEEP|BENCHMARK|WAITFOR\s+DELAY|PG_SLEEP)\s*\(',
        
        # SQL injection with string concatenation
        r'[\'"][^\'")]*(\|\||%2B|&|%26|%7C){2}[^\'")]*[\'"]',
        
        # Time-based blind SQL injection
        r'(SLEEP|BENCHMARK|WAITFOR\s+DELAY|PG_SLEEP)\s*\(\s*\d+\s*\)',
        
        # SQL injection with LIKE operator
        r'(LIKE\s+[\'"]%.*?[\'"])',
        
        # SQL injection with system commands
        r'(EXEC\s+xp_cmdshell|LOAD_FILE|INTO\s+OUTFILE|INTO\s+DUMPFILE)',
        
        # SQL injection with multi-line comments
        r'/\*.*?\*/',
        
        # SQL injection with conditional statements
        r'(CASE\s+WHEN\s+.*?\s+THEN\s+.*?\s+ELSE\s+.*?\s+END)',
    ]
    
    # Check each pattern
    for pattern in patterns:
        if re.search(pattern, value, re.IGNORECASE):
            return True
    
    # Check for suspicious combinations of SQL keywords and syntax
    sql_keywords = [
        'SELECT', 'INSERT', 'UPDATE', 'DELETE', 'DROP', 'ALTER', 'CREATE', 'TRUNCATE',
        'UNION', 'JOIN', 'WHERE', 'HAVING', 'GROUP BY', 'ORDER BY'
    ]
    
    # Only flag as SQL injection if SQL keywords are combined with SQL syntax
    sql_syntax = [';', '--', '/*', '*/', '=', '<', '>', '(', ')', '\'', '"']
    
    has_keyword = any(re.search(r'\b' + keyword + r'\b', value, re.IGNORECASE) for keyword in sql_keywords)
    has_syntax = any(syntax in value for syntax in sql_syntax)
    
    # Only consider it SQL injection if both keyword and syntax are present
    if has_keyword and has_syntax:
        # Additional check to reduce false positives
        # Don't flag if it's just a simple word with SQL keyword as substring
        # For example, "SELECT" in "SELECTED" or "SELECTION"
        for keyword in sql_keywords:
            if re.search(r'\b' + keyword + r'\b', value, re.IGNORECASE):
                return True
    
    return False

def sanitize_sql_input(value: str) -> str:
    """
    Sanitize SQL input to prevent SQL injection.
    
    Args:
        value: The input string to sanitize
        
    Returns:
        Sanitized string
        
    Raises:
        ValueError: If SQL injection is detected
    """
    if is_sql_injection(value):
        raise ValueError("Potential SQL injection detected")
    
    # Escape SQL special characters
    if isinstance(value, str):
        value = value.replace("'", "''")
        value = value.replace("\\", "\\\\")
        value = value.replace("%", "\\%")
        value = value.replace("_", "\\_")
    
    return value

# General input sanitization
def sanitize_input(value: Any, sanitize_type: str = "html") -> Any:
    """
    Sanitize input based on the specified type.
    
    Args:
        value: The input to sanitize
        sanitize_type: The type of sanitization to perform (html, js, command, path, sql)
        
    Returns:
        Sanitized input
    """
    if not value:
        return value
    
    if sanitize_type == "html":
        return sanitize_html_input(value)
    elif sanitize_type == "js":
        return sanitize_js_input(value)
    elif sanitize_type == "command":
        return sanitize_command_input(value)
    elif sanitize_type == "path":
        return sanitize_path_input(value)
    elif sanitize_type == "sql":
        return sanitize_sql_input(value)
    else:
        return value

def sanitize_dict(data: Dict[str, Any], sanitize_type: str = "html") -> Dict[str, Any]:
    """
    Sanitize all string values in a dictionary.
    
    Args:
        data: The dictionary to sanitize
        sanitize_type: The type of sanitization to perform
        
    Returns:
        Sanitized dictionary
    """
    if not isinstance(data, dict):
        return data
    
    result = {}
    for key, value in data.items():
        if isinstance(value, str):
            result[key] = sanitize_input(value, sanitize_type)
        elif isinstance(value, dict):
            result[key] = sanitize_dict(value, sanitize_type)
        elif isinstance(value, list):
            result[key] = sanitize_list(value, sanitize_type)
        else:
            result[key] = value
    
    return result

def sanitize_list(data: List[Any], sanitize_type: str = "html") -> List[Any]:
    """
    Sanitize all string values in a list.
    
    Args:
        data: The list to sanitize
        sanitize_type: The type of sanitization to perform
        
    Returns:
        Sanitized list
    """
    if not isinstance(data, list):
        return data
    
    result = []
    for item in data:
        if isinstance(item, str):
            result.append(sanitize_input(item, sanitize_type))
        elif isinstance(item, dict):
            result.append(sanitize_dict(item, sanitize_type))
        elif isinstance(item, list):
            result.append(sanitize_list(item, sanitize_type))
        else:
            result.append(item)
    
    return result
