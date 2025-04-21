"""
Secure logging utilities.

This module provides functions for secure logging that prevents sensitive data exposure.
"""

import logging
import re
import json
from typing import Any, Dict, List, Optional, Set, Union

class SensitiveDataFilter(logging.Filter):
    """
    Filter for removing sensitive data from log messages.
    
    This filter removes sensitive data like passwords, tokens, and credit card numbers
    from log messages.
    """
    
    def __init__(
        self,
        sensitive_fields: Optional[List[str]] = None,
        mask_char: str = '*',
        partial_visibility: bool = False
    ):
        """
        Initialize the filter.
        
        Args:
            sensitive_fields: List of field names to mask
            mask_char: Character to use for masking
            partial_visibility: Whether to show part of the sensitive data
        """
        super().__init__()
        self.sensitive_fields = set(sensitive_fields or [
            'password', 'secret', 'token', 'key', 'credential', 'auth',
            'credit_card', 'card_number', 'cvv', 'ssn', 'social_security',
            'api_key', 'apikey', 'access_token', 'refresh_token', 'jwt'
        ])
        self.mask_char = mask_char
        self.partial_visibility = partial_visibility
        
        # Compile regex patterns for common sensitive data
        self.patterns = [
            # Credit card numbers
            re.compile(r'\b(?:\d[ -]*?){13,16}\b'),
            # Social Security Numbers
            re.compile(r'\b\d{3}[-]?\d{2}[-]?\d{4}\b'),
            # API keys (various formats)
            re.compile(r'\b[A-Za-z0-9_-]{20,64}\b'),
            # JWT tokens
            re.compile(r'eyJ[a-zA-Z0-9_-]{5,}\.eyJ[a-zA-Z0-9_-]{5,}\.[a-zA-Z0-9_-]{5,}'),
            # Email addresses
            re.compile(r'\b[A-Za-z0-9._%+-]+@[A-Za-z0-9.-]+\.[A-Z|a-z]{2,}\b')
        ]
    
    def _mask_value(self, value: str) -> str:
        """
        Mask a sensitive value.
        
        Args:
            value: Value to mask
            
        Returns:
            Masked value
        """
        if not value or len(value) < 4:
            return self.mask_char * len(value)
            
        if self.partial_visibility:
            # Show first and last character
            return value[0] + self.mask_char * (len(value) - 2) + value[-1]
        else:
            # Mask the entire value
            return self.mask_char * len(value)
    
    def _mask_json(self, json_str: str) -> str:
        """
        Mask sensitive data in a JSON string.
        
        Args:
            json_str: JSON string to mask
            
        Returns:
            Masked JSON string
        """
        try:
            data = json.loads(json_str)
            self._mask_dict(data)
            return json.dumps(data)
        except json.JSONDecodeError:
            # Not valid JSON, use regex-based masking
            return self._mask_with_regex(json_str)
    
    def _mask_dict(self, data: Dict[str, Any], parent_key: str = '') -> None:
        """
        Recursively mask sensitive data in a dictionary.
        
        Args:
            data: Dictionary to mask
            parent_key: Parent key for nested dictionaries
        """
        if not isinstance(data, dict):
            return
            
        for key, value in list(data.items()):
            full_key = f"{parent_key}.{key}" if parent_key else key
            
            # Check if this is a sensitive field
            is_sensitive = any(
                sensitive in full_key.lower() or sensitive in key.lower()
                for sensitive in self.sensitive_fields
            )
            
            if is_sensitive and isinstance(value, str):
                # Mask the sensitive value
                data[key] = self._mask_value(value)
            elif isinstance(value, dict):
                # Recursively mask nested dictionaries
                self._mask_dict(value, full_key)
            elif isinstance(value, list):
                # Recursively mask items in lists
                for i, item in enumerate(value):
                    if isinstance(item, dict):
                        self._mask_dict(item, full_key)
    
    def _mask_with_regex(self, text: str) -> str:
        """
        Mask sensitive data using regex patterns.
        
        Args:
            text: Text to mask
            
        Returns:
            Masked text
        """
        result = text
        
        # Mask sensitive fields
        for pattern in self.patterns:
            result = pattern.sub(
                lambda m: self._mask_value(m.group(0)),
                result
            )
            
        return result
    
    def filter(self, record: logging.LogRecord) -> bool:
        """
        Filter log records to mask sensitive data.
        
        Args:
            record: Log record to filter
            
        Returns:
            True to include the record in the log
        """
        # Check if the message is a string
        if isinstance(record.msg, str):
            # Check if the message looks like JSON
            if record.msg.strip().startswith('{') and record.msg.strip().endswith('}'):
                record.msg = self._mask_json(record.msg)
            else:
                record.msg = self._mask_with_regex(record.msg)
                
        # Check args for sensitive data
        if record.args:
            args = list(record.args)
            for i, arg in enumerate(args):
                if isinstance(arg, str):
                    args[i] = self._mask_with_regex(arg)
                elif isinstance(arg, dict):
                    self._mask_dict(arg)
            record.args = tuple(args)
            
        return True

def setup_secure_logger(
    name: str,
    log_file: Optional[str] = None,
    level: int = logging.INFO,
    sensitive_fields: Optional[List[str]] = None
) -> logging.Logger:
    """
    Set up a secure logger with sensitive data filtering.
    
    Args:
        name: Logger name
        log_file: Path to log file (if None, logs to console only)
        level: Logging level
        sensitive_fields: List of sensitive field names to mask
        
    Returns:
        Configured logger
    """
    logger = logging.getLogger(name)
    logger.setLevel(level)
    
    # Create formatter
    formatter = logging.Formatter(
        '%(asctime)s - %(name)s - %(levelname)s - %(message)s',
        datefmt='%Y-%m-%d %H:%M:%S'
    )
    
    # Create console handler
    console_handler = logging.StreamHandler()
    console_handler.setLevel(level)
    console_handler.setFormatter(formatter)
    
    # Add sensitive data filter
    sensitive_filter = SensitiveDataFilter(sensitive_fields)
    console_handler.addFilter(sensitive_filter)
    
    # Add console handler to logger
    logger.addHandler(console_handler)
    
    # Create file handler if log file is specified
    if log_file:
        file_handler = logging.FileHandler(log_file)
        file_handler.setLevel(level)
        file_handler.setFormatter(formatter)
        file_handler.addFilter(sensitive_filter)
        logger.addHandler(file_handler)
        
    return logger
