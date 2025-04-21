"""
Logging utilities for the Anti-Jamming Service.

This module provides utilities for configuring secure logging.
"""

import logging
import logging.handlers
import os
import json
from typing import Dict, Any, Optional
from pathlib import Path
import re

from anti_jamming_service.utils.config import get_config

# Sensitive data patterns
SENSITIVE_PATTERNS = [
    re.compile(r'password\s*[=:]\s*[\'"]?([^\'"]+)[\'"]?', re.IGNORECASE),
    re.compile(r'token\s*[=:]\s*[\'"]?([^\'"]+)[\'"]?', re.IGNORECASE),
    re.compile(r'secret\s*[=:]\s*[\'"]?([^\'"]+)[\'"]?', re.IGNORECASE),
    re.compile(r'key\s*[=:]\s*[\'"]?([^\'"]+)[\'"]?', re.IGNORECASE),
    re.compile(r'credential\s*[=:]\s*[\'"]?([^\'"]+)[\'"]?', re.IGNORECASE),
    re.compile(r'auth\s*[=:]\s*[\'"]?([^\'"]+)[\'"]?', re.IGNORECASE),
]


class SensitiveDataFilter(logging.Filter):
    """
    Filter for masking sensitive data in log messages.
    
    This filter replaces sensitive data like passwords, tokens, and keys
    with masked values to prevent leaking sensitive information in logs.
    """
    
    def __init__(self, patterns=None):
        """
        Initialize the filter.
        
        Args:
            patterns: Optional list of regex patterns for sensitive data.
        """
        super().__init__()
        self.patterns = patterns or SENSITIVE_PATTERNS
    
    def filter(self, record):
        """
        Filter log record to mask sensitive data.
        
        Args:
            record: Log record to filter.
            
        Returns:
            bool: Always True (record is always processed).
        """
        if isinstance(record.msg, str):
            for pattern in self.patterns:
                record.msg = pattern.sub(r'\1=***MASKED***', record.msg)
        
        if hasattr(record, 'args') and record.args:
            if isinstance(record.args, dict):
                for key, value in record.args.items():
                    if isinstance(value, str):
                        for pattern in self.patterns:
                            record.args[key] = pattern.sub(r'\1=***MASKED***', value)
            else:
                args_list = list(record.args)
                for i, arg in enumerate(args_list):
                    if isinstance(arg, str):
                        for pattern in self.patterns:
                            args_list[i] = pattern.sub(r'\1=***MASKED***', arg)
                record.args = tuple(args_list)
        
        return True


class JSONFormatter(logging.Formatter):
    """
    JSON formatter for structured logging.
    
    This formatter outputs log records as JSON objects for easier parsing
    and analysis by log management systems.
    """
    
    def format(self, record):
        """
        Format log record as JSON.
        
        Args:
            record: Log record to format.
            
        Returns:
            str: JSON-formatted log message.
        """
        log_data = {
            'timestamp': self.formatTime(record, self.datefmt),
            'level': record.levelname,
            'name': record.name,
            'message': record.getMessage(),
            'module': record.module,
            'function': record.funcName,
            'line': record.lineno,
            'process': record.process,
            'thread': record.thread
        }
        
        if hasattr(record, 'request_id'):
            log_data['request_id'] = record.request_id
        
        if record.exc_info:
            log_data['exception'] = self.formatException(record.exc_info)
        
        return json.dumps(log_data)


def setup_logging():
    """
    Set up logging for the Anti-Jamming Service.
    
    This function configures logging based on the configuration,
    including log level, format, and output destination.
    """
    config = get_config()
    log_config = config.get('logging', {})
    
    # Get log level
    level_name = log_config.get('level', 'INFO')
    level = getattr(logging, level_name.upper(), logging.INFO)
    
    # Get log format
    log_format = log_config.get('format', '%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    
    # Create formatter
    if log_config.get('json_format', False):
        formatter = JSONFormatter()
    else:
        formatter = logging.Formatter(log_format)
    
    # Create handlers
    handlers = []
    
    # Console handler
    console_handler = logging.StreamHandler()
    console_handler.setFormatter(formatter)
    handlers.append(console_handler)
    
    # File handler
    log_file = log_config.get('file')
    if log_file:
        # Create log directory if it doesn't exist
        log_dir = os.path.dirname(log_file)
        if log_dir:
            os.makedirs(log_dir, exist_ok=True)
        
        # Get max size and backup count
        max_size = log_config.get('max_size', 10485760)  # 10 MB
        backup_count = log_config.get('backup_count', 5)
        
        # Create rotating file handler
        file_handler = logging.handlers.RotatingFileHandler(
            log_file, maxBytes=max_size, backupCount=backup_count
        )
        file_handler.setFormatter(formatter)
        handlers.append(file_handler)
    
    # Configure root logger
    root_logger = logging.getLogger()
    root_logger.setLevel(level)
    
    # Remove existing handlers
    for handler in root_logger.handlers[:]:
        root_logger.removeHandler(handler)
    
    # Add new handlers
    for handler in handlers:
        root_logger.addHandler(handler)
    
    # Add sensitive data filter
    sensitive_filter = SensitiveDataFilter()
    for handler in root_logger.handlers:
        handler.addFilter(sensitive_filter)
    
    # Configure library loggers
    logging.getLogger('urllib3').setLevel(logging.WARNING)
    logging.getLogger('requests').setLevel(logging.WARNING)
    
    # Log configuration
    logging.info(f"Logging configured with level {level_name}")


def get_logger(name: str) -> logging.Logger:
    """
    Get a logger with the specified name.
    
    Args:
        name: Logger name.
        
    Returns:
        logging.Logger: Logger instance.
    """
    return logging.getLogger(name)
