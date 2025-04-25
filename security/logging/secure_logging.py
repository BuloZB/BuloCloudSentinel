"""
Secure logging utilities for Bulo.Cloud Sentinel.

This module provides functions for securely logging events and errors,
with proper handling of sensitive data.
"""

import os
import json
import logging
import traceback
from typing import Any, Dict, List, Optional, Set, Union
from datetime import datetime, timezone
from pathlib import Path

# Set of sensitive field names that should be masked in logs
SENSITIVE_FIELDS = {
    "password", "token", "secret", "key", "auth", "credential", "jwt", "api_key",
    "access_token", "refresh_token", "private_key", "cookie", "session", "ssn",
    "social_security", "credit_card", "card_number", "cvv", "pin", "passphrase"
}


class SecureLogger:
    """
    A logger that automatically masks sensitive data.
    """
    
    def __init__(
        self,
        name: str,
        log_file: Optional[Union[str, Path]] = None,
        log_level: int = logging.INFO,
        sensitive_fields: Optional[Set[str]] = None,
        mask_char: str = "*",
        console_output: bool = True,
        json_format: bool = False
    ):
        """
        Initialize the secure logger.
        
        Args:
            name: Logger name
            log_file: Path to log file (optional)
            log_level: Logging level
            sensitive_fields: Set of sensitive field names to mask
            mask_char: Character to use for masking
            console_output: Whether to output logs to console
            json_format: Whether to format logs as JSON
        """
        self.name = name
        self.logger = logging.getLogger(name)
        self.logger.setLevel(log_level)
        self.sensitive_fields = sensitive_fields or SENSITIVE_FIELDS
        self.mask_char = mask_char
        self.json_format = json_format
        
        # Create formatter
        if json_format:
            formatter = logging.Formatter('%(message)s')
        else:
            formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        
        # Add console handler if requested
        if console_output:
            console_handler = logging.StreamHandler()
            console_handler.setFormatter(formatter)
            self.logger.addHandler(console_handler)
        
        # Add file handler if log file is specified
        if log_file:
            log_path = Path(log_file)
            log_path.parent.mkdir(parents=True, exist_ok=True)
            
            file_handler = logging.FileHandler(log_path)
            file_handler.setFormatter(formatter)
            self.logger.addHandler(file_handler)
    
    def _mask_sensitive_data(self, data: Any) -> Any:
        """
        Mask sensitive data in logs.
        
        Args:
            data: Data to mask
            
        Returns:
            Masked data
        """
        if isinstance(data, dict):
            masked_data = {}
            for key, value in data.items():
                # Check if key contains any sensitive field name
                is_sensitive = any(field.lower() in key.lower() for field in self.sensitive_fields)
                
                if is_sensitive and isinstance(value, (str, int, float, bool)):
                    # Mask sensitive value
                    if isinstance(value, str) and value:
                        # Show first and last character, mask the rest
                        if len(value) > 2:
                            masked_data[key] = value[0] + self.mask_char * (len(value) - 2) + value[-1]
                        else:
                            masked_data[key] = self.mask_char * len(value)
                    else:
                        masked_data[key] = self.mask_char * 8  # Mask non-string values
                else:
                    # Recursively mask nested data
                    masked_data[key] = self._mask_sensitive_data(value)
            
            return masked_data
        
        elif isinstance(data, list):
            return [self._mask_sensitive_data(item) for item in data]
        
        elif isinstance(data, (str, int, float, bool)) or data is None:
            return data
        
        else:
            # Try to convert to string
            try:
                return str(data)
            except Exception:
                return "<unprintable object>"
    
    def _format_message(self, message: str, extra: Optional[Dict[str, Any]] = None) -> str:
        """
        Format a log message.
        
        Args:
            message: Log message
            extra: Extra data to include
            
        Returns:
            Formatted message
        """
        if not self.json_format:
            if extra:
                masked_extra = self._mask_sensitive_data(extra)
                return f"{message} - {masked_extra}"
            return message
        
        # Format as JSON
        log_data = {
            "timestamp": datetime.now(timezone.utc).isoformat(),
            "logger": self.name,
            "message": message
        }
        
        if extra:
            log_data["data"] = self._mask_sensitive_data(extra)
        
        return json.dumps(log_data)
    
    def debug(self, message: str, extra: Optional[Dict[str, Any]] = None):
        """
        Log a debug message.
        
        Args:
            message: Log message
            extra: Extra data to include
        """
        self.logger.debug(self._format_message(message, extra))
    
    def info(self, message: str, extra: Optional[Dict[str, Any]] = None):
        """
        Log an info message.
        
        Args:
            message: Log message
            extra: Extra data to include
        """
        self.logger.info(self._format_message(message, extra))
    
    def warning(self, message: str, extra: Optional[Dict[str, Any]] = None):
        """
        Log a warning message.
        
        Args:
            message: Log message
            extra: Extra data to include
        """
        self.logger.warning(self._format_message(message, extra))
    
    def error(self, message: str, extra: Optional[Dict[str, Any]] = None, exc_info: bool = False):
        """
        Log an error message.
        
        Args:
            message: Log message
            extra: Extra data to include
            exc_info: Whether to include exception info
        """
        self.logger.error(self._format_message(message, extra), exc_info=exc_info)
    
    def critical(self, message: str, extra: Optional[Dict[str, Any]] = None, exc_info: bool = False):
        """
        Log a critical message.
        
        Args:
            message: Log message
            extra: Extra data to include
            exc_info: Whether to include exception info
        """
        self.logger.critical(self._format_message(message, extra), exc_info=exc_info)
    
    def exception(self, message: str, extra: Optional[Dict[str, Any]] = None):
        """
        Log an exception message.
        
        Args:
            message: Log message
            extra: Extra data to include
        """
        self.logger.exception(self._format_message(message, extra))


class AuditLogger(SecureLogger):
    """
    A logger specifically for audit events.
    """
    
    def __init__(
        self,
        name: str = "audit",
        log_file: Optional[Union[str, Path]] = None,
        log_level: int = logging.INFO,
        sensitive_fields: Optional[Set[str]] = None,
        mask_char: str = "*",
        console_output: bool = False,
        json_format: bool = True
    ):
        """
        Initialize the audit logger.
        
        Args:
            name: Logger name
            log_file: Path to log file (optional)
            log_level: Logging level
            sensitive_fields: Set of sensitive field names to mask
            mask_char: Character to use for masking
            console_output: Whether to output logs to console
            json_format: Whether to format logs as JSON
        """
        super().__init__(
            name=name,
            log_file=log_file,
            log_level=log_level,
            sensitive_fields=sensitive_fields,
            mask_char=mask_char,
            console_output=console_output,
            json_format=json_format
        )
    
    def log_event(
        self,
        event_type: str,
        user_id: Optional[str] = None,
        resource_type: Optional[str] = None,
        resource_id: Optional[str] = None,
        action: Optional[str] = None,
        status: Optional[str] = None,
        details: Optional[Dict[str, Any]] = None,
        ip_address: Optional[str] = None,
        user_agent: Optional[str] = None
    ):
        """
        Log an audit event.
        
        Args:
            event_type: Type of event
            user_id: ID of the user who performed the action
            resource_type: Type of resource affected
            resource_id: ID of resource affected
            action: Action performed
            status: Status of the action
            details: Additional details
            ip_address: IP address of the user
            user_agent: User agent of the user
        """
        event_data = {
            "event_type": event_type,
            "timestamp": datetime.now(timezone.utc).isoformat(),
        }
        
        if user_id:
            event_data["user_id"] = user_id
        
        if resource_type:
            event_data["resource_type"] = resource_type
        
        if resource_id:
            event_data["resource_id"] = resource_id
        
        if action:
            event_data["action"] = action
        
        if status:
            event_data["status"] = status
        
        if details:
            event_data["details"] = details
        
        if ip_address:
            event_data["ip_address"] = ip_address
        
        if user_agent:
            event_data["user_agent"] = user_agent
        
        self.info(f"Audit event: {event_type}", event_data)
    
    def log_auth_event(
        self,
        event_type: str,
        user_id: Optional[str] = None,
        username: Optional[str] = None,
        status: str = "success",
        details: Optional[Dict[str, Any]] = None,
        ip_address: Optional[str] = None,
        user_agent: Optional[str] = None
    ):
        """
        Log an authentication or authorization event.
        
        Args:
            event_type: Type of event (login, logout, etc.)
            user_id: ID of the user
            username: Username of the user
            status: Status of the event (success, failure, etc.)
            details: Additional details
            ip_address: IP address of the user
            user_agent: User agent of the user
        """
        event_data = {
            "event_type": f"auth.{event_type}",
            "timestamp": datetime.now(timezone.utc).isoformat(),
            "status": status
        }
        
        if user_id:
            event_data["user_id"] = user_id
        
        if username:
            event_data["username"] = username
        
        if details:
            event_data["details"] = details
        
        if ip_address:
            event_data["ip_address"] = ip_address
        
        if user_agent:
            event_data["user_agent"] = user_agent
        
        self.info(f"Auth event: {event_type}", event_data)
    
    def log_data_access(
        self,
        user_id: str,
        resource_type: str,
        resource_id: str,
        action: str = "read",
        status: str = "success",
        details: Optional[Dict[str, Any]] = None,
        ip_address: Optional[str] = None
    ):
        """
        Log a data access event.
        
        Args:
            user_id: ID of the user who accessed the data
            resource_type: Type of resource accessed
            resource_id: ID of resource accessed
            action: Action performed (read, write, etc.)
            status: Status of the access (success, failure, etc.)
            details: Additional details
            ip_address: IP address of the user
        """
        event_data = {
            "event_type": "data_access",
            "timestamp": datetime.now(timezone.utc).isoformat(),
            "user_id": user_id,
            "resource_type": resource_type,
            "resource_id": resource_id,
            "action": action,
            "status": status
        }
        
        if details:
            event_data["details"] = details
        
        if ip_address:
            event_data["ip_address"] = ip_address
        
        self.info(f"Data access: {action} {resource_type}/{resource_id}", event_data)


def get_secure_logger(
    name: str,
    log_file: Optional[Union[str, Path]] = None,
    log_level: int = logging.INFO,
    sensitive_fields: Optional[Set[str]] = None,
    mask_char: str = "*",
    console_output: bool = True,
    json_format: bool = False
) -> SecureLogger:
    """
    Get a secure logger instance.
    
    Args:
        name: Logger name
        log_file: Path to log file (optional)
        log_level: Logging level
        sensitive_fields: Set of sensitive field names to mask
        mask_char: Character to use for masking
        console_output: Whether to output logs to console
        json_format: Whether to format logs as JSON
        
    Returns:
        A secure logger instance
    """
    return SecureLogger(
        name=name,
        log_file=log_file,
        log_level=log_level,
        sensitive_fields=sensitive_fields,
        mask_char=mask_char,
        console_output=console_output,
        json_format=json_format
    )


def get_audit_logger(
    log_file: Optional[Union[str, Path]] = None,
    log_level: int = logging.INFO,
    sensitive_fields: Optional[Set[str]] = None,
    mask_char: str = "*",
    console_output: bool = False,
    json_format: bool = True
) -> AuditLogger:
    """
    Get an audit logger instance.
    
    Args:
        log_file: Path to log file (optional)
        log_level: Logging level
        sensitive_fields: Set of sensitive field names to mask
        mask_char: Character to use for masking
        console_output: Whether to output logs to console
        json_format: Whether to format logs as JSON
        
    Returns:
        An audit logger instance
    """
    return AuditLogger(
        log_file=log_file,
        log_level=log_level,
        sensitive_fields=sensitive_fields,
        mask_char=mask_char,
        console_output=console_output,
        json_format=json_format
    )
