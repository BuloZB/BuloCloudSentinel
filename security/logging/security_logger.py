"""
Security logging utilities for Bulo.Cloud Sentinel.

This module provides functions for logging security-relevant events.
"""

import json
import uuid
from typing import Any, Dict, List, Optional, Union
from datetime import datetime, timezone
from enum import Enum, auto

from .secure_logging import get_secure_logger, AuditLogger


class SecurityEventType(Enum):
    """
    Types of security events.
    """
    
    # Authentication events
    LOGIN_SUCCESS = auto()
    LOGIN_FAILURE = auto()
    LOGOUT = auto()
    PASSWORD_CHANGE = auto()
    PASSWORD_RESET = auto()
    MFA_ENABLED = auto()
    MFA_DISABLED = auto()
    MFA_CHALLENGE = auto()
    MFA_SUCCESS = auto()
    MFA_FAILURE = auto()
    
    # Authorization events
    PERMISSION_GRANTED = auto()
    PERMISSION_DENIED = auto()
    ROLE_CHANGE = auto()
    
    # User management events
    USER_CREATED = auto()
    USER_UPDATED = auto()
    USER_DELETED = auto()
    USER_LOCKED = auto()
    USER_UNLOCKED = auto()
    
    # Data access events
    DATA_ACCESS = auto()
    DATA_MODIFICATION = auto()
    DATA_DELETION = auto()
    SENSITIVE_DATA_ACCESS = auto()
    
    # API events
    API_ACCESS = auto()
    API_KEY_CREATED = auto()
    API_KEY_DELETED = auto()
    API_RATE_LIMIT_EXCEEDED = auto()
    
    # System events
    SYSTEM_STARTUP = auto()
    SYSTEM_SHUTDOWN = auto()
    CONFIGURATION_CHANGE = auto()
    
    # Security events
    SECURITY_SCAN = auto()
    VULNERABILITY_DETECTED = auto()
    ATTACK_DETECTED = auto()
    SUSPICIOUS_ACTIVITY = auto()
    BRUTE_FORCE_ATTEMPT = auto()
    
    # File events
    FILE_UPLOAD = auto()
    FILE_DOWNLOAD = auto()
    FILE_DELETION = auto()
    
    # Drone events
    DRONE_COMMAND = auto()
    DRONE_TELEMETRY = auto()
    DRONE_MISSION_START = auto()
    DRONE_MISSION_END = auto()
    DRONE_GEOFENCE_VIOLATION = auto()
    
    # Other events
    OTHER = auto()


class SecuritySeverity(Enum):
    """
    Severity levels for security events.
    """
    
    DEBUG = "debug"
    INFO = "info"
    WARNING = "warning"
    ERROR = "error"
    CRITICAL = "critical"


class SecurityLogger:
    """
    Logger for security events.
    """
    
    def __init__(
        self,
        name: str = "security",
        log_file: Optional[str] = None,
        audit_log_file: Optional[str] = None,
        console_output: bool = True,
        json_format: bool = True
    ):
        """
        Initialize the security logger.
        
        Args:
            name: Logger name
            log_file: Path to log file
            audit_log_file: Path to audit log file
            console_output: Whether to output logs to console
            json_format: Whether to format logs as JSON
        """
        self.logger = get_secure_logger(
            name=name,
            log_file=log_file,
            console_output=console_output,
            json_format=json_format
        )
        
        self.audit_logger = AuditLogger(
            name=f"{name}_audit",
            log_file=audit_log_file,
            console_output=False,
            json_format=True
        )
    
    def log_security_event(
        self,
        event_type: SecurityEventType,
        severity: SecuritySeverity = SecuritySeverity.INFO,
        user_id: Optional[str] = None,
        username: Optional[str] = None,
        resource_type: Optional[str] = None,
        resource_id: Optional[str] = None,
        action: Optional[str] = None,
        status: Optional[str] = None,
        details: Optional[Dict[str, Any]] = None,
        ip_address: Optional[str] = None,
        user_agent: Optional[str] = None,
        correlation_id: Optional[str] = None,
        session_id: Optional[str] = None
    ):
        """
        Log a security event.
        
        Args:
            event_type: Type of security event
            severity: Severity level of the event
            user_id: ID of the user who performed the action
            username: Username of the user who performed the action
            resource_type: Type of resource affected
            resource_id: ID of resource affected
            action: Action performed
            status: Status of the action
            details: Additional details
            ip_address: IP address of the user
            user_agent: User agent of the user
            correlation_id: Correlation ID for tracking related events
            session_id: Session ID for tracking user sessions
        """
        # Generate a correlation ID if not provided
        if not correlation_id:
            correlation_id = str(uuid.uuid4())
        
        # Create event data
        event_data = {
            "event_id": str(uuid.uuid4()),
            "event_type": event_type.name,
            "severity": severity.value,
            "timestamp": datetime.now(timezone.utc).isoformat(),
            "correlation_id": correlation_id
        }
        
        # Add optional fields
        if user_id:
            event_data["user_id"] = user_id
        
        if username:
            event_data["username"] = username
        
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
        
        if session_id:
            event_data["session_id"] = session_id
        
        # Log to appropriate logger based on severity
        log_method = getattr(self.logger, severity.value)
        log_method(f"Security event: {event_type.name}", event_data)
        
        # Log to audit logger
        self.audit_logger.log_event(
            event_type=event_type.name,
            user_id=user_id,
            resource_type=resource_type,
            resource_id=resource_id,
            action=action,
            status=status,
            details=event_data,
            ip_address=ip_address,
            user_agent=user_agent
        )
    
    def log_authentication_event(
        self,
        event_type: SecurityEventType,
        success: bool,
        user_id: Optional[str] = None,
        username: Optional[str] = None,
        details: Optional[Dict[str, Any]] = None,
        ip_address: Optional[str] = None,
        user_agent: Optional[str] = None,
        correlation_id: Optional[str] = None,
        session_id: Optional[str] = None
    ):
        """
        Log an authentication event.
        
        Args:
            event_type: Type of authentication event
            success: Whether the authentication was successful
            user_id: ID of the user
            username: Username of the user
            details: Additional details
            ip_address: IP address of the user
            user_agent: User agent of the user
            correlation_id: Correlation ID for tracking related events
            session_id: Session ID for tracking user sessions
        """
        # Determine severity based on success
        severity = SecuritySeverity.INFO if success else SecuritySeverity.WARNING
        
        # Determine status based on success
        status = "success" if success else "failure"
        
        # Log the event
        self.log_security_event(
            event_type=event_type,
            severity=severity,
            user_id=user_id,
            username=username,
            resource_type="authentication",
            action=event_type.name.lower(),
            status=status,
            details=details,
            ip_address=ip_address,
            user_agent=user_agent,
            correlation_id=correlation_id,
            session_id=session_id
        )
    
    def log_authorization_event(
        self,
        event_type: SecurityEventType,
        success: bool,
        user_id: str,
        resource_type: str,
        resource_id: Optional[str] = None,
        permission: Optional[str] = None,
        details: Optional[Dict[str, Any]] = None,
        ip_address: Optional[str] = None,
        correlation_id: Optional[str] = None,
        session_id: Optional[str] = None
    ):
        """
        Log an authorization event.
        
        Args:
            event_type: Type of authorization event
            success: Whether the authorization was successful
            user_id: ID of the user
            resource_type: Type of resource
            resource_id: ID of resource
            permission: Permission requested
            details: Additional details
            ip_address: IP address of the user
            correlation_id: Correlation ID for tracking related events
            session_id: Session ID for tracking user sessions
        """
        # Determine severity based on success
        severity = SecuritySeverity.INFO if success else SecuritySeverity.WARNING
        
        # Determine status based on success
        status = "success" if success else "failure"
        
        # Create details if not provided
        if not details:
            details = {}
        
        # Add permission to details
        if permission:
            details["permission"] = permission
        
        # Log the event
        self.log_security_event(
            event_type=event_type,
            severity=severity,
            user_id=user_id,
            resource_type=resource_type,
            resource_id=resource_id,
            action=event_type.name.lower(),
            status=status,
            details=details,
            ip_address=ip_address,
            correlation_id=correlation_id,
            session_id=session_id
        )
    
    def log_data_access_event(
        self,
        event_type: SecurityEventType,
        user_id: str,
        resource_type: str,
        resource_id: str,
        action: str,
        success: bool,
        sensitive: bool = False,
        details: Optional[Dict[str, Any]] = None,
        ip_address: Optional[str] = None,
        correlation_id: Optional[str] = None,
        session_id: Optional[str] = None
    ):
        """
        Log a data access event.
        
        Args:
            event_type: Type of data access event
            user_id: ID of the user
            resource_type: Type of resource
            resource_id: ID of resource
            action: Action performed
            success: Whether the access was successful
            sensitive: Whether the data is sensitive
            details: Additional details
            ip_address: IP address of the user
            correlation_id: Correlation ID for tracking related events
            session_id: Session ID for tracking user sessions
        """
        # Determine severity based on success and sensitivity
        if not success:
            severity = SecuritySeverity.WARNING
        elif sensitive:
            severity = SecuritySeverity.INFO
        else:
            severity = SecuritySeverity.DEBUG
        
        # Determine status based on success
        status = "success" if success else "failure"
        
        # Create details if not provided
        if not details:
            details = {}
        
        # Add sensitivity to details
        details["sensitive"] = sensitive
        
        # Log the event
        self.log_security_event(
            event_type=event_type,
            severity=severity,
            user_id=user_id,
            resource_type=resource_type,
            resource_id=resource_id,
            action=action,
            status=status,
            details=details,
            ip_address=ip_address,
            correlation_id=correlation_id,
            session_id=session_id
        )
    
    def log_attack_event(
        self,
        event_type: SecurityEventType,
        attack_type: str,
        severity: SecuritySeverity = SecuritySeverity.WARNING,
        user_id: Optional[str] = None,
        ip_address: Optional[str] = None,
        user_agent: Optional[str] = None,
        details: Optional[Dict[str, Any]] = None,
        correlation_id: Optional[str] = None
    ):
        """
        Log an attack event.
        
        Args:
            event_type: Type of attack event
            attack_type: Type of attack
            severity: Severity level of the attack
            user_id: ID of the user (if known)
            ip_address: IP address of the attacker
            user_agent: User agent of the attacker
            details: Additional details
            correlation_id: Correlation ID for tracking related events
        """
        # Create details if not provided
        if not details:
            details = {}
        
        # Add attack type to details
        details["attack_type"] = attack_type
        
        # Log the event
        self.log_security_event(
            event_type=event_type,
            severity=severity,
            user_id=user_id,
            resource_type="security",
            action="attack",
            status="detected",
            details=details,
            ip_address=ip_address,
            user_agent=user_agent,
            correlation_id=correlation_id
        )
    
    def log_system_event(
        self,
        event_type: SecurityEventType,
        details: Optional[Dict[str, Any]] = None,
        severity: SecuritySeverity = SecuritySeverity.INFO,
        correlation_id: Optional[str] = None
    ):
        """
        Log a system event.
        
        Args:
            event_type: Type of system event
            details: Additional details
            severity: Severity level of the event
            correlation_id: Correlation ID for tracking related events
        """
        # Log the event
        self.log_security_event(
            event_type=event_type,
            severity=severity,
            resource_type="system",
            action=event_type.name.lower(),
            details=details,
            correlation_id=correlation_id
        )


# Create default instance
security_logger = SecurityLogger()
