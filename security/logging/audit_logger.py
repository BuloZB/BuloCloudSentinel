"""
Audit logger for Bulo.Cloud Sentinel Security Module.

This module provides audit logging for security-relevant events.
"""

import json
import logging
import os
import time
import uuid
from datetime import datetime
from typing import Any, Dict, List, Optional, Union

import structlog
from fastapi import Depends, Request
from pydantic import BaseModel

# Configure structlog
structlog.configure(
    processors=[
        structlog.processors.TimeStamper(fmt="iso"),
        structlog.processors.JSONRenderer()
    ],
    logger_factory=structlog.stdlib.LoggerFactory(),
    wrapper_class=structlog.stdlib.BoundLogger,
    cache_logger_on_first_use=True,
)

# Create logger
logger = structlog.get_logger("audit")

# Event types
class EventType:
    """Event types for audit logging."""
    AUTH_SUCCESS = "auth_success"
    AUTH_FAILURE = "auth_failure"
    LOGOUT = "logout"
    PASSWORD_CHANGE = "password_change"
    PASSWORD_RESET = "password_reset"
    USER_CREATE = "user_create"
    USER_UPDATE = "user_update"
    USER_DELETE = "user_delete"
    ROLE_CHANGE = "role_change"
    PERMISSION_CHANGE = "permission_change"
    DATA_ACCESS = "data_access"
    DATA_MODIFY = "data_modify"
    DATA_DELETE = "data_delete"
    ADMIN_ACTION = "admin_action"
    SECURITY_ALERT = "security_alert"
    SYSTEM_ERROR = "system_error"
    API_ACCESS = "api_access"
    CONFIG_CHANGE = "config_change"

# Severity levels
class SeverityLevel:
    """Severity levels for audit logging."""
    INFO = "info"
    WARNING = "warning"
    ERROR = "error"
    CRITICAL = "critical"

# Models
class AuditEvent(BaseModel):
    """Audit event model."""
    id: str
    timestamp: str
    event_type: str
    severity: str
    user_id: Optional[str] = None
    username: Optional[str] = None
    ip_address: Optional[str] = None
    user_agent: Optional[str] = None
    resource: Optional[str] = None
    action: Optional[str] = None
    status: Optional[str] = None
    details: Optional[Dict[str, Any]] = None
    metadata: Optional[Dict[str, Any]] = None

def log_audit_event(
    event_type: str,
    severity: str = SeverityLevel.INFO,
    user_id: Optional[str] = None,
    username: Optional[str] = None,
    ip_address: Optional[str] = None,
    user_agent: Optional[str] = None,
    resource: Optional[str] = None,
    action: Optional[str] = None,
    status: Optional[str] = None,
    details: Optional[Dict[str, Any]] = None,
    metadata: Optional[Dict[str, Any]] = None
) -> str:
    """
    Log an audit event.
    
    Args:
        event_type: Type of event
        severity: Severity level
        user_id: User ID
        username: Username
        ip_address: IP address
        user_agent: User agent
        resource: Resource being accessed
        action: Action being performed
        status: Status of the action
        details: Additional details
        metadata: Additional metadata
        
    Returns:
        Event ID
    """
    # Generate event ID
    event_id = str(uuid.uuid4())
    
    # Create event
    event = AuditEvent(
        id=event_id,
        timestamp=datetime.utcnow().isoformat(),
        event_type=event_type,
        severity=severity,
        user_id=user_id,
        username=username,
        ip_address=ip_address,
        user_agent=user_agent,
        resource=resource,
        action=action,
        status=status,
        details=details,
        metadata=metadata
    )
    
    # Log event
    logger.info(
        event_type,
        event_id=event_id,
        severity=severity,
        user_id=user_id,
        username=username,
        ip_address=ip_address,
        user_agent=user_agent,
        resource=resource,
        action=action,
        status=status,
        details=details,
        metadata=metadata
    )
    
    return event_id

def log_auth_success(
    user_id: str,
    username: str,
    ip_address: Optional[str] = None,
    user_agent: Optional[str] = None,
    details: Optional[Dict[str, Any]] = None
) -> str:
    """
    Log a successful authentication event.
    
    Args:
        user_id: User ID
        username: Username
        ip_address: IP address
        user_agent: User agent
        details: Additional details
        
    Returns:
        Event ID
    """
    return log_audit_event(
        event_type=EventType.AUTH_SUCCESS,
        severity=SeverityLevel.INFO,
        user_id=user_id,
        username=username,
        ip_address=ip_address,
        user_agent=user_agent,
        action="login",
        status="success",
        details=details
    )

def log_auth_failure(
    username: str,
    ip_address: Optional[str] = None,
    user_agent: Optional[str] = None,
    reason: Optional[str] = None,
    details: Optional[Dict[str, Any]] = None
) -> str:
    """
    Log a failed authentication event.
    
    Args:
        username: Username
        ip_address: IP address
        user_agent: User agent
        reason: Reason for failure
        details: Additional details
        
    Returns:
        Event ID
    """
    return log_audit_event(
        event_type=EventType.AUTH_FAILURE,
        severity=SeverityLevel.WARNING,
        username=username,
        ip_address=ip_address,
        user_agent=user_agent,
        action="login",
        status="failure",
        details={"reason": reason, **(details or {})}
    )

def log_logout(
    user_id: str,
    username: str,
    ip_address: Optional[str] = None,
    user_agent: Optional[str] = None
) -> str:
    """
    Log a logout event.
    
    Args:
        user_id: User ID
        username: Username
        ip_address: IP address
        user_agent: User agent
        
    Returns:
        Event ID
    """
    return log_audit_event(
        event_type=EventType.LOGOUT,
        severity=SeverityLevel.INFO,
        user_id=user_id,
        username=username,
        ip_address=ip_address,
        user_agent=user_agent,
        action="logout",
        status="success"
    )

def log_password_change(
    user_id: str,
    username: str,
    ip_address: Optional[str] = None,
    user_agent: Optional[str] = None,
    initiated_by: Optional[str] = None
) -> str:
    """
    Log a password change event.
    
    Args:
        user_id: User ID
        username: Username
        ip_address: IP address
        user_agent: User agent
        initiated_by: User who initiated the change
        
    Returns:
        Event ID
    """
    return log_audit_event(
        event_type=EventType.PASSWORD_CHANGE,
        severity=SeverityLevel.INFO,
        user_id=user_id,
        username=username,
        ip_address=ip_address,
        user_agent=user_agent,
        action="password_change",
        status="success",
        details={"initiated_by": initiated_by or user_id}
    )

def log_password_reset(
    user_id: str,
    username: str,
    ip_address: Optional[str] = None,
    user_agent: Optional[str] = None,
    initiated_by: Optional[str] = None
) -> str:
    """
    Log a password reset event.
    
    Args:
        user_id: User ID
        username: Username
        ip_address: IP address
        user_agent: User agent
        initiated_by: User who initiated the reset
        
    Returns:
        Event ID
    """
    return log_audit_event(
        event_type=EventType.PASSWORD_RESET,
        severity=SeverityLevel.INFO,
        user_id=user_id,
        username=username,
        ip_address=ip_address,
        user_agent=user_agent,
        action="password_reset",
        status="success",
        details={"initiated_by": initiated_by or user_id}
    )

def log_user_create(
    created_user_id: str,
    created_username: str,
    creator_id: str,
    creator_username: str,
    ip_address: Optional[str] = None,
    user_agent: Optional[str] = None,
    roles: Optional[List[str]] = None
) -> str:
    """
    Log a user creation event.
    
    Args:
        created_user_id: Created user ID
        created_username: Created username
        creator_id: Creator user ID
        creator_username: Creator username
        ip_address: IP address
        user_agent: User agent
        roles: Assigned roles
        
    Returns:
        Event ID
    """
    return log_audit_event(
        event_type=EventType.USER_CREATE,
        severity=SeverityLevel.INFO,
        user_id=creator_id,
        username=creator_username,
        ip_address=ip_address,
        user_agent=user_agent,
        resource=f"user/{created_user_id}",
        action="create",
        status="success",
        details={
            "created_user_id": created_user_id,
            "created_username": created_username,
            "roles": roles
        }
    )

def log_user_update(
    updated_user_id: str,
    updated_username: str,
    updater_id: str,
    updater_username: str,
    ip_address: Optional[str] = None,
    user_agent: Optional[str] = None,
    changes: Optional[Dict[str, Any]] = None
) -> str:
    """
    Log a user update event.
    
    Args:
        updated_user_id: Updated user ID
        updated_username: Updated username
        updater_id: Updater user ID
        updater_username: Updater username
        ip_address: IP address
        user_agent: User agent
        changes: Changes made
        
    Returns:
        Event ID
    """
    return log_audit_event(
        event_type=EventType.USER_UPDATE,
        severity=SeverityLevel.INFO,
        user_id=updater_id,
        username=updater_username,
        ip_address=ip_address,
        user_agent=user_agent,
        resource=f"user/{updated_user_id}",
        action="update",
        status="success",
        details={
            "updated_user_id": updated_user_id,
            "updated_username": updated_username,
            "changes": changes
        }
    )

def log_user_delete(
    deleted_user_id: str,
    deleted_username: str,
    deleter_id: str,
    deleter_username: str,
    ip_address: Optional[str] = None,
    user_agent: Optional[str] = None,
    reason: Optional[str] = None
) -> str:
    """
    Log a user deletion event.
    
    Args:
        deleted_user_id: Deleted user ID
        deleted_username: Deleted username
        deleter_id: Deleter user ID
        deleter_username: Deleter username
        ip_address: IP address
        user_agent: User agent
        reason: Reason for deletion
        
    Returns:
        Event ID
    """
    return log_audit_event(
        event_type=EventType.USER_DELETE,
        severity=SeverityLevel.WARNING,
        user_id=deleter_id,
        username=deleter_username,
        ip_address=ip_address,
        user_agent=user_agent,
        resource=f"user/{deleted_user_id}",
        action="delete",
        status="success",
        details={
            "deleted_user_id": deleted_user_id,
            "deleted_username": deleted_username,
            "reason": reason
        }
    )

def log_role_change(
    target_user_id: str,
    target_username: str,
    changer_id: str,
    changer_username: str,
    ip_address: Optional[str] = None,
    user_agent: Optional[str] = None,
    old_roles: Optional[List[str]] = None,
    new_roles: Optional[List[str]] = None,
    reason: Optional[str] = None
) -> str:
    """
    Log a role change event.
    
    Args:
        target_user_id: Target user ID
        target_username: Target username
        changer_id: Changer user ID
        changer_username: Changer username
        ip_address: IP address
        user_agent: User agent
        old_roles: Old roles
        new_roles: New roles
        reason: Reason for change
        
    Returns:
        Event ID
    """
    return log_audit_event(
        event_type=EventType.ROLE_CHANGE,
        severity=SeverityLevel.WARNING,
        user_id=changer_id,
        username=changer_username,
        ip_address=ip_address,
        user_agent=user_agent,
        resource=f"user/{target_user_id}/roles",
        action="update",
        status="success",
        details={
            "target_user_id": target_user_id,
            "target_username": target_username,
            "old_roles": old_roles,
            "new_roles": new_roles,
            "reason": reason
        }
    )

def log_data_access(
    user_id: str,
    username: str,
    ip_address: Optional[str] = None,
    user_agent: Optional[str] = None,
    resource: str,
    resource_id: Optional[str] = None,
    access_type: str = "read"
) -> str:
    """
    Log a data access event.
    
    Args:
        user_id: User ID
        username: Username
        ip_address: IP address
        user_agent: User agent
        resource: Resource type
        resource_id: Resource ID
        access_type: Type of access
        
    Returns:
        Event ID
    """
    resource_path = f"{resource}/{resource_id}" if resource_id else resource
    
    return log_audit_event(
        event_type=EventType.DATA_ACCESS,
        severity=SeverityLevel.INFO,
        user_id=user_id,
        username=username,
        ip_address=ip_address,
        user_agent=user_agent,
        resource=resource_path,
        action=access_type,
        status="success"
    )

def log_security_alert(
    alert_type: str,
    severity: str,
    ip_address: Optional[str] = None,
    user_agent: Optional[str] = None,
    user_id: Optional[str] = None,
    username: Optional[str] = None,
    resource: Optional[str] = None,
    details: Optional[Dict[str, Any]] = None
) -> str:
    """
    Log a security alert event.
    
    Args:
        alert_type: Type of alert
        severity: Severity level
        ip_address: IP address
        user_agent: User agent
        user_id: User ID
        username: Username
        resource: Resource
        details: Additional details
        
    Returns:
        Event ID
    """
    return log_audit_event(
        event_type=EventType.SECURITY_ALERT,
        severity=severity,
        user_id=user_id,
        username=username,
        ip_address=ip_address,
        user_agent=user_agent,
        resource=resource,
        action=alert_type,
        status="alert",
        details=details
    )

def log_api_access(
    request: Request,
    user_id: Optional[str] = None,
    username: Optional[str] = None,
    status_code: int = 200,
    response_time: Optional[float] = None,
    error: Optional[str] = None
) -> str:
    """
    Log an API access event.
    
    Args:
        request: FastAPI request
        user_id: User ID
        username: Username
        status_code: HTTP status code
        response_time: Response time in seconds
        error: Error message
        
    Returns:
        Event ID
    """
    # Get request details
    method = request.method
    path = request.url.path
    query_params = dict(request.query_params)
    headers = dict(request.headers)
    
    # Remove sensitive headers
    if "authorization" in headers:
        headers["authorization"] = "REDACTED"
    
    if "cookie" in headers:
        headers["cookie"] = "REDACTED"
    
    # Determine status
    status = "success" if status_code < 400 else "failure"
    
    # Determine severity
    severity = SeverityLevel.INFO
    if 400 <= status_code < 500:
        severity = SeverityLevel.WARNING
    elif status_code >= 500:
        severity = SeverityLevel.ERROR
    
    return log_audit_event(
        event_type=EventType.API_ACCESS,
        severity=severity,
        user_id=user_id,
        username=username,
        ip_address=request.client.host if request.client else None,
        user_agent=headers.get("user-agent"),
        resource=path,
        action=method,
        status=status,
        details={
            "status_code": status_code,
            "query_params": query_params,
            "response_time": response_time,
            "error": error
        },
        metadata={
            "headers": headers
        }
    )
