"""
Security monitoring API for Bulo.Cloud Sentinel.

This module provides API endpoints for security monitoring and alerting.
"""

import logging
from typing import Dict, List, Optional, Any, Union
from fastapi import APIRouter, Depends, HTTPException, status, Security, Query
from pydantic import BaseModel, Field

from ..auth.jwt_handler import has_role, has_permission
from .security_events import (
    SecurityEvent,
    SecurityEventSeverity,
    SecurityEventType,
    emit_event,
    add_webhook_handler as add_event_webhook_handler
)
from .alerting import (
    SecurityAlert,
    AlertStatus,
    create_alert,
    get_alert,
    update_alert_status,
    assign_alert,
    add_alert_comment,
    add_webhook_handler as add_alert_webhook_handler
)

# Configure logging
logger = logging.getLogger(__name__)

# Create router
router = APIRouter(
    prefix="/security",
    tags=["security"]
)


# Models
class SecurityEventCreate(BaseModel):
    """Model for creating a security event."""
    event_type: str = Field(..., description="Event type")
    message: str = Field(..., description="Event message")
    severity: str = Field("info", description="Event severity")
    source: Optional[str] = Field(None, description="Event source")
    details: Optional[Dict[str, Any]] = Field(None, description="Event details")


class SecurityAlertCreate(BaseModel):
    """Model for creating a security alert."""
    title: str = Field(..., description="Alert title")
    description: str = Field(..., description="Alert description")
    severity: str = Field("medium", description="Alert severity")
    source: Optional[str] = Field(None, description="Alert source")
    details: Optional[Dict[str, Any]] = Field(None, description="Alert details")


class SecurityAlertUpdate(BaseModel):
    """Model for updating a security alert."""
    status: str = Field(..., description="Alert status")
    comment: Optional[str] = Field(None, description="Comment")
    user: Optional[str] = Field(None, description="User making the update")


class SecurityAlertAssign(BaseModel):
    """Model for assigning a security alert."""
    user: str = Field(..., description="User to assign the alert to")
    comment: Optional[str] = Field(None, description="Comment")


class SecurityAlertComment(BaseModel):
    """Model for adding a comment to a security alert."""
    comment: str = Field(..., description="Comment text")
    user: Optional[str] = Field(None, description="User adding the comment")


class WebhookConfig(BaseModel):
    """Model for configuring a webhook."""
    webhook_url: str = Field(..., description="Webhook URL")
    name: str = Field(..., description="Webhook name")
    min_severity: str = Field("medium", description="Minimum severity to send")
    headers: Optional[Dict[str, str]] = Field(None, description="HTTP headers")


# Endpoints
@router.post("/events", status_code=status.HTTP_201_CREATED)
async def create_security_event(
    event: SecurityEventCreate,
    current_user: str = Security(has_permission("security:create_event"))
) -> Dict[str, Any]:
    """
    Create a security event.
    
    Args:
        event: Security event data
        current_user: Current user
        
    Returns:
        Created event
    """
    try:
        # Emit event
        emit_event(
            event_type=event.event_type,
            message=event.message,
            severity=event.severity,
            source=event.source,
            details=event.details
        )
        
        return {
            "success": True,
            "message": "Security event created"
        }
    except Exception as e:
        logger.error(f"Error creating security event: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error creating security event: {str(e)}"
        )


@router.post("/alerts", status_code=status.HTTP_201_CREATED)
async def create_security_alert(
    alert: SecurityAlertCreate,
    current_user: str = Security(has_permission("security:create_alert"))
) -> Dict[str, Any]:
    """
    Create a security alert.
    
    Args:
        alert: Security alert data
        current_user: Current user
        
    Returns:
        Created alert
    """
    try:
        # Create alert
        created_alert = create_alert(
            title=alert.title,
            description=alert.description,
            severity=alert.severity,
            source=alert.source,
            details=alert.details
        )
        
        return {
            "success": True,
            "message": "Security alert created",
            "alert_id": created_alert.alert_id
        }
    except Exception as e:
        logger.error(f"Error creating security alert: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error creating security alert: {str(e)}"
        )


@router.get("/alerts")
async def list_security_alerts(
    status: Optional[str] = Query(None, description="Filter by status"),
    severity: Optional[str] = Query(None, description="Filter by severity"),
    assigned_to: Optional[str] = Query(None, description="Filter by assigned user"),
    current_user: str = Security(has_permission("security:view_alerts"))
) -> List[Dict[str, Any]]:
    """
    List security alerts.
    
    Args:
        status: Filter by status
        severity: Filter by severity
        assigned_to: Filter by assigned user
        current_user: Current user
        
    Returns:
        List of security alerts
    """
    try:
        from .alerting import alert_manager
        
        # Get all alerts
        alerts = list(alert_manager.alerts.values())
        
        # Apply filters
        if status:
            alerts = [a for a in alerts if a.status.value == status]
        
        if severity:
            alerts = [a for a in alerts if a.severity.value == severity]
        
        if assigned_to:
            alerts = [a for a in alerts if a.assigned_to == assigned_to]
        
        # Convert to dictionaries
        return [alert.to_dict() for alert in alerts]
    
    except Exception as e:
        logger.error(f"Error listing security alerts: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error listing security alerts: {str(e)}"
        )


@router.get("/alerts/{alert_id}")
async def get_security_alert(
    alert_id: str,
    current_user: str = Security(has_permission("security:view_alerts"))
) -> Dict[str, Any]:
    """
    Get a security alert.
    
    Args:
        alert_id: Alert ID
        current_user: Current user
        
    Returns:
        Security alert
    """
    try:
        # Get alert
        alert = get_alert(alert_id)
        if not alert:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail=f"Alert not found: {alert_id}"
            )
        
        return alert.to_dict()
    
    except HTTPException:
        raise
    
    except Exception as e:
        logger.error(f"Error getting security alert: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error getting security alert: {str(e)}"
        )


@router.put("/alerts/{alert_id}/status")
async def update_security_alert_status(
    alert_id: str,
    update: SecurityAlertUpdate,
    current_user: str = Security(has_permission("security:update_alerts"))
) -> Dict[str, Any]:
    """
    Update security alert status.
    
    Args:
        alert_id: Alert ID
        update: Status update data
        current_user: Current user
        
    Returns:
        Success status
    """
    try:
        # Update alert status
        success = update_alert_status(
            alert_id=alert_id,
            status=AlertStatus(update.status),
            comment=update.comment,
            user=update.user or current_user
        )
        
        if not success:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail=f"Alert not found: {alert_id}"
            )
        
        return {
            "success": True,
            "message": f"Alert status updated to {update.status}"
        }
    
    except HTTPException:
        raise
    
    except Exception as e:
        logger.error(f"Error updating security alert status: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error updating security alert status: {str(e)}"
        )


@router.put("/alerts/{alert_id}/assign")
async def assign_security_alert(
    alert_id: str,
    assignment: SecurityAlertAssign,
    current_user: str = Security(has_permission("security:update_alerts"))
) -> Dict[str, Any]:
    """
    Assign security alert to a user.
    
    Args:
        alert_id: Alert ID
        assignment: Assignment data
        current_user: Current user
        
    Returns:
        Success status
    """
    try:
        # Assign alert
        success = assign_alert(
            alert_id=alert_id,
            user=assignment.user,
            comment=assignment.comment
        )
        
        if not success:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail=f"Alert not found: {alert_id}"
            )
        
        return {
            "success": True,
            "message": f"Alert assigned to {assignment.user}"
        }
    
    except HTTPException:
        raise
    
    except Exception as e:
        logger.error(f"Error assigning security alert: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error assigning security alert: {str(e)}"
        )


@router.post("/alerts/{alert_id}/comments")
async def add_security_alert_comment(
    alert_id: str,
    comment: SecurityAlertComment,
    current_user: str = Security(has_permission("security:update_alerts"))
) -> Dict[str, Any]:
    """
    Add a comment to a security alert.
    
    Args:
        alert_id: Alert ID
        comment: Comment data
        current_user: Current user
        
    Returns:
        Success status
    """
    try:
        # Add comment
        success = add_alert_comment(
            alert_id=alert_id,
            comment=comment.comment,
            user=comment.user or current_user
        )
        
        if not success:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail=f"Alert not found: {alert_id}"
            )
        
        return {
            "success": True,
            "message": "Comment added to alert"
        }
    
    except HTTPException:
        raise
    
    except Exception as e:
        logger.error(f"Error adding comment to security alert: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error adding comment to security alert: {str(e)}"
        )


@router.post("/webhooks/events")
async def configure_event_webhook(
    config: WebhookConfig,
    current_user: str = Security(has_permission("security:configure_webhooks"))
) -> Dict[str, Any]:
    """
    Configure a webhook for security events.
    
    Args:
        config: Webhook configuration
        current_user: Current user
        
    Returns:
        Success status
    """
    try:
        # Add webhook handler
        add_event_webhook_handler(
            webhook_url=config.webhook_url,
            name=config.name,
            min_severity=config.min_severity,
            headers=config.headers
        )
        
        return {
            "success": True,
            "message": f"Event webhook '{config.name}' configured"
        }
    
    except Exception as e:
        logger.error(f"Error configuring event webhook: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error configuring event webhook: {str(e)}"
        )


@router.post("/webhooks/alerts")
async def configure_alert_webhook(
    config: WebhookConfig,
    current_user: str = Security(has_permission("security:configure_webhooks"))
) -> Dict[str, Any]:
    """
    Configure a webhook for security alerts.
    
    Args:
        config: Webhook configuration
        current_user: Current user
        
    Returns:
        Success status
    """
    try:
        # Add webhook handler
        add_alert_webhook_handler(
            webhook_url=config.webhook_url,
            name=config.name,
            min_severity=config.min_severity,
            headers=config.headers
        )
        
        return {
            "success": True,
            "message": f"Alert webhook '{config.name}' configured"
        }
    
    except Exception as e:
        logger.error(f"Error configuring alert webhook: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error configuring alert webhook: {str(e)}"
        )
