"""
Security alerting for Bulo.Cloud Sentinel.

This module provides functionality for generating and sending security alerts.
"""

import logging
import json
import time
import threading
import queue
from typing import Dict, List, Optional, Any, Callable, Union
from datetime import datetime
from enum import Enum

from .security_events import (
    SecurityEvent,
    SecurityEventSeverity,
    SecurityEventType,
    SecurityEventHandler,
    emit_event
)

# Configure logging
logger = logging.getLogger(__name__)


class AlertStatus(str, Enum):
    """Alert status."""
    NEW = "new"
    ACKNOWLEDGED = "acknowledged"
    IN_PROGRESS = "in_progress"
    RESOLVED = "resolved"
    CLOSED = "closed"
    FALSE_POSITIVE = "false_positive"


class SecurityAlert:
    """
    Security alert for notification and response.
    
    This class represents a security alert that requires attention.
    """
    
    def __init__(
        self,
        title: str,
        description: str,
        severity: Union[SecurityEventSeverity, str] = SecurityEventSeverity.MEDIUM,
        source: Optional[str] = None,
        details: Optional[Dict[str, Any]] = None,
        timestamp: Optional[float] = None,
        alert_id: Optional[str] = None,
        status: AlertStatus = AlertStatus.NEW,
        assigned_to: Optional[str] = None,
        events: Optional[List[SecurityEvent]] = None
    ):
        """
        Initialize security alert.
        
        Args:
            title: Alert title
            description: Alert description
            severity: Alert severity
            source: Alert source
            details: Alert details
            timestamp: Alert timestamp (default: current time)
            alert_id: Alert ID (default: generated)
            status: Alert status
            assigned_to: User assigned to the alert
            events: Related security events
        """
        self.title = title
        self.description = description
        self.severity = severity if isinstance(severity, SecurityEventSeverity) else SecurityEventSeverity(severity)
        self.source = source or "security_alerting"
        self.details = details or {}
        self.timestamp = timestamp or time.time()
        self.alert_id = alert_id or f"alert-{int(self.timestamp)}-{hash(self.title) % 10000:04d}"
        self.status = status
        self.assigned_to = assigned_to
        self.events = events or []
        self.updates: List[Dict[str, Any]] = []
    
    def add_event(self, event: SecurityEvent):
        """
        Add a related security event.
        
        Args:
            event: Security event
        """
        self.events.append(event)
    
    def update_status(self, status: AlertStatus, comment: Optional[str] = None, user: Optional[str] = None):
        """
        Update alert status.
        
        Args:
            status: New status
            comment: Optional comment
            user: User making the update
        """
        old_status = self.status
        self.status = status
        
        # Add update to history
        update = {
            "timestamp": time.time(),
            "user": user or "system",
            "old_status": old_status,
            "new_status": status,
            "comment": comment
        }
        
        self.updates.append(update)
        
        # Emit security event
        emit_event(
            event_type=SecurityEventType.SYSTEM,
            message=f"Alert {self.alert_id} status updated from {old_status} to {status}",
            severity=SecurityEventSeverity.INFO,
            source="security_alerting",
            details={
                "alert_id": self.alert_id,
                "old_status": old_status,
                "new_status": status,
                "user": user or "system",
                "comment": comment
            }
        )
    
    def assign(self, user: str, comment: Optional[str] = None):
        """
        Assign alert to a user.
        
        Args:
            user: User to assign the alert to
            comment: Optional comment
        """
        old_assigned_to = self.assigned_to
        self.assigned_to = user
        
        # Add update to history
        update = {
            "timestamp": time.time(),
            "user": user,
            "old_assigned_to": old_assigned_to,
            "new_assigned_to": user,
            "comment": comment
        }
        
        self.updates.append(update)
        
        # Emit security event
        emit_event(
            event_type=SecurityEventType.SYSTEM,
            message=f"Alert {self.alert_id} assigned to {user}",
            severity=SecurityEventSeverity.INFO,
            source="security_alerting",
            details={
                "alert_id": self.alert_id,
                "old_assigned_to": old_assigned_to,
                "new_assigned_to": user,
                "comment": comment
            }
        )
    
    def add_comment(self, comment: str, user: Optional[str] = None):
        """
        Add a comment to the alert.
        
        Args:
            comment: Comment text
            user: User adding the comment
        """
        # Add update to history
        update = {
            "timestamp": time.time(),
            "user": user or "system",
            "comment": comment
        }
        
        self.updates.append(update)
    
    def to_dict(self) -> Dict[str, Any]:
        """
        Convert alert to dictionary.
        
        Returns:
            Alert as dictionary
        """
        return {
            "alert_id": self.alert_id,
            "title": self.title,
            "description": self.description,
            "severity": self.severity.value,
            "source": self.source,
            "details": self.details,
            "timestamp": self.timestamp,
            "timestamp_iso": datetime.fromtimestamp(self.timestamp).isoformat(),
            "status": self.status.value,
            "assigned_to": self.assigned_to,
            "events": [event.to_dict() for event in self.events],
            "updates": self.updates
        }
    
    def to_json(self) -> str:
        """
        Convert alert to JSON.
        
        Returns:
            Alert as JSON string
        """
        return json.dumps(self.to_dict(), indent=2)
    
    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "SecurityAlert":
        """
        Create alert from dictionary.
        
        Args:
            data: Alert data
            
        Returns:
            Security alert
        """
        # Create alert
        alert = cls(
            title=data["title"],
            description=data["description"],
            severity=data["severity"],
            source=data["source"],
            details=data["details"],
            timestamp=data["timestamp"],
            alert_id=data["alert_id"],
            status=AlertStatus(data["status"]),
            assigned_to=data.get("assigned_to")
        )
        
        # Add events
        for event_data in data.get("events", []):
            alert.add_event(SecurityEvent.from_dict(event_data))
        
        # Add updates
        alert.updates = data.get("updates", [])
        
        return alert
    
    @classmethod
    def from_json(cls, json_str: str) -> "SecurityAlert":
        """
        Create alert from JSON.
        
        Args:
            json_str: Alert as JSON string
            
        Returns:
            Security alert
        """
        data = json.loads(json_str)
        return cls.from_dict(data)


class AlertHandler:
    """
    Handler for security alerts.
    
    This class handles security alerts by processing them and
    optionally forwarding them to other systems.
    """
    
    def __init__(
        self,
        name: str,
        min_severity: SecurityEventSeverity = SecurityEventSeverity.INFO
    ):
        """
        Initialize alert handler.
        
        Args:
            name: Handler name
            min_severity: Minimum severity to handle
        """
        self.name = name
        self.min_severity = min_severity
        
        logger.info(f"Initialized alert handler: {name}")
    
    def can_handle(self, alert: SecurityAlert) -> bool:
        """
        Check if handler can handle an alert.
        
        Args:
            alert: Security alert
            
        Returns:
            True if handler can handle alert, False otherwise
        """
        # Check severity
        if SecurityEventSeverity[self.min_severity.upper()].value > SecurityEventSeverity[alert.severity.upper()].value:
            return False
        
        return True
    
    def handle(self, alert: SecurityAlert):
        """
        Handle a security alert.
        
        Args:
            alert: Security alert
        """
        if not self.can_handle(alert):
            return
        
        try:
            self._handle_alert(alert)
        except Exception as e:
            logger.error(f"Error handling security alert in {self.name}: {str(e)}")
    
    def _handle_alert(self, alert: SecurityAlert):
        """
        Handle a security alert (to be implemented by subclasses).
        
        Args:
            alert: Security alert
        """
        raise NotImplementedError("Subclasses must implement _handle_alert")


class LoggingAlertHandler(AlertHandler):
    """Alert handler that logs alerts."""
    
    def __init__(
        self,
        name: str = "logging",
        min_severity: SecurityEventSeverity = SecurityEventSeverity.INFO,
        logger_name: str = "security_alerts"
    ):
        """
        Initialize logging alert handler.
        
        Args:
            name: Handler name
            min_severity: Minimum severity to handle
            logger_name: Logger name
        """
        super().__init__(name, min_severity)
        self.logger = logging.getLogger(logger_name)
    
    def _handle_alert(self, alert: SecurityAlert):
        """
        Handle a security alert by logging it.
        
        Args:
            alert: Security alert
        """
        # Map severity to log level
        log_level = {
            SecurityEventSeverity.INFO: logging.INFO,
            SecurityEventSeverity.LOW: logging.INFO,
            SecurityEventSeverity.MEDIUM: logging.WARNING,
            SecurityEventSeverity.HIGH: logging.ERROR,
            SecurityEventSeverity.CRITICAL: logging.CRITICAL
        }.get(alert.severity, logging.INFO)
        
        # Log alert
        self.logger.log(
            log_level,
            f"[ALERT] {alert.title}",
            extra={
                "alert_id": alert.alert_id,
                "title": alert.title,
                "description": alert.description,
                "severity": alert.severity.value,
                "source": alert.source,
                "status": alert.status.value,
                "assigned_to": alert.assigned_to,
                "timestamp": alert.timestamp,
                "timestamp_iso": datetime.fromtimestamp(alert.timestamp).isoformat()
            }
        )


class EmailAlertHandler(AlertHandler):
    """Alert handler that sends email notifications."""
    
    def __init__(
        self,
        smtp_server: str,
        smtp_port: int,
        sender: str,
        recipients: List[str],
        name: str = "email",
        min_severity: SecurityEventSeverity = SecurityEventSeverity.MEDIUM,
        use_ssl: bool = True,
        username: Optional[str] = None,
        password: Optional[str] = None
    ):
        """
        Initialize email alert handler.
        
        Args:
            smtp_server: SMTP server
            smtp_port: SMTP port
            sender: Sender email address
            recipients: Recipient email addresses
            name: Handler name
            min_severity: Minimum severity to handle
            use_ssl: Whether to use SSL
            username: SMTP username
            password: SMTP password
        """
        super().__init__(name, min_severity)
        self.smtp_server = smtp_server
        self.smtp_port = smtp_port
        self.sender = sender
        self.recipients = recipients
        self.use_ssl = use_ssl
        self.username = username
        self.password = password
        
        # Import email modules lazily
        try:
            import smtplib
            from email.mime.text import MIMEText
            from email.mime.multipart import MIMEMultipart
            
            self.smtplib = smtplib
            self.MIMEText = MIMEText
            self.MIMEMultipart = MIMEMultipart
        except ImportError:
            logger.error("email modules are required for EmailAlertHandler")
            raise ImportError("email modules are required for EmailAlertHandler")
    
    def _handle_alert(self, alert: SecurityAlert):
        """
        Handle a security alert by sending an email notification.
        
        Args:
            alert: Security alert
        """
        try:
            # Create message
            msg = self.MIMEMultipart()
            msg["From"] = self.sender
            msg["To"] = ", ".join(self.recipients)
            msg["Subject"] = f"[{alert.severity.value.upper()}] Security Alert: {alert.title}"
            
            # Create HTML body
            html = f"""
            <html>
            <head>
                <style>
                    body {{ font-family: Arial, sans-serif; }}
                    .alert {{ padding: 10px; border: 1px solid #ddd; border-radius: 5px; }}
                    .info {{ background-color: #d1ecf1; }}
                    .low {{ background-color: #d4edda; }}
                    .medium {{ background-color: #fff3cd; }}
                    .high {{ background-color: #f8d7da; }}
                    .critical {{ background-color: #dc3545; color: white; }}
                    .details {{ margin-top: 10px; }}
                    .events {{ margin-top: 10px; }}
                    .event {{ padding: 5px; border: 1px solid #ddd; margin-top: 5px; }}
                </style>
            </head>
            <body>
                <div class="alert {alert.severity.value}">
                    <h2>{alert.title}</h2>
                    <p><strong>ID:</strong> {alert.alert_id}</p>
                    <p><strong>Severity:</strong> {alert.severity.value.upper()}</p>
                    <p><strong>Source:</strong> {alert.source}</p>
                    <p><strong>Time:</strong> {datetime.fromtimestamp(alert.timestamp).isoformat()}</p>
                    <p><strong>Status:</strong> {alert.status.value}</p>
                    <p><strong>Assigned To:</strong> {alert.assigned_to or "Unassigned"}</p>
                    <p><strong>Description:</strong> {alert.description}</p>
                    
                    <div class="details">
                        <h3>Details</h3>
                        <pre>{json.dumps(alert.details, indent=2)}</pre>
                    </div>
                    
                    <div class="events">
                        <h3>Related Events ({len(alert.events)})</h3>
                        {"".join([f'<div class="event"><p><strong>Event ID:</strong> {event.event_id}</p><p><strong>Type:</strong> {event.event_type.value}</p><p><strong>Message:</strong> {event.message}</p></div>' for event in alert.events])}
                    </div>
                </div>
            </body>
            </html>
            """
            
            # Attach HTML body
            msg.attach(self.MIMEText(html, "html"))
            
            # Send email
            if self.use_ssl:
                server = self.smtplib.SMTP_SSL(self.smtp_server, self.smtp_port)
            else:
                server = self.smtplib.SMTP(self.smtp_server, self.smtp_port)
                server.starttls()
            
            if self.username and self.password:
                server.login(self.username, self.password)
            
            server.sendmail(self.sender, self.recipients, msg.as_string())
            server.quit()
            
            logger.info(f"Sent email notification for alert {alert.alert_id}")
        
        except Exception as e:
            logger.error(f"Error sending email notification for alert {alert.alert_id}: {str(e)}")


class WebhookAlertHandler(AlertHandler):
    """Alert handler that sends alerts to a webhook."""
    
    def __init__(
        self,
        webhook_url: str,
        name: str = "webhook",
        min_severity: SecurityEventSeverity = SecurityEventSeverity.MEDIUM,
        headers: Optional[Dict[str, str]] = None,
        timeout: int = 5
    ):
        """
        Initialize webhook alert handler.
        
        Args:
            webhook_url: Webhook URL
            name: Handler name
            min_severity: Minimum severity to handle
            headers: HTTP headers
            timeout: Request timeout in seconds
        """
        super().__init__(name, min_severity)
        self.webhook_url = webhook_url
        self.headers = headers or {"Content-Type": "application/json"}
        self.timeout = timeout
        
        # Import requests lazily
        try:
            import requests
            self.requests = requests
        except ImportError:
            logger.error("requests package is required for WebhookAlertHandler")
            raise ImportError("requests package is required for WebhookAlertHandler")
    
    def _handle_alert(self, alert: SecurityAlert):
        """
        Handle a security alert by sending it to a webhook.
        
        Args:
            alert: Security alert
        """
        try:
            response = self.requests.post(
                self.webhook_url,
                headers=self.headers,
                json=alert.to_dict(),
                timeout=self.timeout
            )
            
            if response.status_code >= 400:
                logger.error(f"Error sending security alert to webhook: {response.status_code} {response.text}")
            else:
                logger.debug(f"Sent security alert to webhook: {alert.alert_id}")
        
        except Exception as e:
            logger.error(f"Error sending security alert to webhook: {str(e)}")


class AlertManager:
    """
    Manager for security alerts.
    
    This class manages security alerts and forwards them to handlers.
    """
    
    def __init__(self, queue_size: int = 1000):
        """
        Initialize alert manager.
        
        Args:
            queue_size: Maximum queue size
        """
        self.handlers: List[AlertHandler] = []
        self.alerts: Dict[str, SecurityAlert] = {}
        self.alert_queue = queue.Queue(maxsize=queue_size)
        self.processing_thread: Optional[threading.Thread] = None
        self.stop_event = threading.Event()
        
        logger.info("Initialized alert manager")
    
    def add_handler(self, handler: AlertHandler):
        """
        Add an alert handler.
        
        Args:
            handler: Alert handler
        """
        self.handlers.append(handler)
        logger.info(f"Added alert handler: {handler.name}")
    
    def remove_handler(self, name: str):
        """
        Remove an alert handler.
        
        Args:
            name: Handler name
        """
        self.handlers = [h for h in self.handlers if h.name != name]
        logger.info(f"Removed alert handler: {name}")
    
    def create_alert(
        self,
        title: str,
        description: str,
        severity: Union[SecurityEventSeverity, str] = SecurityEventSeverity.MEDIUM,
        source: Optional[str] = None,
        details: Optional[Dict[str, Any]] = None,
        events: Optional[List[SecurityEvent]] = None
    ) -> SecurityAlert:
        """
        Create a security alert.
        
        Args:
            title: Alert title
            description: Alert description
            severity: Alert severity
            source: Alert source
            details: Alert details
            events: Related security events
            
        Returns:
            Security alert
        """
        # Create alert
        alert = SecurityAlert(
            title=title,
            description=description,
            severity=severity,
            source=source,
            details=details,
            events=events
        )
        
        # Store alert
        self.alerts[alert.alert_id] = alert
        
        # Queue alert for processing
        try:
            self.alert_queue.put(alert, block=False)
            logger.debug(f"Queued security alert: {alert.alert_id}")
        except queue.Full:
            logger.error("Alert queue is full, alert may not be processed")
        
        # Emit security event
        emit_event(
            event_type=SecurityEventType.SYSTEM,
            message=f"Security alert created: {alert.title}",
            severity=SecurityEventSeverity.INFO,
            source="security_alerting",
            details={
                "alert_id": alert.alert_id,
                "title": alert.title,
                "severity": alert.severity.value,
                "source": alert.source
            }
        )
        
        return alert
    
    def get_alert(self, alert_id: str) -> Optional[SecurityAlert]:
        """
        Get a security alert.
        
        Args:
            alert_id: Alert ID
            
        Returns:
            Security alert or None if not found
        """
        return self.alerts.get(alert_id)
    
    def update_alert_status(
        self,
        alert_id: str,
        status: AlertStatus,
        comment: Optional[str] = None,
        user: Optional[str] = None
    ) -> bool:
        """
        Update alert status.
        
        Args:
            alert_id: Alert ID
            status: New status
            comment: Optional comment
            user: User making the update
            
        Returns:
            True if successful, False otherwise
        """
        alert = self.get_alert(alert_id)
        if not alert:
            logger.warning(f"Alert not found: {alert_id}")
            return False
        
        alert.update_status(status, comment, user)
        return True
    
    def assign_alert(
        self,
        alert_id: str,
        user: str,
        comment: Optional[str] = None
    ) -> bool:
        """
        Assign alert to a user.
        
        Args:
            alert_id: Alert ID
            user: User to assign the alert to
            comment: Optional comment
            
        Returns:
            True if successful, False otherwise
        """
        alert = self.get_alert(alert_id)
        if not alert:
            logger.warning(f"Alert not found: {alert_id}")
            return False
        
        alert.assign(user, comment)
        return True
    
    def add_alert_comment(
        self,
        alert_id: str,
        comment: str,
        user: Optional[str] = None
    ) -> bool:
        """
        Add a comment to an alert.
        
        Args:
            alert_id: Alert ID
            comment: Comment text
            user: User adding the comment
            
        Returns:
            True if successful, False otherwise
        """
        alert = self.get_alert(alert_id)
        if not alert:
            logger.warning(f"Alert not found: {alert_id}")
            return False
        
        alert.add_comment(comment, user)
        return True
    
    def start(self):
        """Start the alert processing thread."""
        if self.processing_thread and self.processing_thread.is_alive():
            logger.warning("Alert manager already running")
            return
        
        self.stop_event.clear()
        
        def process_alerts():
            logger.info("Started alert processing thread")
            
            while not self.stop_event.is_set():
                try:
                    # Get alert from queue with timeout
                    try:
                        alert = self.alert_queue.get(timeout=1)
                    except queue.Empty:
                        continue
                    
                    # Process alert
                    for handler in self.handlers:
                        try:
                            handler.handle(alert)
                        except Exception as e:
                            logger.error(f"Error in alert handler {handler.name}: {str(e)}")
                    
                    # Mark task as done
                    self.alert_queue.task_done()
                
                except Exception as e:
                    logger.error(f"Error processing security alert: {str(e)}")
            
            logger.info("Stopped alert processing thread")
        
        self.processing_thread = threading.Thread(
            target=process_alerts,
            daemon=True,
            name="AlertManager"
        )
        self.processing_thread.start()
        
        logger.info("Started alert manager")
    
    def stop(self):
        """Stop the alert processing thread."""
        if not self.processing_thread or not self.processing_thread.is_alive():
            logger.warning("Alert manager not running")
            return
        
        self.stop_event.set()
        self.processing_thread.join(timeout=10)
        
        if self.processing_thread.is_alive():
            logger.warning("Alert processing thread did not stop gracefully")
        else:
            logger.info("Stopped alert manager")
    
    def wait_for_queue(self, timeout: Optional[float] = None):
        """
        Wait for the alert queue to be empty.
        
        Args:
            timeout: Timeout in seconds
        """
        self.alert_queue.join()


# Create global alert manager
alert_manager = AlertManager()

# Add default logging handler
alert_manager.add_handler(LoggingAlertHandler())

# Start the manager
alert_manager.start()


def create_alert(
    title: str,
    description: str,
    severity: Union[SecurityEventSeverity, str] = SecurityEventSeverity.MEDIUM,
    source: Optional[str] = None,
    details: Optional[Dict[str, Any]] = None,
    events: Optional[List[SecurityEvent]] = None
) -> SecurityAlert:
    """
    Create a security alert.
    
    Args:
        title: Alert title
        description: Alert description
        severity: Alert severity
        source: Alert source
        details: Alert details
        events: Related security events
        
    Returns:
        Security alert
    """
    return alert_manager.create_alert(
        title=title,
        description=description,
        severity=severity,
        source=source,
        details=details,
        events=events
    )


def get_alert(alert_id: str) -> Optional[SecurityAlert]:
    """
    Get a security alert.
    
    Args:
        alert_id: Alert ID
        
    Returns:
        Security alert or None if not found
    """
    return alert_manager.get_alert(alert_id)


def update_alert_status(
    alert_id: str,
    status: AlertStatus,
    comment: Optional[str] = None,
    user: Optional[str] = None
) -> bool:
    """
    Update alert status.
    
    Args:
        alert_id: Alert ID
        status: New status
        comment: Optional comment
        user: User making the update
        
    Returns:
        True if successful, False otherwise
    """
    return alert_manager.update_alert_status(alert_id, status, comment, user)


def assign_alert(
    alert_id: str,
    user: str,
    comment: Optional[str] = None
) -> bool:
    """
    Assign alert to a user.
    
    Args:
        alert_id: Alert ID
        user: User to assign the alert to
        comment: Optional comment
        
    Returns:
        True if successful, False otherwise
    """
    return alert_manager.assign_alert(alert_id, user, comment)


def add_alert_comment(
    alert_id: str,
    comment: str,
    user: Optional[str] = None
) -> bool:
    """
    Add a comment to an alert.
    
    Args:
        alert_id: Alert ID
        comment: Comment text
        user: User adding the comment
        
    Returns:
        True if successful, False otherwise
    """
    return alert_manager.add_alert_comment(alert_id, comment, user)


def add_alert_handler(handler: AlertHandler):
    """
    Add an alert handler.
    
    Args:
        handler: Alert handler
    """
    alert_manager.add_handler(handler)


def remove_alert_handler(name: str):
    """
    Remove an alert handler.
    
    Args:
        name: Handler name
    """
    alert_manager.remove_handler(name)


def add_email_handler(
    smtp_server: str,
    smtp_port: int,
    sender: str,
    recipients: List[str],
    name: str = "email",
    min_severity: SecurityEventSeverity = SecurityEventSeverity.MEDIUM,
    use_ssl: bool = True,
    username: Optional[str] = None,
    password: Optional[str] = None
):
    """
    Add an email alert handler.
    
    Args:
        smtp_server: SMTP server
        smtp_port: SMTP port
        sender: Sender email address
        recipients: Recipient email addresses
        name: Handler name
        min_severity: Minimum severity to handle
        use_ssl: Whether to use SSL
        username: SMTP username
        password: SMTP password
    """
    handler = EmailAlertHandler(
        smtp_server=smtp_server,
        smtp_port=smtp_port,
        sender=sender,
        recipients=recipients,
        name=name,
        min_severity=min_severity,
        use_ssl=use_ssl,
        username=username,
        password=password
    )
    
    alert_manager.add_handler(handler)


def add_webhook_handler(
    webhook_url: str,
    name: str = "webhook",
    min_severity: SecurityEventSeverity = SecurityEventSeverity.MEDIUM,
    headers: Optional[Dict[str, str]] = None
):
    """
    Add a webhook alert handler.
    
    Args:
        webhook_url: Webhook URL
        name: Handler name
        min_severity: Minimum severity to handle
        headers: HTTP headers
    """
    handler = WebhookAlertHandler(
        webhook_url=webhook_url,
        name=name,
        min_severity=min_severity,
        headers=headers
    )
    
    alert_manager.add_handler(handler)
