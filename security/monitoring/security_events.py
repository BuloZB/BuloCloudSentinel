"""
Security event monitoring for Bulo.Cloud Sentinel.

This module provides functionality for monitoring and alerting on security events.
"""

import logging
import json
import time
import threading
import queue
from typing import Dict, List, Optional, Any, Callable, Union
from datetime import datetime
from enum import Enum

# Configure logging
logger = logging.getLogger(__name__)


class SecurityEventSeverity(str, Enum):
    """Security event severity levels."""
    INFO = "info"
    LOW = "low"
    MEDIUM = "medium"
    HIGH = "high"
    CRITICAL = "critical"


class SecurityEventType(str, Enum):
    """Security event types."""
    AUTHENTICATION = "authentication"
    AUTHORIZATION = "authorization"
    ACCESS_CONTROL = "access_control"
    DATA_ACCESS = "data_access"
    CONFIGURATION = "configuration"
    NETWORK = "network"
    SYSTEM = "system"
    APPLICATION = "application"
    MALWARE = "malware"
    VULNERABILITY = "vulnerability"
    COMPLIANCE = "compliance"
    OTHER = "other"


class SecurityEvent:
    """
    Security event for monitoring and alerting.
    
    This class represents a security event that can be monitored and alerted on.
    """
    
    def __init__(
        self,
        event_type: Union[SecurityEventType, str],
        message: str,
        severity: Union[SecurityEventSeverity, str] = SecurityEventSeverity.INFO,
        source: Optional[str] = None,
        details: Optional[Dict[str, Any]] = None,
        timestamp: Optional[float] = None,
        event_id: Optional[str] = None
    ):
        """
        Initialize security event.
        
        Args:
            event_type: Event type
            message: Event message
            severity: Event severity
            source: Event source
            details: Event details
            timestamp: Event timestamp (default: current time)
            event_id: Event ID (default: generated)
        """
        self.event_type = event_type if isinstance(event_type, SecurityEventType) else SecurityEventType(event_type)
        self.message = message
        self.severity = severity if isinstance(severity, SecurityEventSeverity) else SecurityEventSeverity(severity)
        self.source = source or "security_monitoring"
        self.details = details or {}
        self.timestamp = timestamp or time.time()
        self.event_id = event_id or f"{int(self.timestamp)}-{hash(self.message) % 10000:04d}"
    
    def to_dict(self) -> Dict[str, Any]:
        """
        Convert event to dictionary.
        
        Returns:
            Event as dictionary
        """
        return {
            "event_id": self.event_id,
            "event_type": self.event_type.value,
            "message": self.message,
            "severity": self.severity.value,
            "source": self.source,
            "details": self.details,
            "timestamp": self.timestamp,
            "timestamp_iso": datetime.fromtimestamp(self.timestamp).isoformat()
        }
    
    def to_json(self) -> str:
        """
        Convert event to JSON.
        
        Returns:
            Event as JSON string
        """
        return json.dumps(self.to_dict())
    
    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "SecurityEvent":
        """
        Create event from dictionary.
        
        Args:
            data: Event data
            
        Returns:
            Security event
        """
        return cls(
            event_type=data["event_type"],
            message=data["message"],
            severity=data["severity"],
            source=data["source"],
            details=data["details"],
            timestamp=data["timestamp"],
            event_id=data["event_id"]
        )
    
    @classmethod
    def from_json(cls, json_str: str) -> "SecurityEvent":
        """
        Create event from JSON.
        
        Args:
            json_str: Event as JSON string
            
        Returns:
            Security event
        """
        data = json.loads(json_str)
        return cls.from_dict(data)


class SecurityEventHandler:
    """
    Handler for security events.
    
    This class handles security events by processing them and
    optionally forwarding them to other systems.
    """
    
    def __init__(
        self,
        name: str,
        min_severity: SecurityEventSeverity = SecurityEventSeverity.INFO,
        event_types: Optional[List[SecurityEventType]] = None
    ):
        """
        Initialize security event handler.
        
        Args:
            name: Handler name
            min_severity: Minimum severity to handle
            event_types: Event types to handle (None = all)
        """
        self.name = name
        self.min_severity = min_severity
        self.event_types = event_types
        
        logger.info(f"Initialized security event handler: {name}")
    
    def can_handle(self, event: SecurityEvent) -> bool:
        """
        Check if handler can handle an event.
        
        Args:
            event: Security event
            
        Returns:
            True if handler can handle event, False otherwise
        """
        # Check severity
        if SecurityEventSeverity[self.min_severity.upper()].value > SecurityEventSeverity[event.severity.upper()].value:
            return False
        
        # Check event type
        if self.event_types and event.event_type not in self.event_types:
            return False
        
        return True
    
    def handle(self, event: SecurityEvent):
        """
        Handle a security event.
        
        Args:
            event: Security event
        """
        if not self.can_handle(event):
            return
        
        try:
            self._handle_event(event)
        except Exception as e:
            logger.error(f"Error handling security event in {self.name}: {str(e)}")
    
    def _handle_event(self, event: SecurityEvent):
        """
        Handle a security event (to be implemented by subclasses).
        
        Args:
            event: Security event
        """
        raise NotImplementedError("Subclasses must implement _handle_event")


class LoggingEventHandler(SecurityEventHandler):
    """Security event handler that logs events."""
    
    def __init__(
        self,
        name: str = "logging",
        min_severity: SecurityEventSeverity = SecurityEventSeverity.INFO,
        event_types: Optional[List[SecurityEventType]] = None,
        logger_name: str = "security_events"
    ):
        """
        Initialize logging event handler.
        
        Args:
            name: Handler name
            min_severity: Minimum severity to handle
            event_types: Event types to handle (None = all)
            logger_name: Logger name
        """
        super().__init__(name, min_severity, event_types)
        self.logger = logging.getLogger(logger_name)
    
    def _handle_event(self, event: SecurityEvent):
        """
        Handle a security event by logging it.
        
        Args:
            event: Security event
        """
        # Map severity to log level
        log_level = {
            SecurityEventSeverity.INFO: logging.INFO,
            SecurityEventSeverity.LOW: logging.INFO,
            SecurityEventSeverity.MEDIUM: logging.WARNING,
            SecurityEventSeverity.HIGH: logging.ERROR,
            SecurityEventSeverity.CRITICAL: logging.CRITICAL
        }.get(event.severity, logging.INFO)
        
        # Log event
        self.logger.log(
            log_level,
            f"[{event.event_type.value}] {event.message}",
            extra={
                "event_id": event.event_id,
                "event_type": event.event_type.value,
                "severity": event.severity.value,
                "source": event.source,
                "details": event.details,
                "timestamp": event.timestamp,
                "timestamp_iso": datetime.fromtimestamp(event.timestamp).isoformat()
            }
        )


class CallbackEventHandler(SecurityEventHandler):
    """Security event handler that calls a callback function."""
    
    def __init__(
        self,
        callback: Callable[[SecurityEvent], None],
        name: str = "callback",
        min_severity: SecurityEventSeverity = SecurityEventSeverity.INFO,
        event_types: Optional[List[SecurityEventType]] = None
    ):
        """
        Initialize callback event handler.
        
        Args:
            callback: Callback function
            name: Handler name
            min_severity: Minimum severity to handle
            event_types: Event types to handle (None = all)
        """
        super().__init__(name, min_severity, event_types)
        self.callback = callback
    
    def _handle_event(self, event: SecurityEvent):
        """
        Handle a security event by calling the callback function.
        
        Args:
            event: Security event
        """
        self.callback(event)


class WebhookEventHandler(SecurityEventHandler):
    """Security event handler that sends events to a webhook."""
    
    def __init__(
        self,
        webhook_url: str,
        name: str = "webhook",
        min_severity: SecurityEventSeverity = SecurityEventSeverity.INFO,
        event_types: Optional[List[SecurityEventType]] = None,
        headers: Optional[Dict[str, str]] = None,
        timeout: int = 5
    ):
        """
        Initialize webhook event handler.
        
        Args:
            webhook_url: Webhook URL
            name: Handler name
            min_severity: Minimum severity to handle
            event_types: Event types to handle (None = all)
            headers: HTTP headers
            timeout: Request timeout in seconds
        """
        super().__init__(name, min_severity, event_types)
        self.webhook_url = webhook_url
        self.headers = headers or {"Content-Type": "application/json"}
        self.timeout = timeout
        
        # Import requests lazily
        try:
            import requests
            self.requests = requests
        except ImportError:
            logger.error("requests package is required for WebhookEventHandler")
            raise ImportError("requests package is required for WebhookEventHandler")
    
    def _handle_event(self, event: SecurityEvent):
        """
        Handle a security event by sending it to a webhook.
        
        Args:
            event: Security event
        """
        try:
            response = self.requests.post(
                self.webhook_url,
                headers=self.headers,
                json=event.to_dict(),
                timeout=self.timeout
            )
            
            if response.status_code >= 400:
                logger.error(f"Error sending security event to webhook: {response.status_code} {response.text}")
            else:
                logger.debug(f"Sent security event to webhook: {event.event_id}")
        
        except Exception as e:
            logger.error(f"Error sending security event to webhook: {str(e)}")


class SecurityEventMonitor:
    """
    Security event monitor for processing and forwarding events.
    
    This class monitors security events and forwards them to handlers.
    """
    
    def __init__(self, queue_size: int = 1000):
        """
        Initialize security event monitor.
        
        Args:
            queue_size: Maximum queue size
        """
        self.handlers: List[SecurityEventHandler] = []
        self.event_queue = queue.Queue(maxsize=queue_size)
        self.processing_thread: Optional[threading.Thread] = None
        self.stop_event = threading.Event()
        
        logger.info("Initialized security event monitor")
    
    def add_handler(self, handler: SecurityEventHandler):
        """
        Add an event handler.
        
        Args:
            handler: Security event handler
        """
        self.handlers.append(handler)
        logger.info(f"Added security event handler: {handler.name}")
    
    def remove_handler(self, name: str):
        """
        Remove an event handler.
        
        Args:
            name: Handler name
        """
        self.handlers = [h for h in self.handlers if h.name != name]
        logger.info(f"Removed security event handler: {name}")
    
    def emit_event(self, event: SecurityEvent):
        """
        Emit a security event.
        
        Args:
            event: Security event
        """
        try:
            self.event_queue.put(event, block=False)
            logger.debug(f"Emitted security event: {event.event_id}")
        except queue.Full:
            logger.error("Security event queue is full, event dropped")
    
    def start(self):
        """Start the event processing thread."""
        if self.processing_thread and self.processing_thread.is_alive():
            logger.warning("Security event monitor already running")
            return
        
        self.stop_event.clear()
        
        def process_events():
            logger.info("Started security event processing thread")
            
            while not self.stop_event.is_set():
                try:
                    # Get event from queue with timeout
                    try:
                        event = self.event_queue.get(timeout=1)
                    except queue.Empty:
                        continue
                    
                    # Process event
                    for handler in self.handlers:
                        try:
                            handler.handle(event)
                        except Exception as e:
                            logger.error(f"Error in security event handler {handler.name}: {str(e)}")
                    
                    # Mark task as done
                    self.event_queue.task_done()
                
                except Exception as e:
                    logger.error(f"Error processing security event: {str(e)}")
            
            logger.info("Stopped security event processing thread")
        
        self.processing_thread = threading.Thread(
            target=process_events,
            daemon=True,
            name="SecurityEventMonitor"
        )
        self.processing_thread.start()
        
        logger.info("Started security event monitor")
    
    def stop(self):
        """Stop the event processing thread."""
        if not self.processing_thread or not self.processing_thread.is_alive():
            logger.warning("Security event monitor not running")
            return
        
        self.stop_event.set()
        self.processing_thread.join(timeout=10)
        
        if self.processing_thread.is_alive():
            logger.warning("Security event processing thread did not stop gracefully")
        else:
            logger.info("Stopped security event monitor")
    
    def wait_for_queue(self, timeout: Optional[float] = None):
        """
        Wait for the event queue to be empty.
        
        Args:
            timeout: Timeout in seconds
        """
        self.event_queue.join()


# Create global security event monitor
security_event_monitor = SecurityEventMonitor()

# Add default logging handler
security_event_monitor.add_handler(LoggingEventHandler())

# Start the monitor
security_event_monitor.start()


def emit_event(
    event_type: Union[SecurityEventType, str],
    message: str,
    severity: Union[SecurityEventSeverity, str] = SecurityEventSeverity.INFO,
    source: Optional[str] = None,
    details: Optional[Dict[str, Any]] = None
):
    """
    Emit a security event.
    
    Args:
        event_type: Event type
        message: Event message
        severity: Event severity
        source: Event source
        details: Event details
    """
    event = SecurityEvent(
        event_type=event_type,
        message=message,
        severity=severity,
        source=source,
        details=details
    )
    
    security_event_monitor.emit_event(event)


def add_handler(handler: SecurityEventHandler):
    """
    Add an event handler.
    
    Args:
        handler: Security event handler
    """
    security_event_monitor.add_handler(handler)


def remove_handler(name: str):
    """
    Remove an event handler.
    
    Args:
        name: Handler name
    """
    security_event_monitor.remove_handler(name)


def add_webhook_handler(
    webhook_url: str,
    name: str = "webhook",
    min_severity: SecurityEventSeverity = SecurityEventSeverity.MEDIUM,
    event_types: Optional[List[SecurityEventType]] = None,
    headers: Optional[Dict[str, str]] = None
):
    """
    Add a webhook event handler.
    
    Args:
        webhook_url: Webhook URL
        name: Handler name
        min_severity: Minimum severity to handle
        event_types: Event types to handle (None = all)
        headers: HTTP headers
    """
    handler = WebhookEventHandler(
        webhook_url=webhook_url,
        name=name,
        min_severity=min_severity,
        event_types=event_types,
        headers=headers
    )
    
    security_event_monitor.add_handler(handler)


def add_callback_handler(
    callback: Callable[[SecurityEvent], None],
    name: str = "callback",
    min_severity: SecurityEventSeverity = SecurityEventSeverity.INFO,
    event_types: Optional[List[SecurityEventType]] = None
):
    """
    Add a callback event handler.
    
    Args:
        callback: Callback function
        name: Handler name
        min_severity: Minimum severity to handle
        event_types: Event types to handle (None = all)
    """
    handler = CallbackEventHandler(
        callback=callback,
        name=name,
        min_severity=min_severity,
        event_types=event_types
    )
    
    security_event_monitor.add_handler(handler)
