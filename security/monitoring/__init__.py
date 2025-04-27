"""
Security monitoring utilities for Bulo.Cloud Sentinel.

This package provides functions for monitoring security-relevant events,
detecting anomalies, and generating security alerts.
"""

from .anomaly_detection import (
    AnomalyType,
    AnomalyDetector,
    StatisticalAnomalyDetector,
    RateAnomalyDetector,
    PatternAnomalyDetector,
    AnomalyDetectionManager,
    anomaly_detection_manager,
    auth_failure_detector,
    api_usage_detector,
    user_activity_detector,
)

from .security_events import (
    SecurityEvent,
    SecurityEventSeverity,
    SecurityEventType,
    emit_event,
    add_handler as add_event_handler,
    remove_handler as remove_event_handler,
    add_webhook_handler as add_event_webhook_handler,
    add_callback_handler as add_event_callback_handler
)

from .alerting import (
    SecurityAlert,
    AlertStatus,
    create_alert,
    get_alert,
    update_alert_status,
    assign_alert,
    add_alert_comment,
    add_alert_handler,
    remove_alert_handler,
    add_email_handler,
    add_webhook_handler as add_alert_webhook_handler
)

__all__ = [
    # Anomaly detection
    "AnomalyType",
    "AnomalyDetector",
    "StatisticalAnomalyDetector",
    "RateAnomalyDetector",
    "PatternAnomalyDetector",
    "AnomalyDetectionManager",
    "anomaly_detection_manager",
    "auth_failure_detector",
    "api_usage_detector",
    "user_activity_detector",

    # Security events
    "SecurityEvent",
    "SecurityEventSeverity",
    "SecurityEventType",
    "emit_event",
    "add_event_handler",
    "remove_event_handler",
    "add_event_webhook_handler",
    "add_event_callback_handler",

    # Security alerts
    "SecurityAlert",
    "AlertStatus",
    "create_alert",
    "get_alert",
    "update_alert_status",
    "assign_alert",
    "add_alert_comment",
    "add_alert_handler",
    "remove_alert_handler",
    "add_email_handler",
    "add_alert_webhook_handler"
]
