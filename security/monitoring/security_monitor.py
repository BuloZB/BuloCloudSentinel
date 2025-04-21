"""
Security monitoring for Bulo.Cloud Sentinel Security Module.

This module provides security monitoring and alerting functionality.
"""

import time
from datetime import datetime, timedelta
from typing import Dict, List, Optional, Any, Callable

from fastapi import Request
from pydantic import BaseModel

from ..logging.audit_logger import log_security_alert, SeverityLevel

# Models
class SecurityAlert(BaseModel):
    """Security alert model."""
    id: str
    timestamp: datetime
    alert_type: str
    severity: str
    source: str
    message: str
    details: Optional[Dict[str, Any]] = None
    status: str = "new"  # new, acknowledged, resolved, false_positive

class SecurityRule(BaseModel):
    """Security rule model."""
    id: str
    name: str
    description: str
    severity: str
    enabled: bool = True
    conditions: Dict[str, Any]
    actions: List[Dict[str, Any]]

# Alert types
class AlertType:
    """Alert types for security monitoring."""
    BRUTE_FORCE = "brute_force"
    SUSPICIOUS_LOGIN = "suspicious_login"
    ACCOUNT_LOCKOUT = "account_lockout"
    PRIVILEGE_ESCALATION = "privilege_escalation"
    UNAUTHORIZED_ACCESS = "unauthorized_access"
    DATA_EXFILTRATION = "data_exfiltration"
    MALICIOUS_IP = "malicious_ip"
    UNUSUAL_ACTIVITY = "unusual_activity"
    CONFIGURATION_CHANGE = "configuration_change"
    SYSTEM_ERROR = "system_error"

# Security monitor
class SecurityMonitor:
    """Security monitor for detecting and alerting on security events."""
    
    def __init__(self):
        """Initialize security monitor."""
        self.rules: Dict[str, SecurityRule] = {}
        self.alerts: List[SecurityAlert] = []
        self.failed_logins: Dict[str, List[datetime]] = {}  # username -> list of failed login times
        self.ip_failed_logins: Dict[str, List[datetime]] = {}  # ip -> list of failed login times
        self.suspicious_ips: Dict[str, int] = {}  # ip -> suspicion score
        self.user_sessions: Dict[str, Dict[str, Any]] = {}  # user_id -> session info
    
    def add_rule(self, rule: SecurityRule):
        """
        Add a security rule.
        
        Args:
            rule: Security rule to add
        """
        self.rules[rule.id] = rule
    
    def remove_rule(self, rule_id: str) -> bool:
        """
        Remove a security rule.
        
        Args:
            rule_id: ID of rule to remove
            
        Returns:
            True if rule was removed, False if not found
        """
        if rule_id in self.rules:
            del self.rules[rule_id]
            return True
        return False
    
    def get_rule(self, rule_id: str) -> Optional[SecurityRule]:
        """
        Get a security rule.
        
        Args:
            rule_id: ID of rule to get
            
        Returns:
            Security rule or None if not found
        """
        return self.rules.get(rule_id)
    
    def get_rules(self) -> List[SecurityRule]:
        """
        Get all security rules.
        
        Returns:
            List of security rules
        """
        return list(self.rules.values())
    
    def add_alert(self, alert: SecurityAlert):
        """
        Add a security alert.
        
        Args:
            alert: Security alert to add
        """
        self.alerts.append(alert)
    
    def get_alerts(
        self,
        start_time: Optional[datetime] = None,
        end_time: Optional[datetime] = None,
        severity: Optional[str] = None,
        status: Optional[str] = None,
        limit: int = 100
    ) -> List[SecurityAlert]:
        """
        Get security alerts.
        
        Args:
            start_time: Start time filter
            end_time: End time filter
            severity: Severity filter
            status: Status filter
            limit: Maximum number of alerts to return
            
        Returns:
            List of security alerts
        """
        filtered_alerts = self.alerts
        
        if start_time:
            filtered_alerts = [a for a in filtered_alerts if a.timestamp >= start_time]
        
        if end_time:
            filtered_alerts = [a for a in filtered_alerts if a.timestamp <= end_time]
        
        if severity:
            filtered_alerts = [a for a in filtered_alerts if a.severity == severity]
        
        if status:
            filtered_alerts = [a for a in filtered_alerts if a.status == status]
        
        # Sort by timestamp (newest first)
        filtered_alerts.sort(key=lambda a: a.timestamp, reverse=True)
        
        return filtered_alerts[:limit]
    
    def update_alert_status(self, alert_id: str, status: str) -> bool:
        """
        Update the status of a security alert.
        
        Args:
            alert_id: ID of alert to update
            status: New status
            
        Returns:
            True if alert was updated, False if not found
        """
        for alert in self.alerts:
            if alert.id == alert_id:
                alert.status = status
                return True
        return False
    
    def track_failed_login(self, username: str, ip_address: str):
        """
        Track a failed login attempt.
        
        Args:
            username: Username
            ip_address: IP address
        """
        now = datetime.utcnow()
        
        # Track by username
        if username not in self.failed_logins:
            self.failed_logins[username] = []
        self.failed_logins[username].append(now)
        
        # Track by IP
        if ip_address not in self.ip_failed_logins:
            self.ip_failed_logins[ip_address] = []
        self.ip_failed_logins[ip_address].append(now)
        
        # Check for brute force attacks
        self._check_brute_force(username, ip_address)
    
    def track_successful_login(
        self,
        user_id: str,
        username: str,
        ip_address: str,
        user_agent: str,
        location: Optional[Dict[str, Any]] = None
    ):
        """
        Track a successful login.
        
        Args:
            user_id: User ID
            username: Username
            ip_address: IP address
            user_agent: User agent
            location: Location information
        """
        now = datetime.utcnow()
        
        # Store session info
        self.user_sessions[user_id] = {
            "username": username,
            "ip_address": ip_address,
            "user_agent": user_agent,
            "login_time": now,
            "location": location
        }
        
        # Check for suspicious login
        self._check_suspicious_login(user_id, username, ip_address, user_agent, location)
    
    def track_api_request(
        self,
        request: Request,
        user_id: Optional[str] = None,
        username: Optional[str] = None,
        response_time: float = 0.0,
        status_code: int = 200
    ):
        """
        Track an API request.
        
        Args:
            request: FastAPI request
            user_id: User ID
            username: Username
            response_time: Response time in seconds
            status_code: HTTP status code
        """
        # Get request details
        method = request.method
        path = request.url.path
        ip_address = request.client.host if request.client else None
        user_agent = request.headers.get("user-agent")
        
        # Check for suspicious activity
        if status_code >= 400:
            self._check_suspicious_activity(
                user_id,
                username,
                ip_address,
                method,
                path,
                status_code
            )
    
    def _check_brute_force(self, username: str, ip_address: str):
        """
        Check for brute force attacks.
        
        Args:
            username: Username
            ip_address: IP address
        """
        now = datetime.utcnow()
        window = timedelta(minutes=10)
        threshold = 5
        
        # Check username
        recent_failures = [
            t for t in self.failed_logins.get(username, [])
            if now - t <= window
        ]
        
        if len(recent_failures) >= threshold:
            # Alert on username brute force
            alert_id = log_security_alert(
                alert_type=AlertType.BRUTE_FORCE,
                severity=SeverityLevel.WARNING,
                ip_address=ip_address,
                username=username,
                details={
                    "username": username,
                    "ip_address": ip_address,
                    "attempts": len(recent_failures),
                    "window_minutes": window.total_seconds() / 60
                }
            )
        
        # Check IP
        recent_ip_failures = [
            t for t in self.ip_failed_logins.get(ip_address, [])
            if now - t <= window
        ]
        
        if len(recent_ip_failures) >= threshold * 2:
            # Alert on IP brute force (higher threshold)
            alert_id = log_security_alert(
                alert_type=AlertType.BRUTE_FORCE,
                severity=SeverityLevel.WARNING,
                ip_address=ip_address,
                details={
                    "ip_address": ip_address,
                    "attempts": len(recent_ip_failures),
                    "window_minutes": window.total_seconds() / 60
                }
            )
            
            # Add to suspicious IPs
            self.suspicious_ips[ip_address] = self.suspicious_ips.get(ip_address, 0) + 10
    
    def _check_suspicious_login(
        self,
        user_id: str,
        username: str,
        ip_address: str,
        user_agent: str,
        location: Optional[Dict[str, Any]] = None
    ):
        """
        Check for suspicious logins.
        
        Args:
            user_id: User ID
            username: Username
            ip_address: IP address
            user_agent: User agent
            location: Location information
        """
        # Check if user has previous sessions
        if user_id in self.user_sessions:
            prev_session = self.user_sessions[user_id]
            
            # Check for IP change
            if prev_session["ip_address"] != ip_address:
                # Calculate time since last login
                time_diff = datetime.utcnow() - prev_session["login_time"]
                
                # If IP changed and login was recent, might be suspicious
                if time_diff < timedelta(hours=24):
                    alert_id = log_security_alert(
                        alert_type=AlertType.SUSPICIOUS_LOGIN,
                        severity=SeverityLevel.WARNING,
                        ip_address=ip_address,
                        user_id=user_id,
                        username=username,
                        details={
                            "previous_ip": prev_session["ip_address"],
                            "new_ip": ip_address,
                            "time_since_last_login_hours": time_diff.total_seconds() / 3600
                        }
                    )
            
            # Check for user agent change
            if prev_session["user_agent"] != user_agent:
                alert_id = log_security_alert(
                    alert_type=AlertType.SUSPICIOUS_LOGIN,
                    severity=SeverityLevel.INFO,
                    ip_address=ip_address,
                    user_id=user_id,
                    username=username,
                    details={
                        "previous_user_agent": prev_session["user_agent"],
                        "new_user_agent": user_agent
                    }
                )
        
        # Check if IP is suspicious
        if ip_address in self.suspicious_ips and self.suspicious_ips[ip_address] > 5:
            alert_id = log_security_alert(
                alert_type=AlertType.MALICIOUS_IP,
                severity=SeverityLevel.WARNING,
                ip_address=ip_address,
                user_id=user_id,
                username=username,
                details={
                    "suspicion_score": self.suspicious_ips[ip_address]
                }
            )
    
    def _check_suspicious_activity(
        self,
        user_id: Optional[str],
        username: Optional[str],
        ip_address: Optional[str],
        method: str,
        path: str,
        status_code: int
    ):
        """
        Check for suspicious API activity.
        
        Args:
            user_id: User ID
            username: Username
            ip_address: IP address
            method: HTTP method
            path: Request path
            status_code: HTTP status code
        """
        # Check for unauthorized access (403)
        if status_code == 403:
            alert_id = log_security_alert(
                alert_type=AlertType.UNAUTHORIZED_ACCESS,
                severity=SeverityLevel.WARNING,
                ip_address=ip_address,
                user_id=user_id,
                username=username,
                resource=path,
                details={
                    "method": method,
                    "path": path,
                    "status_code": status_code
                }
            )
        
        # Check for not found (404) on sensitive paths
        if status_code == 404 and any(s in path for s in ["/admin", "/api/v1/users", "/api/v1/config"]):
            alert_id = log_security_alert(
                alert_type=AlertType.SUSPICIOUS_ACTIVITY,
                severity=SeverityLevel.INFO,
                ip_address=ip_address,
                user_id=user_id,
                username=username,
                resource=path,
                details={
                    "method": method,
                    "path": path,
                    "status_code": status_code
                }
            )
        
        # Check for server errors on sensitive operations
        if status_code >= 500 and method in ["POST", "PUT", "DELETE"]:
            alert_id = log_security_alert(
                alert_type=AlertType.SYSTEM_ERROR,
                severity=SeverityLevel.ERROR,
                ip_address=ip_address,
                user_id=user_id,
                username=username,
                resource=path,
                details={
                    "method": method,
                    "path": path,
                    "status_code": status_code
                }
            )
        
        # Increment suspicion score for IP with 4xx errors
        if status_code >= 400 and status_code < 500 and ip_address:
            self.suspicious_ips[ip_address] = self.suspicious_ips.get(ip_address, 0) + 1

# Create global security monitor instance
security_monitor = SecurityMonitor()
