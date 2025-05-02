"""
Audit logging for Bulo.Cloud Sentinel.

This module provides comprehensive audit logging for security-relevant events.
"""

import os
import sys
import logging
import time
import json
import uuid
import threading
from typing import Dict, Any, Optional, List, Tuple
from datetime import datetime
from pathlib import Path

# Add project root to path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))

# Import local modules
from ai.inference.config import ConfigManager
from ai.inference.monitoring import structured_logger

# Setup logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Create configuration manager
config = ConfigManager()


class AuditLogger:
    """
    Audit logger for Bulo.Cloud Sentinel.
    
    This class provides comprehensive audit logging for security-relevant events.
    """
    
    def __init__(
        self,
        audit_dir: Optional[str] = None,
        max_audit_files: int = 10,
        max_audit_age: int = 86400 * 30,  # 30 days
        include_timestamp: bool = True,
        include_hostname: bool = True,
        include_pid: bool = True
    ):
        """
        Initialize the audit logger.
        
        Args:
            audit_dir: Directory for audit logs
            max_audit_files: Maximum number of audit files to keep
            max_audit_age: Maximum age of audit files in seconds
            include_timestamp: Whether to include timestamp in audit records
            include_hostname: Whether to include hostname in audit records
            include_pid: Whether to include process ID in audit records
        """
        # Set audit directory
        self.audit_dir = audit_dir or os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            "../../logs/audit"
        )
        
        # Create audit directory if it doesn't exist
        os.makedirs(self.audit_dir, exist_ok=True)
        
        # Set audit parameters
        self.max_audit_files = max_audit_files
        self.max_audit_age = max_audit_age
        self.include_timestamp = include_timestamp
        self.include_hostname = include_hostname
        self.include_pid = include_pid
        
        # Initialize audit file
        self.audit_file = None
        self.audit_file_path = None
        self.audit_file_lock = threading.Lock()
        
        # Initialize audit file
        self._init_audit_file()
        
        # Clean up old audit files
        self._clean_audit_files()
    
    def _init_audit_file(self):
        """Initialize audit file."""
        try:
            # Create audit file path
            self.audit_file_path = os.path.join(
                self.audit_dir,
                f"audit_{datetime.now().strftime('%Y%m%d')}.log"
            )
            
            # Create audit file
            self.audit_file = open(self.audit_file_path, "a")
            
            # Set secure permissions for audit file
            os.chmod(self.audit_file_path, 0o600)
            
            structured_logger.info(
                "Audit file initialized",
                logger_name="audit",
                audit_file=self.audit_file_path
            )
        
        except Exception as e:
            # Log error
            structured_logger.error(
                "Error initializing audit file",
                logger_name="audit",
                error=str(e)
            )
    
    def _clean_audit_files(self):
        """Clean up old audit files."""
        try:
            # Get current time
            current_time = time.time()
            
            # Get audit files
            audit_files = []
            for file_path in Path(self.audit_dir).glob("audit_*.log"):
                # Get file modification time
                mod_time = file_path.stat().st_mtime
                
                # Add file to list
                audit_files.append((file_path, mod_time))
            
            # Sort files by modification time (newest first)
            audit_files.sort(key=lambda x: x[1], reverse=True)
            
            # Delete old files
            for file_path, mod_time in audit_files[self.max_audit_files:]:
                try:
                    # Check if file is too old
                    if current_time - mod_time > self.max_audit_age:
                        # Delete file
                        os.remove(file_path)
                        structured_logger.debug(
                            "Deleted old audit file",
                            logger_name="audit",
                            file_path=str(file_path)
                        )
                except Exception as e:
                    structured_logger.error(
                        "Error deleting audit file",
                        logger_name="audit",
                        error=str(e),
                        file_path=str(file_path)
                    )
        
        except Exception as e:
            # Log error
            structured_logger.error(
                "Error cleaning audit files",
                logger_name="audit",
                error=str(e)
            )
    
    def log(
        self,
        event_type: str,
        event_data: Dict[str, Any],
        user: Optional[str] = None,
        source: str = "system",
        severity: str = "info"
    ):
        """
        Log an audit event.
        
        Args:
            event_type: Type of event
            event_data: Event data
            user: User who performed the action
            source: Source of the event
            severity: Severity of the event
        """
        try:
            # Check if audit file is initialized
            if not self.audit_file:
                self._init_audit_file()
            
            # Create audit record
            audit_record = {
                "event_id": str(uuid.uuid4()),
                "event_type": event_type,
                "event_data": event_data,
                "source": source,
                "severity": severity
            }
            
            # Add user if provided
            if user:
                audit_record["user"] = user
            
            # Add standard fields
            if self.include_timestamp:
                audit_record["timestamp"] = datetime.now().isoformat()
            
            if self.include_hostname:
                import socket
                audit_record["hostname"] = socket.gethostname()
            
            if self.include_pid:
                import os
                audit_record["pid"] = os.getpid()
            
            # Convert audit record to JSON
            audit_json = json.dumps(audit_record)
            
            # Write audit record to file
            with self.audit_file_lock:
                # Check if audit file is for the current day
                current_day = datetime.now().strftime("%Y%m%d")
                if not self.audit_file_path or current_day not in self.audit_file_path:
                    # Close current audit file
                    if self.audit_file:
                        self.audit_file.close()
                    
                    # Initialize new audit file
                    self._init_audit_file()
                
                # Write audit record
                self.audit_file.write(audit_json + "\n")
                self.audit_file.flush()
        
        except Exception as e:
            # Log error
            structured_logger.error(
                "Error logging audit event",
                logger_name="audit",
                error=str(e)
            )
    
    def log_authentication(
        self,
        user: str,
        success: bool,
        source: str = "auth_ui",
        details: Optional[Dict[str, Any]] = None
    ):
        """
        Log an authentication event.
        
        Args:
            user: User who attempted to authenticate
            success: Whether authentication was successful
            source: Source of the event
            details: Additional details
        """
        # Create event data
        event_data = {
            "success": success
        }
        
        # Add details if provided
        if details:
            event_data.update(details)
        
        # Determine severity
        severity = "info" if success else "warning"
        
        # Log event
        self.log(
            event_type="authentication",
            event_data=event_data,
            user=user,
            source=source,
            severity=severity
        )
    
    def log_authorization(
        self,
        user: str,
        action: str,
        resource: str,
        success: bool,
        source: str = "system",
        details: Optional[Dict[str, Any]] = None
    ):
        """
        Log an authorization event.
        
        Args:
            user: User who attempted to perform the action
            action: Action that was attempted
            resource: Resource that was accessed
            success: Whether authorization was successful
            source: Source of the event
            details: Additional details
        """
        # Create event data
        event_data = {
            "action": action,
            "resource": resource,
            "success": success
        }
        
        # Add details if provided
        if details:
            event_data.update(details)
        
        # Determine severity
        severity = "info" if success else "warning"
        
        # Log event
        self.log(
            event_type="authorization",
            event_data=event_data,
            user=user,
            source=source,
            severity=severity
        )
    
    def log_data_access(
        self,
        user: str,
        action: str,
        resource: str,
        success: bool,
        source: str = "system",
        details: Optional[Dict[str, Any]] = None
    ):
        """
        Log a data access event.
        
        Args:
            user: User who accessed the data
            action: Action that was performed
            resource: Resource that was accessed
            success: Whether access was successful
            source: Source of the event
            details: Additional details
        """
        # Create event data
        event_data = {
            "action": action,
            "resource": resource,
            "success": success
        }
        
        # Add details if provided
        if details:
            event_data.update(details)
        
        # Determine severity
        severity = "info" if success else "warning"
        
        # Log event
        self.log(
            event_type="data_access",
            event_data=event_data,
            user=user,
            source=source,
            severity=severity
        )
    
    def log_configuration_change(
        self,
        user: str,
        action: str,
        resource: str,
        old_value: Any,
        new_value: Any,
        source: str = "system",
        details: Optional[Dict[str, Any]] = None
    ):
        """
        Log a configuration change event.
        
        Args:
            user: User who changed the configuration
            action: Action that was performed
            resource: Resource that was changed
            old_value: Old value
            new_value: New value
            source: Source of the event
            details: Additional details
        """
        # Create event data
        event_data = {
            "action": action,
            "resource": resource,
            "old_value": old_value,
            "new_value": new_value
        }
        
        # Add details if provided
        if details:
            event_data.update(details)
        
        # Log event
        self.log(
            event_type="configuration_change",
            event_data=event_data,
            user=user,
            source=source,
            severity="info"
        )
    
    def log_security_event(
        self,
        event_name: str,
        severity: str,
        user: Optional[str] = None,
        source: str = "system",
        details: Optional[Dict[str, Any]] = None
    ):
        """
        Log a security event.
        
        Args:
            event_name: Name of the event
            severity: Severity of the event
            user: User associated with the event
            source: Source of the event
            details: Additional details
        """
        # Create event data
        event_data = {
            "event_name": event_name
        }
        
        # Add details if provided
        if details:
            event_data.update(details)
        
        # Log event
        self.log(
            event_type="security_event",
            event_data=event_data,
            user=user,
            source=source,
            severity=severity
        )
    
    def log_system_event(
        self,
        event_name: str,
        severity: str,
        source: str = "system",
        details: Optional[Dict[str, Any]] = None
    ):
        """
        Log a system event.
        
        Args:
            event_name: Name of the event
            severity: Severity of the event
            source: Source of the event
            details: Additional details
        """
        # Create event data
        event_data = {
            "event_name": event_name
        }
        
        # Add details if provided
        if details:
            event_data.update(details)
        
        # Log event
        self.log(
            event_type="system_event",
            event_data=event_data,
            user=None,
            source=source,
            severity=severity
        )


# Create global audit logger
audit_logger = AuditLogger()
