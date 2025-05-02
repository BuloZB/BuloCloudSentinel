"""
Logging and monitoring module for the inference engine.

This module provides logging and monitoring utilities for the inference engine,
including structured logging, metrics collection, and alerting.
"""

import os
import sys
import json
import time
import logging
import threading
import traceback
from typing import Dict, Any, Optional, List, Union, Callable
from datetime import datetime
from pathlib import Path
import uuid

# Setup basic logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class StructuredLogger:
    """
    Structured logger for the inference engine.
    
    This class provides structured logging for the inference engine.
    """
    
    def __init__(
        self,
        log_dir: Optional[str] = None,
        log_level: int = logging.INFO,
        max_log_size: int = 10 * 1024 * 1024,  # 10MB
        max_log_files: int = 10,
        include_timestamp: bool = True,
        include_hostname: bool = True,
        include_pid: bool = True
    ):
        """
        Initialize the structured logger.
        
        Args:
            log_dir: Directory for log files
            log_level: Logging level
            max_log_size: Maximum log file size in bytes
            max_log_files: Maximum number of log files to keep
            include_timestamp: Whether to include timestamp in log records
            include_hostname: Whether to include hostname in log records
            include_pid: Whether to include process ID in log records
        """
        # Set log directory
        self.log_dir = log_dir or os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            "../../logs"
        )
        
        # Create log directory if it doesn't exist
        os.makedirs(self.log_dir, exist_ok=True)
        
        # Set logging parameters
        self.log_level = log_level
        self.max_log_size = max_log_size
        self.max_log_files = max_log_files
        self.include_timestamp = include_timestamp
        self.include_hostname = include_hostname
        self.include_pid = include_pid
        
        # Initialize loggers
        self.loggers = {}
        
        # Initialize default logger
        self.get_logger("inference")
    
    def get_logger(self, name: str) -> logging.Logger:
        """
        Get a logger.
        
        Args:
            name: Logger name
            
        Returns:
            Logger
        """
        try:
            # Check if logger already exists
            if name in self.loggers:
                return self.loggers[name]
            
            # Create logger
            logger = logging.getLogger(name)
            logger.setLevel(self.log_level)
            
            # Remove existing handlers
            for handler in logger.handlers[:]:
                logger.removeHandler(handler)
            
            # Create log file path
            log_file = os.path.join(self.log_dir, f"{name}.log")
            
            # Create file handler with rotation
            from logging.handlers import RotatingFileHandler
            file_handler = RotatingFileHandler(
                log_file,
                maxBytes=self.max_log_size,
                backupCount=self.max_log_files
            )
            
            # Create formatter
            formatter = logging.Formatter(
                "%(asctime)s - %(name)s - %(levelname)s - %(message)s"
            )
            
            # Set formatter
            file_handler.setFormatter(formatter)
            
            # Add handler to logger
            logger.addHandler(file_handler)
            
            # Store logger
            self.loggers[name] = logger
            
            return logger
        except Exception as e:
            # Use default logger
            logger.error(f"Error creating logger {name}: {str(e)}")
            return logging.getLogger(name)
    
    def log(
        self,
        level: int,
        message: str,
        logger_name: str = "inference",
        **kwargs
    ):
        """
        Log a message.
        
        Args:
            level: Logging level
            message: Log message
            logger_name: Logger name
            **kwargs: Additional log data
        """
        try:
            # Get logger
            logger = self.get_logger(logger_name)
            
            # Create log data
            log_data = kwargs.copy()
            
            # Add standard fields
            if self.include_timestamp:
                log_data["timestamp"] = datetime.now().isoformat()
            
            if self.include_hostname:
                import socket
                log_data["hostname"] = socket.gethostname()
            
            if self.include_pid:
                import os
                log_data["pid"] = os.getpid()
            
            # Add message
            log_data["message"] = message
            
            # Convert log data to JSON
            log_message = json.dumps(log_data)
            
            # Log message
            logger.log(level, log_message)
        except Exception as e:
            # Use default logger
            logger.error(f"Error logging message: {str(e)}")
            logger.log(level, message)
    
    def debug(self, message: str, logger_name: str = "inference", **kwargs):
        """
        Log a debug message.
        
        Args:
            message: Log message
            logger_name: Logger name
            **kwargs: Additional log data
        """
        self.log(logging.DEBUG, message, logger_name, **kwargs)
    
    def info(self, message: str, logger_name: str = "inference", **kwargs):
        """
        Log an info message.
        
        Args:
            message: Log message
            logger_name: Logger name
            **kwargs: Additional log data
        """
        self.log(logging.INFO, message, logger_name, **kwargs)
    
    def warning(self, message: str, logger_name: str = "inference", **kwargs):
        """
        Log a warning message.
        
        Args:
            message: Log message
            logger_name: Logger name
            **kwargs: Additional log data
        """
        self.log(logging.WARNING, message, logger_name, **kwargs)
    
    def error(self, message: str, logger_name: str = "inference", **kwargs):
        """
        Log an error message.
        
        Args:
            message: Log message
            logger_name: Logger name
            **kwargs: Additional log data
        """
        self.log(logging.ERROR, message, logger_name, **kwargs)
    
    def exception(self, message: str, logger_name: str = "inference", **kwargs):
        """
        Log an exception message.
        
        Args:
            message: Log message
            logger_name: Logger name
            **kwargs: Additional log data
        """
        try:
            # Get exception info
            exc_type, exc_value, exc_traceback = sys.exc_info()
            
            # Format traceback
            tb_lines = traceback.format_exception(exc_type, exc_value, exc_traceback)
            tb_text = "".join(tb_lines)
            
            # Add traceback to log data
            kwargs["traceback"] = tb_text
            
            # Log error message
            self.error(message, logger_name, **kwargs)
        except Exception as e:
            # Use default logger
            logger.error(f"Error logging exception: {str(e)}")
            logger.exception(message)


class MetricsCollector:
    """
    Metrics collector for the inference engine.
    
    This class provides metrics collection for the inference engine.
    """
    
    def __init__(
        self,
        metrics_dir: Optional[str] = None,
        collection_interval: float = 60.0,
        max_metrics_files: int = 10,
        max_metrics_age: int = 86400  # 1 day
    ):
        """
        Initialize the metrics collector.
        
        Args:
            metrics_dir: Directory for metrics files
            collection_interval: Metrics collection interval in seconds
            max_metrics_files: Maximum number of metrics files to keep
            max_metrics_age: Maximum age of metrics files in seconds
        """
        # Set metrics directory
        self.metrics_dir = metrics_dir or os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            "../../metrics"
        )
        
        # Create metrics directory if it doesn't exist
        os.makedirs(self.metrics_dir, exist_ok=True)
        
        # Set metrics parameters
        self.collection_interval = collection_interval
        self.max_metrics_files = max_metrics_files
        self.max_metrics_age = max_metrics_age
        
        # Initialize metrics
        self.metrics = {}
        
        # Initialize metrics lock
        self.metrics_lock = threading.Lock()
        
        # Initialize collection thread
        self.collection_thread = None
        self.running = False
        
        # Initialize callbacks
        self.callbacks = []
    
    def start(self):
        """Start metrics collection."""
        try:
            # Check if already running
            if self.running:
                logger.warning("Metrics collector is already running")
                return
            
            # Set running flag
            self.running = True
            
            # Start collection thread
            self.collection_thread = threading.Thread(target=self._collection_loop)
            self.collection_thread.daemon = True
            self.collection_thread.start()
            
            logger.info("Metrics collector started")
        except Exception as e:
            logger.error(f"Error starting metrics collector: {str(e)}")
    
    def stop(self):
        """Stop metrics collection."""
        try:
            # Check if running
            if not self.running:
                logger.warning("Metrics collector is not running")
                return
            
            # Clear running flag
            self.running = False
            
            # Wait for collection thread to finish
            if self.collection_thread:
                self.collection_thread.join(timeout=1.0)
            
            logger.info("Metrics collector stopped")
        except Exception as e:
            logger.error(f"Error stopping metrics collector: {str(e)}")
    
    def _collection_loop(self):
        """Metrics collection loop."""
        try:
            logger.info("Metrics collection thread started")
            
            while self.running:
                try:
                    # Collect metrics
                    self._collect_metrics()
                    
                    # Clean up old metrics files
                    self._clean_metrics_files()
                    
                    # Wait for next collection
                    time.sleep(self.collection_interval)
                except Exception as e:
                    logger.error(f"Error in metrics collection loop: {str(e)}")
                    time.sleep(1.0)
            
            logger.info("Metrics collection thread stopped")
        except Exception as e:
            logger.error(f"Error in metrics collection thread: {str(e)}")
    
    def _collect_metrics(self):
        """Collect metrics."""
        try:
            # Get current time
            current_time = time.time()
            
            # Create metrics data
            metrics_data = {
                "timestamp": current_time,
                "metrics": {}
            }
            
            # Get metrics
            with self.metrics_lock:
                metrics_data["metrics"] = self.metrics.copy()
            
            # Create metrics file path
            metrics_file = os.path.join(
                self.metrics_dir,
                f"metrics_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
            )
            
            # Save metrics to file
            with open(metrics_file, "w") as f:
                json.dump(metrics_data, f, indent=2)
            
            # Call callbacks
            for callback in self.callbacks:
                try:
                    callback(metrics_data)
                except Exception as e:
                    logger.error(f"Error in metrics callback: {str(e)}")
        except Exception as e:
            logger.error(f"Error collecting metrics: {str(e)}")
    
    def _clean_metrics_files(self):
        """Clean up old metrics files."""
        try:
            # Get current time
            current_time = time.time()
            
            # Get metrics files
            metrics_files = []
            for file_path in Path(self.metrics_dir).glob("metrics_*.json"):
                # Get file modification time
                mod_time = file_path.stat().st_mtime
                
                # Add file to list
                metrics_files.append((file_path, mod_time))
            
            # Sort files by modification time (newest first)
            metrics_files.sort(key=lambda x: x[1], reverse=True)
            
            # Delete old files
            for file_path, mod_time in metrics_files[self.max_metrics_files:]:
                try:
                    # Check if file is too old
                    if current_time - mod_time > self.max_metrics_age:
                        # Delete file
                        os.remove(file_path)
                        logger.debug(f"Deleted old metrics file: {file_path}")
                except Exception as e:
                    logger.error(f"Error deleting metrics file {file_path}: {str(e)}")
        except Exception as e:
            logger.error(f"Error cleaning metrics files: {str(e)}")
    
    def set_metric(self, name: str, value: Any):
        """
        Set a metric value.
        
        Args:
            name: Metric name
            value: Metric value
        """
        try:
            # Set metric value
            with self.metrics_lock:
                self.metrics[name] = value
        except Exception as e:
            logger.error(f"Error setting metric {name}: {str(e)}")
    
    def increment_metric(self, name: str, value: float = 1.0):
        """
        Increment a metric value.
        
        Args:
            name: Metric name
            value: Increment value
        """
        try:
            # Increment metric value
            with self.metrics_lock:
                if name not in self.metrics:
                    self.metrics[name] = 0.0
                
                self.metrics[name] += value
        except Exception as e:
            logger.error(f"Error incrementing metric {name}: {str(e)}")
    
    def get_metric(self, name: str, default: Any = None) -> Any:
        """
        Get a metric value.
        
        Args:
            name: Metric name
            default: Default value if the metric doesn't exist
            
        Returns:
            Metric value
        """
        try:
            # Get metric value
            with self.metrics_lock:
                return self.metrics.get(name, default)
        except Exception as e:
            logger.error(f"Error getting metric {name}: {str(e)}")
            return default
    
    def get_metrics(self) -> Dict[str, Any]:
        """
        Get all metrics.
        
        Returns:
            Dictionary with all metrics
        """
        try:
            # Get all metrics
            with self.metrics_lock:
                return self.metrics.copy()
        except Exception as e:
            logger.error(f"Error getting metrics: {str(e)}")
            return {}
    
    def add_callback(self, callback: Callable[[Dict[str, Any]], None]):
        """
        Add a metrics callback.
        
        Args:
            callback: Callback function
        """
        try:
            # Add callback
            self.callbacks.append(callback)
        except Exception as e:
            logger.error(f"Error adding metrics callback: {str(e)}")
    
    def remove_callback(self, callback: Callable[[Dict[str, Any]], None]):
        """
        Remove a metrics callback.
        
        Args:
            callback: Callback function
        """
        try:
            # Remove callback
            if callback in self.callbacks:
                self.callbacks.remove(callback)
        except Exception as e:
            logger.error(f"Error removing metrics callback: {str(e)}")


class AlertManager:
    """
    Alert manager for the inference engine.
    
    This class provides alerting for the inference engine.
    """
    
    def __init__(
        self,
        alert_dir: Optional[str] = None,
        max_alerts: int = 1000,
        max_alert_age: int = 86400 * 7  # 7 days
    ):
        """
        Initialize the alert manager.
        
        Args:
            alert_dir: Directory for alert files
            max_alerts: Maximum number of alerts to keep
            max_alert_age: Maximum age of alerts in seconds
        """
        # Set alert directory
        self.alert_dir = alert_dir or os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            "../../alerts"
        )
        
        # Create alert directory if it doesn't exist
        os.makedirs(self.alert_dir, exist_ok=True)
        
        # Set alert parameters
        self.max_alerts = max_alerts
        self.max_alert_age = max_alert_age
        
        # Initialize alerts
        self.alerts = []
        
        # Initialize alert lock
        self.alert_lock = threading.Lock()
        
        # Initialize callbacks
        self.callbacks = []
        
        # Load alerts
        self._load_alerts()
    
    def _load_alerts(self):
        """Load alerts from files."""
        try:
            # Get alert files
            alert_files = []
            for file_path in Path(self.alert_dir).glob("alert_*.json"):
                # Get file modification time
                mod_time = file_path.stat().st_mtime
                
                # Add file to list
                alert_files.append((file_path, mod_time))
            
            # Sort files by modification time (newest first)
            alert_files.sort(key=lambda x: x[1], reverse=True)
            
            # Load alerts
            with self.alert_lock:
                self.alerts = []
                
                for file_path, _ in alert_files[:self.max_alerts]:
                    try:
                        # Load alert from file
                        with open(file_path, "r") as f:
                            alert = json.load(f)
                        
                        # Add alert to list
                        self.alerts.append(alert)
                    except Exception as e:
                        logger.error(f"Error loading alert from {file_path}: {str(e)}")
            
            logger.info(f"Loaded {len(self.alerts)} alerts")
        except Exception as e:
            logger.error(f"Error loading alerts: {str(e)}")
    
    def _save_alert(self, alert: Dict[str, Any]):
        """
        Save an alert to a file.
        
        Args:
            alert: Alert data
        """
        try:
            # Create alert file path
            alert_file = os.path.join(
                self.alert_dir,
                f"alert_{alert['id']}.json"
            )
            
            # Save alert to file
            with open(alert_file, "w") as f:
                json.dump(alert, f, indent=2)
        except Exception as e:
            logger.error(f"Error saving alert: {str(e)}")
    
    def _clean_alerts(self):
        """Clean up old alerts."""
        try:
            # Get current time
            current_time = time.time()
            
            # Clean up old alerts
            with self.alert_lock:
                # Remove old alerts
                self.alerts = [
                    alert for alert in self.alerts
                    if current_time - alert.get("timestamp", 0) <= self.max_alert_age
                ]
                
                # Limit number of alerts
                if len(self.alerts) > self.max_alerts:
                    self.alerts = self.alerts[:self.max_alerts]
            
            # Clean up old alert files
            alert_files = []
            for file_path in Path(self.alert_dir).glob("alert_*.json"):
                # Get file modification time
                mod_time = file_path.stat().st_mtime
                
                # Add file to list
                alert_files.append((file_path, mod_time))
            
            # Sort files by modification time (newest first)
            alert_files.sort(key=lambda x: x[1], reverse=True)
            
            # Delete old files
            for file_path, mod_time in alert_files[self.max_alerts:]:
                try:
                    # Check if file is too old
                    if current_time - mod_time > self.max_alert_age:
                        # Delete file
                        os.remove(file_path)
                        logger.debug(f"Deleted old alert file: {file_path}")
                except Exception as e:
                    logger.error(f"Error deleting alert file {file_path}: {str(e)}")
        except Exception as e:
            logger.error(f"Error cleaning alerts: {str(e)}")
    
    def create_alert(
        self,
        level: str,
        message: str,
        source: str = "inference",
        **kwargs
    ) -> Dict[str, Any]:
        """
        Create an alert.
        
        Args:
            level: Alert level (info, warning, error, critical)
            message: Alert message
            source: Alert source
            **kwargs: Additional alert data
            
        Returns:
            Alert data
        """
        try:
            # Get current time
            current_time = time.time()
            
            # Create alert ID
            alert_id = str(uuid.uuid4())
            
            # Create alert data
            alert = {
                "id": alert_id,
                "level": level,
                "message": message,
                "source": source,
                "timestamp": current_time,
                "data": kwargs
            }
            
            # Add alert to list
            with self.alert_lock:
                self.alerts.append(alert)
            
            # Save alert to file
            self._save_alert(alert)
            
            # Clean up old alerts
            self._clean_alerts()
            
            # Call callbacks
            for callback in self.callbacks:
                try:
                    callback(alert)
                except Exception as e:
                    logger.error(f"Error in alert callback: {str(e)}")
            
            return alert
        except Exception as e:
            logger.error(f"Error creating alert: {str(e)}")
            
            # Create minimal alert
            return {
                "id": str(uuid.uuid4()),
                "level": level,
                "message": message,
                "source": source,
                "timestamp": time.time()
            }
    
    def get_alerts(
        self,
        level: Optional[str] = None,
        source: Optional[str] = None,
        limit: int = 100
    ) -> List[Dict[str, Any]]:
        """
        Get alerts.
        
        Args:
            level: Filter by alert level
            source: Filter by alert source
            limit: Maximum number of alerts to return
            
        Returns:
            List of alerts
        """
        try:
            # Get alerts
            with self.alert_lock:
                # Filter alerts
                filtered_alerts = self.alerts
                
                if level:
                    filtered_alerts = [
                        alert for alert in filtered_alerts
                        if alert.get("level") == level
                    ]
                
                if source:
                    filtered_alerts = [
                        alert for alert in filtered_alerts
                        if alert.get("source") == source
                    ]
                
                # Limit number of alerts
                if limit > 0:
                    filtered_alerts = filtered_alerts[:limit]
                
                return filtered_alerts
        except Exception as e:
            logger.error(f"Error getting alerts: {str(e)}")
            return []
    
    def get_alert(self, alert_id: str) -> Optional[Dict[str, Any]]:
        """
        Get an alert by ID.
        
        Args:
            alert_id: Alert ID
            
        Returns:
            Alert data if found, None otherwise
        """
        try:
            # Get alert
            with self.alert_lock:
                for alert in self.alerts:
                    if alert.get("id") == alert_id:
                        return alert
            
            return None
        except Exception as e:
            logger.error(f"Error getting alert {alert_id}: {str(e)}")
            return None
    
    def add_callback(self, callback: Callable[[Dict[str, Any]], None]):
        """
        Add an alert callback.
        
        Args:
            callback: Callback function
        """
        try:
            # Add callback
            self.callbacks.append(callback)
        except Exception as e:
            logger.error(f"Error adding alert callback: {str(e)}")
    
    def remove_callback(self, callback: Callable[[Dict[str, Any]], None]):
        """
        Remove an alert callback.
        
        Args:
            callback: Callback function
        """
        try:
            # Remove callback
            if callback in self.callbacks:
                self.callbacks.remove(callback)
        except Exception as e:
            logger.error(f"Error removing alert callback: {str(e)}")


# Create global instances
structured_logger = StructuredLogger()
metrics_collector = MetricsCollector()
alert_manager = AlertManager()
