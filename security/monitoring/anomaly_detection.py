"""
Anomaly detection utilities for Bulo.Cloud Sentinel.

This module provides functions for detecting anomalies in system behavior,
user activity, and network traffic.
"""

import time
import math
import statistics
from typing import Any, Dict, List, Optional, Set, Tuple, Union
from datetime import datetime, timedelta, timezone
from enum import Enum, auto
from collections import defaultdict, deque

from ..logging.security_logger import SecurityLogger, SecurityEventType, SecuritySeverity


class AnomalyType(Enum):
    """
    Types of anomalies.
    """
    
    # Authentication anomalies
    AUTH_FAILURE_RATE = auto()
    AUTH_TIME_ANOMALY = auto()
    AUTH_LOCATION_ANOMALY = auto()
    AUTH_DEVICE_ANOMALY = auto()
    
    # User behavior anomalies
    USER_ACTIVITY_VOLUME = auto()
    USER_ACTIVITY_PATTERN = auto()
    USER_PERMISSION_ANOMALY = auto()
    USER_DATA_ACCESS_ANOMALY = auto()
    
    # API usage anomalies
    API_USAGE_VOLUME = auto()
    API_USAGE_PATTERN = auto()
    API_ERROR_RATE = auto()
    
    # Network anomalies
    NETWORK_TRAFFIC_VOLUME = auto()
    NETWORK_TRAFFIC_PATTERN = auto()
    NETWORK_CONNECTION_ANOMALY = auto()
    
    # System anomalies
    SYSTEM_RESOURCE_USAGE = auto()
    SYSTEM_ERROR_RATE = auto()
    SYSTEM_CONFIGURATION_CHANGE = auto()
    
    # Drone anomalies
    DRONE_TELEMETRY_ANOMALY = auto()
    DRONE_COMMAND_ANOMALY = auto()
    DRONE_POSITION_ANOMALY = auto()
    
    # Other anomalies
    OTHER = auto()


class AnomalyDetector:
    """
    Base class for anomaly detectors.
    """
    
    def __init__(
        self,
        name: str,
        anomaly_type: AnomalyType,
        threshold: float = 0.95,
        logger: Optional[SecurityLogger] = None
    ):
        """
        Initialize the anomaly detector.
        
        Args:
            name: Detector name
            anomaly_type: Type of anomaly to detect
            threshold: Threshold for anomaly detection (0.0 to 1.0)
            logger: Security logger
        """
        self.name = name
        self.anomaly_type = anomaly_type
        self.threshold = threshold
        self.logger = logger or SecurityLogger()
    
    def detect(self, data: Any) -> Tuple[bool, float, Optional[Dict[str, Any]]]:
        """
        Detect anomalies in data.
        
        Args:
            data: Data to analyze
            
        Returns:
            Tuple of (is_anomaly, confidence, details)
        """
        raise NotImplementedError("Subclasses must implement detect()")
    
    def log_anomaly(
        self,
        is_anomaly: bool,
        confidence: float,
        details: Optional[Dict[str, Any]] = None,
        user_id: Optional[str] = None,
        resource_type: Optional[str] = None,
        resource_id: Optional[str] = None,
        ip_address: Optional[str] = None
    ):
        """
        Log an anomaly detection result.
        
        Args:
            is_anomaly: Whether an anomaly was detected
            confidence: Confidence level of the detection
            details: Additional details
            user_id: ID of the user
            resource_type: Type of resource
            resource_id: ID of resource
            ip_address: IP address
        """
        if not is_anomaly:
            return
        
        # Determine severity based on confidence
        if confidence >= 0.9:
            severity = SecuritySeverity.WARNING
        elif confidence >= 0.7:
            severity = SecuritySeverity.INFO
        else:
            severity = SecuritySeverity.DEBUG
        
        # Create details if not provided
        if not details:
            details = {}
        
        # Add detector information to details
        details["detector"] = self.name
        details["anomaly_type"] = self.anomaly_type.name
        details["confidence"] = confidence
        
        # Log the event
        self.logger.log_security_event(
            event_type=SecurityEventType.SUSPICIOUS_ACTIVITY,
            severity=severity,
            user_id=user_id,
            resource_type=resource_type or "anomaly",
            resource_id=resource_id,
            action="anomaly_detected",
            status="detected",
            details=details,
            ip_address=ip_address
        )


class StatisticalAnomalyDetector(AnomalyDetector):
    """
    Anomaly detector based on statistical methods.
    """
    
    def __init__(
        self,
        name: str,
        anomaly_type: AnomalyType,
        window_size: int = 100,
        threshold: float = 0.95,
        z_score_threshold: float = 3.0,
        logger: Optional[SecurityLogger] = None
    ):
        """
        Initialize the statistical anomaly detector.
        
        Args:
            name: Detector name
            anomaly_type: Type of anomaly to detect
            window_size: Size of the sliding window
            threshold: Threshold for anomaly detection (0.0 to 1.0)
            z_score_threshold: Z-score threshold for anomaly detection
            logger: Security logger
        """
        super().__init__(name, anomaly_type, threshold, logger)
        self.window_size = window_size
        self.z_score_threshold = z_score_threshold
        self.values = deque(maxlen=window_size)
    
    def add_value(self, value: float):
        """
        Add a value to the sliding window.
        
        Args:
            value: Value to add
        """
        self.values.append(value)
    
    def detect(self, data: float) -> Tuple[bool, float, Optional[Dict[str, Any]]]:
        """
        Detect anomalies in data using statistical methods.
        
        Args:
            data: Data point to analyze
            
        Returns:
            Tuple of (is_anomaly, confidence, details)
        """
        # Add the value to the sliding window
        self.add_value(data)
        
        # If we don't have enough data, return no anomaly
        if len(self.values) < 10:
            return False, 0.0, None
        
        # Calculate mean and standard deviation
        mean = statistics.mean(self.values)
        stdev = statistics.stdev(self.values) if len(self.values) > 1 else 0.0
        
        # If standard deviation is 0, return no anomaly
        if stdev == 0:
            return False, 0.0, None
        
        # Calculate Z-score
        z_score = abs((data - mean) / stdev)
        
        # Calculate confidence
        confidence = min(1.0, z_score / self.z_score_threshold)
        
        # Determine if it's an anomaly
        is_anomaly = z_score > self.z_score_threshold
        
        # Create details
        details = {
            "value": data,
            "mean": mean,
            "stdev": stdev,
            "z_score": z_score,
            "threshold": self.z_score_threshold
        }
        
        return is_anomaly, confidence, details


class RateAnomalyDetector(AnomalyDetector):
    """
    Anomaly detector for rate-based anomalies.
    """
    
    def __init__(
        self,
        name: str,
        anomaly_type: AnomalyType,
        window_size: int = 10,
        threshold: float = 0.95,
        rate_threshold: float = 0.5,
        logger: Optional[SecurityLogger] = None
    ):
        """
        Initialize the rate anomaly detector.
        
        Args:
            name: Detector name
            anomaly_type: Type of anomaly to detect
            window_size: Size of the sliding window
            threshold: Threshold for anomaly detection (0.0 to 1.0)
            rate_threshold: Rate threshold for anomaly detection
            logger: Security logger
        """
        super().__init__(name, anomaly_type, threshold, logger)
        self.window_size = window_size
        self.rate_threshold = rate_threshold
        self.events = deque(maxlen=window_size)
        self.timestamps = deque(maxlen=window_size)
    
    def add_event(self, event: bool, timestamp: Optional[float] = None):
        """
        Add an event to the sliding window.
        
        Args:
            event: Event (True for success, False for failure)
            timestamp: Event timestamp (current time if not provided)
        """
        self.events.append(event)
        self.timestamps.append(timestamp or time.time())
    
    def detect(self, data: bool) -> Tuple[bool, float, Optional[Dict[str, Any]]]:
        """
        Detect anomalies in event rates.
        
        Args:
            data: Event to analyze (True for success, False for failure)
            
        Returns:
            Tuple of (is_anomaly, confidence, details)
        """
        # Add the event to the sliding window
        self.add_event(data)
        
        # If we don't have enough data, return no anomaly
        if len(self.events) < self.window_size:
            return False, 0.0, None
        
        # Calculate failure rate
        failure_count = sum(1 for event in self.events if not event)
        failure_rate = failure_count / len(self.events)
        
        # Calculate confidence
        confidence = min(1.0, failure_rate / self.rate_threshold)
        
        # Determine if it's an anomaly
        is_anomaly = failure_rate > self.rate_threshold
        
        # Create details
        details = {
            "failure_rate": failure_rate,
            "failure_count": failure_count,
            "total_count": len(self.events),
            "threshold": self.rate_threshold
        }
        
        return is_anomaly, confidence, details


class PatternAnomalyDetector(AnomalyDetector):
    """
    Anomaly detector for pattern-based anomalies.
    """
    
    def __init__(
        self,
        name: str,
        anomaly_type: AnomalyType,
        window_size: int = 100,
        threshold: float = 0.95,
        pattern_threshold: float = 0.5,
        logger: Optional[SecurityLogger] = None
    ):
        """
        Initialize the pattern anomaly detector.
        
        Args:
            name: Detector name
            anomaly_type: Type of anomaly to detect
            window_size: Size of the sliding window
            threshold: Threshold for anomaly detection (0.0 to 1.0)
            pattern_threshold: Pattern threshold for anomaly detection
            logger: Security logger
        """
        super().__init__(name, anomaly_type, threshold, logger)
        self.window_size = window_size
        self.pattern_threshold = pattern_threshold
        self.patterns = defaultdict(int)
        self.total_patterns = 0
        self.sequence = deque(maxlen=window_size)
    
    def add_event(self, event: str):
        """
        Add an event to the sequence.
        
        Args:
            event: Event to add
        """
        self.sequence.append(event)
        
        # If we have at least 2 events, update patterns
        if len(self.sequence) >= 2:
            pattern = (self.sequence[-2], self.sequence[-1])
            self.patterns[pattern] += 1
            self.total_patterns += 1
    
    def detect(self, data: str) -> Tuple[bool, float, Optional[Dict[str, Any]]]:
        """
        Detect anomalies in event patterns.
        
        Args:
            data: Event to analyze
            
        Returns:
            Tuple of (is_anomaly, confidence, details)
        """
        # If we don't have enough data, return no anomaly
        if len(self.sequence) < 1:
            self.add_event(data)
            return False, 0.0, None
        
        # Check if the transition is known
        pattern = (self.sequence[-1], data)
        pattern_count = self.patterns.get(pattern, 0)
        
        # Add the event to the sequence
        self.add_event(data)
        
        # If we don't have enough patterns, return no anomaly
        if self.total_patterns < 10:
            return False, 0.0, None
        
        # Calculate pattern probability
        pattern_probability = pattern_count / self.total_patterns if self.total_patterns > 0 else 0.0
        
        # Calculate anomaly score (1.0 - probability)
        anomaly_score = 1.0 - pattern_probability
        
        # Calculate confidence
        confidence = min(1.0, anomaly_score / (1.0 - self.pattern_threshold))
        
        # Determine if it's an anomaly
        is_anomaly = anomaly_score > (1.0 - self.pattern_threshold)
        
        # Create details
        details = {
            "pattern": pattern,
            "pattern_count": pattern_count,
            "total_patterns": self.total_patterns,
            "pattern_probability": pattern_probability,
            "anomaly_score": anomaly_score,
            "threshold": 1.0 - self.pattern_threshold
        }
        
        return is_anomaly, confidence, details


class AnomalyDetectionManager:
    """
    Manager for anomaly detectors.
    """
    
    def __init__(self, logger: Optional[SecurityLogger] = None):
        """
        Initialize the anomaly detection manager.
        
        Args:
            logger: Security logger
        """
        self.logger = logger or SecurityLogger()
        self.detectors = {}
    
    def add_detector(self, detector: AnomalyDetector):
        """
        Add an anomaly detector.
        
        Args:
            detector: Anomaly detector to add
        """
        self.detectors[detector.name] = detector
    
    def remove_detector(self, name: str):
        """
        Remove an anomaly detector.
        
        Args:
            name: Name of the detector to remove
        """
        if name in self.detectors:
            del self.detectors[name]
    
    def get_detector(self, name: str) -> Optional[AnomalyDetector]:
        """
        Get an anomaly detector by name.
        
        Args:
            name: Name of the detector
            
        Returns:
            Anomaly detector or None if not found
        """
        return self.detectors.get(name)
    
    def detect(
        self,
        detector_name: str,
        data: Any,
        user_id: Optional[str] = None,
        resource_type: Optional[str] = None,
        resource_id: Optional[str] = None,
        ip_address: Optional[str] = None
    ) -> Tuple[bool, float, Optional[Dict[str, Any]]]:
        """
        Detect anomalies using a specific detector.
        
        Args:
            detector_name: Name of the detector to use
            data: Data to analyze
            user_id: ID of the user
            resource_type: Type of resource
            resource_id: ID of resource
            ip_address: IP address
            
        Returns:
            Tuple of (is_anomaly, confidence, details)
            
        Raises:
            ValueError: If the detector is not found
        """
        detector = self.get_detector(detector_name)
        if not detector:
            raise ValueError(f"Detector not found: {detector_name}")
        
        # Detect anomalies
        is_anomaly, confidence, details = detector.detect(data)
        
        # Log the anomaly
        detector.log_anomaly(
            is_anomaly=is_anomaly,
            confidence=confidence,
            details=details,
            user_id=user_id,
            resource_type=resource_type,
            resource_id=resource_id,
            ip_address=ip_address
        )
        
        return is_anomaly, confidence, details


# Create default instance
anomaly_detection_manager = AnomalyDetectionManager()

# Create default detectors
auth_failure_detector = RateAnomalyDetector(
    name="auth_failure_detector",
    anomaly_type=AnomalyType.AUTH_FAILURE_RATE,
    window_size=10,
    rate_threshold=0.3
)

api_usage_detector = StatisticalAnomalyDetector(
    name="api_usage_detector",
    anomaly_type=AnomalyType.API_USAGE_VOLUME,
    window_size=100,
    z_score_threshold=3.0
)

user_activity_detector = PatternAnomalyDetector(
    name="user_activity_detector",
    anomaly_type=AnomalyType.USER_ACTIVITY_PATTERN,
    window_size=100,
    pattern_threshold=0.1
)

# Add detectors to manager
anomaly_detection_manager.add_detector(auth_failure_detector)
anomaly_detection_manager.add_detector(api_usage_detector)
anomaly_detection_manager.add_detector(user_activity_detector)
