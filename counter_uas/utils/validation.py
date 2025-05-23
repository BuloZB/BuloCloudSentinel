"""
Validation utilities for the Counter-UAS module.

This module provides utilities for validating input data.
"""

import re
from typing import Any, Dict, List, Optional, Union, Callable


def validate_frequency(frequency: int) -> bool:
    """
    Validate a frequency value.
    
    Args:
        frequency: Frequency in Hz.
        
    Returns:
        bool: True if the frequency is valid, False otherwise.
    """
    # Frequency must be positive
    if frequency <= 0:
        return False
    
    # Frequency must be within reasonable range (1 kHz to 10 GHz)
    if frequency < 1000 or frequency > 10000000000:
        return False
    
    return True


def validate_sample_rate(sample_rate: int) -> bool:
    """
    Validate a sample rate value.
    
    Args:
        sample_rate: Sample rate in samples per second.
        
    Returns:
        bool: True if the sample rate is valid, False otherwise.
    """
    # Sample rate must be positive
    if sample_rate <= 0:
        return False
    
    # Sample rate must be within reasonable range (1 kHz to 100 MHz)
    if sample_rate < 1000 or sample_rate > 100000000:
        return False
    
    return True


def validate_gain(gain: float) -> bool:
    """
    Validate a gain value.
    
    Args:
        gain: Gain in dB.
        
    Returns:
        bool: True if the gain is valid, False otherwise.
    """
    # Gain must be within reasonable range (-30 dB to 70 dB)
    if gain < -30.0 or gain > 70.0:
        return False
    
    return True


def validate_range(start_m: float, end_m: float) -> bool:
    """
    Validate a range.
    
    Args:
        start_m: Start range in meters.
        end_m: End range in meters.
        
    Returns:
        bool: True if the range is valid, False otherwise.
    """
    # Range must be positive
    if start_m < 0.0 or end_m < 0.0:
        return False
    
    # Start must be less than end
    if start_m >= end_m:
        return False
    
    # Range must be within reasonable limits (0 to 1000 meters)
    if start_m > 1000.0 or end_m > 1000.0:
        return False
    
    return True


def validate_update_rate(rate_hz: float) -> bool:
    """
    Validate an update rate.
    
    Args:
        rate_hz: Update rate in Hz.
        
    Returns:
        bool: True if the update rate is valid, False otherwise.
    """
    # Update rate must be positive
    if rate_hz <= 0.0:
        return False
    
    # Update rate must be within reasonable limits (0.1 Hz to 1000 Hz)
    if rate_hz < 0.1 or rate_hz > 1000.0:
        return False
    
    return True


def validate_ip_address(ip: str) -> bool:
    """
    Validate an IP address.
    
    Args:
        ip: IP address.
        
    Returns:
        bool: True if the IP address is valid, False otherwise.
    """
    # Simple regex for IPv4 address
    pattern = r"^(\d{1,3})\.(\d{1,3})\.(\d{1,3})\.(\d{1,3})$"
    match = re.match(pattern, ip)
    
    if not match:
        return False
    
    # Check that each octet is in range 0-255
    for octet in match.groups():
        if int(octet) > 255:
            return False
    
    return True


def validate_port(port: int) -> bool:
    """
    Validate a port number.
    
    Args:
        port: Port number.
        
    Returns:
        bool: True if the port number is valid, False otherwise.
    """
    # Port must be in range 1-65535
    if port < 1 or port > 65535:
        return False
    
    return True


def validate_dict(data: Dict[str, Any], schema: Dict[str, Dict[str, Any]]) -> List[str]:
    """
    Validate a dictionary against a schema.
    
    Args:
        data: Dictionary to validate.
        schema: Schema to validate against. Each key in the schema is a key in the data,
                and each value is a dictionary with the following keys:
                - required: Whether the key is required.
                - type: The expected type of the value.
                - validator: Optional function to validate the value.
                
    Returns:
        List[str]: List of validation errors. Empty if validation passed.
    """
    errors = []
    
    # Check required keys
    for key, key_schema in schema.items():
        if key_schema.get("required", False) and key not in data:
            errors.append(f"Missing required key: {key}")
    
    # Check types and validators
    for key, value in data.items():
        if key in schema:
            key_schema = schema[key]
            
            # Check type
            expected_type = key_schema.get("type")
            if expected_type and not isinstance(value, expected_type):
                errors.append(f"Invalid type for {key}: expected {expected_type.__name__}, got {type(value).__name__}")
            
            # Check validator
            validator = key_schema.get("validator")
            if validator and not validator(value):
                errors.append(f"Validation failed for {key}")
    
    return errors
