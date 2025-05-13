"""
Tests for the common validators module.

This module contains tests for the common validators module.
"""

from __future__ import annotations

import re
from datetime import datetime, timedelta

import pytest

from common import utc_now
from common.exceptions import ValidationError
from common.validators import (
    validate_latitude,
    validate_longitude,
    validate_future_date,
    validate_string_length,
    validate_regex,
    validate_enum,
    validate_numeric_range,
    GeoCoordinates,
)


def test_validate_latitude() -> None:
    """Test the validate_latitude function."""
    # Test valid latitudes
    assert validate_latitude(0) == 0
    assert validate_latitude(90) == 90
    assert validate_latitude(-90) == -90
    assert validate_latitude(45.5) == 45.5
    
    # Test invalid latitudes
    with pytest.raises(ValidationError) as excinfo:
        validate_latitude(91)
    assert "Latitude must be between -90 and 90 degrees" in str(excinfo.value)
    
    with pytest.raises(ValidationError) as excinfo:
        validate_latitude(-91)
    assert "Latitude must be between -90 and 90 degrees" in str(excinfo.value)


def test_validate_longitude() -> None:
    """Test the validate_longitude function."""
    # Test valid longitudes
    assert validate_longitude(0) == 0
    assert validate_longitude(180) == 180
    assert validate_longitude(-180) == -180
    assert validate_longitude(45.5) == 45.5
    
    # Test invalid longitudes
    with pytest.raises(ValidationError) as excinfo:
        validate_longitude(181)
    assert "Longitude must be between -180 and 180 degrees" in str(excinfo.value)
    
    with pytest.raises(ValidationError) as excinfo:
        validate_longitude(-181)
    assert "Longitude must be between -180 and 180 degrees" in str(excinfo.value)


def test_validate_future_date() -> None:
    """Test the validate_future_date function."""
    # Test valid future date
    future_date = utc_now() + timedelta(days=1)
    assert validate_future_date(future_date) == future_date
    
    # Test invalid past date
    past_date = utc_now() - timedelta(days=1)
    with pytest.raises(ValidationError) as excinfo:
        validate_future_date(past_date)
    assert "Date must be in the future" in str(excinfo.value)
    
    # Test invalid current date
    current_date = utc_now()
    with pytest.raises(ValidationError) as excinfo:
        validate_future_date(current_date)
    assert "Date must be in the future" in str(excinfo.value)


def test_validate_string_length() -> None:
    """Test the validate_string_length function."""
    # Create validators
    min_validator = validate_string_length(min_length=5)
    max_validator = validate_string_length(max_length=10)
    range_validator = validate_string_length(min_length=5, max_length=10)
    
    # Test min_validator
    assert min_validator("hello") == "hello"
    assert min_validator("hello world") == "hello world"
    
    with pytest.raises(ValidationError) as excinfo:
        min_validator("hi")
    assert "String must be at least 5 characters long" in str(excinfo.value)
    
    # Test max_validator
    assert max_validator("hello") == "hello"
    assert max_validator("helloworld") == "helloworld"
    
    with pytest.raises(ValidationError) as excinfo:
        max_validator("hello world!")
    assert "String must be at most 10 characters long" in str(excinfo.value)
    
    # Test range_validator
    assert range_validator("hello") == "hello"
    assert range_validator("helloworl") == "helloworl"
    
    with pytest.raises(ValidationError) as excinfo:
        range_validator("hi")
    assert "String must be at least 5 characters long" in str(excinfo.value)
    
    with pytest.raises(ValidationError) as excinfo:
        range_validator("hello world!")
    assert "String must be at most 10 characters long" in str(excinfo.value)


def test_validate_regex() -> None:
    """Test the validate_regex function."""
    # Create validators
    email_validator = validate_regex(r"^[a-zA-Z0-9_.+-]+@[a-zA-Z0-9-]+\.[a-zA-Z0-9-.]+$")
    phone_validator = validate_regex(re.compile(r"^\d{3}-\d{3}-\d{4}$"))
    
    # Test email_validator
    assert email_validator("user@example.com") == "user@example.com"
    
    with pytest.raises(ValidationError) as excinfo:
        email_validator("invalid-email")
    assert "String does not match the required pattern" in str(excinfo.value)
    
    # Test phone_validator
    assert phone_validator("123-456-7890") == "123-456-7890"
    
    with pytest.raises(ValidationError) as excinfo:
        phone_validator("1234567890")
    assert "String does not match the required pattern" in str(excinfo.value)


def test_validate_enum() -> None:
    """Test the validate_enum function."""
    # Create validator
    color_validator = validate_enum(["red", "green", "blue"])
    
    # Test valid values
    assert color_validator("red") == "red"
    assert color_validator("green") == "green"
    assert color_validator("blue") == "blue"
    
    # Test invalid value
    with pytest.raises(ValidationError) as excinfo:
        color_validator("yellow")
    assert "Value must be one of ['red', 'green', 'blue']" in str(excinfo.value)


def test_validate_numeric_range() -> None:
    """Test the validate_numeric_range function."""
    # Create validators
    min_validator = validate_numeric_range(min_value=0)
    max_validator = validate_numeric_range(max_value=100)
    range_validator = validate_numeric_range(min_value=0, max_value=100)
    
    # Test min_validator
    assert min_validator(0) == 0
    assert min_validator(50) == 50
    assert min_validator(100) == 100
    
    with pytest.raises(ValidationError) as excinfo:
        min_validator(-1)
    assert "Value must be at least 0" in str(excinfo.value)
    
    # Test max_validator
    assert max_validator(0) == 0
    assert max_validator(50) == 50
    assert max_validator(100) == 100
    
    with pytest.raises(ValidationError) as excinfo:
        max_validator(101)
    assert "Value must be at most 100" in str(excinfo.value)
    
    # Test range_validator
    assert range_validator(0) == 0
    assert range_validator(50) == 50
    assert range_validator(100) == 100
    
    with pytest.raises(ValidationError) as excinfo:
        range_validator(-1)
    assert "Value must be at least 0" in str(excinfo.value)
    
    with pytest.raises(ValidationError) as excinfo:
        range_validator(101)
    assert "Value must be at most 100" in str(excinfo.value)


def test_geo_coordinates() -> None:
    """Test the GeoCoordinates model."""
    # Test valid coordinates
    coords = GeoCoordinates(latitude=40.7128, longitude=-74.0060)
    assert coords.latitude == 40.7128
    assert coords.longitude == -74.0060
    assert coords.altitude is None
    
    coords = GeoCoordinates(latitude=40.7128, longitude=-74.0060, altitude=10.5)
    assert coords.latitude == 40.7128
    assert coords.longitude == -74.0060
    assert coords.altitude == 10.5
    
    # Test invalid latitude
    with pytest.raises(ValueError) as excinfo:
        GeoCoordinates(latitude=91, longitude=-74.0060)
    assert "latitude" in str(excinfo.value)
    assert "less than or equal to 90" in str(excinfo.value)
    
    # Test invalid longitude
    with pytest.raises(ValueError) as excinfo:
        GeoCoordinates(latitude=40.7128, longitude=181)
    assert "longitude" in str(excinfo.value)
    assert "less than or equal to 180" in str(excinfo.value)
