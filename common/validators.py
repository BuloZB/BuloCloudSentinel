"""
Common validators for the BuloCloud Sentinel project.

This module provides a set of common validators that can be used across all modules.
"""

from __future__ import annotations

import re
from datetime import datetime, UTC
from typing import Any, Callable, List, Optional, Pattern, TypeVar, Union

from pydantic import BaseModel, Field, field_validator

from common.exceptions import ValidationError

# Type variables for generic functions
T = TypeVar("T")


def validate_latitude(latitude: float) -> float:
    """
    Validate a latitude value.

    Args:
        latitude: The latitude value to validate

    Returns:
        The validated latitude value

    Raises:
        ValidationError: If the latitude is invalid
    """
    if not -90 <= latitude <= 90:
        raise ValidationError(
            message="Latitude must be between -90 and 90 degrees",
            field="latitude",
        )
    return latitude


def validate_longitude(longitude: float) -> float:
    """
    Validate a longitude value.

    Args:
        longitude: The longitude value to validate

    Returns:
        The validated longitude value

    Raises:
        ValidationError: If the longitude is invalid
    """
    if not -180 <= longitude <= 180:
        raise ValidationError(
            message="Longitude must be between -180 and 180 degrees",
            field="longitude",
        )
    return longitude


def validate_future_date(date: datetime) -> datetime:
    """
    Validate that a date is in the future.

    Args:
        date: The date to validate

    Returns:
        The validated date

    Raises:
        ValidationError: If the date is not in the future
    """
    now = datetime.now(UTC)
    if date <= now:
        raise ValidationError(
            message="Date must be in the future",
            field="date",
        )
    return date


def validate_string_length(
    min_length: Optional[int] = None, max_length: Optional[int] = None
) -> Callable[[str], str]:
    """
    Create a validator for string length.

    Args:
        min_length: Minimum length of the string (optional)
        max_length: Maximum length of the string (optional)

    Returns:
        A validator function for string length
    """
    def validator_func(value: str) -> str:
        if min_length is not None and len(value) < min_length:
            raise ValidationError(
                message=f"String must be at least {min_length} characters long",
                field="string",
            )
        if max_length is not None and len(value) > max_length:
            raise ValidationError(
                message=f"String must be at most {max_length} characters long",
                field="string",
            )
        return value
    return validator_func


def validate_regex(pattern: Union[str, Pattern]) -> Callable[[str], str]:
    """
    Create a validator for regex pattern matching.

    Args:
        pattern: The regex pattern to match

    Returns:
        A validator function for regex pattern matching
    """
    if isinstance(pattern, str):
        compiled_pattern = re.compile(pattern)
    else:
        compiled_pattern = pattern

    def validator_func(value: str) -> str:
        if not compiled_pattern.match(value):
            raise ValidationError(
                message="String does not match the required pattern",
                field="string",
            )
        return value
    return validator_func


def validate_enum(enum_values: List[Any]) -> Callable[[Any], Any]:
    """
    Create a validator for enum values.

    Args:
        enum_values: The list of valid enum values

    Returns:
        A validator function for enum values
    """
    def validator_func(value: Any) -> Any:
        if value not in enum_values:
            raise ValidationError(
                message=f"Value must be one of {enum_values}",
                field="enum",
            )
        return value
    return validator_func


def validate_numeric_range(
    min_value: Optional[float] = None, max_value: Optional[float] = None
) -> Callable[[float], float]:
    """
    Create a validator for numeric range.

    Args:
        min_value: Minimum value (optional)
        max_value: Maximum value (optional)

    Returns:
        A validator function for numeric range
    """
    def validator_func(value: float) -> float:
        if min_value is not None and value < min_value:
            raise ValidationError(
                message=f"Value must be at least {min_value}",
                field="numeric",
            )
        if max_value is not None and value > max_value:
            raise ValidationError(
                message=f"Value must be at most {max_value}",
                field="numeric",
            )
        return value
    return validator_func


class GeoCoordinates(BaseModel):
    """Model for geographic coordinates."""

    latitude: float = Field(..., ge=-90, le=90, description="Latitude in degrees")
    longitude: float = Field(..., ge=-180, le=180, description="Longitude in degrees")
    altitude: Optional[float] = Field(None, description="Altitude in meters")

    @field_validator("latitude")
    def validate_latitude(cls, v: float) -> float:
        """Validate latitude."""
        return validate_latitude(v)

    @field_validator("longitude")
    def validate_longitude(cls, v: float) -> float:
        """Validate longitude."""
        return validate_longitude(v)
