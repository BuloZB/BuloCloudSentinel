"""
Tests for the common utils module.

This module contains tests for the common utils module.
"""

from __future__ import annotations

import asyncio
import time
from datetime import datetime, UTC
from typing import Any

import pytest
from pytest_mock import MockerFixture

from common.utils import (
    timed,
    async_timed,
    utc_now,
    format_datetime,
    parse_datetime,
    truncate_string,
    safe_divide,
)


def test_timed_decorator(mocker: MockerFixture) -> None:
    """Test the timed decorator."""
    # Mock the logger
    mock_logger = mocker.patch("common.utils.logger")

    # Define a function to decorate
    @timed
    def example_function(x: int, y: int) -> int:
        time.sleep(0.1)  # Sleep to ensure measurable execution time
        return x + y

    # Call the decorated function
    result = example_function(2, 3)

    # Check the result
    assert result == 5

    # Check that the logger was called
    mock_logger.debug.assert_called_once()
    # The first argument is the format string, which doesn't have the function name yet
    # It will be formatted with the function name and execution time


@pytest.mark.asyncio
async def test_async_timed_decorator(mocker: MockerFixture) -> None:
    """Test the async_timed decorator."""
    # Mock the logger
    mock_logger = mocker.patch("common.utils.logger")

    # Define an async function to decorate
    @async_timed
    async def example_async_function(x: int, y: int) -> int:
        await asyncio.sleep(0.1)  # Sleep to ensure measurable execution time
        return x + y

    # Call the decorated function
    result = await example_async_function(2, 3)

    # Check the result
    assert result == 5

    # Check that the logger was called
    mock_logger.debug.assert_called_once()
    # The first argument is the format string, which doesn't have the function name yet
    # It will be formatted with the function name and execution time


def test_utc_now() -> None:
    """Test the utc_now function."""
    # Get the current time
    now = utc_now()

    # Check that it's a datetime object
    assert isinstance(now, datetime)

    # Check that it has UTC timezone
    assert now.tzinfo is UTC


def test_format_datetime() -> None:
    """Test the format_datetime function."""
    # Create a datetime object
    dt = datetime(2023, 1, 15, 12, 30, 45, tzinfo=UTC)

    # Format with default format
    formatted = format_datetime(dt)
    assert formatted == "2023-01-15 12:30:45"

    # Format with custom format
    formatted = format_datetime(dt, fmt="%Y/%m/%d %H:%M")
    assert formatted == "2023/01/15 12:30"


def test_parse_datetime() -> None:
    """Test the parse_datetime function."""
    # Parse a datetime string with default format
    dt_str = "2023-01-15 12:30:45"
    dt = parse_datetime(dt_str)

    # Check the parsed datetime
    assert dt.year == 2023
    assert dt.month == 1
    assert dt.day == 15
    assert dt.hour == 12
    assert dt.minute == 30
    assert dt.second == 45
    assert dt.tzinfo is UTC

    # Parse with custom format
    dt_str = "2023/01/15 12:30"
    dt = parse_datetime(dt_str, fmt="%Y/%m/%d %H:%M")

    # Check the parsed datetime
    assert dt.year == 2023
    assert dt.month == 1
    assert dt.day == 15
    assert dt.hour == 12
    assert dt.minute == 30
    assert dt.second == 0
    assert dt.tzinfo is UTC


def test_truncate_string() -> None:
    """Test the truncate_string function."""
    # Test with a short string
    short_str = "Hello, world!"
    truncated = truncate_string(short_str, max_length=100)
    assert truncated == short_str

    # Test with a long string
    long_str = "This is a very long string that needs to be truncated."
    truncated = truncate_string(long_str, max_length=20)
    assert truncated == "This is a very lo..."
    assert len(truncated) == 20

    # Test with custom suffix
    truncated = truncate_string(long_str, max_length=20, suffix="[...]")
    assert truncated == "This is a very [...]"
    assert len(truncated) == 20


def test_safe_divide() -> None:
    """Test the safe_divide function."""
    # Test normal division
    result = safe_divide(10, 2)
    assert result == 5.0

    # Test division by zero with default value
    result = safe_divide(10, 0)
    assert result == 0.0

    # Test division by zero with custom default value
    result = safe_divide(10, 0, default=-1.0)
    assert result == -1.0
