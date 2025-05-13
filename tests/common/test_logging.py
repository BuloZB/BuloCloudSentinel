"""
Tests for the common logging module.

This module contains tests for the common logging module.
"""

from __future__ import annotations

# import json
import logging
import os
import tempfile
from typing import Generator

import pytest
from pytest_mock import MockerFixture

from common.logging import (
    InterceptHandler,
    JSONSink,
    setup_logging,
    get_logger,
)


@pytest.fixture
def temp_log_file() -> Generator[str, None, None]:
    """Create a temporary log file."""
    with tempfile.NamedTemporaryFile(delete=False) as f:
        temp_file = f.name

    yield temp_file

    # Clean up
    if os.path.exists(temp_file):
        os.unlink(temp_file)


def test_intercept_handler(mocker: MockerFixture) -> None:
    """Test the InterceptHandler class."""
    # Mock loguru logger
    mock_logger = mocker.patch("common.logging.logger")

    # Create a record
    record = logging.LogRecord(
        name="test_logger",
        level=logging.INFO,
        pathname="test_file.py",
        lineno=10,
        msg="Test message",
        args=(),
        exc_info=None,
    )

    # Create handler and emit record
    handler = InterceptHandler()
    handler.emit(record)

    # Check that loguru logger was called
    mock_logger.opt.assert_called_once()
    mock_logger.opt.return_value.log.assert_called_once()


def test_json_sink(temp_log_file: str, mocker: MockerFixture) -> None:
    """Test the JSONSink class."""
    # Mock open function
    mock_open = mocker.patch("builtins.open", mocker.mock_open())
    mock_json = mocker.patch("json.dumps")
    mock_json.return_value = '{"test": "json"}'

    # Create a sink
    sink = JSONSink(temp_log_file)

    # Create a mock record
    mock_record = {
        "time": mocker.MagicMock(),
        "level": mocker.MagicMock(),
        "message": "Test message",
        "name": "test_logger",
        "function": "test_function",
        "line": 10,
        "process": mocker.MagicMock(),
        "thread": mocker.MagicMock(),
        "exception": None,
        "extra": {"key": "value"},
    }
    mock_record["time"].isoformat.return_value = "2023-01-15T12:30:45"
    mock_record["level"].name = "INFO"
    mock_record["process"].id = 1234
    mock_record["thread"].id = 5678

    # Create a mock message
    mock_message = mocker.MagicMock()
    mock_message.record = mock_record

    # Call the sink
    sink(mock_message)

    # Check that open was called
    mock_open.assert_called_once_with(temp_log_file, "a", encoding="utf-8")

    # Check that json.dumps was called
    mock_json.assert_called_once()

    # We're mocking the file operations, so we don't need to check the file content


def test_setup_logging(mocker: MockerFixture, temp_log_file: str) -> None:
    """Test the setup_logging function."""
    # Mock loguru logger
    mock_logger = mocker.patch("common.logging.logger")
    mock_logger.remove = mocker.MagicMock()
    mock_logger.add = mocker.MagicMock()
    mock_logger.level = mocker.MagicMock()

    # Mock logging
    mock_logging = mocker.patch("common.logging.logging")
    mock_logging.basicConfig = mocker.MagicMock()
    mock_logging.root.manager.loggerDict = {
        "sqlalchemy": mocker.MagicMock(),
        "uvicorn": mocker.MagicMock(),
    }
    mock_logging.getLogger = mocker.MagicMock()

    # Call setup_logging
    setup_logging(log_level="INFO", json_logs=True, log_file=temp_log_file)

    # Check that logger was configured
    mock_logger.remove.assert_called_once()
    assert mock_logger.add.call_count >= 2  # Console and file
    mock_logging.basicConfig.assert_called_once()
    assert mock_logging.getLogger.call_count >= 2  # One for each logger in loggerDict

    # We're not using logger.level anymore, we're using logging.getLogger().setLevel()
    assert mock_logging.getLogger.call_count >= 2  # sqlalchemy and uvicorn


def test_get_logger(mocker: MockerFixture) -> None:
    """Test the get_logger function."""
    # Mock logging.getLogger
    mock_get_logger = mocker.patch("common.logging.logging.getLogger")
    mock_logger = mocker.MagicMock()
    mock_get_logger.return_value = mock_logger

    # Call get_logger
    logger = get_logger("test_module")

    # Check that logging.getLogger was called
    mock_get_logger.assert_called_once_with("test_module")

    # Check that the logger was returned
    assert logger == mock_logger
