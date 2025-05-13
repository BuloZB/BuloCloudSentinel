"""
Pytest configuration file.

This file contains fixtures and configuration for pytest.
"""

from __future__ import annotations

from typing import Dict

import pytest

from common import setup_logging


@pytest.fixture(scope="session", autouse=True)
def setup_test_logging() -> None:
    """Set up logging for tests."""
    setup_logging(log_level="DEBUG")


# We don't need to define our own event_loop fixture anymore
# pytest-asyncio provides one for us


@pytest.fixture
def mock_env_vars(monkeypatch: pytest.MonkeyPatch) -> Dict[str, str]:
    """Mock environment variables."""
    env_vars = {
        "DATABASE_URL": "postgresql://user:password@localhost:5432/testdb",
        "JWT_SECRET": "test_secret",
        "MQTT_ENABLED": "true",
        "MQTT_HOST": "localhost",
        "MQTT_PORT": "1883",
        "MQTT_USERNAME": "test_user",
        "MQTT_PASSWORD": "test_password",
        "REDIS_URL": "redis://localhost:6379/0",
        "WIND_THRESHOLD": "10.0",
        "RAIN_THRESHOLD": "5.0",
    }

    for key, value in env_vars.items():
        monkeypatch.setenv(key, value)

    return env_vars
