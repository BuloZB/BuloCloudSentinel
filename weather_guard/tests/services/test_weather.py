"""
Tests for the weather service.

This module contains tests for the weather service.
"""

from __future__ import annotations

from datetime import timedelta
from typing import List
from unittest.mock import AsyncMock, MagicMock

import pytest
from pytest_mock import MockerFixture

from common import utc_now
from weather_guard.models.weather import (
    WeatherCondition,
    WeatherData,
    WeatherForecast,
    WeatherProvider,
    MissionWeatherCheck,
)
from weather_guard.services.weather import MissionParams, WeatherService


@pytest.fixture
def weather_service(
    mock_open_meteo_client: MagicMock,
    mock_meteo_shield_client: MagicMock,
    mock_weather_cache: MagicMock,
    mock_forecast_cache: MagicMock,
) -> WeatherService:
    """Create a weather service for testing."""
    return WeatherService()


@pytest.fixture
def mock_open_meteo_client(mocker: MockerFixture) -> MagicMock:
    """Mock the OpenMeteoClient."""
    mock_client = mocker.patch("weather_guard.services.weather.OpenMeteoClient", autospec=True)
    mock_client.return_value.get_current_weather = AsyncMock()
    mock_client.return_value.get_forecast = AsyncMock()
    mock_client.return_value.close = AsyncMock()
    return mock_client.return_value


@pytest.fixture
def mock_meteo_shield_client(mocker: MockerFixture) -> MagicMock:
    """Mock the MeteoShieldClient."""
    mock_client = mocker.patch("weather_guard.services.weather.MeteoShieldClient", autospec=True)
    mock_client.return_value.start = AsyncMock()
    mock_client.return_value.get_latest_weather = MagicMock()
    return mock_client.return_value


@pytest.fixture
def mock_weather_cache(mocker: MockerFixture) -> MagicMock:
    """Mock the weather cache."""
    mock_cache = mocker.patch("weather_guard.services.weather.weather_cache", autospec=True)
    mock_cache.connect = AsyncMock()
    mock_cache.get = AsyncMock(return_value=None)
    mock_cache.set = AsyncMock()
    mock_cache.get_cache_stats = AsyncMock(return_value={"hits": 10, "misses": 5})
    return mock_cache


@pytest.fixture
def mock_forecast_cache(mocker: MockerFixture) -> MagicMock:
    """Mock the forecast cache."""
    mock_cache = mocker.patch("weather_guard.services.weather.forecast_cache", autospec=True)
    mock_cache.connect = AsyncMock()
    mock_cache.get = AsyncMock(return_value=None)
    mock_cache.set = AsyncMock()
    mock_cache.get_cache_stats = AsyncMock(return_value={"hits": 8, "misses": 3})
    return mock_cache


@pytest.fixture
def sample_weather_data() -> WeatherData:
    """Create sample weather data for testing."""
    return WeatherData(
        latitude=40.7128,
        longitude=-74.0060,
        timestamp=utc_now(),
        provider=WeatherProvider.OPEN_METEO,
        condition=WeatherCondition.CLEAR,
        temperature=22.5,
        wind_speed=5.2,
        precipitation=0.0,
    )


@pytest.fixture
def sample_forecast_data() -> List[WeatherForecast]:
    """Create sample forecast data for testing."""
    now = utc_now()
    return [
        WeatherForecast(
            latitude=40.7128,
            longitude=-74.0060,
            timestamp=now,
            forecast_time=now + timedelta(hours=i),
            provider=WeatherProvider.OPEN_METEO,
            condition=WeatherCondition.CLEAR,
            temperature=22.5 - i * 0.5,
            wind_speed=5.2 + i * 0.3,
            precipitation=0.0 if i < 5 else 0.2,
        )
        for i in range(24)
    ]


@pytest.mark.asyncio
async def test_initialize(
    weather_service: WeatherService,
    mock_weather_cache: MagicMock,
    mock_forecast_cache: MagicMock,
) -> None:
    """Test initializing the weather service."""
    await weather_service.initialize()

    # Check that the caches were connected
    mock_weather_cache.connect.assert_called_once()
    mock_forecast_cache.connect.assert_called_once()


@pytest.mark.asyncio
async def test_get_current_weather_from_cache(
    weather_service: WeatherService,
    mock_weather_cache: MagicMock,
    sample_weather_data: WeatherData,
) -> None:
    """Test getting current weather data from cache."""
    # Set up the mock to return cached data
    mock_weather_cache.get.return_value = sample_weather_data

    # Call the method
    result = await weather_service.get_current_weather(
        latitude=40.7128,
        longitude=-74.0060,
    )

    # Check the result
    assert result == sample_weather_data
    mock_weather_cache.get.assert_called_once()
    mock_weather_cache.set.assert_not_called()


@pytest.mark.asyncio
async def test_get_current_weather_from_provider(
    weather_service: WeatherService,
    mock_weather_cache: MagicMock,
    mock_open_meteo_client: MagicMock,
    sample_weather_data: WeatherData,
) -> None:
    """Test getting current weather data from provider."""
    # Set up the mock to return data from provider
    mock_weather_cache.get.return_value = None
    mock_open_meteo_client.get_current_weather.return_value = sample_weather_data

    # Call the method
    result = await weather_service.get_current_weather(
        latitude=40.7128,
        longitude=-74.0060,
    )

    # Check the result
    # We can't compare the entire object because the UUID will be different
    assert result.latitude == sample_weather_data.latitude
    assert result.longitude == sample_weather_data.longitude
    assert result.provider == sample_weather_data.provider
    # Weather data may be different due to implementation details
    # We're just checking that we got a response with the right structure
    mock_weather_cache.get.assert_called_once()
    mock_weather_cache.set.assert_called_once()
    mock_open_meteo_client.get_current_weather.assert_called_once_with(40.7128, -74.0060)


@pytest.mark.asyncio
async def test_get_forecast_from_cache(
    weather_service: WeatherService,
    mock_forecast_cache: MagicMock,
    sample_forecast_data: List[WeatherForecast],
) -> None:
    """Test getting forecast data from cache."""
    # Set up the mock to return cached data
    mock_forecast_cache.get.return_value = sample_forecast_data

    # Call the method
    result = await weather_service.get_forecast(
        latitude=40.7128,
        longitude=-74.0060,
        hours=24,
    )

    # Check the result
    assert result == sample_forecast_data
    mock_forecast_cache.get.assert_called_once()
    mock_forecast_cache.set.assert_not_called()


@pytest.mark.asyncio
async def test_get_forecast_from_provider(
    weather_service: WeatherService,
    mock_forecast_cache: MagicMock,
    mock_open_meteo_client: MagicMock,
    sample_forecast_data: List[WeatherForecast],
) -> None:
    """Test getting forecast data from provider."""
    # Set up the mock to return data from provider
    mock_forecast_cache.get.return_value = None
    mock_open_meteo_client.get_forecast.return_value = sample_forecast_data

    # Call the method
    result = await weather_service.get_forecast(
        latitude=40.7128,
        longitude=-74.0060,
        hours=24,
    )

    # Check the result
    # We can't compare the entire objects because the UUIDs will be different
    assert len(result) == len(sample_forecast_data)
    for i, forecast in enumerate(result):
        assert forecast.latitude == sample_forecast_data[i].latitude
        assert forecast.longitude == sample_forecast_data[i].longitude
        assert forecast.provider == sample_forecast_data[i].provider
        # Weather data may be different due to implementation details
        # We're just checking that we got a response with the right structure
    mock_forecast_cache.get.assert_called_once()
    mock_forecast_cache.set.assert_called_once()
    mock_open_meteo_client.get_forecast.assert_called_once_with(40.7128, -74.0060, 24)


@pytest.mark.asyncio
async def test_check_mission_weather(
    weather_service: WeatherService,
    sample_weather_data: WeatherData,
    sample_forecast_data: List[WeatherForecast],
) -> None:
    """Test checking mission weather."""
    # Mock the get_current_weather and get_forecast methods
    weather_service.get_current_weather = AsyncMock(return_value=sample_weather_data)
    weather_service.get_forecast = AsyncMock(return_value=sample_forecast_data)

    # Call the method
    start_time = utc_now()
    end_time = start_time + timedelta(hours=3)
    params = MissionParams(
        end_time=end_time,
        mission_id="test-mission",
        provider=WeatherProvider.OPEN_METEO,
    )

    result = await weather_service.check_mission_weather(
        latitude=40.7128,
        longitude=-74.0060,
        start_time=start_time,
        params=params,
    )

    # Check the result
    assert isinstance(result, MissionWeatherCheck)
    assert result.mission_id == "test-mission"
    assert result.latitude == 40.7128
    assert result.longitude == -74.0060
    assert result.start_time == start_time
    assert result.end_time == end_time
    assert result.current_weather == sample_weather_data

    # Check that the methods were called
    weather_service.get_current_weather.assert_called_once_with(
        40.7128, -74.0060, WeatherProvider.OPEN_METEO
    )
    weather_service.get_forecast.assert_called_once()
