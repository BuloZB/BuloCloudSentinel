"""
Tests for the Open-Meteo API client.

This module provides tests for the Open-Meteo API client, including
mocked API responses and error handling.
"""

import json
import pytest
from datetime import datetime
from unittest.mock import AsyncMock, patch

import httpx
import pytest_asyncio

from weather_guard.clients.open_meteo import OpenMeteoClient
from weather_guard.models.weather import WeatherCondition, WeatherProvider


@pytest.fixture
def mock_current_weather_response():
    """Mock response for current weather."""
    return {
        "latitude": 52.52,
        "longitude": 13.41,
        "generationtime_ms": 0.2510547637939453,
        "utc_offset_seconds": 0,
        "timezone": "UTC",
        "timezone_abbreviation": "UTC",
        "elevation": 38.0,
        "current": {
            "time": "2023-10-15T12:00",
            "temperature_2m": 15.3,
            "relative_humidity_2m": 65,
            "apparent_temperature": 14.8,
            "precipitation": 0.1,
            "rain": 0.1,
            "weather_code": 3,
            "cloud_cover": 100,
            "pressure_msl": 1012.0,
            "wind_speed_10m": 5.0,
            "wind_direction_10m": 270,
            "wind_gusts_10m": 8.0
        }
    }


@pytest.fixture
def mock_forecast_response():
    """Mock response for forecast."""
    return {
        "latitude": 52.52,
        "longitude": 13.41,
        "generationtime_ms": 0.2510547637939453,
        "utc_offset_seconds": 0,
        "timezone": "UTC",
        "timezone_abbreviation": "UTC",
        "elevation": 38.0,
        "hourly": {
            "time": [
                "2023-10-15T12:00",
                "2023-10-15T13:00",
                "2023-10-15T14:00"
            ],
            "temperature_2m": [15.3, 16.1, 16.5],
            "relative_humidity_2m": [65, 63, 60],
            "precipitation": [0.1, 0.0, 0.0],
            "rain": [0.1, 0.0, 0.0],
            "weather_code": [3, 2, 1],
            "cloud_cover": [100, 75, 50],
            "wind_speed_10m": [5.0, 4.8, 4.5],
            "wind_direction_10m": [270, 275, 280]
        }
    }


@pytest_asyncio.fixture
async def open_meteo_client():
    """Create an Open-Meteo client for testing."""
    client = OpenMeteoClient(api_url="https://test.example.com/v1/forecast")
    yield client
    await client.close()


@pytest.mark.asyncio
async def test_get_current_weather(open_meteo_client, mock_current_weather_response):
    """Test getting current weather."""
    # Mock the HTTP response
    with patch.object(open_meteo_client.client, "get") as mock_get:
        mock_response = AsyncMock()
        mock_response.raise_for_status = AsyncMock()
        mock_response.json.return_value = mock_current_weather_response
        mock_get.return_value = mock_response
        
        # Call the method
        result = await open_meteo_client.get_current_weather(52.52, 13.41)
        
        # Check the result
        assert result.latitude == 52.52
        assert result.longitude == 13.41
        assert result.provider == WeatherProvider.OPEN_METEO
        assert result.condition == WeatherCondition.PARTLY_CLOUDY
        assert result.temperature == 15.3
        assert result.feels_like == 14.8
        assert result.humidity == 65
        assert result.pressure == 1012.0
        assert result.wind_speed == 5.0
        assert result.wind_direction == 270
        assert result.wind_gust == 8.0
        assert result.cloud_cover == 100
        assert result.precipitation == 0.1


@pytest.mark.asyncio
async def test_get_forecast(open_meteo_client, mock_forecast_response):
    """Test getting forecast."""
    # Mock the HTTP response
    with patch.object(open_meteo_client.client, "get") as mock_get:
        mock_response = AsyncMock()
        mock_response.raise_for_status = AsyncMock()
        mock_response.json.return_value = mock_forecast_response
        mock_get.return_value = mock_response
        
        # Call the method
        result = await open_meteo_client.get_forecast(52.52, 13.41, 3)
        
        # Check the result
        assert len(result) == 3
        assert result[0].latitude == 52.52
        assert result[0].longitude == 13.41
        assert result[0].provider == WeatherProvider.OPEN_METEO
        assert result[0].condition == WeatherCondition.PARTLY_CLOUDY
        assert result[0].temperature == 15.3
        assert result[0].wind_speed == 5.0
        assert result[0].wind_direction == 270
        assert result[0].precipitation == 0.1
        assert result[0].humidity == 65
        assert result[0].cloud_cover == 100
        
        # Check second forecast
        assert result[1].temperature == 16.1
        assert result[1].condition == WeatherCondition.PARTLY_CLOUDY
        
        # Check third forecast
        assert result[2].temperature == 16.5
        assert result[2].condition == WeatherCondition.PARTLY_CLOUDY


@pytest.mark.asyncio
async def test_http_error_handling(open_meteo_client):
    """Test handling of HTTP errors."""
    # Mock the HTTP response to raise an error
    with patch.object(open_meteo_client.client, "get") as mock_get:
        mock_get.side_effect = httpx.HTTPError("HTTP Error")
        
        # Call the method and check that it raises the error
        with pytest.raises(httpx.HTTPError):
            await open_meteo_client.get_current_weather(52.52, 13.41)


@pytest.mark.asyncio
async def test_invalid_response_handling(open_meteo_client):
    """Test handling of invalid responses."""
    # Mock the HTTP response with invalid data
    with patch.object(open_meteo_client.client, "get") as mock_get:
        mock_response = AsyncMock()
        mock_response.raise_for_status = AsyncMock()
        mock_response.json.return_value = {"invalid": "data"}
        mock_get.return_value = mock_response
        
        # Call the method and check that it raises a ValueError
        with pytest.raises(ValueError):
            await open_meteo_client.get_current_weather(52.52, 13.41)
