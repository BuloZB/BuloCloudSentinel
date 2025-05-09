"""
Tests for the Weather Service.

This module provides tests for the Weather Service, including
mocked API responses, caching, and error handling.
"""

import json
import pytest
from datetime import datetime, timedelta, timezone
from unittest.mock import AsyncMock, MagicMock, patch

import httpx
import pytest_asyncio
import redis.asyncio as redis

from weather_guard.clients.open_meteo import OpenMeteoClient
from weather_guard.clients.meteo_shield import MeteoShieldClient
from weather_guard.models.weather import (
    WeatherCondition,
    WeatherData,
    WeatherForecast,
    WeatherProvider,
    WeatherSeverity,
    MissionWeatherCheck,
)
from weather_guard.services.cache import RedisCache
from weather_guard.services.weather import WeatherService


@pytest.fixture
def mock_weather_data():
    """Mock weather data."""
    return WeatherData(
        latitude=52.52,
        longitude=13.41,
        timestamp=datetime.now(timezone.utc),
        provider=WeatherProvider.OPEN_METEO,
        condition=WeatherCondition.PARTLY_CLOUDY,
        temperature=15.3,
        feels_like=14.8,
        humidity=65,
        pressure=1012.0,
        wind_speed=5.0,
        wind_direction=270,
        wind_gust=8.0,
        cloud_cover=100,
        precipitation=0.1,
    )


@pytest.fixture
def mock_forecast_data():
    """Mock forecast data."""
    now = datetime.now(timezone.utc)
    return [
        WeatherForecast(
            latitude=52.52,
            longitude=13.41,
            timestamp=now,
            forecast_time=now + timedelta(hours=i),
            provider=WeatherProvider.OPEN_METEO,
            condition=WeatherCondition.PARTLY_CLOUDY,
            temperature=15.3 + i * 0.2,
            wind_speed=5.0 - i * 0.1,
            wind_direction=270,
            precipitation=0.1 if i < 2 else 0.0,
            humidity=65,
            cloud_cover=100 - i * 5,
        )
        for i in range(5)
    ]


@pytest_asyncio.fixture
async def mock_redis_cache():
    """Mock Redis cache."""
    with patch("weather_guard.services.cache.redis.from_url") as mock_redis:
        # Create mock Redis client
        mock_client = AsyncMock()
        mock_client.get.return_value = None
        mock_client.setex.return_value = "OK"
        mock_client.keys.return_value = []
        mock_client.info.return_value = {"used_memory_human": "1MB"}
        mock_client.ping.return_value = True
        
        # Return mock Redis client from from_url
        mock_redis.return_value = mock_client
        
        yield mock_client


@pytest_asyncio.fixture
async def weather_service(mock_redis_cache):
    """Create a weather service for testing."""
    # Mock OpenMeteoClient
    with patch("weather_guard.services.weather.OpenMeteoClient") as mock_client_class:
        # Create mock client
        mock_client = AsyncMock()
        mock_client_class.return_value = mock_client
        
        # Create weather service
        service = WeatherService()
        await service.initialize()
        
        yield service, mock_client
        
        # Cleanup
        await service.close()


@pytest.mark.asyncio
async def test_get_current_weather(weather_service, mock_weather_data):
    """Test getting current weather."""
    service, mock_client = weather_service
    
    # Mock the client response
    mock_client.get_current_weather.return_value = mock_weather_data
    
    # Call the method
    result = await service.get_current_weather(52.52, 13.41)
    
    # Check the result
    assert result.latitude == 52.52
    assert result.longitude == 13.41
    assert result.provider == WeatherProvider.OPEN_METEO
    assert result.condition == WeatherCondition.PARTLY_CLOUDY
    assert result.temperature == 15.3
    assert result.wind_speed == 5.0
    assert result.precipitation == 0.1
    
    # Verify the client was called
    mock_client.get_current_weather.assert_called_once_with(52.52, 13.41)


@pytest.mark.asyncio
async def test_get_forecast(weather_service, mock_forecast_data):
    """Test getting forecast."""
    service, mock_client = weather_service
    
    # Mock the client response
    mock_client.get_forecast.return_value = mock_forecast_data
    
    # Call the method
    result = await service.get_forecast(52.52, 13.41, 5)
    
    # Check the result
    assert len(result) == 5
    assert result[0].latitude == 52.52
    assert result[0].longitude == 13.41
    assert result[0].provider == WeatherProvider.OPEN_METEO
    assert result[0].condition == WeatherCondition.PARTLY_CLOUDY
    assert result[0].temperature == 15.3
    assert result[0].wind_speed == 5.0
    assert result[0].precipitation == 0.1
    
    # Verify the client was called
    mock_client.get_forecast.assert_called_once_with(52.52, 13.41, 5)


@pytest.mark.asyncio
async def test_check_mission_weather(weather_service, mock_weather_data, mock_forecast_data):
    """Test checking mission weather."""
    service, mock_client = weather_service
    
    # Mock the client responses
    mock_client.get_current_weather.return_value = mock_weather_data
    mock_client.get_forecast.return_value = mock_forecast_data
    
    # Call the method
    start_time = datetime.now(timezone.utc) + timedelta(hours=1)
    end_time = start_time + timedelta(hours=2)
    result = await service.check_mission_weather(
        52.52, 13.41, start_time, end_time, "test-mission"
    )
    
    # Check the result
    assert result.mission_id == "test-mission"
    assert result.latitude == 52.52
    assert result.longitude == 13.41
    assert result.start_time == start_time
    assert result.end_time == end_time
    assert result.is_flyable is True  # Wind and rain are below thresholds
    assert result.severity == WeatherSeverity.NONE
    assert len(result.forecast) > 0
    
    # Verify the clients were called
    mock_client.get_current_weather.assert_called_once_with(52.52, 13.41, None)
    mock_client.get_forecast.assert_called_once()


@pytest.mark.asyncio
async def test_error_handling(weather_service):
    """Test error handling."""
    service, mock_client = weather_service
    
    # Mock the client to raise an error
    mock_client.get_current_weather.side_effect = httpx.HTTPError("HTTP Error")
    
    # Call the method
    result = await service.get_current_weather(52.52, 13.41)
    
    # Check the result (should be default data)
    assert result.latitude == 52.52
    assert result.longitude == 13.41
    assert result.condition == WeatherCondition.UNKNOWN
    assert result.temperature == 20.0
    assert result.wind_speed == 0.0
    assert result.precipitation == 0.0
    
    # Verify the error count was incremented
    assert service.error_count == 1


@pytest.mark.asyncio
async def test_meteo_shield_integration():
    """Test MeteoShield integration."""
    # Mock MeteoShieldClient
    with patch("weather_guard.services.weather.MeteoShieldClient") as mock_client_class, \
         patch("weather_guard.services.weather.settings") as mock_settings, \
         patch("weather_guard.services.weather.weather_cache") as mock_cache, \
         patch("weather_guard.services.weather.forecast_cache") as mock_forecast_cache, \
         patch("weather_guard.services.weather.OpenMeteoClient"):
        
        # Enable MQTT
        mock_settings.MQTT_ENABLED = True
        
        # Mock cache
        mock_cache.connect = AsyncMock()
        mock_cache.close = AsyncMock()
        mock_cache.get = AsyncMock(return_value=None)
        mock_cache.set = AsyncMock()
        
        mock_forecast_cache.connect = AsyncMock()
        mock_forecast_cache.close = AsyncMock()
        
        # Create mock MeteoShield client
        mock_client = AsyncMock()
        mock_client_class.return_value = mock_client
        
        # Create mock weather data
        mock_weather = WeatherData(
            latitude=52.52,
            longitude=13.41,
            timestamp=datetime.now(timezone.utc),
            provider=WeatherProvider.METEO_SHIELD,
            condition=WeatherCondition.CLEAR,
            temperature=18.5,
            humidity=55,
            pressure=1015.0,
            wind_speed=3.0,
            precipitation=0.0,
        )
        
        # Mock get_latest_weather
        mock_client.get_latest_weather.return_value = mock_weather
        
        # Create weather service
        service = WeatherService()
        await service.initialize()
        
        # Verify MeteoShield client was started
        mock_client.start.assert_called_once()
        
        # Test getting weather from MeteoShield
        result = await service.get_current_weather(52.52, 13.41, WeatherProvider.METEO_SHIELD)
        
        # Check the result
        assert result.provider == WeatherProvider.METEO_SHIELD
        assert result.temperature == 18.5
        assert result.wind_speed == 3.0
        
        # Verify get_latest_weather was called
        mock_client.get_latest_weather.assert_called_once()
        
        # Cleanup
        await service.close()
        
        # Verify MeteoShield client was stopped
        mock_client.stop.assert_called_once()
