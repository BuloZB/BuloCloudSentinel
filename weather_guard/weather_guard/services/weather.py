"""
Weather service for Weather Guard.

This module provides the main weather service for the Weather Guard module,
including weather data fetching, caching, and mission weather checks.
"""

from __future__ import annotations

import asyncio
import time
from dataclasses import dataclass
from datetime import datetime, timedelta
from typing import Any

from common import get_logger, utc_now, async_timed
from weather_guard.clients.meteo_shield import MeteoShieldClient
from weather_guard.clients.open_meteo import OpenMeteoClient
from weather_guard.core.config import settings
from weather_guard.models.weather import (
    MissionWeatherCheck,
    WeatherCondition,
    WeatherData,
    WeatherForecast,
    WeatherProvider,
    WeatherServiceStatus,
    WeatherSeverity,
)
from weather_guard.services.cache import forecast_cache, weather_cache

logger = get_logger(__name__)


@dataclass
class MissionParams:
    """Parameters for a mission weather check."""

    end_time: datetime | None = None
    mission_id: str | None = None
    provider: WeatherProvider | None = None


class WeatherService:
    """Weather service for Weather Guard."""

    def __init__(self) -> None:
        """Initialize the weather service."""
        self.open_meteo_client = OpenMeteoClient()
        self.meteo_shield_client = None
        self.start_time = time.time()
        self.error_count = 0

    async def initialize(self) -> None:
        """Initialize the weather service."""
        # Connect to Redis
        await weather_cache.connect()
        await forecast_cache.connect()

        # Initialize MeteoShield client if enabled
        if settings.MQTT_ENABLED:
            try:
                self.meteo_shield_client = MeteoShieldClient(
                    callback=self._on_meteo_shield_data,
                )
                await self.meteo_shield_client.start()
                logger.info("MeteoShield client started")
            except Exception:
                logger.exception("Error starting MeteoShield client")
                self.meteo_shield_client = None

    async def close(self) -> None:
        """Close the weather service."""
        await self.open_meteo_client.close()

        # Stop MeteoShield client if running
        if self.meteo_shield_client:
            await self.meteo_shield_client.stop()

        await weather_cache.close()
        await forecast_cache.close()

    def _on_meteo_shield_data(self, weather_data: WeatherData) -> None:
        """Handle new weather data from MeteoShield.

        Args:
            weather_data: Weather data from MeteoShield
        """
        # Cache the data asynchronously
        task = asyncio.create_task(self._cache_meteo_shield_data(weather_data))
        # Add a callback to handle any exceptions
        task.add_done_callback(lambda t: t.exception() if t.exception() else None)

    async def _cache_meteo_shield_data(self, weather_data: WeatherData) -> None:
        """Cache weather data from MeteoShield.

        Args:
            weather_data: Weather data from MeteoShield
        """
        try:
            cache_key = (
                f"current:{WeatherProvider.METEO_SHIELD}:"
                f"{weather_data.latitude:.4f}:{weather_data.longitude:.4f}"
            )
            await weather_cache.set(cache_key, weather_data)
            logger.debug(
                "Cached MeteoShield data for %s, %s",
                weather_data.latitude,
                weather_data.longitude,
            )
        except Exception:
            logger.exception("Error caching MeteoShield data")

    async def _fetch_weather_from_provider(
        self, latitude: float, longitude: float, provider: WeatherProvider,
    ) -> WeatherData:
        """Fetch weather data from the specified provider.

        Args:
            latitude: Latitude of the location
            longitude: Longitude of the location
            provider: Weather data provider

        Returns:
            Weather data from the provider

        Raises:
            ValueError: If the provider is not supported or data is not available
        """
        if provider == WeatherProvider.OPEN_METEO:
            return await self.open_meteo_client.get_current_weather(latitude, longitude)

        if provider == WeatherProvider.METEO_SHIELD:
            if not self.meteo_shield_client:
                error_msg = "MeteoShield client not initialized"
                raise ValueError(error_msg)

            weather_data = self.meteo_shield_client.get_latest_weather()
            if not weather_data:
                error_msg = "No data available from MeteoShield"
                raise ValueError(error_msg)

            return weather_data

        # Unsupported provider
        error_msg = f"Unsupported weather provider: {provider}"
        raise ValueError(error_msg)

    @async_timed
    async def get_current_weather(
        self, latitude: float, longitude: float, provider: WeatherProvider | None = None,
    ) -> WeatherData:
        """Get current weather data for a location.

        Args:
            latitude: Latitude of the location
            longitude: Longitude of the location
            provider: Weather data provider

        Returns:
            Current weather data
        """
        provider = provider or WeatherProvider.OPEN_METEO
        cache_key = f"current:{provider}:{latitude:.4f}:{longitude:.4f}"

        # Try to get from cache
        cached_data = await weather_cache.get(cache_key)
        if cached_data:
            logger.debug("Using cached weather data for %s, %s", latitude, longitude)
            return cached_data

        try:
            # Fetch from provider
            weather_data = await self._fetch_weather_from_provider(latitude, longitude, provider)

            # Cache the result
            await weather_cache.set(cache_key, weather_data)
        except Exception:
            logger.exception("Error fetching weather data")
            self.error_count += 1

            # Try to get last cached data even if expired
            if cached_data:
                logger.warning("Using expired cached data as fallback")
                return cached_data

            # Create default data as last resort
            return WeatherData(
                latitude=latitude,
                longitude=longitude,
                timestamp=utc_now(),
                provider=provider,
                condition=WeatherCondition.UNKNOWN,
                temperature=20.0,
                wind_speed=0.0,
                precipitation=0.0,
            )
        else:
            return weather_data

    async def _fetch_forecast_from_provider(
        self, latitude: float, longitude: float, hours: int, provider: WeatherProvider,
    ) -> list[WeatherForecast]:
        """Fetch forecast data from the specified provider.

        Args:
            latitude: Latitude of the location
            longitude: Longitude of the location
            hours: Number of hours to forecast
            provider: Weather data provider

        Returns:
            List of weather forecasts from the provider

        Raises:
            ValueError: If the provider is not supported
        """
        if provider == WeatherProvider.OPEN_METEO:
            return await self.open_meteo_client.get_forecast(
                latitude, longitude, hours,
            )

        # Unsupported provider
        error_msg = f"Unsupported weather provider: {provider}"
        raise ValueError(error_msg)

    @async_timed
    async def get_forecast(
        self,
        latitude: float,
        longitude: float,
        hours: int = 24,
        provider: WeatherProvider | None = None,
    ) -> list[WeatherForecast]:
        """Get weather forecast for a location.

        Args:
            latitude: Latitude of the location
            longitude: Longitude of the location
            hours: Number of hours to forecast
            provider: Weather data provider

        Returns:
            List of weather forecasts
        """
        provider = provider or WeatherProvider.OPEN_METEO
        cache_key = f"forecast:{provider}:{latitude:.4f}:{longitude:.4f}:{hours}"

        # Try to get from cache
        cached_data = await forecast_cache.get(cache_key)
        if cached_data:
            logger.debug("Using cached forecast data for %s, %s", latitude, longitude)
            return cached_data

        try:
            # Fetch from provider
            forecast_data = await self._fetch_forecast_from_provider(
                latitude, longitude, hours, provider,
            )

            # Cache the result
            await forecast_cache.set(cache_key, forecast_data)
        except Exception:
            logger.exception("Error fetching forecast data")
            self.error_count += 1

            # Try to get last cached data even if expired
            if cached_data:
                logger.warning("Using expired cached data as fallback")
                return cached_data

            # Create default forecast as last resort
            now = utc_now()
            return [
                WeatherForecast(
                    latitude=latitude,
                    longitude=longitude,
                    timestamp=now,
                    forecast_time=now + timedelta(hours=i),
                    provider=provider,
                    condition=WeatherCondition.UNKNOWN,
                    temperature=20.0,
                    wind_speed=0.0,
                    precipitation=0.0,
                )
                for i in range(hours)
            ]
        else:
            return forecast_data

    @async_timed
    async def check_mission_weather(
        self,
        latitude: float,
        longitude: float,
        start_time: datetime,
        params: MissionParams = None,
    ) -> MissionWeatherCheck:
        """Check if weather conditions are suitable for a mission.

        Args:
            latitude: Latitude of the mission location
            longitude: Longitude of the mission location
            start_time: Mission start time
            params: Additional mission parameters (end_time, mission_id, provider)

        Returns:
            Mission weather check result
        """
        # Initialize params if None
        if params is None:
            params = MissionParams()

        # Default end time is 1 hour after start time
        end_time = params.end_time or start_time + timedelta(hours=1)

        # Get current weather
        current_weather = await self.get_current_weather(latitude, longitude, params.provider)

        # Get forecast for the mission duration
        now = utc_now()
        hours_until_end = max(1, int((end_time - now).total_seconds() / 3600) + 1)
        forecast = await self.get_forecast(
            latitude, longitude, hours_until_end, params.provider,
        )

        # Filter forecast to mission time window
        mission_forecast = [
            f for f in forecast
            if start_time <= f.forecast_time <= end_time
        ]

        # Check if weather conditions are suitable
        is_flyable, severity, limitations, recommendations = self._evaluate_weather_conditions(
            current_weather, mission_forecast,
        )

        # Create mission weather check result
        return MissionWeatherCheck(
            mission_id=params.mission_id,
            latitude=latitude,
            longitude=longitude,
            start_time=start_time,
            end_time=end_time,
            is_flyable=is_flyable,
            severity=severity,
            current_weather=current_weather,
            forecast=mission_forecast,
            limitations=limitations,
            recommendations=recommendations,
        )

    def _evaluate_weather_conditions(
        self, current: WeatherData, forecast: list[WeatherForecast],
    ) -> tuple[bool, WeatherSeverity, list[dict[str, Any]], list[str]]:
        """Evaluate weather conditions for a mission.

        Args:
            current: Current weather data
            forecast: Weather forecast for the mission duration

        Returns:
            Tuple of (is_flyable, severity, limitations, recommendations)
        """
        is_flyable = True
        severity = WeatherSeverity.NONE
        limitations: list[dict[str, Any]] = []
        recommendations: list[str] = []

        # Check wind speed
        max_wind = max([current.wind_speed] + [f.wind_speed for f in forecast])
        if max_wind > settings.WIND_THRESHOLD:
            is_flyable = False
            severity = WeatherSeverity.HIGH
            limitations.append({
                "type": "wind",
                "value": max_wind,
                "threshold": settings.WIND_THRESHOLD,
                "description": (
                    f"Wind speed exceeds threshold ({max_wind:.1f} m/s > "
                    f"{settings.WIND_THRESHOLD} m/s)"
                ),
            })
            recommendations.append("Consider rescheduling the mission when wind conditions improve")
        elif max_wind > settings.WIND_THRESHOLD * 0.7:
            severity = max(severity, WeatherSeverity.MEDIUM)
            limitations.append({
                "type": "wind",
                "value": max_wind,
                "threshold": settings.WIND_THRESHOLD,
                "description": f"Wind speed approaching threshold ({max_wind:.1f} m/s)",
            })
            recommendations.append("Monitor wind conditions closely during the mission")

        # Check precipitation
        max_precip = max([current.precipitation] + [f.precipitation for f in forecast])
        if max_precip > settings.RAIN_THRESHOLD:
            is_flyable = False
            severity = WeatherSeverity.HIGH
            limitations.append({
                "type": "precipitation",
                "value": max_precip,
                "threshold": settings.RAIN_THRESHOLD,
                "description": (
                    f"Precipitation exceeds threshold ({max_precip:.1f} mm/h > "
                    f"{settings.RAIN_THRESHOLD} mm/h)"
                ),
            })
            recommendations.append(
                "Consider indoor alternatives or reschedule when precipitation subsides",
            )
        elif max_precip > settings.RAIN_THRESHOLD * 0.7:
            severity = max(severity, WeatherSeverity.MEDIUM)
            limitations.append({
                "type": "precipitation",
                "value": max_precip,
                "threshold": settings.RAIN_THRESHOLD,
                "description": f"Precipitation approaching threshold ({max_precip:.1f} mm/h)",
            })
            recommendations.append(
                "Prepare indoor fallback options in case precipitation increases",
            )

        # Check for severe weather conditions
        severe_conditions = [
            f for f in forecast
            if f.condition in [
                WeatherCondition.THUNDERSTORM,
                WeatherCondition.HEAVY_RAIN,
                WeatherCondition.SNOW,
                WeatherCondition.HAIL,
            ]
        ]

        if severe_conditions:
            is_flyable = False
            severity = WeatherSeverity.EXTREME
            limitations.append({
                "type": "severe_weather",
                "value": severe_conditions[0].condition,
                "description": (
                    f"Severe weather conditions detected: {severe_conditions[0].condition}"
                ),
            })
            recommendations.append("Mission not recommended due to severe weather conditions")

        # If no specific recommendations, add general ones
        if not recommendations:
            if is_flyable:
                recommendations.append("Weather conditions are suitable for the mission")
            else:
                recommendations.append(
                    "Consider rescheduling the mission when weather conditions improve",
                )

        return is_flyable, severity, limitations, recommendations

    async def get_service_status(self) -> WeatherServiceStatus:
        """Get the status of the weather service.

        Returns:
            Weather service status
        """
        # Get cache statistics
        weather_stats = await weather_cache.get_cache_stats()
        forecast_stats = await forecast_cache.get_cache_stats()

        # Get available providers
        providers = [WeatherProvider.OPEN_METEO]
        if self.meteo_shield_client:
            providers.append(WeatherProvider.METEO_SHIELD)

        # Create status object
        return WeatherServiceStatus(
            status="operational",
            version=settings.PROJECT_NAME,
            providers=providers,
            last_update=utc_now(),
            cache_status={
                "weather": weather_stats,
                "forecast": forecast_stats,
            },
            error_count=self.error_count,
            uptime=time.time() - self.start_time,
        )
