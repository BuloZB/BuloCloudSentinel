"""
Weather service for Weather Guard.

This module provides the main weather service for the Weather Guard module,
including weather data fetching, caching, and mission weather checks.
"""

import asyncio
import logging
import time
from datetime import datetime, timedelta, timezone
from typing import Dict, List, Optional, Any, Tuple, Union

from weather_guard.clients.open_meteo import OpenMeteoClient
from weather_guard.clients.meteo_shield import MeteoShieldClient
from weather_guard.core.config import settings
from weather_guard.models.weather import (
    WeatherCondition,
    WeatherData,
    WeatherForecast,
    WeatherProvider,
    WeatherSeverity,
    MissionWeatherCheck,
    WeatherServiceStatus,
)
from weather_guard.services.cache import weather_cache, forecast_cache

logger = logging.getLogger(__name__)


class WeatherService:
    """Weather service for Weather Guard."""

    def __init__(self):
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
                    callback=self._on_meteo_shield_data
                )
                await self.meteo_shield_client.start()
                logger.info("MeteoShield client started")
            except Exception as e:
                logger.error(f"Error starting MeteoShield client: {str(e)}")
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
        asyncio.create_task(self._cache_meteo_shield_data(weather_data))

    async def _cache_meteo_shield_data(self, weather_data: WeatherData) -> None:
        """Cache weather data from MeteoShield.

        Args:
            weather_data: Weather data from MeteoShield
        """
        try:
            cache_key = f"current:{WeatherProvider.METEO_SHIELD}:{weather_data.latitude:.4f}:{weather_data.longitude:.4f}"
            await weather_cache.set(cache_key, weather_data)
            logger.debug(f"Cached MeteoShield data for {weather_data.latitude}, {weather_data.longitude}")
        except Exception as e:
            logger.error(f"Error caching MeteoShield data: {str(e)}")

    async def get_current_weather(
        self, latitude: float, longitude: float, provider: Optional[WeatherProvider] = None
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
            logger.debug(f"Using cached weather data for {latitude}, {longitude}")
            return cached_data

        try:
            # Fetch from provider
            if provider == WeatherProvider.OPEN_METEO:
                weather_data = await self.open_meteo_client.get_current_weather(latitude, longitude)
            elif provider == WeatherProvider.METEO_SHIELD:
                # Get data from MeteoShield client
                if self.meteo_shield_client:
                    weather_data = self.meteo_shield_client.get_latest_weather()
                    if not weather_data:
                        raise ValueError("No data available from MeteoShield")
                else:
                    raise ValueError("MeteoShield client not initialized")
            else:
                raise ValueError(f"Unsupported weather provider: {provider}")

            # Cache the result
            await weather_cache.set(cache_key, weather_data)

            return weather_data
        except Exception as e:
            logger.error(f"Error fetching weather data: {str(e)}")
            self.error_count += 1

            # Try to get last cached data even if expired
            if cached_data:
                logger.warning("Using expired cached data as fallback")
                return cached_data

            # Create default data as last resort
            return WeatherData(
                latitude=latitude,
                longitude=longitude,
                timestamp=datetime.now(timezone.utc),
                provider=provider,
                condition=WeatherCondition.UNKNOWN,
                temperature=20.0,
                wind_speed=0.0,
                precipitation=0.0,
            )

    async def get_forecast(
        self, latitude: float, longitude: float, hours: int = 24, provider: Optional[WeatherProvider] = None
    ) -> List[WeatherForecast]:
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
            logger.debug(f"Using cached forecast data for {latitude}, {longitude}")
            return cached_data

        try:
            # Fetch from provider
            if provider == WeatherProvider.OPEN_METEO:
                forecast_data = await self.open_meteo_client.get_forecast(latitude, longitude, hours)
            else:
                raise ValueError(f"Unsupported weather provider: {provider}")

            # Cache the result
            await forecast_cache.set(cache_key, forecast_data)

            return forecast_data
        except Exception as e:
            logger.error(f"Error fetching forecast data: {str(e)}")
            self.error_count += 1

            # Try to get last cached data even if expired
            if cached_data:
                logger.warning("Using expired cached data as fallback")
                return cached_data

            # Create default forecast as last resort
            now = datetime.now(timezone.utc)
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

    async def check_mission_weather(
        self,
        latitude: float,
        longitude: float,
        start_time: datetime,
        end_time: Optional[datetime] = None,
        mission_id: Optional[str] = None,
        provider: Optional[WeatherProvider] = None,
    ) -> MissionWeatherCheck:
        """Check if weather conditions are suitable for a mission.

        Args:
            latitude: Latitude of the mission location
            longitude: Longitude of the mission location
            start_time: Mission start time
            end_time: Mission end time (optional)
            mission_id: Mission ID (optional)
            provider: Weather data provider

        Returns:
            Mission weather check result
        """
        # Default end time is 1 hour after start time
        if not end_time:
            end_time = start_time + timedelta(hours=1)

        # Get current weather
        current_weather = await self.get_current_weather(latitude, longitude, provider)

        # Get forecast for the mission duration
        now = datetime.now(timezone.utc)
        hours_until_end = max(1, int((end_time - now).total_seconds() / 3600) + 1)
        forecast = await self.get_forecast(latitude, longitude, hours_until_end, provider)

        # Filter forecast to mission time window
        mission_forecast = [
            f for f in forecast
            if start_time <= f.forecast_time <= end_time
        ]

        # Check if weather conditions are suitable
        is_flyable, severity, limitations, recommendations = self._evaluate_weather_conditions(
            current_weather, mission_forecast
        )

        # Create mission weather check result
        check_result = MissionWeatherCheck(
            mission_id=mission_id,
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

        return check_result

    def _evaluate_weather_conditions(
        self, current: WeatherData, forecast: List[WeatherForecast]
    ) -> Tuple[bool, WeatherSeverity, List[Dict[str, Any]], List[str]]:
        """Evaluate weather conditions for a mission.

        Args:
            current: Current weather data
            forecast: Weather forecast for the mission duration

        Returns:
            Tuple of (is_flyable, severity, limitations, recommendations)
        """
        is_flyable = True
        severity = WeatherSeverity.NONE
        limitations = []
        recommendations = []

        # Check wind speed
        max_wind = max([current.wind_speed] + [f.wind_speed for f in forecast])
        if max_wind > settings.WIND_THRESHOLD:
            is_flyable = False
            severity = WeatherSeverity.HIGH
            limitations.append({
                "type": "wind",
                "value": max_wind,
                "threshold": settings.WIND_THRESHOLD,
                "description": f"Wind speed exceeds threshold ({max_wind:.1f} m/s > {settings.WIND_THRESHOLD} m/s)"
            })
            recommendations.append("Consider rescheduling the mission when wind conditions improve")
        elif max_wind > settings.WIND_THRESHOLD * 0.7:
            severity = max(severity, WeatherSeverity.MEDIUM)
            limitations.append({
                "type": "wind",
                "value": max_wind,
                "threshold": settings.WIND_THRESHOLD,
                "description": f"Wind speed approaching threshold ({max_wind:.1f} m/s)"
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
                "description": f"Precipitation exceeds threshold ({max_precip:.1f} mm/h > {settings.RAIN_THRESHOLD} mm/h)"
            })
            recommendations.append("Consider indoor alternatives or reschedule when precipitation subsides")
        elif max_precip > settings.RAIN_THRESHOLD * 0.7:
            severity = max(severity, WeatherSeverity.MEDIUM)
            limitations.append({
                "type": "precipitation",
                "value": max_precip,
                "threshold": settings.RAIN_THRESHOLD,
                "description": f"Precipitation approaching threshold ({max_precip:.1f} mm/h)"
            })
            recommendations.append("Prepare indoor fallback options in case precipitation increases")

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
                "description": f"Severe weather conditions detected: {severe_conditions[0].condition}"
            })
            recommendations.append("Mission not recommended due to severe weather conditions")

        # If no specific recommendations, add general ones
        if not recommendations:
            if is_flyable:
                recommendations.append("Weather conditions are suitable for the mission")
            else:
                recommendations.append("Consider rescheduling the mission when weather conditions improve")

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
        status = WeatherServiceStatus(
            status="operational",
            version=settings.PROJECT_NAME,
            providers=providers,
            last_update=datetime.now(timezone.utc),
            cache_status={
                "weather": weather_stats,
                "forecast": forecast_stats,
            },
            error_count=self.error_count,
            uptime=time.time() - self.start_time,
        )

        return status
