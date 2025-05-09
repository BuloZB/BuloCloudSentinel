"""
Prometheus metrics exporter for Weather Guard service.

This module provides a Prometheus metrics exporter for the Weather Guard service,
which exports weather data as Prometheus metrics.
"""

import asyncio
import logging
import time
from datetime import datetime, timezone
from typing import Dict, List, Optional, Any, Set

from prometheus_client import Gauge, Counter, Info, start_http_server

from weather_guard.core.config import settings
from weather_guard.models.weather import (
    WeatherCondition,
    WeatherData,
    WeatherForecast,
    WeatherProvider,
    WeatherSeverity,
)
from weather_guard.services.weather import WeatherService

logger = logging.getLogger(__name__)


class MetricsExporter:
    """Prometheus metrics exporter for Weather Guard service."""
    
    def __init__(
        self,
        weather_service: WeatherService,
        port: int = 9090,
        update_interval: int = 60,
    ):
        """Initialize the metrics exporter.
        
        Args:
            weather_service: Weather service instance
            port: Prometheus metrics server port
            update_interval: Metrics update interval in seconds
        """
        self.weather_service = weather_service
        self.port = port
        self.update_interval = update_interval
        
        self.task = None
        self.running = False
        
        # Define metrics
        self.info = Info("weather_service", "Weather service information")
        
        # Current weather metrics
        self.temperature = Gauge(
            "weather_temperature", "Current temperature in Celsius", ["location", "provider"]
        )
        self.feels_like = Gauge(
            "weather_feels_like", "Current feels like temperature in Celsius", ["location", "provider"]
        )
        self.humidity = Gauge(
            "weather_humidity", "Current humidity in percent", ["location", "provider"]
        )
        self.pressure = Gauge(
            "weather_pressure", "Current pressure in hPa", ["location", "provider"]
        )
        self.wind_speed = Gauge(
            "weather_wind_speed", "Current wind speed in m/s", ["location", "provider"]
        )
        self.wind_direction = Gauge(
            "weather_wind_direction", "Current wind direction in degrees", ["location", "provider"]
        )
        self.wind_gust = Gauge(
            "weather_wind_gust", "Current wind gust in m/s", ["location", "provider"]
        )
        self.precipitation = Gauge(
            "weather_precipitation", "Current precipitation in mm/h", ["location", "provider"]
        )
        self.cloud_cover = Gauge(
            "weather_cloud_cover", "Current cloud cover in percent", ["location", "provider"]
        )
        self.uv_index = Gauge(
            "weather_uv_index", "Current UV index", ["location", "provider"]
        )
        
        # Forecast metrics
        self.forecast_temperature = Gauge(
            "weather_forecast_temperature",
            "Forecast temperature in Celsius",
            ["location", "provider", "time_offset"],
        )
        self.forecast_wind_speed = Gauge(
            "weather_forecast_wind_speed",
            "Forecast wind speed in m/s",
            ["location", "provider", "time_offset"],
        )
        self.forecast_precipitation = Gauge(
            "weather_forecast_precipitation",
            "Forecast precipitation in mm/h",
            ["location", "provider", "time_offset"],
        )
        
        # Mission metrics
        self.mission_flyable = Gauge(
            "weather_mission_flyable",
            "Whether weather conditions are suitable for a mission (1=yes, 0=no)",
            ["location"],
        )
        self.mission_severity = Gauge(
            "weather_mission_severity",
            "Weather severity for a mission (0=none, 1=low, 2=medium, 3=high, 4=extreme)",
            ["location"],
        )
        
        # Service metrics
        self.error_count = Counter(
            "weather_service_errors_total", "Total number of weather service errors"
        )
        self.api_requests = Counter(
            "weather_service_api_requests_total", "Total number of API requests", ["endpoint"]
        )
        self.cache_hits = Counter(
            "weather_service_cache_hits_total", "Total number of cache hits", ["cache"]
        )
        self.cache_misses = Counter(
            "weather_service_cache_misses_total", "Total number of cache misses", ["cache"]
        )
        
        # Monitored locations
        self.locations = [
            {
                "id": "default",
                "latitude": 52.52,
                "longitude": 13.41,
                "name": "Default Location",
            }
        ]
    
    def start(self) -> None:
        """Start the metrics exporter."""
        if self.running:
            return
        
        # Start Prometheus HTTP server
        start_http_server(self.port)
        logger.info(f"Prometheus metrics server started on port {self.port}")
        
        # Set service info
        self.info.info({
            "version": settings.PROJECT_NAME,
            "update_interval": str(self.update_interval),
        })
        
        # Start update task
        self.running = True
        self.task = asyncio.create_task(self._update_metrics())
        logger.info("Metrics exporter started")
    
    async def stop(self) -> None:
        """Stop the metrics exporter."""
        if not self.running:
            return
        
        self.running = False
        if self.task:
            self.task.cancel()
            try:
                await self.task
            except asyncio.CancelledError:
                pass
            self.task = None
        
        logger.info("Metrics exporter stopped")
    
    async def _update_metrics(self) -> None:
        """Update metrics periodically."""
        while self.running:
            try:
                # Update metrics for each location
                for location in self.locations:
                    await self._update_location_metrics(location)
                
                # Update service metrics
                self.error_count._value.set(self.weather_service.error_count)
                
                # Wait for next update
                await asyncio.sleep(self.update_interval)
            
            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error(f"Error updating metrics: {str(e)}")
                # Wait before retrying
                await asyncio.sleep(10)
    
    async def _update_location_metrics(self, location: Dict[str, Any]) -> None:
        """Update metrics for a location.
        
        Args:
            location: Location information
        """
        try:
            # Get location details
            location_id = location["id"]
            latitude = location["latitude"]
            longitude = location["longitude"]
            
            # Get current weather
            current = await self.weather_service.get_current_weather(latitude, longitude)
            
            # Update current weather metrics
            self._update_current_metrics(location_id, current)
            
            # Get forecast
            forecast = await self.weather_service.get_forecast(latitude, longitude, 24)
            
            # Update forecast metrics
            self._update_forecast_metrics(location_id, forecast)
            
            # Check mission weather
            now = datetime.now(timezone.utc)
            mission_check = await self.weather_service.check_mission_weather(
                latitude, longitude, now, now + settings.UPDATE_INTERVAL * 60
            )
            
            # Update mission metrics
            self.mission_flyable.labels(location=location_id).set(1 if mission_check.is_flyable else 0)
            
            # Map severity to numeric value
            severity_value = {
                WeatherSeverity.NONE: 0,
                WeatherSeverity.LOW: 1,
                WeatherSeverity.MEDIUM: 2,
                WeatherSeverity.HIGH: 3,
                WeatherSeverity.EXTREME: 4,
            }.get(mission_check.severity, 0)
            
            self.mission_severity.labels(location=location_id).set(severity_value)
        
        except Exception as e:
            logger.error(f"Error updating metrics for location {location.get('name', 'unknown')}: {str(e)}")
    
    def _update_current_metrics(self, location_id: str, weather: WeatherData) -> None:
        """Update current weather metrics.
        
        Args:
            location_id: Location ID
            weather: Current weather data
        """
        provider = str(weather.provider.value)
        
        self.temperature.labels(location=location_id, provider=provider).set(weather.temperature)
        
        if weather.feels_like is not None:
            self.feels_like.labels(location=location_id, provider=provider).set(weather.feels_like)
        
        if weather.humidity is not None:
            self.humidity.labels(location=location_id, provider=provider).set(weather.humidity)
        
        if weather.pressure is not None:
            self.pressure.labels(location=location_id, provider=provider).set(weather.pressure)
        
        self.wind_speed.labels(location=location_id, provider=provider).set(weather.wind_speed)
        
        if weather.wind_direction is not None:
            self.wind_direction.labels(location=location_id, provider=provider).set(weather.wind_direction)
        
        if weather.wind_gust is not None:
            self.wind_gust.labels(location=location_id, provider=provider).set(weather.wind_gust)
        
        self.precipitation.labels(location=location_id, provider=provider).set(weather.precipitation)
        
        if weather.cloud_cover is not None:
            self.cloud_cover.labels(location=location_id, provider=provider).set(weather.cloud_cover)
        
        if weather.uv_index is not None:
            self.uv_index.labels(location=location_id, provider=provider).set(weather.uv_index)
    
    def _update_forecast_metrics(self, location_id: str, forecast: List[WeatherForecast]) -> None:
        """Update forecast metrics.
        
        Args:
            location_id: Location ID
            forecast: Weather forecast
        """
        for i, f in enumerate(forecast):
            if i >= 24:  # Limit to 24 hours
                break
            
            provider = str(f.provider.value)
            time_offset = str(i)
            
            self.forecast_temperature.labels(
                location=location_id, provider=provider, time_offset=time_offset
            ).set(f.temperature)
            
            self.forecast_wind_speed.labels(
                location=location_id, provider=provider, time_offset=time_offset
            ).set(f.wind_speed)
            
            self.forecast_precipitation.labels(
                location=location_id, provider=provider, time_offset=time_offset
            ).set(f.precipitation)
    
    def add_location(self, latitude: float, longitude: float, name: Optional[str] = None) -> str:
        """Add a location to monitor.
        
        Args:
            latitude: Latitude of the location
            longitude: Longitude of the location
            name: Name of the location
            
        Returns:
            Location ID
        """
        location_id = f"loc_{len(self.locations)}"
        
        self.locations.append({
            "id": location_id,
            "latitude": latitude,
            "longitude": longitude,
            "name": name or f"{latitude}, {longitude}",
        })
        
        return location_id
    
    def remove_location(self, location_id: str) -> bool:
        """Remove a location from monitoring.
        
        Args:
            location_id: Location ID
            
        Returns:
            True if location was removed, False otherwise
        """
        for i, location in enumerate(self.locations):
            if location["id"] == location_id:
                self.locations.pop(i)
                return True
        
        return False
    
    def get_locations(self) -> List[Dict[str, Any]]:
        """Get all monitored locations.
        
        Returns:
            List of monitored locations
        """
        return self.locations
