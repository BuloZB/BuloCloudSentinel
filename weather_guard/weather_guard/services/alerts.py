"""
Weather alert service for Weather Guard.

This module provides a service for generating and managing weather alerts,
including notifications when weather windows open for mission execution.
"""

import asyncio
import json
import logging
from datetime import datetime, timedelta, timezone
from typing import Dict, List, Optional, Any, Set, Tuple

from weather_guard.core.config import settings
from weather_guard.models.weather import (
    WeatherAlert,
    WeatherCondition,
    WeatherData,
    WeatherForecast,
    WeatherProvider,
    WeatherSeverity,
)
from weather_guard.services.weather import WeatherService

logger = logging.getLogger(__name__)


class AlertService:
    """Alert service for Weather Guard."""
    
    def __init__(self, weather_service: Optional[WeatherService] = None):
        """Initialize the alert service.
        
        Args:
            weather_service: Weather service instance
        """
        self.weather_service = weather_service
        self.task = None
        self.running = False
        self.check_interval = 30 * 60  # 30 minutes
        
        # Store locations to monitor
        self.monitored_locations: List[Dict[str, Any]] = []
        
        # Store active alerts
        self.active_alerts: Dict[str, WeatherAlert] = {}
        
        # Store weather windows
        self.weather_windows: Dict[str, Dict[str, Any]] = {}
        
        # Store notification callbacks
        self.notification_callbacks: List[callable] = []
    
    async def start(self) -> None:
        """Start the alert service."""
        if self.running:
            return
        
        self.running = True
        self.task = asyncio.create_task(self._run())
        logger.info("Alert service started")
    
    async def stop(self) -> None:
        """Stop the alert service."""
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
        
        logger.info("Alert service stopped")
    
    async def _run(self) -> None:
        """Run the alert service loop."""
        while self.running:
            try:
                # Check all monitored locations
                await self._check_all_locations()
                
                # Wait for next check
                await asyncio.sleep(self.check_interval)
            
            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error(f"Error in alert service loop: {str(e)}")
                # Wait before retrying
                await asyncio.sleep(60)
    
    async def _check_all_locations(self) -> None:
        """Check all monitored locations for alerts and weather windows."""
        if not self.weather_service:
            logger.warning("Weather service not available")
            return
        
        for location in self.monitored_locations:
            try:
                # Get location details
                location_id = location["id"]
                latitude = location["latitude"]
                longitude = location["longitude"]
                name = location.get("name", f"{latitude}, {longitude}")
                
                # Get current weather and forecast
                current = await self.weather_service.get_current_weather(latitude, longitude)
                forecast = await self.weather_service.get_forecast(latitude, longitude, 24)
                
                # Check for alerts
                alerts = self._check_for_alerts(location_id, name, current, forecast)
                
                # Check for weather windows
                windows = self._check_for_weather_windows(location_id, name, current, forecast)
                
                # Send notifications for new alerts and windows
                await self._send_notifications(alerts, windows)
            
            except Exception as e:
                logger.error(f"Error checking location {location.get('name', 'unknown')}: {str(e)}")
    
    def _check_for_alerts(
        self, location_id: str, name: str, current: WeatherData, forecast: List[WeatherForecast]
    ) -> List[WeatherAlert]:
        """Check for weather alerts.
        
        Args:
            location_id: Location ID
            name: Location name
            current: Current weather data
            forecast: Weather forecast
            
        Returns:
            List of new alerts
        """
        new_alerts = []
        
        # Check for severe wind
        max_wind = max([current.wind_speed] + [f.wind_speed for f in forecast[:12]])  # Next 12 hours
        if max_wind > settings.WIND_THRESHOLD:
            alert_id = f"{location_id}:wind:{datetime.now(timezone.utc).isoformat()}"
            
            # Check if this alert is already active
            if not any(a.startswith(f"{location_id}:wind:") for a in self.active_alerts):
                # Create alert
                alert = WeatherAlert(
                    id=alert_id,
                    latitude=current.latitude,
                    longitude=current.longitude,
                    start_time=datetime.now(timezone.utc),
                    end_time=datetime.now(timezone.utc) + timedelta(hours=12),
                    provider=current.provider,
                    severity=WeatherSeverity.HIGH if max_wind > settings.WIND_THRESHOLD * 1.5 else WeatherSeverity.MEDIUM,
                    title=f"High Wind Alert for {name}",
                    description=f"Wind speeds up to {max_wind:.1f} m/s expected in the next 12 hours",
                    recommendation="Consider postponing outdoor drone operations",
                )
                
                # Add to active alerts
                self.active_alerts[alert_id] = alert
                new_alerts.append(alert)
        
        # Check for severe precipitation
        max_precip = max([current.precipitation] + [f.precipitation for f in forecast[:12]])  # Next 12 hours
        if max_precip > settings.RAIN_THRESHOLD:
            alert_id = f"{location_id}:precipitation:{datetime.now(timezone.utc).isoformat()}"
            
            # Check if this alert is already active
            if not any(a.startswith(f"{location_id}:precipitation:") for a in self.active_alerts):
                # Create alert
                alert = WeatherAlert(
                    id=alert_id,
                    latitude=current.latitude,
                    longitude=current.longitude,
                    start_time=datetime.now(timezone.utc),
                    end_time=datetime.now(timezone.utc) + timedelta(hours=12),
                    provider=current.provider,
                    severity=WeatherSeverity.HIGH if max_precip > settings.RAIN_THRESHOLD * 1.5 else WeatherSeverity.MEDIUM,
                    title=f"Precipitation Alert for {name}",
                    description=f"Precipitation up to {max_precip:.1f} mm/h expected in the next 12 hours",
                    recommendation="Consider indoor alternatives for drone operations",
                )
                
                # Add to active alerts
                self.active_alerts[alert_id] = alert
                new_alerts.append(alert)
        
        # Check for severe weather conditions
        severe_conditions = [
            f for f in forecast[:12]  # Next 12 hours
            if f.condition in [
                WeatherCondition.THUNDERSTORM,
                WeatherCondition.HEAVY_RAIN,
                WeatherCondition.SNOW,
                WeatherCondition.HAIL,
            ]
        ]
        
        if severe_conditions:
            alert_id = f"{location_id}:severe:{datetime.now(timezone.utc).isoformat()}"
            
            # Check if this alert is already active
            if not any(a.startswith(f"{location_id}:severe:") for a in self.active_alerts):
                # Create alert
                alert = WeatherAlert(
                    id=alert_id,
                    latitude=current.latitude,
                    longitude=current.longitude,
                    start_time=datetime.now(timezone.utc),
                    end_time=datetime.now(timezone.utc) + timedelta(hours=12),
                    provider=current.provider,
                    severity=WeatherSeverity.EXTREME,
                    title=f"Severe Weather Alert for {name}",
                    description=f"Severe weather conditions ({severe_conditions[0].condition}) expected in the next 12 hours",
                    recommendation="Outdoor drone operations not recommended",
                )
                
                # Add to active alerts
                self.active_alerts[alert_id] = alert
                new_alerts.append(alert)
        
        # Clean up expired alerts
        now = datetime.now(timezone.utc)
        expired_alerts = [
            alert_id for alert_id, alert in self.active_alerts.items()
            if alert.end_time < now
        ]
        
        for alert_id in expired_alerts:
            del self.active_alerts[alert_id]
        
        return new_alerts
    
    def _check_for_weather_windows(
        self, location_id: str, name: str, current: WeatherData, forecast: List[WeatherForecast]
    ) -> List[Dict[str, Any]]:
        """Check for weather windows.
        
        Args:
            location_id: Location ID
            name: Location name
            current: Current weather data
            forecast: Weather forecast
            
        Returns:
            List of new weather windows
        """
        new_windows = []
        
        # Find continuous periods of good weather
        good_weather_periods = []
        current_period = None
        
        for i, f in enumerate(forecast):
            # Check if weather is good for flying
            is_good = (
                f.wind_speed <= settings.WIND_THRESHOLD and
                f.precipitation <= settings.RAIN_THRESHOLD and
                f.condition not in [
                    WeatherCondition.THUNDERSTORM,
                    WeatherCondition.HEAVY_RAIN,
                    WeatherCondition.SNOW,
                    WeatherCondition.HAIL,
                ]
            )
            
            if is_good:
                if current_period is None:
                    # Start a new period
                    current_period = {
                        "start_index": i,
                        "start_time": f.forecast_time,
                        "end_index": i,
                        "end_time": f.forecast_time,
                    }
                else:
                    # Extend current period
                    current_period["end_index"] = i
                    current_period["end_time"] = f.forecast_time
            elif current_period is not None:
                # End current period
                if current_period["end_index"] - current_period["start_index"] >= 2:  # At least 3 hours
                    good_weather_periods.append(current_period)
                current_period = None
        
        # Add last period if it exists
        if current_period is not None and current_period["end_index"] - current_period["start_index"] >= 2:
            good_weather_periods.append(current_period)
        
        # Create weather windows for good weather periods
        for period in good_weather_periods:
            window_id = f"{location_id}:window:{period['start_time'].isoformat()}"
            
            # Check if this window is already known
            if window_id not in self.weather_windows:
                # Create window
                window = {
                    "id": window_id,
                    "location_id": location_id,
                    "location_name": name,
                    "latitude": current.latitude,
                    "longitude": current.longitude,
                    "start_time": period["start_time"],
                    "end_time": period["end_time"],
                    "duration_hours": (period["end_time"] - period["start_time"]).total_seconds() / 3600,
                    "forecast": forecast[period["start_index"]:period["end_index"] + 1],
                }
                
                # Add to weather windows
                self.weather_windows[window_id] = window
                new_windows.append(window)
        
        # Clean up expired windows
        now = datetime.now(timezone.utc)
        expired_windows = [
            window_id for window_id, window in self.weather_windows.items()
            if window["end_time"] < now
        ]
        
        for window_id in expired_windows:
            del self.weather_windows[window_id]
        
        return new_windows
    
    async def _send_notifications(
        self, alerts: List[WeatherAlert], windows: List[Dict[str, Any]]
    ) -> None:
        """Send notifications for new alerts and weather windows.
        
        Args:
            alerts: New alerts
            windows: New weather windows
        """
        # Send notifications for new alerts
        for alert in alerts:
            for callback in self.notification_callbacks:
                try:
                    await callback({
                        "type": "alert",
                        "alert": alert.model_dump(),
                    })
                except Exception as e:
                    logger.error(f"Error sending alert notification: {str(e)}")
        
        # Send notifications for new weather windows
        for window in windows:
            for callback in self.notification_callbacks:
                try:
                    await callback({
                        "type": "window",
                        "window": {
                            "id": window["id"],
                            "location_id": window["location_id"],
                            "location_name": window["location_name"],
                            "latitude": window["latitude"],
                            "longitude": window["longitude"],
                            "start_time": window["start_time"].isoformat(),
                            "end_time": window["end_time"].isoformat(),
                            "duration_hours": window["duration_hours"],
                        },
                    })
                except Exception as e:
                    logger.error(f"Error sending window notification: {str(e)}")
    
    def add_location(self, latitude: float, longitude: float, name: Optional[str] = None) -> str:
        """Add a location to monitor.
        
        Args:
            latitude: Latitude of the location
            longitude: Longitude of the location
            name: Name of the location
            
        Returns:
            Location ID
        """
        location_id = f"loc_{len(self.monitored_locations)}"
        
        self.monitored_locations.append({
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
        for i, location in enumerate(self.monitored_locations):
            if location["id"] == location_id:
                self.monitored_locations.pop(i)
                return True
        
        return False
    
    def get_locations(self) -> List[Dict[str, Any]]:
        """Get all monitored locations.
        
        Returns:
            List of monitored locations
        """
        return self.monitored_locations
    
    def get_active_alerts(self) -> List[WeatherAlert]:
        """Get all active alerts.
        
        Returns:
            List of active alerts
        """
        return list(self.active_alerts.values())
    
    def get_weather_windows(self) -> List[Dict[str, Any]]:
        """Get all weather windows.
        
        Returns:
            List of weather windows
        """
        return list(self.weather_windows.values())
    
    def add_notification_callback(self, callback: callable) -> None:
        """Add a notification callback.
        
        Args:
            callback: Callback function
        """
        self.notification_callbacks.append(callback)
    
    def remove_notification_callback(self, callback: callable) -> bool:
        """Remove a notification callback.
        
        Args:
            callback: Callback function
            
        Returns:
            True if callback was removed, False otherwise
        """
        if callback in self.notification_callbacks:
            self.notification_callbacks.remove(callback)
            return True
        
        return False
