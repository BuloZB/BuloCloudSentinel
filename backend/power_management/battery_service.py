"""
Battery Monitoring Service for Bulo.Cloud Sentinel.

This module provides services for monitoring drone battery status,
storing historical data, and generating alerts.
"""

import asyncio
import logging
import json
import time
from typing import Dict, List, Any, Optional, Set
from datetime import datetime, timedelta
import aioredis

from backend.power_management.models import (
    BatteryMetrics, BatteryThresholds, BatteryStatus,
    BatteryHealthRecord
)
from dronecore.drone_command_telemetry_hub import DroneCommandHub
from backend.notification.service import NotificationService

logger = logging.getLogger(__name__)

class BatteryMonitoringService:
    """
    Service for monitoring drone battery status.
    
    This service collects battery telemetry data, processes it,
    stores historical data, and generates alerts when thresholds
    are exceeded.
    """
    
    def __init__(
        self,
        db_session=None,
        redis_url=None,
        drone_command_hub=None,
        notification_service=None
    ):
        """
        Initialize the battery monitoring service.
        
        Args:
            db_session: Database session for persistence
            redis_url: URL for Redis cache
            drone_command_hub: Drone command hub for telemetry
            notification_service: Notification service for alerts
        """
        self.db = db_session
        self.redis_url = redis_url
        self.redis = None
        self.drone_command_hub = drone_command_hub or DroneCommandHub()
        self.notification_service = notification_service or NotificationService()
        
        # In-memory cache of current battery metrics
        self._battery_metrics: Dict[str, BatteryMetrics] = {}
        
        # In-memory cache of battery thresholds
        self._battery_thresholds: Dict[str, BatteryThresholds] = {}
        
        # Set to track drones with active alerts to prevent alert spam
        self._active_alerts: Dict[str, Set[str]] = {}
        
        # Background tasks
        self._background_tasks = set()
        self._running = False
    
    async def start(self):
        """Start the battery monitoring service."""
        if self._running:
            return
        
        self._running = True
        
        # Connect to Redis if URL provided
        if self.redis_url:
            self.redis = await aioredis.from_url(self.redis_url)
        
        # Load thresholds
        await self._load_thresholds()
        
        # Start monitoring task
        task = asyncio.create_task(self._monitor_batteries())
        self._background_tasks.add(task)
        task.add_done_callback(self._background_tasks.remove)
        
        logger.info("Battery monitoring service started")
    
    async def stop(self):
        """Stop the battery monitoring service."""
        self._running = False
        
        # Cancel all background tasks
        for task in self._background_tasks:
            task.cancel()
        
        # Wait for all tasks to complete
        if self._background_tasks:
            await asyncio.gather(*self._background_tasks, return_exceptions=True)
        
        # Close Redis connection
        if self.redis:
            await self.redis.close()
        
        logger.info("Battery monitoring service stopped")
    
    async def get_battery_metrics(self, drone_id: str) -> Optional[BatteryMetrics]:
        """
        Get current battery metrics for a drone.
        
        Args:
            drone_id: ID of the drone
            
        Returns:
            Battery metrics if available, None otherwise
        """
        # Check in-memory cache first
        if drone_id in self._battery_metrics:
            return self._battery_metrics[drone_id]
        
        # Check Redis cache
        if self.redis:
            cache_key = f"battery:metrics:{drone_id}"
            cached_data = await self.redis.get(cache_key)
            if cached_data:
                metrics_dict = json.loads(cached_data)
                return BatteryMetrics(**metrics_dict)
        
        # Get from telemetry
        try:
            telemetry = await self.drone_command_hub.get_telemetry(drone_id)
            if telemetry and "battery" in telemetry:
                battery_data = telemetry["battery"]
                
                # Create metrics
                metrics = BatteryMetrics(
                    drone_id=drone_id,
                    voltage=battery_data.get("voltage", 0.0),
                    current=battery_data.get("current", 0.0),
                    capacity_percent=battery_data.get("percentage", 0.0),
                    temperature=battery_data.get("temperature", 0.0),
                    discharge_rate=battery_data.get("discharge_rate"),
                    cycle_count=battery_data.get("cycle_count"),
                    health_percent=battery_data.get("health_percent"),
                    estimated_time_remaining=battery_data.get("time_remaining"),
                    raw_data=battery_data
                )
                
                # Update status based on thresholds
                await self._update_battery_status(metrics)
                
                # Cache metrics
                self._battery_metrics[drone_id] = metrics
                
                # Cache in Redis
                if self.redis:
                    cache_key = f"battery:metrics:{drone_id}"
                    await self.redis.set(
                        cache_key,
                        json.dumps(metrics.dict()),
                        ex=60  # Expire after 60 seconds
                    )
                
                return metrics
        except Exception as e:
            logger.error(f"Error getting battery metrics for drone {drone_id}: {str(e)}")
        
        return None
    
    async def get_battery_history(
        self,
        drone_id: str,
        start_time: Optional[datetime] = None,
        end_time: Optional[datetime] = None,
        interval: str = "1m"
    ) -> List[BatteryMetrics]:
        """
        Get historical battery metrics for a drone.
        
        Args:
            drone_id: ID of the drone
            start_time: Start time for history (default: 1 hour ago)
            end_time: End time for history (default: now)
            interval: Data interval (1s, 1m, 5m, 1h)
            
        Returns:
            List of historical battery metrics
        """
        if not self.db:
            raise ValueError("Database session required for historical data")
        
        # Set default time range if not provided
        if not end_time:
            end_time = datetime.utcnow()
        
        if not start_time:
            start_time = end_time - timedelta(hours=1)
        
        # This is a placeholder - in a real implementation, you would query the database
        # with appropriate time aggregation based on the interval
        # For example, using SQL window functions to downsample high-frequency data
        
        # For now, we'll just return an empty list
        return []
    
    async def get_fleet_battery_metrics(self, fleet_id: str) -> Dict[str, BatteryMetrics]:
        """
        Get battery metrics for all drones in a fleet.
        
        Args:
            fleet_id: ID of the fleet
            
        Returns:
            Dictionary mapping drone IDs to battery metrics
        """
        # Get drones in fleet
        # This is a placeholder - in a real implementation, you would get the drones from the fleet service
        drone_ids = []  # await fleet_service.get_fleet_drone_ids(fleet_id)
        
        # Get battery metrics for each drone
        metrics = {}
        for drone_id in drone_ids:
            drone_metrics = await self.get_battery_metrics(drone_id)
            if drone_metrics:
                metrics[drone_id] = drone_metrics
        
        return metrics
    
    async def set_battery_thresholds(
        self,
        drone_id: str,
        warning_threshold: float,
        critical_threshold: float,
        temperature_max: float,
        voltage_min: Optional[float] = None,
        auto_return_threshold: float = 20.0
    ) -> BatteryThresholds:
        """
        Set battery alert thresholds for a drone.
        
        Args:
            drone_id: ID of the drone
            warning_threshold: Warning threshold percentage
            critical_threshold: Critical threshold percentage
            temperature_max: Maximum temperature threshold
            voltage_min: Minimum voltage threshold
            auto_return_threshold: Auto-return threshold percentage
            
        Returns:
            Updated battery thresholds
        """
        # Validate thresholds
        if warning_threshold <= critical_threshold:
            raise ValueError("Warning threshold must be greater than critical threshold")
        
        if auto_return_threshold <= critical_threshold:
            raise ValueError("Auto-return threshold must be greater than critical threshold")
        
        # Create or update thresholds
        thresholds = BatteryThresholds(
            drone_id=drone_id,
            warning_threshold=warning_threshold,
            critical_threshold=critical_threshold,
            temperature_max=temperature_max,
            voltage_min=voltage_min,
            auto_return_threshold=auto_return_threshold
        )
        
        # Update in-memory cache
        self._battery_thresholds[drone_id] = thresholds
        
        # Persist to database if available
        if self.db:
            # This is a placeholder - in a real implementation, you would save to the database
            pass
        
        # Cache in Redis
        if self.redis:
            cache_key = f"battery:thresholds:{drone_id}"
            await self.redis.set(
                cache_key,
                json.dumps(thresholds.dict()),
                ex=3600  # Expire after 1 hour
            )
        
        logger.info(f"Set battery thresholds for drone {drone_id}")
        
        return thresholds
    
    async def get_battery_thresholds(self, drone_id: str) -> BatteryThresholds:
        """
        Get battery alert thresholds for a drone.
        
        Args:
            drone_id: ID of the drone
            
        Returns:
            Battery thresholds
        """
        # Check in-memory cache first
        if drone_id in self._battery_thresholds:
            return self._battery_thresholds[drone_id]
        
        # Check Redis cache
        if self.redis:
            cache_key = f"battery:thresholds:{drone_id}"
            cached_data = await self.redis.get(cache_key)
            if cached_data:
                thresholds_dict = json.loads(cached_data)
                thresholds = BatteryThresholds(**thresholds_dict)
                self._battery_thresholds[drone_id] = thresholds
                return thresholds
        
        # Get from database
        if self.db:
            # This is a placeholder - in a real implementation, you would query the database
            pass
        
        # Return default thresholds
        thresholds = BatteryThresholds(
            drone_id=drone_id,
            warning_threshold=30.0,
            critical_threshold=15.0,
            temperature_max=60.0,
            auto_return_threshold=20.0
        )
        
        self._battery_thresholds[drone_id] = thresholds
        return thresholds
    
    async def get_battery_health(
        self,
        drone_id: str,
        battery_id: Optional[str] = None
    ) -> Optional[BatteryHealthRecord]:
        """
        Get battery health information.
        
        Args:
            drone_id: ID of the drone
            battery_id: Optional ID of the specific battery
            
        Returns:
            Battery health record if available, None otherwise
        """
        if not self.db:
            raise ValueError("Database session required for battery health data")
        
        # This is a placeholder - in a real implementation, you would query the database
        # for the most recent battery health record
        
        # For now, we'll just return a dummy record
        return BatteryHealthRecord(
            drone_id=drone_id,
            battery_id=battery_id or "default_battery",
            cycle_count=50,
            health_percent=95.0,
            initial_capacity=5000.0,
            current_capacity=4750.0,
            estimated_remaining_cycles=450,
            recommendation="Battery is in good condition"
        )
    
    async def trigger_return_to_home(self, drone_id: str, reason: str) -> bool:
        """
        Trigger return-to-home for a drone.
        
        Args:
            drone_id: ID of the drone
            reason: Reason for RTH
            
        Returns:
            True if RTH was triggered successfully, False otherwise
        """
        try:
            # Send RTH command to drone
            success = await self.drone_command_hub.return_to_home(drone_id)
            
            if success:
                logger.info(f"Triggered RTH for drone {drone_id}: {reason}")
                
                # Send notification
                await self.notification_service.send_notification(
                    title="Return to Home Triggered",
                    message=f"Drone {drone_id} is returning to home: {reason}",
                    level="warning",
                    drone_id=drone_id
                )
            
            return success
        except Exception as e:
            logger.error(f"Error triggering RTH for drone {drone_id}: {str(e)}")
            return False
    
    async def _load_thresholds(self):
        """Load battery thresholds from database."""
        if self.db:
            # This is a placeholder - in a real implementation, you would load from the database
            pass
    
    async def _monitor_batteries(self):
        """Background task for monitoring battery status."""
        while self._running:
            try:
                # Get connected drones
                drones = await self.drone_command_hub.get_connected_drones()
                
                for drone_id in drones:
                    try:
                        # Get battery metrics
                        metrics = await self.get_battery_metrics(drone_id)
                        if not metrics:
                            continue
                        
                        # Store metrics in database
                        if self.db:
                            await self._store_metrics(metrics)
                        
                        # Check thresholds and generate alerts
                        await self._check_thresholds(metrics)
                    except Exception as e:
                        logger.error(f"Error monitoring battery for drone {drone_id}: {str(e)}")
            except Exception as e:
                logger.error(f"Error in battery monitoring: {str(e)}")
            
            # Sleep for 5 seconds
            await asyncio.sleep(5)
    
    async def _store_metrics(self, metrics: BatteryMetrics):
        """
        Store battery metrics in database.
        
        Args:
            metrics: Battery metrics to store
        """
        # This is a placeholder - in a real implementation, you would store in the database
        pass
    
    async def _update_battery_status(self, metrics: BatteryMetrics):
        """
        Update battery status based on thresholds.
        
        Args:
            metrics: Battery metrics to update
        """
        # Get thresholds
        thresholds = await self.get_battery_thresholds(metrics.drone_id)
        
        # Update status
        if metrics.capacity_percent <= thresholds.critical_threshold:
            metrics.status = BatteryStatus.CRITICAL
        elif metrics.capacity_percent <= thresholds.warning_threshold:
            metrics.status = BatteryStatus.WARNING
        else:
            metrics.status = BatteryStatus.NORMAL
        
        # Check temperature
        if metrics.temperature >= thresholds.temperature_max:
            metrics.status = BatteryStatus.CRITICAL
        
        # Check voltage
        if thresholds.voltage_min is not None and metrics.voltage <= thresholds.voltage_min:
            metrics.status = BatteryStatus.CRITICAL
    
    async def _check_thresholds(self, metrics: BatteryMetrics):
        """
        Check battery thresholds and generate alerts.
        
        Args:
            metrics: Battery metrics to check
        """
        drone_id = metrics.drone_id
        
        # Initialize active alerts set if not exists
        if drone_id not in self._active_alerts:
            self._active_alerts[drone_id] = set()
        
        # Get thresholds
        thresholds = await self.get_battery_thresholds(drone_id)
        
        # Check critical threshold
        if metrics.capacity_percent <= thresholds.critical_threshold:
            alert_key = "critical_battery"
            if alert_key not in self._active_alerts[drone_id]:
                # Send notification
                await self.notification_service.send_notification(
                    title="Critical Battery Level",
                    message=f"Drone {drone_id} battery at {metrics.capacity_percent:.1f}% - Critical level!",
                    level="critical",
                    drone_id=drone_id
                )
                
                # Add to active alerts
                self._active_alerts[drone_id].add(alert_key)
                
                # Trigger RTH
                await self.trigger_return_to_home(drone_id, "Critical battery level")
        else:
            # Remove from active alerts if exists
            self._active_alerts[drone_id].discard("critical_battery")
        
        # Check warning threshold
        if metrics.capacity_percent <= thresholds.warning_threshold and metrics.capacity_percent > thresholds.critical_threshold:
            alert_key = "warning_battery"
            if alert_key not in self._active_alerts[drone_id]:
                # Send notification
                await self.notification_service.send_notification(
                    title="Low Battery Warning",
                    message=f"Drone {drone_id} battery at {metrics.capacity_percent:.1f}% - Consider returning soon",
                    level="warning",
                    drone_id=drone_id
                )
                
                # Add to active alerts
                self._active_alerts[drone_id].add(alert_key)
        else:
            # Remove from active alerts if exists
            self._active_alerts[drone_id].discard("warning_battery")
        
        # Check auto-return threshold
        if metrics.capacity_percent <= thresholds.auto_return_threshold and metrics.capacity_percent > thresholds.critical_threshold:
            alert_key = "auto_return"
            if alert_key not in self._active_alerts[drone_id]:
                # Send notification
                await self.notification_service.send_notification(
                    title="Auto-Return Threshold Reached",
                    message=f"Drone {drone_id} battery at {metrics.capacity_percent:.1f}% - Initiating auto-return",
                    level="warning",
                    drone_id=drone_id
                )
                
                # Add to active alerts
                self._active_alerts[drone_id].add(alert_key)
                
                # Trigger RTH
                await self.trigger_return_to_home(drone_id, "Auto-return threshold reached")
        else:
            # Remove from active alerts if exists
            self._active_alerts[drone_id].discard("auto_return")
        
        # Check temperature
        if metrics.temperature >= thresholds.temperature_max:
            alert_key = "high_temperature"
            if alert_key not in self._active_alerts[drone_id]:
                # Send notification
                await self.notification_service.send_notification(
                    title="High Battery Temperature",
                    message=f"Drone {drone_id} battery temperature at {metrics.temperature:.1f}°C - Critical level!",
                    level="critical",
                    drone_id=drone_id
                )
                
                # Add to active alerts
                self._active_alerts[drone_id].add(alert_key)
                
                # Trigger RTH
                await self.trigger_return_to_home(drone_id, "High battery temperature")
        else:
            # Remove from active alerts if exists
            self._active_alerts[drone_id].discard("high_temperature")
        
        # Check voltage
        if thresholds.voltage_min is not None and metrics.voltage <= thresholds.voltage_min:
            alert_key = "low_voltage"
            if alert_key not in self._active_alerts[drone_id]:
                # Send notification
                await self.notification_service.send_notification(
                    title="Low Battery Voltage",
                    message=f"Drone {drone_id} battery voltage at {metrics.voltage:.2f}V - Critical level!",
                    level="critical",
                    drone_id=drone_id
                )
                
                # Add to active alerts
                self._active_alerts[drone_id].add(alert_key)
                
                # Trigger RTH
                await self.trigger_return_to_home(drone_id, "Low battery voltage")
        else:
            # Remove from active alerts if exists
            self._active_alerts[drone_id].discard("low_voltage")
