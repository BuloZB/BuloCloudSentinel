"""
Power Management module for Bulo.Cloud Sentinel.

This module provides functionality for monitoring battery status,
predicting energy consumption, and optimizing power usage.
"""

from backend.power_management.models import (
    BatteryMetrics, BatteryStatus, BatteryThresholds,
    EnergyPrediction, EnergyPredictionModel,
    PowerOptimizationSettings, BatteryHealthRecord
)
from backend.power_management.battery_service import BatteryMonitoringService
from backend.power_management.energy_prediction import EnergyPredictionService
