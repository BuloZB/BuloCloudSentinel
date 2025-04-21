"""
Unit tests for the power management services.
"""

import pytest
import asyncio
from datetime import datetime

from backend.power_management.models import (
    BatteryMetrics, BatteryStatus, BatteryThresholds,
    EnergyPrediction, EnergyPredictionModel
)
from backend.power_management.battery_service import BatteryMonitoringService
from backend.power_management.energy_prediction import EnergyPredictionService


@pytest.fixture
async def battery_service():
    """Create a battery monitoring service for testing."""
    service = BatteryMonitoringService()
    await service.start()
    yield service
    await service.stop()


@pytest.fixture
async def energy_prediction_service():
    """Create an energy prediction service for testing."""
    service = EnergyPredictionService()
    yield service


@pytest.mark.asyncio
async def test_set_battery_thresholds(battery_service):
    """Test setting battery thresholds."""
    # Set thresholds
    thresholds = await battery_service.set_battery_thresholds(
        drone_id="test_drone",
        warning_threshold=35.0,
        critical_threshold=15.0,
        temperature_max=55.0,
        voltage_min=10.5,
        auto_return_threshold=25.0
    )
    
    # Check that thresholds were set
    assert thresholds.drone_id == "test_drone"
    assert thresholds.warning_threshold == 35.0
    assert thresholds.critical_threshold == 15.0
    assert thresholds.temperature_max == 55.0
    assert thresholds.voltage_min == 10.5
    assert thresholds.auto_return_threshold == 25.0
    
    # Get thresholds
    retrieved_thresholds = await battery_service.get_battery_thresholds("test_drone")
    
    # Check that retrieved thresholds match
    assert retrieved_thresholds.drone_id == thresholds.drone_id
    assert retrieved_thresholds.warning_threshold == thresholds.warning_threshold
    assert retrieved_thresholds.critical_threshold == thresholds.critical_threshold
    assert retrieved_thresholds.temperature_max == thresholds.temperature_max
    assert retrieved_thresholds.voltage_min == thresholds.voltage_min
    assert retrieved_thresholds.auto_return_threshold == thresholds.auto_return_threshold


@pytest.mark.asyncio
async def test_update_battery_status(battery_service):
    """Test updating battery status based on thresholds."""
    # Set thresholds
    await battery_service.set_battery_thresholds(
        drone_id="test_drone",
        warning_threshold=30.0,
        critical_threshold=15.0,
        temperature_max=60.0,
        voltage_min=10.5,
        auto_return_threshold=20.0
    )
    
    # Create metrics with normal battery level
    metrics = BatteryMetrics(
        drone_id="test_drone",
        voltage=12.0,
        current=5.0,
        capacity_percent=50.0,
        temperature=30.0
    )
    
    # Update status
    await battery_service._update_battery_status(metrics)
    
    # Check that status is normal
    assert metrics.status == BatteryStatus.NORMAL
    
    # Create metrics with warning battery level
    metrics = BatteryMetrics(
        drone_id="test_drone",
        voltage=11.5,
        current=5.0,
        capacity_percent=25.0,
        temperature=35.0
    )
    
    # Update status
    await battery_service._update_battery_status(metrics)
    
    # Check that status is warning
    assert metrics.status == BatteryStatus.WARNING
    
    # Create metrics with critical battery level
    metrics = BatteryMetrics(
        drone_id="test_drone",
        voltage=10.8,
        current=5.0,
        capacity_percent=10.0,
        temperature=40.0
    )
    
    # Update status
    await battery_service._update_battery_status(metrics)
    
    # Check that status is critical
    assert metrics.status == BatteryStatus.CRITICAL
    
    # Create metrics with high temperature
    metrics = BatteryMetrics(
        drone_id="test_drone",
        voltage=12.0,
        current=5.0,
        capacity_percent=50.0,
        temperature=65.0
    )
    
    # Update status
    await battery_service._update_battery_status(metrics)
    
    # Check that status is critical due to high temperature
    assert metrics.status == BatteryStatus.CRITICAL
    
    # Create metrics with low voltage
    metrics = BatteryMetrics(
        drone_id="test_drone",
        voltage=10.0,
        current=5.0,
        capacity_percent=50.0,
        temperature=30.0
    )
    
    # Update status
    await battery_service._update_battery_status(metrics)
    
    # Check that status is critical due to low voltage
    assert metrics.status == BatteryStatus.CRITICAL


@pytest.mark.asyncio
async def test_predict_energy_consumption(energy_prediction_service):
    """Test predicting energy consumption for a mission."""
    # Create waypoints
    waypoints = [
        {"latitude": 47.6062, "longitude": -122.3321, "altitude": 50},
        {"latitude": 47.6162, "longitude": -122.3221, "altitude": 70},
        {"latitude": 47.6262, "longitude": -122.3121, "altitude": 50}
    ]
    
    # Predict energy consumption
    prediction = await energy_prediction_service.predict_energy_consumption(
        drone_id="test_drone",
        waypoints=waypoints,
        payload_weight=0.5,
        wind_speed=5.0,
        wind_direction=45.0,
        temperature=25.0,
        model_type=EnergyPredictionModel.LINEAR
    )
    
    # Check prediction
    assert prediction.drone_id == "test_drone"
    assert prediction.model_type == EnergyPredictionModel.LINEAR
    assert prediction.estimated_consumption > 0
    assert prediction.estimated_duration > 0
    assert prediction.estimated_range > 0
    assert 0 <= prediction.confidence <= 1
    assert prediction.margin_of_error > 0
    assert isinstance(prediction.is_feasible, bool)
    assert isinstance(prediction.factors, dict)
    assert len(prediction.factors) > 0
    
    # Check that recommendations are provided
    assert isinstance(prediction.recommendations, list)


@pytest.mark.asyncio
async def test_haversine_distance(energy_prediction_service):
    """Test calculating distance between coordinates."""
    # Seattle to Portland
    distance = energy_prediction_service._haversine_distance(
        lat1=47.6062,
        lon1=-122.3321,
        lat2=45.5051,
        lon2=-122.6750
    )
    
    # Distance should be approximately 234 km
    assert 230 <= distance <= 240
