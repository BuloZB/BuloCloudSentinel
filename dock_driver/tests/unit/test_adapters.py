"""
Unit Tests for Dock Adapters

This module contains unit tests for the dock adapters.
"""

import pytest
import asyncio
from unittest.mock import patch, MagicMock, AsyncMock
from datetime import datetime

from dock_driver.adapters.interface import DockStatus, ChargingStatus, DockAdapterFactory
from dock_driver.adapters.dji.adapter import DJIDockAdapter
from dock_driver.adapters.heisha.adapter import HeishaDockAdapter
from dock_driver.adapters.esp32.adapter import ESP32DockAdapter


# Test DockAdapterFactory
def test_dock_adapter_factory():
    """Test the DockAdapterFactory."""
    # Test DJI adapter creation
    dji_adapter = DockAdapterFactory.create_adapter("dji", {"api_key": "test", "api_secret": "test"})
    assert isinstance(dji_adapter, DJIDockAdapter)
    
    # Test Heisha adapter creation
    heisha_adapter = DockAdapterFactory.create_adapter("heisha", {"rest_api_url": "test", "modbus_host": "test"})
    assert isinstance(heisha_adapter, HeishaDockAdapter)
    
    # Test ESP32 adapter creation
    esp32_adapter = DockAdapterFactory.create_adapter("esp32", {"mqtt_broker": "test"})
    assert isinstance(esp32_adapter, ESP32DockAdapter)
    
    # Test invalid adapter type
    invalid_adapter = DockAdapterFactory.create_adapter("invalid", {})
    assert invalid_adapter is None


# Test DJI Dock Adapter
@pytest.mark.asyncio
async def test_dji_dock_adapter():
    """Test the DJI Dock Adapter."""
    # Mock httpx.AsyncClient
    with patch("httpx.AsyncClient") as mock_client:
        # Mock response for token request
        mock_token_response = MagicMock()
        mock_token_response.status_code = 200
        mock_token_response.json.return_value = {
            "code": 0,
            "data": {
                "access_token": "test_token",
                "expires_in": 7200
            }
        }
        
        # Mock response for status request
        mock_status_response = MagicMock()
        mock_status_response.status_code = 200
        mock_status_response.json.return_value = {
            "code": 0,
            "data": {
                "status": 1,
                "charging_status": 0,
                "door_state": "closed",
                "drone_connected": False,
                "error_code": 0,
                "error_message": ""
            }
        }
        
        # Mock response for telemetry request
        mock_telemetry_response = MagicMock()
        mock_telemetry_response.status_code = 200
        mock_telemetry_response.json.return_value = {
            "code": 0,
            "data": {
                "temperature": 25.0,
                "humidity": 50.0,
                "charging_voltage": 12.0,
                "charging_current": 2.0,
                "battery_level": 80,
                "network_signal": 90
            }
        }
        
        # Mock response for control requests
        mock_control_response = MagicMock()
        mock_control_response.status_code = 200
        mock_control_response.json.return_value = {
            "code": 0,
            "data": {}
        }
        
        # Set up mock client
        mock_client_instance = AsyncMock()
        mock_client_instance.__aenter__.return_value = mock_client_instance
        mock_client_instance.post.side_effect = [
            mock_token_response,  # For token request
            mock_control_response,  # For open_dock
            mock_control_response,  # For close_dock
            mock_control_response,  # For start_charging
            mock_control_response   # For stop_charging
        ]
        mock_client_instance.get.side_effect = [
            mock_status_response,   # For get_status
            mock_telemetry_response  # For get_telemetry
        ]
        mock_client.return_value = mock_client_instance
        
        # Create adapter
        adapter = DJIDockAdapter({
            "api_key": "test_key",
            "api_secret": "test_secret",
            "dock_sn": "test_dock",
            "region": "us-east-1",
            "refresh_interval": 30
        })
        
        # Initialize adapter
        success = await adapter.initialize()
        assert success is True
        
        # Test get_status
        status = await adapter.get_status()
        assert status["dock_id"] == "test_dock"
        assert status["status"] == DockStatus.ONLINE
        assert status["charging_status"] == ChargingStatus.IDLE
        
        # Test get_telemetry
        telemetry = await adapter.get_telemetry()
        assert telemetry["dock_id"] == "test_dock"
        assert telemetry["temperature"] == 25.0
        assert telemetry["humidity"] == 50.0
        assert telemetry["charging_voltage"] == 12.0
        assert telemetry["charging_current"] == 2.0
        assert telemetry["battery_level"] == 80
        assert telemetry["network_signal"] == 90
        
        # Test open_dock
        success = await adapter.open_dock()
        assert success is True
        
        # Test close_dock
        success = await adapter.close_dock()
        assert success is True
        
        # Test start_charging
        success = await adapter.start_charging()
        assert success is True
        
        # Test stop_charging
        success = await adapter.stop_charging()
        assert success is True
        
        # Shutdown adapter
        await adapter.shutdown()


# Test Heisha Dock Adapter
@pytest.mark.asyncio
async def test_heisha_dock_adapter():
    """Test the Heisha Dock Adapter."""
    # Mock pymodbus.client.AsyncModbusTcpClient
    with patch("dock_driver.adapters.heisha.adapter.AsyncModbusTcpClient") as mock_modbus_client, \
         patch("httpx.AsyncClient") as mock_http_client:
        # Mock modbus client
        mock_modbus_instance = MagicMock()
        mock_modbus_instance.connect = AsyncMock(return_value=True)
        mock_modbus_instance.connected = True
        mock_modbus_instance.close = AsyncMock()
        
        # Mock modbus read response
        mock_read_response = MagicMock()
        mock_read_response.isError.return_value = False
        mock_read_response.registers = [1, 0, 80, 20, 120, 25, 50, 30, 0]
        mock_modbus_instance.read_holding_registers = AsyncMock(return_value=mock_read_response)
        
        # Mock modbus write response
        mock_write_response = MagicMock()
        mock_write_response.isError.return_value = False
        mock_modbus_instance.write_register = AsyncMock(return_value=mock_write_response)
        
        mock_modbus_client.return_value = mock_modbus_instance
        
        # Mock HTTP client
        mock_http_instance = AsyncMock()
        mock_http_instance.__aenter__.return_value = mock_http_instance
        
        # Mock HTTP responses
        mock_status_response = MagicMock()
        mock_status_response.status_code = 200
        mock_status_response.json.return_value = {
            "door_state": "closed",
            "drone_connected": False,
            "error_code": 0,
            "error_message": ""
        }
        
        mock_telemetry_response = MagicMock()
        mock_telemetry_response.status_code = 200
        mock_telemetry_response.json.return_value = {
            "network_signal": 90
        }
        
        mock_http_instance.get.side_effect = [
            mock_status_response,   # For _get_rest_status
            mock_telemetry_response  # For _get_rest_telemetry
        ]
        
        mock_http_client.return_value = mock_http_instance
        
        # Create adapter
        adapter = HeishaDockAdapter({
            "rest_api_url": "http://test-heisha:8080/api",
            "modbus_host": "test-heisha",
            "modbus_port": 502,
            "modbus_unit_id": 1,
            "refresh_interval": 15
        })
        
        # Initialize adapter
        success = await adapter.initialize()
        assert success is True
        
        # Test get_status
        status = await adapter.get_status()
        assert status["dock_id"] == "heisha-test-heisha"
        assert status["status"] == DockStatus.ONLINE
        assert status["charging_status"] == ChargingStatus.IDLE
        assert status["door_state"] == "closed"
        assert status["drone_connected"] is False
        
        # Test get_telemetry
        telemetry = await adapter.get_telemetry()
        assert telemetry["dock_id"] == "heisha-test-heisha"
        assert telemetry["battery_level"] == 80
        assert telemetry["charging_current"] == 2.0
        assert telemetry["charging_voltage"] == 12.0
        assert telemetry["temperature"] == 2.5
        assert telemetry["humidity"] == 50
        assert telemetry["fan_speed"] == 30
        assert telemetry["network_signal"] == 90
        
        # Test open_dock
        success = await adapter.open_dock()
        assert success is True
        
        # Test close_dock
        success = await adapter.close_dock()
        assert success is True
        
        # Test start_charging
        success = await adapter.start_charging()
        assert success is True
        
        # Test stop_charging
        success = await adapter.stop_charging()
        assert success is True
        
        # Test set_fan_speed
        success = await adapter.set_fan_speed(50)
        assert success is True
        
        # Test set_heater_state
        success = await adapter.set_heater_state(True)
        assert success is True
        
        # Shutdown adapter
        await adapter.shutdown()


# Test ESP32 Dock Adapter
@pytest.mark.asyncio
async def test_esp32_dock_adapter():
    """Test the ESP32 Dock Adapter."""
    # Mock aiomqtt.Client
    with patch("dock_driver.adapters.esp32.adapter.aiomqtt.Client") as mock_mqtt_client:
        # Mock MQTT client
        mock_mqtt_instance = AsyncMock()
        mock_mqtt_instance.connect = AsyncMock()
        mock_mqtt_instance.disconnect = AsyncMock()
        mock_mqtt_instance.subscribe = AsyncMock()
        mock_mqtt_instance.publish = AsyncMock()
        
        # Mock messages context manager
        mock_messages_context = AsyncMock()
        mock_mqtt_instance.messages.return_value = mock_messages_context
        
        mock_mqtt_client.return_value = mock_mqtt_instance
        
        # Create adapter
        adapter = ESP32DockAdapter({
            "mqtt_broker": "test-mqtt",
            "mqtt_port": 1883,
            "mqtt_topic_prefix": "esp32_dock",
            "dock_id": "esp32-dock-1",
            "refresh_interval": 10
        })
        
        # Set some values for testing
        adapter.available = True
        adapter.relay_state = True
        adapter.voltage = 12.0
        adapter.current = 2.0
        adapter.temperature = 25.0
        
        # Initialize adapter
        success = await adapter.initialize()
        assert success is True
        
        # Test get_status
        status = await adapter.get_status()
        assert status["dock_id"] == "esp32-dock-1"
        assert status["status"] == DockStatus.ONLINE
        assert status["charging_status"] == ChargingStatus.CHARGING
        assert status["relay_state"] is True
        assert status["available"] is True
        
        # Test get_telemetry
        telemetry = await adapter.get_telemetry()
        assert telemetry["dock_id"] == "esp32-dock-1"
        assert telemetry["voltage"] == 12.0
        assert telemetry["current"] == 2.0
        assert telemetry["temperature"] == 25.0
        assert telemetry["relay_state"] is True
        
        # Test set_relay_state
        success = await adapter.set_relay_state(False)
        assert success is True
        
        # Test start_charging
        success = await adapter.start_charging()
        assert success is True
        
        # Test stop_charging
        success = await adapter.stop_charging()
        assert success is True
        
        # Shutdown adapter
        await adapter.shutdown()
