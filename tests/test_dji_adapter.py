"""
Tests for DJI Adapter

This module contains tests for the DJI adapter.
"""

import unittest
import asyncio
from unittest.mock import patch, MagicMock

from dronecore.dji_adapter import DJIAdapter
from dronecore.adapter_factory import AdapterFactory

class TestDJIAdapter(unittest.TestCase):
    """Test cases for DJI adapter."""

    def setUp(self):
        """Set up test fixtures."""
        self.connection_params = {
            "app_id": "test_app_id",
            "app_key": "test_app_key",
            "connection_type": "USB",
            "drone_model": "Mavic 3",
            "serial_number": "test_serial_number"
        }
        
        # Create adapter
        self.adapter = DJIAdapter(self.connection_params)
    
    def test_init(self):
        """Test adapter initialization."""
        self.assertEqual(self.adapter.app_id, "test_app_id")
        self.assertEqual(self.adapter.app_key, "test_app_key")
        self.assertEqual(self.adapter.connection_type, "USB")
        self.assertEqual(self.adapter.drone_model, "Mavic 3")
        self.assertEqual(self.adapter.serial_number, "test_serial_number")
        self.assertFalse(self.adapter._connected)
    
    @patch('asyncio.sleep', return_value=None)
    async def test_connect(self, mock_sleep):
        """Test connect method."""
        # Mock _dji_sdk_available
        self.adapter._dji_sdk_available = True
        
        # Connect
        result = await self.adapter.connect()
        
        # Check result
        self.assertTrue(result)
        self.assertTrue(self.adapter._connected)
        self.assertIsNotNone(self.adapter._telemetry_task)
        
        # Clean up
        await self.adapter.disconnect()
    
    @patch('asyncio.sleep', return_value=None)
    async def test_disconnect(self, mock_sleep):
        """Test disconnect method."""
        # Mock _dji_sdk_available and connect
        self.adapter._dji_sdk_available = True
        await self.adapter.connect()
        
        # Disconnect
        result = await self.adapter.disconnect()
        
        # Check result
        self.assertTrue(result)
        self.assertFalse(self.adapter._connected)
    
    def test_send_command(self):
        """Test send_command method."""
        # Mock _connected
        self.adapter._connected = True
        
        # Test takeoff command
        takeoff_command = {
            "command": "takeoff",
            "parameters": {}
        }
        result = self.adapter.send_command(takeoff_command)
        self.assertTrue(result)
        
        # Test land command
        land_command = {
            "command": "land",
            "parameters": {}
        }
        result = self.adapter.send_command(land_command)
        self.assertTrue(result)
        
        # Test goto command
        goto_command = {
            "command": "goto",
            "parameters": {
                "latitude": 37.7749,
                "longitude": -122.4194,
                "altitude": 100
            }
        }
        result = self.adapter.send_command(goto_command)
        self.assertTrue(result)
        
        # Test return_to_home command
        rth_command = {
            "command": "return_to_home",
            "parameters": {}
        }
        result = self.adapter.send_command(rth_command)
        self.assertTrue(result)
        
        # Test unknown command
        unknown_command = {
            "command": "unknown",
            "parameters": {}
        }
        result = self.adapter.send_command(unknown_command)
        self.assertFalse(result)
    
    def test_receive_telemetry(self):
        """Test receive_telemetry method."""
        # Set telemetry data
        self.adapter._telemetry_data = {
            "battery": 75,
            "position": {
                "latitude": 37.7749,
                "longitude": -122.4194,
                "altitude": 100,
            },
            "attitude": {
                "roll": 0,
                "pitch": 0,
                "yaw": 90,
            }
        }
        
        # Get telemetry
        telemetry = self.adapter.receive_telemetry()
        
        # Check telemetry
        self.assertEqual(telemetry["battery"], 75)
        self.assertEqual(telemetry["position"]["latitude"], 37.7749)
        self.assertEqual(telemetry["position"]["longitude"], -122.4194)
        self.assertEqual(telemetry["position"]["altitude"], 100)
        self.assertEqual(telemetry["attitude"]["roll"], 0)
        self.assertEqual(telemetry["attitude"]["pitch"], 0)
        self.assertEqual(telemetry["attitude"]["yaw"], 90)

class TestAdapterFactory(unittest.TestCase):
    """Test cases for AdapterFactory."""

    def test_create_dji_adapter(self):
        """Test creating DJI adapter."""
        connection_params = {
            "app_id": "test_app_id",
            "app_key": "test_app_key",
            "connection_type": "USB",
            "drone_model": "Mavic 3",
            "serial_number": "test_serial_number"
        }
        
        # Create adapter
        adapter = AdapterFactory.create_adapter("dji", connection_params)
        
        # Check adapter
        self.assertIsInstance(adapter, DJIAdapter)
        self.assertEqual(adapter.app_id, "test_app_id")
        self.assertEqual(adapter.app_key, "test_app_key")
        self.assertEqual(adapter.connection_type, "USB")
        self.assertEqual(adapter.drone_model, "Mavic 3")
        self.assertEqual(adapter.serial_number, "test_serial_number")

if __name__ == '__main__':
    unittest.main()
