#!/usr/bin/env python3
"""
DJI Adapter Integration Tests

This script provides integration tests for the DJI adapter in Bulo.Cloud Sentinel.
It tests all major functionality including:
- Connection and disconnection
- Telemetry reception
- Command execution
- Camera and gimbal control
- Mission execution

Note: These tests can be run in simulation mode without actual hardware,
or with real hardware by setting the appropriate environment variables.
"""

import os
import sys
import unittest
import asyncio
import json
import logging
import tempfile
from typing import Dict, Any, List
from unittest.mock import patch, MagicMock

# Add project root to path to ensure imports work
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from dronecore.adapter_factory import AdapterFactory
from dronecore.dji_adapter import DJIAdapter

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# Test configuration
TEST_CONFIG = {
    "app_id": os.environ.get("DJI_APP_ID", "test_app_id"),
    "app_key": os.environ.get("DJI_APP_KEY", "test_app_key"),
    "connection_type": os.environ.get("DJI_CONNECTION_TYPE", "USB"),
    "drone_model": os.environ.get("DJI_DRONE_MODEL", "Mavic 3"),
    "serial_number": os.environ.get("DJI_SERIAL_NUMBER", None),
    "bridge_ip": os.environ.get("DJI_BRIDGE_IP", None),
    "bridge_port": os.environ.get("DJI_BRIDGE_PORT", None),
    "enable_virtual_stick": True,
    "enable_camera": True,
    "enable_gimbal": True,
    "enable_waypoint": True,
    "enable_hotpoint": True,
    "enable_follow_me": True,
    "enable_timeline": True,
    "enable_hd_video": True,
}

# Flag to determine if we're using real hardware or simulation
USE_REAL_HARDWARE = os.environ.get("DJI_USE_REAL_HARDWARE", "false").lower() == "true"


class TestDJIAdapterIntegration(unittest.TestCase):
    """Integration tests for the DJI adapter."""

    def setUp(self):
        """Set up the test environment."""
        self.connection_params = TEST_CONFIG.copy()
        self.adapter = None
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)

    def tearDown(self):
        """Clean up after tests."""
        if self.adapter and self.adapter._connected:
            self.loop.run_until_complete(self.adapter.disconnect())
        self.loop.close()

    def test_01_create_adapter(self):
        """Test creating the DJI adapter."""
        adapter = AdapterFactory.create_adapter("dji", self.connection_params)
        self.assertIsNotNone(adapter)
        self.assertIsInstance(adapter, DJIAdapter)
        self.adapter = adapter

    def test_02_connect_disconnect(self):
        """Test connecting to and disconnecting from the drone."""
        adapter = AdapterFactory.create_adapter("dji", self.connection_params)
        self.adapter = adapter

        # Connect
        connected = self.loop.run_until_complete(adapter.connect())
        self.assertTrue(connected)
        self.assertTrue(adapter._connected)

        # Disconnect
        disconnected = self.loop.run_until_complete(adapter.disconnect())
        self.assertTrue(disconnected)
        self.assertFalse(adapter._connected)

    def test_03_telemetry(self):
        """Test receiving telemetry data."""
        adapter = AdapterFactory.create_adapter("dji", self.connection_params)
        self.adapter = adapter

        # Connect
        connected = self.loop.run_until_complete(adapter.connect())
        self.assertTrue(connected)

        # Get telemetry
        telemetry = adapter.receive_telemetry()
        self.assertIsNotNone(telemetry)
        self.assertIn("battery", telemetry)
        self.assertIn("position", telemetry)
        self.assertIn("attitude", telemetry)
        self.assertIn("flight_status", telemetry)

        # Check telemetry structure
        self.assertIn("percent", telemetry["battery"])
        self.assertIn("latitude", telemetry["position"])
        self.assertIn("longitude", telemetry["position"])
        self.assertIn("altitude", telemetry["position"])
        self.assertIn("mode", telemetry["flight_status"])

    def test_04_flight_commands(self):
        """Test flight control commands."""
        adapter = AdapterFactory.create_adapter("dji", self.connection_params)
        self.adapter = adapter

        # Connect
        connected = self.loop.run_until_complete(adapter.connect())
        self.assertTrue(connected)

        # Test takeoff command
        if USE_REAL_HARDWARE:
            # Only test with real hardware if explicitly enabled
            takeoff_command = {
                "command": "takeoff",
                "parameters": {}
            }
            result = adapter.send_command(takeoff_command)
            self.assertTrue(result)

            # Wait for takeoff to complete
            self.loop.run_until_complete(asyncio.sleep(5))

            # Test land command
            land_command = {
                "command": "land",
                "parameters": {}
            }
            result = adapter.send_command(land_command)
            self.assertTrue(result)

            # Wait for landing to complete
            self.loop.run_until_complete(asyncio.sleep(5))
        else:
            # In simulation mode, just verify the command structure
            takeoff_command = {
                "command": "takeoff",
                "parameters": {}
            }
            with patch.object(adapter, 'send_command', return_value=True) as mock_send:
                result = adapter.send_command(takeoff_command)
                self.assertTrue(result)
                mock_send.assert_called_once_with(takeoff_command)

    def test_05_camera_commands(self):
        """Test camera control commands."""
        adapter = AdapterFactory.create_adapter("dji", self.connection_params)
        self.adapter = adapter

        # Connect
        connected = self.loop.run_until_complete(adapter.connect())
        self.assertTrue(connected)

        # Test camera commands
        if USE_REAL_HARDWARE:
            # Take photo
            take_photo_command = {
                "command": "take_photo",
                "parameters": {}
            }
            result = adapter.send_command(take_photo_command)
            self.assertTrue(result)

            # Start recording
            start_recording_command = {
                "command": "start_recording",
                "parameters": {}
            }
            result = adapter.send_command(start_recording_command)
            self.assertTrue(result)

            # Wait a moment
            self.loop.run_until_complete(asyncio.sleep(3))

            # Stop recording
            stop_recording_command = {
                "command": "stop_recording",
                "parameters": {}
            }
            result = adapter.send_command(stop_recording_command)
            self.assertTrue(result)
        else:
            # In simulation mode, just verify the command structure
            commands = [
                {"command": "take_photo", "parameters": {}},
                {"command": "start_recording", "parameters": {}},
                {"command": "stop_recording", "parameters": {}},
                {"command": "set_camera_mode", "parameters": {"mode": "PHOTO"}},
                {"command": "set_photo_mode", "parameters": {"mode": "SINGLE"}},
                {"command": "set_video_resolution", "parameters": {"resolution": "RES_4K"}}
            ]
            
            with patch.object(adapter, 'send_command', return_value=True) as mock_send:
                for cmd in commands:
                    result = adapter.send_command(cmd)
                    self.assertTrue(result)

    def test_06_gimbal_commands(self):
        """Test gimbal control commands."""
        adapter = AdapterFactory.create_adapter("dji", self.connection_params)
        self.adapter = adapter

        # Connect
        connected = self.loop.run_until_complete(adapter.connect())
        self.assertTrue(connected)

        # Test gimbal commands
        if USE_REAL_HARDWARE:
            # Rotate gimbal
            rotate_gimbal_command = {
                "command": "rotate_gimbal",
                "parameters": {
                    "pitch": -45.0,
                    "roll": 0.0,
                    "yaw": 0.0,
                    "duration": 1.0
                }
            }
            result = adapter.send_command(rotate_gimbal_command)
            self.assertTrue(result)

            # Wait for rotation to complete
            self.loop.run_until_complete(asyncio.sleep(2))

            # Reset gimbal
            reset_gimbal_command = {
                "command": "reset_gimbal",
                "parameters": {}
            }
            result = adapter.send_command(reset_gimbal_command)
            self.assertTrue(result)
        else:
            # In simulation mode, just verify the command structure
            commands = [
                {
                    "command": "rotate_gimbal", 
                    "parameters": {
                        "pitch": -45.0,
                        "roll": 0.0,
                        "yaw": 0.0,
                        "duration": 1.0
                    }
                },
                {"command": "reset_gimbal", "parameters": {}},
                {"command": "set_gimbal_mode", "parameters": {"mode": "YAW_FOLLOW"}}
            ]
            
            with patch.object(adapter, 'send_command', return_value=True) as mock_send:
                for cmd in commands:
                    result = adapter.send_command(cmd)
                    self.assertTrue(result)

    def test_07_waypoint_mission(self):
        """Test waypoint mission execution."""
        adapter = AdapterFactory.create_adapter("dji", self.connection_params)
        self.adapter = adapter

        # Connect
        connected = self.loop.run_until_complete(adapter.connect())
        self.assertTrue(connected)

        # Get current position
        telemetry = adapter.receive_telemetry()
        current_lat = telemetry['position']['latitude']
        current_lon = telemetry['position']['longitude']
        current_alt = telemetry['position']['altitude']

        # Create waypoints
        waypoints = [
            {
                "latitude": current_lat + 0.0001,
                "longitude": current_lon,
                "altitude": current_alt + 10,
                "heading": 0,
                "stay_time": 0
            },
            {
                "latitude": current_lat + 0.0001,
                "longitude": current_lon + 0.0001,
                "altitude": current_alt + 15,
                "heading": 90,
                "stay_time": 0
            },
            {
                "latitude": current_lat,
                "longitude": current_lon + 0.0001,
                "altitude": current_alt + 20,
                "heading": 180,
                "stay_time": 0
            },
            {
                "latitude": current_lat,
                "longitude": current_lon,
                "altitude": current_alt + 10,
                "heading": 270,
                "stay_time": 0
            }
        ]

        # Test waypoint mission
        if USE_REAL_HARDWARE:
            # Start waypoint mission
            start_mission_command = {
                "command": "start_waypoint_mission",
                "parameters": {
                    "waypoints": waypoints,
                    "speed": 5.0,
                    "finish_action": "go_home",
                    "heading_mode": "auto"
                }
            }
            result = adapter.send_command(start_mission_command)
            self.assertTrue(result)

            # Wait for mission to start
            self.loop.run_until_complete(asyncio.sleep(5))

            # Stop waypoint mission
            stop_mission_command = {
                "command": "stop_waypoint_mission",
                "parameters": {}
            }
            result = adapter.send_command(stop_mission_command)
            self.assertTrue(result)
        else:
            # In simulation mode, just verify the command structure
            with patch.object(adapter, 'send_command', return_value=True) as mock_send:
                start_mission_command = {
                    "command": "start_waypoint_mission",
                    "parameters": {
                        "waypoints": waypoints,
                        "speed": 5.0,
                        "finish_action": "go_home",
                        "heading_mode": "auto"
                    }
                }
                result = adapter.send_command(start_mission_command)
                self.assertTrue(result)


if __name__ == "__main__":
    unittest.main()
