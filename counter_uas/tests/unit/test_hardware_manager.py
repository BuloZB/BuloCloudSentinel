"""
Unit tests for the hardware manager.
"""

import asyncio
import unittest
from unittest.mock import patch, MagicMock

from counter_uas.hardware.hardware_manager import HardwareManager
from counter_uas.hardware.kerberos_sdr import KerberosSDR
from counter_uas.hardware.acconeer_radar import AcconeerRadar


class TestHardwareManager(unittest.TestCase):
    """Test cases for the hardware manager."""
    
    def setUp(self):
        """Set up the test case."""
        # Create a test configuration
        self.config = {
            "hardware": {
                "kerberos_sdr_devices": [
                    {
                        "device_id": "0",
                        "center_frequency": 915000000,
                        "sample_rate": 2400000,
                        "gain": 30.0,
                        "reference_channel": 0
                    }
                ],
                "acconeer_radar_devices": [
                    {
                        "device_id": "0",
                        "mode": "iq",
                        "start_range": 0.2,
                        "end_range": 5.0,
                        "update_rate": 10.0
                    }
                ]
            }
        }
        
        # Create a hardware manager with the test configuration
        self.hardware_manager = HardwareManager(self.config)
    
    def tearDown(self):
        """Tear down the test case."""
        # Run the shutdown method in the event loop
        loop = asyncio.get_event_loop()
        loop.run_until_complete(self.hardware_manager.shutdown())
    
    @patch("counter_uas.hardware.kerberos_sdr.KerberosSDR.initialize")
    @patch("counter_uas.hardware.acconeer_radar.AcconeerRadar.initialize")
    def test_initialize(self, mock_acconeer_initialize, mock_kerberos_initialize):
        """Test initialize method."""
        # Configure mocks
        mock_kerberos_initialize.return_value = asyncio.Future()
        mock_kerberos_initialize.return_value.set_result(True)
        mock_acconeer_initialize.return_value = asyncio.Future()
        mock_acconeer_initialize.return_value.set_result(True)
        
        # Run the initialize method in the event loop
        loop = asyncio.get_event_loop()
        result = loop.run_until_complete(self.hardware_manager.initialize())
        
        self.assertTrue(result)
        self.assertTrue(self.hardware_manager.initialized)
        mock_kerberos_initialize.assert_called_once()
        mock_acconeer_initialize.assert_called_once()
    
    @patch("counter_uas.hardware.kerberos_sdr.KerberosSDR.shutdown")
    @patch("counter_uas.hardware.acconeer_radar.AcconeerRadar.shutdown")
    def test_shutdown(self, mock_acconeer_shutdown, mock_kerberos_shutdown):
        """Test shutdown method."""
        # Configure mocks
        mock_kerberos_shutdown.return_value = asyncio.Future()
        mock_kerberos_shutdown.return_value.set_result(True)
        mock_acconeer_shutdown.return_value = asyncio.Future()
        mock_acconeer_shutdown.return_value.set_result(True)
        
        # Initialize first
        self.hardware_manager.initialized = True
        
        # Then shutdown
        loop = asyncio.get_event_loop()
        result = loop.run_until_complete(self.hardware_manager.shutdown())
        
        self.assertTrue(result)
        self.assertFalse(self.hardware_manager.initialized)
        mock_kerberos_shutdown.assert_called_once()
        mock_acconeer_shutdown.assert_called_once()
    
    @patch("counter_uas.hardware.kerberos_sdr.KerberosSDR.get_status")
    @patch("counter_uas.hardware.acconeer_radar.AcconeerRadar.get_status")
    def test_get_status(self, mock_acconeer_get_status, mock_kerberos_get_status):
        """Test get_status method."""
        # Configure mocks
        mock_kerberos_get_status.return_value = asyncio.Future()
        mock_kerberos_get_status.return_value.set_result({
            "device_index": 0,
            "initialized": True,
            "running": True
        })
        mock_acconeer_get_status.return_value = asyncio.Future()
        mock_acconeer_get_status.return_value.set_result({
            "device_id": "0",
            "initialized": True,
            "running": True
        })
        
        # Initialize first
        self.hardware_manager.initialized = True
        
        # Then get_status
        loop = asyncio.get_event_loop()
        status = loop.run_until_complete(self.hardware_manager.get_status())
        
        self.assertIsInstance(status, dict)
        self.assertTrue(status["initialized"])
        self.assertIsInstance(status["devices"], dict)
        self.assertIn("kerberos_sdr_0", status["devices"])
        self.assertIn("acconeer_radar_0", status["devices"])
        mock_kerberos_get_status.assert_called_once()
        mock_acconeer_get_status.assert_called_once()
    
    def test_get_device(self):
        """Test get_device method."""
        # Initialize first
        loop = asyncio.get_event_loop()
        loop.run_until_complete(self.hardware_manager.initialize())
        
        # Then get_device
        kerberos_device = self.hardware_manager.get_device("kerberos_sdr_0")
        acconeer_device = self.hardware_manager.get_device("acconeer_radar_0")
        
        self.assertIsInstance(kerberos_device, KerberosSDR)
        self.assertIsInstance(acconeer_device, AcconeerRadar)
    
    def test_get_kerberos_sdr_devices(self):
        """Test get_kerberos_sdr_devices method."""
        # Initialize first
        loop = asyncio.get_event_loop()
        loop.run_until_complete(self.hardware_manager.initialize())
        
        # Then get_kerberos_sdr_devices
        devices = self.hardware_manager.get_kerberos_sdr_devices()
        
        self.assertIsInstance(devices, list)
        self.assertEqual(len(devices), 1)
        self.assertIsInstance(devices[0], KerberosSDR)
    
    def test_get_acconeer_radar_devices(self):
        """Test get_acconeer_radar_devices method."""
        # Initialize first
        loop = asyncio.get_event_loop()
        loop.run_until_complete(self.hardware_manager.initialize())
        
        # Then get_acconeer_radar_devices
        devices = self.hardware_manager.get_acconeer_radar_devices()
        
        self.assertIsInstance(devices, list)
        self.assertEqual(len(devices), 1)
        self.assertIsInstance(devices[0], AcconeerRadar)
    
    @patch("counter_uas.hardware.kerberos_sdr.KerberosSDR.configure")
    def test_configure_device(self, mock_configure):
        """Test configure_device method."""
        # Configure mock
        mock_configure.return_value = asyncio.Future()
        mock_configure.return_value.set_result(True)
        
        # Initialize first
        loop = asyncio.get_event_loop()
        loop.run_until_complete(self.hardware_manager.initialize())
        
        # Then configure_device
        config = {"center_frequency": 2400000000}
        result = loop.run_until_complete(self.hardware_manager.configure_device("kerberos_sdr_0", config))
        
        self.assertTrue(result)
        mock_configure.assert_called_once_with(config)
    
    @patch("counter_uas.hardware.kerberos_sdr.KerberosSDR.start_rx")
    @patch("counter_uas.hardware.acconeer_radar.AcconeerRadar.start_measurement")
    def test_start_all_devices(self, mock_start_measurement, mock_start_rx):
        """Test start_all_devices method."""
        # Configure mocks
        mock_start_rx.return_value = asyncio.Future()
        mock_start_rx.return_value.set_result(True)
        mock_start_measurement.return_value = asyncio.Future()
        mock_start_measurement.return_value.set_result(True)
        
        # Initialize first
        loop = asyncio.get_event_loop()
        loop.run_until_complete(self.hardware_manager.initialize())
        
        # Then start_all_devices
        result = loop.run_until_complete(self.hardware_manager.start_all_devices())
        
        self.assertTrue(result)
        mock_start_rx.assert_called_once()
        mock_start_measurement.assert_called_once()
    
    @patch("counter_uas.hardware.kerberos_sdr.KerberosSDR.stop_rx")
    @patch("counter_uas.hardware.acconeer_radar.AcconeerRadar.stop_measurement")
    def test_stop_all_devices(self, mock_stop_measurement, mock_stop_rx):
        """Test stop_all_devices method."""
        # Configure mocks
        mock_stop_rx.return_value = asyncio.Future()
        mock_stop_rx.return_value.set_result(True)
        mock_stop_measurement.return_value = asyncio.Future()
        mock_stop_measurement.return_value.set_result(True)
        
        # Initialize first
        loop = asyncio.get_event_loop()
        loop.run_until_complete(self.hardware_manager.initialize())
        
        # Then stop_all_devices
        result = loop.run_until_complete(self.hardware_manager.stop_all_devices())
        
        self.assertTrue(result)
        mock_stop_rx.assert_called_once()
        mock_stop_measurement.assert_called_once()


if __name__ == "__main__":
    unittest.main()
