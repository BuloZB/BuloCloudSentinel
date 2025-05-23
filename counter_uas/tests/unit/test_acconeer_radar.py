"""
Unit tests for the Acconeer radar implementation.
"""

import asyncio
import unittest
import numpy as np
from unittest.mock import patch, MagicMock

from counter_uas.hardware.acconeer_radar import AcconeerRadar


class TestAcconeerRadar(unittest.TestCase):
    """Test cases for the Acconeer radar implementation."""
    
    def setUp(self):
        """Set up the test case."""
        self.acconeer_radar = AcconeerRadar(device_id="0")
    
    def tearDown(self):
        """Tear down the test case."""
        # Run the shutdown method in the event loop
        loop = asyncio.get_event_loop()
        loop.run_until_complete(self.acconeer_radar.shutdown())
    
    def test_initialization(self):
        """Test initialization of AcconeerRadar."""
        self.assertEqual(self.acconeer_radar.device_id, "0")
        self.assertEqual(self.acconeer_radar.mode, "iq")
        self.assertEqual(self.acconeer_radar.start_range, 0.2)
        self.assertEqual(self.acconeer_radar.end_range, 5.0)
        self.assertEqual(self.acconeer_radar.update_rate, 10.0)
        self.assertFalse(self.acconeer_radar.initialized)
        self.assertFalse(self.acconeer_radar.running)
    
    def test_initialize(self):
        """Test initialize method."""
        # Run the initialize method in the event loop
        loop = asyncio.get_event_loop()
        result = loop.run_until_complete(self.acconeer_radar.initialize())
        
        self.assertTrue(result)
        self.assertTrue(self.acconeer_radar.initialized)
    
    def test_shutdown(self):
        """Test shutdown method."""
        # Initialize first
        loop = asyncio.get_event_loop()
        loop.run_until_complete(self.acconeer_radar.initialize())
        
        # Then shutdown
        result = loop.run_until_complete(self.acconeer_radar.shutdown())
        
        self.assertTrue(result)
        self.assertFalse(self.acconeer_radar.initialized)
    
    def test_get_status(self):
        """Test get_status method."""
        # Run the get_status method in the event loop
        loop = asyncio.get_event_loop()
        status = loop.run_until_complete(self.acconeer_radar.get_status())
        
        self.assertIsInstance(status, dict)
        self.assertEqual(status["device_id"], "0")
        self.assertEqual(status["mode"], "iq")
        self.assertEqual(status["start_range"], 0.2)
        self.assertEqual(status["end_range"], 5.0)
        self.assertEqual(status["update_rate"], 10.0)
        self.assertFalse(status["initialized"])
        self.assertFalse(status["running"])
    
    def test_configure(self):
        """Test configure method."""
        # Define test configuration
        config = {
            "mode": "envelope",
            "start_range": 0.5,
            "end_range": 3.0,
            "update_rate": 5.0
        }
        
        # Run the configure method in the event loop
        loop = asyncio.get_event_loop()
        result = loop.run_until_complete(self.acconeer_radar.configure(config))
        
        self.assertTrue(result)
        self.assertEqual(self.acconeer_radar.mode, "envelope")
        self.assertEqual(self.acconeer_radar.start_range, 0.5)
        self.assertEqual(self.acconeer_radar.end_range, 3.0)
        self.assertEqual(self.acconeer_radar.update_rate, 5.0)
    
    def test_set_mode(self):
        """Test set_mode method."""
        # Run the set_mode method in the event loop
        loop = asyncio.get_event_loop()
        result = loop.run_until_complete(self.acconeer_radar.set_mode("envelope"))
        
        self.assertTrue(result)
        self.assertEqual(self.acconeer_radar.mode, "envelope")
    
    def test_set_range(self):
        """Test set_range method."""
        # Run the set_range method in the event loop
        loop = asyncio.get_event_loop()
        result = loop.run_until_complete(self.acconeer_radar.set_range(0.5, 3.0))
        
        self.assertTrue(result)
        self.assertEqual(self.acconeer_radar.start_range, 0.5)
        self.assertEqual(self.acconeer_radar.end_range, 3.0)
    
    def test_set_update_rate(self):
        """Test set_update_rate method."""
        # Run the set_update_rate method in the event loop
        loop = asyncio.get_event_loop()
        result = loop.run_until_complete(self.acconeer_radar.set_update_rate(5.0))
        
        self.assertTrue(result)
        self.assertEqual(self.acconeer_radar.update_rate, 5.0)
    
    def test_start_measurement(self):
        """Test start_measurement method."""
        # Initialize first
        loop = asyncio.get_event_loop()
        loop.run_until_complete(self.acconeer_radar.initialize())
        
        # Then start_measurement
        result = loop.run_until_complete(self.acconeer_radar.start_measurement())
        
        self.assertTrue(result)
        self.assertTrue(self.acconeer_radar.running)
    
    def test_stop_measurement(self):
        """Test stop_measurement method."""
        # Initialize and start_measurement first
        loop = asyncio.get_event_loop()
        loop.run_until_complete(self.acconeer_radar.initialize())
        loop.run_until_complete(self.acconeer_radar.start_measurement())
        
        # Then stop_measurement
        result = loop.run_until_complete(self.acconeer_radar.stop_measurement())
        
        self.assertTrue(result)
        self.assertFalse(self.acconeer_radar.running)
    
    def test_get_next_frame(self):
        """Test get_next_frame method."""
        # Initialize and start_measurement first
        loop = asyncio.get_event_loop()
        loop.run_until_complete(self.acconeer_radar.initialize())
        loop.run_until_complete(self.acconeer_radar.start_measurement())
        
        # Wait for a frame to be generated
        loop.run_until_complete(asyncio.sleep(0.2))
        
        # Then get_next_frame
        frame = loop.run_until_complete(self.acconeer_radar.get_next_frame())
        
        self.assertIsInstance(frame, dict)
        self.assertEqual(frame["mode"], "iq")
        self.assertEqual(frame["start_range"], 0.2)
        self.assertEqual(frame["end_range"], 5.0)
        self.assertIsInstance(frame["data"], list)
        self.assertIsInstance(frame["distances"], list)
        self.assertIsInstance(frame["timestamp"], float)
        self.assertIsInstance(frame["num_points"], int)
    
    def test_invalid_mode(self):
        """Test setting an invalid mode."""
        # Run the set_mode method with an invalid mode
        loop = asyncio.get_event_loop()
        result = loop.run_until_complete(self.acconeer_radar.set_mode("invalid_mode"))
        
        self.assertFalse(result)
        self.assertEqual(self.acconeer_radar.mode, "iq")  # Mode should not change
    
    def test_invalid_range(self):
        """Test setting an invalid range."""
        # Run the set_range method with an invalid range
        loop = asyncio.get_event_loop()
        
        # Start > end
        result1 = loop.run_until_complete(self.acconeer_radar.set_range(5.0, 0.2))
        self.assertFalse(result1)
        
        # Negative range
        result2 = loop.run_until_complete(self.acconeer_radar.set_range(-1.0, 5.0))
        self.assertFalse(result2)
        
        # Range should not change
        self.assertEqual(self.acconeer_radar.start_range, 0.2)
        self.assertEqual(self.acconeer_radar.end_range, 5.0)
    
    def test_invalid_update_rate(self):
        """Test setting an invalid update rate."""
        # Run the set_update_rate method with an invalid update rate
        loop = asyncio.get_event_loop()
        result = loop.run_until_complete(self.acconeer_radar.set_update_rate(-1.0))
        
        self.assertFalse(result)
        self.assertEqual(self.acconeer_radar.update_rate, 10.0)  # Update rate should not change


if __name__ == "__main__":
    unittest.main()
