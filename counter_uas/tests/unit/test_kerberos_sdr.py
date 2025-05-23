"""
Unit tests for the KerberosSDR implementation.
"""

import asyncio
import unittest
import numpy as np
from unittest.mock import patch, MagicMock

from counter_uas.hardware.kerberos_sdr import KerberosSDR


class TestKerberosSDR(unittest.TestCase):
    """Test cases for the KerberosSDR implementation."""
    
    def setUp(self):
        """Set up the test case."""
        self.kerberos_sdr = KerberosSDR(device_index=0)
    
    def tearDown(self):
        """Tear down the test case."""
        # Run the shutdown method in the event loop
        loop = asyncio.get_event_loop()
        loop.run_until_complete(self.kerberos_sdr.shutdown())
    
    def test_initialization(self):
        """Test initialization of KerberosSDR."""
        self.assertEqual(self.kerberos_sdr.device_index, 0)
        self.assertEqual(self.kerberos_sdr.center_frequency, 915000000)
        self.assertEqual(self.kerberos_sdr.sample_rate, 2400000)
        self.assertEqual(self.kerberos_sdr.gain, 30.0)
        self.assertEqual(self.kerberos_sdr.reference_channel, 0)
        self.assertTrue(self.kerberos_sdr.coherent_mode)
        self.assertFalse(self.kerberos_sdr.initialized)
        self.assertFalse(self.kerberos_sdr.running)
    
    def test_initialize(self):
        """Test initialize method."""
        # Run the initialize method in the event loop
        loop = asyncio.get_event_loop()
        result = loop.run_until_complete(self.kerberos_sdr.initialize())
        
        self.assertTrue(result)
        self.assertTrue(self.kerberos_sdr.initialized)
    
    def test_shutdown(self):
        """Test shutdown method."""
        # Initialize first
        loop = asyncio.get_event_loop()
        loop.run_until_complete(self.kerberos_sdr.initialize())
        
        # Then shutdown
        result = loop.run_until_complete(self.kerberos_sdr.shutdown())
        
        self.assertTrue(result)
        self.assertFalse(self.kerberos_sdr.initialized)
    
    def test_get_status(self):
        """Test get_status method."""
        # Run the get_status method in the event loop
        loop = asyncio.get_event_loop()
        status = loop.run_until_complete(self.kerberos_sdr.get_status())
        
        self.assertIsInstance(status, dict)
        self.assertEqual(status["device_index"], 0)
        self.assertEqual(status["center_frequency"], 915000000)
        self.assertEqual(status["sample_rate"], 2400000)
        self.assertEqual(status["gain"], 30.0)
        self.assertEqual(status["reference_channel"], 0)
        self.assertTrue(status["coherent_mode"])
        self.assertFalse(status["initialized"])
        self.assertFalse(status["running"])
    
    def test_configure(self):
        """Test configure method."""
        # Define test configuration
        config = {
            "center_frequency": 2400000000,  # 2.4 GHz
            "sample_rate": 1000000,  # 1 MSPS
            "gain": 20.0,  # 20 dB
            "reference_channel": 1
        }
        
        # Run the configure method in the event loop
        loop = asyncio.get_event_loop()
        result = loop.run_until_complete(self.kerberos_sdr.configure(config))
        
        self.assertTrue(result)
        self.assertEqual(self.kerberos_sdr.center_frequency, 2400000000)
        self.assertEqual(self.kerberos_sdr.sample_rate, 1000000)
        self.assertEqual(self.kerberos_sdr.gain, 20.0)
        self.assertEqual(self.kerberos_sdr.reference_channel, 1)
    
    def test_set_frequency(self):
        """Test set_frequency method."""
        # Run the set_frequency method in the event loop
        loop = asyncio.get_event_loop()
        result = loop.run_until_complete(self.kerberos_sdr.set_frequency(2400000000))
        
        self.assertTrue(result)
        self.assertEqual(self.kerberos_sdr.center_frequency, 2400000000)
    
    def test_set_sample_rate(self):
        """Test set_sample_rate method."""
        # Run the set_sample_rate method in the event loop
        loop = asyncio.get_event_loop()
        result = loop.run_until_complete(self.kerberos_sdr.set_sample_rate(1000000))
        
        self.assertTrue(result)
        self.assertEqual(self.kerberos_sdr.sample_rate, 1000000)
    
    def test_set_gain(self):
        """Test set_gain method."""
        # Run the set_gain method in the event loop
        loop = asyncio.get_event_loop()
        result = loop.run_until_complete(self.kerberos_sdr.set_gain(20.0))
        
        self.assertTrue(result)
        self.assertEqual(self.kerberos_sdr.gain, 20.0)
    
    def test_start_rx(self):
        """Test start_rx method."""
        # Initialize first
        loop = asyncio.get_event_loop()
        loop.run_until_complete(self.kerberos_sdr.initialize())
        
        # Then start_rx
        result = loop.run_until_complete(self.kerberos_sdr.start_rx())
        
        self.assertTrue(result)
        self.assertTrue(self.kerberos_sdr.running)
    
    def test_stop_rx(self):
        """Test stop_rx method."""
        # Initialize and start_rx first
        loop = asyncio.get_event_loop()
        loop.run_until_complete(self.kerberos_sdr.initialize())
        loop.run_until_complete(self.kerberos_sdr.start_rx())
        
        # Then stop_rx
        result = loop.run_until_complete(self.kerberos_sdr.stop_rx())
        
        self.assertTrue(result)
        self.assertFalse(self.kerberos_sdr.running)
    
    def test_get_samples(self):
        """Test get_samples method."""
        # Initialize and start_rx first
        loop = asyncio.get_event_loop()
        loop.run_until_complete(self.kerberos_sdr.initialize())
        loop.run_until_complete(self.kerberos_sdr.start_rx())
        
        # Then get_samples
        samples = loop.run_until_complete(self.kerberos_sdr.get_samples(1000))
        
        self.assertIsInstance(samples, np.ndarray)
        self.assertEqual(samples.shape, (1000,))
        self.assertTrue(np.iscomplex(samples[0]))
    
    def test_get_doa_estimate(self):
        """Test get_doa_estimate method."""
        # Initialize and start_rx first
        loop = asyncio.get_event_loop()
        loop.run_until_complete(self.kerberos_sdr.initialize())
        loop.run_until_complete(self.kerberos_sdr.start_rx())
        
        # Then get_doa_estimate
        azimuth, elevation, confidence = loop.run_until_complete(self.kerberos_sdr.get_doa_estimate())
        
        self.assertIsInstance(azimuth, float)
        self.assertIsInstance(elevation, float)
        self.assertIsInstance(confidence, float)
        self.assertGreaterEqual(azimuth, 0.0)
        self.assertLessEqual(azimuth, 360.0)
        self.assertGreaterEqual(elevation, 0.0)
        self.assertLessEqual(elevation, 90.0)
        self.assertGreaterEqual(confidence, 0.0)
        self.assertLessEqual(confidence, 1.0)


if __name__ == "__main__":
    unittest.main()
