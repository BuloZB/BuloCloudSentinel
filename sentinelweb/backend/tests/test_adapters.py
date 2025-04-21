"""
Tests for SentinelWeb adapters.
"""

import asyncio
import unittest
from unittest.mock import patch, MagicMock

from sentinel_web.drone_adapter.main import DroneAdapter
from sentinel_web.mission_adapter.main import MissionAdapter
from sentinel_web.telemetry_adapter.main import TelemetryAdapter
from sentinel_web.video_adapter.main import VideoAdapter


class TestDroneAdapter(unittest.TestCase):
    """Tests for DroneAdapter."""
    
    def setUp(self):
        self.adapter = DroneAdapter("http://test-api:8000", "test-token")
    
    @patch("httpx.AsyncClient.get")
    def test_get_drones(self, mock_get):
        """Test get_drones method."""
        # Setup mock
        mock_response = MagicMock()
        mock_response.json.return_value = [{"id": "drone1"}, {"id": "drone2"}]
        mock_response.raise_for_status = MagicMock()
        mock_get.return_value.__aenter__.return_value = mock_response
        
        # Run test
        result = asyncio.run(self.adapter.get_drones())
        
        # Verify
        self.assertEqual(len(result), 2)
        self.assertEqual(result[0]["id"], "drone1")
        self.assertEqual(result[1]["id"], "drone2")
        mock_get.assert_called_once_with(
            "http://test-api:8000/api/drones",
            headers={"Authorization": "Bearer test-token", "Content-Type": "application/json"},
            timeout=10.0
        )


class TestMissionAdapter(unittest.TestCase):
    """Tests for MissionAdapter."""
    
    def setUp(self):
        self.adapter = MissionAdapter("http://test-api:8000", "test-token")
    
    @patch("httpx.AsyncClient.get")
    def test_get_missions(self, mock_get):
        """Test get_missions method."""
        # Setup mock
        mock_response = MagicMock()
        mock_response.json.return_value = [{"id": "mission1"}, {"id": "mission2"}]
        mock_response.raise_for_status = MagicMock()
        mock_get.return_value.__aenter__.return_value = mock_response
        
        # Run test
        result = asyncio.run(self.adapter.get_missions())
        
        # Verify
        self.assertEqual(len(result), 2)
        self.assertEqual(result[0]["id"], "mission1")
        self.assertEqual(result[1]["id"], "mission2")
        mock_get.assert_called_once_with(
            "http://test-api:8000/api/missions",
            headers={"Authorization": "Bearer test-token", "Content-Type": "application/json"},
            timeout=10.0
        )


class TestTelemetryAdapter(unittest.TestCase):
    """Tests for TelemetryAdapter."""
    
    def setUp(self):
        self.adapter = TelemetryAdapter("http://test-api:8000", "test-token")
    
    @patch("httpx.AsyncClient.get")
    def test_get_telemetry(self, mock_get):
        """Test get_telemetry method."""
        # Setup mock
        mock_response = MagicMock()
        mock_response.json.return_value = {"battery": 80, "altitude": 100}
        mock_response.raise_for_status = MagicMock()
        mock_get.return_value.__aenter__.return_value = mock_response
        
        # Run test
        result = asyncio.run(self.adapter.get_telemetry("drone1"))
        
        # Verify
        self.assertEqual(result["battery"], 80)
        self.assertEqual(result["altitude"], 100)
        mock_get.assert_called_once_with(
            "http://test-api:8000/api/drones/drone1/telemetry",
            headers={"Authorization": "Bearer test-token", "Content-Type": "application/json"},
            timeout=10.0
        )


class TestVideoAdapter(unittest.TestCase):
    """Tests for VideoAdapter."""
    
    def setUp(self):
        self.adapter = VideoAdapter("http://test-api:8000", "rtmp://test-server:1935", "test-token")
    
    @patch("httpx.AsyncClient.get")
    def test_get_video_streams(self, mock_get):
        """Test get_video_streams method."""
        # Setup mock
        mock_response = MagicMock()
        mock_response.json.return_value = [{"id": "stream1"}, {"id": "stream2"}]
        mock_response.raise_for_status = MagicMock()
        mock_get.return_value.__aenter__.return_value = mock_response
        
        # Run test
        result = asyncio.run(self.adapter.get_video_streams())
        
        # Verify
        self.assertEqual(len(result), 2)
        self.assertEqual(result[0]["id"], "stream1")
        self.assertEqual(result[1]["id"], "stream2")
        mock_get.assert_called_once_with(
            "http://test-api:8000/api/video/streams",
            headers={"Authorization": "Bearer test-token", "Content-Type": "application/json"},
            timeout=10.0
        )
    
    def test_get_rtmp_url(self):
        """Test get_rtmp_url method."""
        result = self.adapter.get_rtmp_url("test-stream")
        self.assertEqual(result, "rtmp://test-server:1935/test-stream")


if __name__ == "__main__":
    unittest.main()
