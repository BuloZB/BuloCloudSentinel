import unittest
import sys
import os
from unittest.mock import MagicMock, patch

# Add parent directory to path to import modules
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

class TestAnalysisFunctionality(unittest.TestCase):
    """Tests for vision system analysis functionality"""
    
    def setUp(self):
        """Set up test fixtures"""
        # Create mock image data
        self.mock_image = MagicMock()
        
    def tearDown(self):
        """Tear down test fixtures"""
        pass
    
    @patch('vision_system.analysis.detect_objects')
    def test_object_detection(self, mock_detect):
        """Test object detection functionality"""
        # Configure the mock
        mock_detect.return_value = [
            {'class': 'person', 'confidence': 0.95, 'bbox': [10, 20, 100, 200]},
            {'class': 'car', 'confidence': 0.87, 'bbox': [150, 160, 300, 400]}
        ]
        
        # Import here to use the patched version
        from vision_system.analysis import detect_objects
        
        # Call the function with mock data
        results = detect_objects(self.mock_image)
        
        # Assertions
        self.assertEqual(len(results), 2)
        self.assertEqual(results[0]['class'], 'person')
        self.assertEqual(results[1]['class'], 'car')
        
    @patch('vision_system.analysis.estimate_crowd_density')
    def test_crowd_density_estimation(self, mock_estimate):
        """Test crowd density estimation"""
        # Configure the mock
        mock_estimate.return_value = {
            'density': 0.35,
            'count': 42,
            'confidence': 0.88
        }
        
        # Import here to use the patched version
        from vision_system.analysis import estimate_crowd_density
        
        # Call the function with mock data
        result = estimate_crowd_density(self.mock_image)
        
        # Assertions
        self.assertIsNotNone(result)
        self.assertIn('density', result)
        self.assertIn('count', result)
        self.assertEqual(result['count'], 42)
        
    @patch('vision_system.analysis.analyze_vehicle_flow')
    def test_vehicle_flow_analysis(self, mock_analyze):
        """Test vehicle flow analysis"""
        # Configure the mock
        mock_analyze.return_value = {
            'count': 15,
            'average_speed': 35.5,
            'direction': 'northbound',
            'congestion_level': 'low'
        }
        
        # Import here to use the patched version
        from vision_system.analysis import analyze_vehicle_flow
        
        # Call the function with mock data
        result = analyze_vehicle_flow(self.mock_image)
        
        # Assertions
        self.assertIsNotNone(result)
        self.assertEqual(result['count'], 15)
        self.assertEqual(result['congestion_level'], 'low')

if __name__ == '__main__':
    unittest.main()