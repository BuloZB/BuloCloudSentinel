import pytest
import os
import sys
import numpy as np
from unittest.mock import MagicMock

# Add parent directory to path to import modules
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

@pytest.fixture
def sample_image():
    """Fixture providing a sample image for testing"""
    # Create a simple numpy array representing an image
    return np.zeros((480, 640, 3), dtype=np.uint8)

@pytest.fixture
def mock_video_stream():
    """Fixture providing a mock video stream for testing"""
    mock_stream = MagicMock()
    
    # Configure the mock to return sample frames when read
    mock_stream.read.return_value = (True, np.zeros((480, 640, 3), dtype=np.uint8))
    mock_stream.isOpened.return_value = True
    mock_stream.get.return_value = 30.0  # fps
    
    return mock_stream

@pytest.fixture
def sample_detection_results():
    """Fixture providing sample object detection results"""
    return [
        {'class': 'person', 'confidence': 0.95, 'bbox': [10, 20, 100, 200]},
        {'class': 'car', 'confidence': 0.87, 'bbox': [150, 160, 300, 400]},
        {'class': 'truck', 'confidence': 0.76, 'bbox': [350, 200, 500, 350]},
        {'class': 'bicycle', 'confidence': 0.82, 'bbox': [50, 300, 120, 380]}
    ]

@pytest.fixture
def sample_crowd_analysis():
    """Fixture providing sample crowd analysis results"""
    return {
        'density': 0.35,
        'count': 42,
        'hotspots': [
            {'x': 100, 'y': 150, 'intensity': 0.8},
            {'x': 300, 'y': 200, 'intensity': 0.6}
        ],
        'flow_vectors': [
            {'x1': 100, 'y1': 150, 'x2': 120, 'y2': 160, 'magnitude': 0.3},
            {'x1': 300, 'y1': 200, 'x2': 290, 'y2': 220, 'magnitude': 0.4}
        ]
    }

@pytest.fixture
def sample_vehicle_analysis():
    """Fixture providing sample vehicle analysis results"""
    return {
        'count': 15,
        'types': {
            'car': 10,
            'truck': 3,
            'bus': 2
        },
        'average_speed': 35.5,
        'direction': 'northbound',
        'congestion_level': 'low',
        'flow_rate': 300  # vehicles per hour
    }

@pytest.fixture
def mock_api_client():
    """Fixture providing a mock API client for testing"""
    mock_client = MagicMock()
    
    # Configure common API responses
    mock_client.get_status.return_value = {'status': 'ok', 'version': '1.0.0'}
    mock_client.get_analytics.return_value = {'data': 'sample_data'}
    
    return mock_client