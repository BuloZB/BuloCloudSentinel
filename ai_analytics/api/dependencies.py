"""
API dependencies for the AI Analytics module.

This module provides FastAPI dependencies for the AI Analytics module.
"""

import logging
from typing import Dict, Any
from fastapi import Depends

from utils.config import Config, get_config
from services.video_stream_manager import VideoStreamManager
from services.event_publisher import EventPublisher
from services.detection import DetectionService
from services.recognition import RecognitionService
from services.behavior import BehaviorService
from services.analytics import AnalyticsService
from services.multimodal_detection import MultimodalDetectionService
from services.inference_service import InferenceService
from services.enhanced_detection import EnhancedDetectionService

# Setup logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Service instances
_video_stream_manager = None
_event_publisher = None
_detection_service = None
_recognition_service = None
_behavior_service = None
_analytics_service = None
_multimodal_detection_service = None
_inference_service = None
_enhanced_detection_service = None


def get_video_stream_manager(config: Config = Depends(get_config)) -> VideoStreamManager:
    """
    Get the video stream manager.

    Args:
        config: Configuration object

    Returns:
        Video stream manager
    """
    global _video_stream_manager

    if _video_stream_manager is None:
        _video_stream_manager = VideoStreamManager(config)

    return _video_stream_manager


def get_event_publisher(config: Config = Depends(get_config)) -> EventPublisher:
    """
    Get the event publisher.

    Args:
        config: Configuration object

    Returns:
        Event publisher
    """
    global _event_publisher

    if _event_publisher is None:
        _event_publisher = EventPublisher(config)

    return _event_publisher


def get_detection_service(
    config: Config = Depends(get_config),
    video_stream_manager: VideoStreamManager = Depends(get_video_stream_manager),
    event_publisher: EventPublisher = Depends(get_event_publisher)
) -> DetectionService:
    """
    Get the detection service.

    Args:
        config: Configuration object
        video_stream_manager: Video stream manager
        event_publisher: Event publisher

    Returns:
        Detection service
    """
    global _detection_service

    if _detection_service is None:
        _detection_service = DetectionService(config, video_stream_manager, event_publisher)

    return _detection_service


def get_recognition_service(
    config: Config = Depends(get_config),
    video_stream_manager: VideoStreamManager = Depends(get_video_stream_manager),
    event_publisher: EventPublisher = Depends(get_event_publisher)
) -> RecognitionService:
    """
    Get the recognition service.

    Args:
        config: Configuration object
        video_stream_manager: Video stream manager
        event_publisher: Event publisher

    Returns:
        Recognition service
    """
    global _recognition_service

    if _recognition_service is None:
        _recognition_service = RecognitionService(config, video_stream_manager, event_publisher)

    return _recognition_service


def get_behavior_service(
    config: Config = Depends(get_config),
    video_stream_manager: VideoStreamManager = Depends(get_video_stream_manager),
    event_publisher: EventPublisher = Depends(get_event_publisher)
) -> BehaviorService:
    """
    Get the behavior service.

    Args:
        config: Configuration object
        video_stream_manager: Video stream manager
        event_publisher: Event publisher

    Returns:
        Behavior service
    """
    global _behavior_service

    if _behavior_service is None:
        _behavior_service = BehaviorService(config, video_stream_manager, event_publisher)

    return _behavior_service


def get_analytics_service(
    config: Config = Depends(get_config),
    event_publisher: EventPublisher = Depends(get_event_publisher)
) -> AnalyticsService:
    """
    Get the analytics service.

    Args:
        config: Configuration object
        event_publisher: Event publisher

    Returns:
        Analytics service
    """
    global _analytics_service

    if _analytics_service is None:
        _analytics_service = AnalyticsService(config, event_publisher)

    return _analytics_service


def get_multimodal_detection_service(
    config: Config = Depends(get_config),
    video_stream_manager: VideoStreamManager = Depends(get_video_stream_manager),
    event_publisher: EventPublisher = Depends(get_event_publisher)
) -> MultimodalDetectionService:
    """
    Get the multimodal detection service.

    Args:
        config: Configuration object
        video_stream_manager: Video stream manager
        event_publisher: Event publisher

    Returns:
        Multimodal detection service
    """
    global _multimodal_detection_service

    if _multimodal_detection_service is None:
        _multimodal_detection_service = MultimodalDetectionService(
            config, video_stream_manager, event_publisher
        )

    return _multimodal_detection_service


def get_inference_service(
    config: Config = Depends(get_config)
) -> InferenceService:
    """
    Get the inference service.

    Args:
        config: Configuration object

    Returns:
        Inference service
    """
    global _inference_service

    if _inference_service is None:
        _inference_service = InferenceService(config.get("inference", {}))

    return _inference_service


def get_enhanced_detection_service(
    config: Config = Depends(get_config),
    video_stream_manager: VideoStreamManager = Depends(get_video_stream_manager),
    event_publisher: EventPublisher = Depends(get_event_publisher)
) -> EnhancedDetectionService:
    """
    Get the enhanced detection service.

    Args:
        config: Configuration object
        video_stream_manager: Video stream manager
        event_publisher: Event publisher

    Returns:
        Enhanced detection service
    """
    global _enhanced_detection_service

    if _enhanced_detection_service is None:
        _enhanced_detection_service = EnhancedDetectionService(
            config, video_stream_manager, event_publisher
        )

    return _enhanced_detection_service
