"""
Multimodal detection models for Bulo.Cloud Sentinel.

This package provides models for multimodal detection, combining visual,
thermal, and other sensor data for improved object detection.
"""

from .fusion import (
    FeatureFusion,
    DecisionFusion,
    HybridFusion,
)

__all__ = [
    "FeatureFusion",
    "DecisionFusion",
    "HybridFusion",
]
