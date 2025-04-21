"""
Visualization schemas for the Vision System.
"""

from pydantic import BaseModel, Field
from typing import List, Dict, Any, Optional
from datetime import datetime
from enum import Enum

class ColorMap(str, Enum):
    """Color map enumeration for heat maps."""
    JET = "jet"
    VIRIDIS = "viridis"
    PLASMA = "plasma"
    INFERNO = "inferno"
    MAGMA = "magma"
    CIVIDIS = "cividis"
    HOT = "hot"
    COOL = "cool"
    RAINBOW = "rainbow"
    CUSTOM = "custom"

class HeatMapConfig(BaseModel):
    """Heat map configuration model."""
    color_map: ColorMap = ColorMap.JET
    opacity: float = 0.7
    resolution: Dict[str, int] = {"width": 640, "height": 480}
    interpolation: str = "bicubic"
    normalize: bool = True
    show_legend: bool = True
    custom_colors: Optional[List[str]] = None  # For custom color map

class FlowMapConfig(BaseModel):
    """Flow map configuration model."""
    line_color: str = "rainbow"  # color or "rainbow" for direction-based coloring
    line_width: int = 2
    line_opacity: float = 0.8
    min_track_length: int = 10  # frames
    max_tracks: int = 100
    show_direction: bool = True
    show_speed: bool = True
    arrow_scale: float = 1.0

class OverlayConfig(BaseModel):
    """Overlay configuration model."""
    show_bounding_boxes: bool = True
    show_labels: bool = True
    show_confidence: bool = True
    show_tracks: bool = False
    show_density: bool = False
    show_counts: bool = True
    box_thickness: int = 2
    text_size: float = 0.5
    text_thickness: int = 1
    person_color: str = "#00FF00"  # Green
    vehicle_color: str = "#FF0000"  # Red
    track_color: str = "#0000FF"  # Blue
    count_position: Dict[str, float] = {"x": 0.05, "y": 0.05}

class VisualizationConfig(BaseModel):
    """Visualization configuration model."""
    heat_map: HeatMapConfig = Field(default_factory=HeatMapConfig)
    flow_map: FlowMapConfig = Field(default_factory=FlowMapConfig)
    overlay: OverlayConfig = Field(default_factory=OverlayConfig)
    default_resolution: Dict[str, int] = {"width": 1280, "height": 720}
    default_format: str = "png"
    max_resolution: Dict[str, int] = {"width": 3840, "height": 2160}

class VisualizationResult(BaseModel):
    """Visualization result model."""
    id: str
    stream_id: str
    visualization_type: str  # heat_map, flow_map, overlay
    analysis_type: str  # crowd, vehicle, flow
    timestamp: datetime
    image_path: str
    parameters: Optional[Dict[str, Any]] = None
    created_at: datetime = Field(default_factory=datetime.now)
