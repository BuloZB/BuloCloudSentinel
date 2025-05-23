"""
API schemas for the Mapping Service.
"""

import uuid
from datetime import datetime
from typing import Any, Dict, List, Optional, Union

from pydantic import BaseModel, Field


class ProjectBase(BaseModel):
    """Base project schema."""
    
    name: str = Field(..., description="Project name")
    description: Optional[str] = Field(None, description="Project description")


class ProjectCreate(ProjectBase):
    """Project creation schema."""
    pass


class ProjectUpdate(BaseModel):
    """Project update schema."""
    
    name: Optional[str] = Field(None, description="Project name")
    description: Optional[str] = Field(None, description="Project description")


class ProjectResponse(ProjectBase):
    """Project response schema."""
    
    id: uuid.UUID = Field(..., description="Project ID")
    created_at: datetime = Field(..., description="Creation timestamp")
    updated_at: datetime = Field(..., description="Last update timestamp")
    user_id: uuid.UUID = Field(..., description="User ID")
    status: str = Field(..., description="Project status")
    image_count: int = Field(..., description="Number of images")
    processing_time: Optional[int] = Field(None, description="Processing time in seconds")
    area_coverage: Optional[float] = Field(None, description="Area coverage in square meters")
    resolution: Optional[float] = Field(None, description="Resolution in cm/pixel")
    
    class Config:
        orm_mode = True


class SourceImageBase(BaseModel):
    """Base source image schema."""
    
    filename: str = Field(..., description="Image filename")
    captured_at: Optional[datetime] = Field(None, description="Capture timestamp")
    altitude: Optional[float] = Field(None, description="Altitude in meters")
    heading: Optional[float] = Field(None, description="Heading in degrees")


class SourceImageResponse(SourceImageBase):
    """Source image response schema."""
    
    id: uuid.UUID = Field(..., description="Image ID")
    project_id: uuid.UUID = Field(..., description="Project ID")
    storage_path: str = Field(..., description="Storage path")
    location: Optional[Dict[str, Any]] = Field(None, description="Location as GeoJSON")
    metadata: Optional[Dict[str, Any]] = Field(None, description="Image metadata")
    
    class Config:
        orm_mode = True


class AssetBase(BaseModel):
    """Base asset schema."""
    
    type: str = Field(..., description="Asset type")
    version: int = Field(..., description="Asset version")
    is_current: bool = Field(..., description="Whether this is the current version")


class AssetResponse(AssetBase):
    """Asset response schema."""
    
    id: uuid.UUID = Field(..., description="Asset ID")
    project_id: uuid.UUID = Field(..., description="Project ID")
    created_at: datetime = Field(..., description="Creation timestamp")
    storage_path: str = Field(..., description="Storage path")
    file_size: Optional[int] = Field(None, description="File size in bytes")
    metadata: Optional[Dict[str, Any]] = Field(None, description="Asset metadata")
    url: Optional[str] = Field(None, description="Public URL")
    
    class Config:
        orm_mode = True


class ProcessingJobBase(BaseModel):
    """Base processing job schema."""
    
    parameters: Dict[str, Any] = Field(..., description="Processing parameters")


class ProcessingJobCreate(ProcessingJobBase):
    """Processing job creation schema."""
    pass


class ProcessingJobResponse(ProcessingJobBase):
    """Processing job response schema."""
    
    id: uuid.UUID = Field(..., description="Job ID")
    project_id: uuid.UUID = Field(..., description="Project ID")
    status: str = Field(..., description="Job status")
    created_at: datetime = Field(..., description="Creation timestamp")
    started_at: Optional[datetime] = Field(None, description="Start timestamp")
    completed_at: Optional[datetime] = Field(None, description="Completion timestamp")
    error_message: Optional[str] = Field(None, description="Error message")
    worker_id: Optional[str] = Field(None, description="Worker ID")
    
    class Config:
        orm_mode = True


class ProjectDetailResponse(ProjectResponse):
    """Project detail response schema."""
    
    source_images: List[SourceImageResponse] = Field([], description="Source images")
    assets: List[AssetResponse] = Field([], description="Assets")
    processing_jobs: List[ProcessingJobResponse] = Field([], description="Processing jobs")
    
    class Config:
        orm_mode = True


class ErrorResponse(BaseModel):
    """Error response schema."""
    
    detail: str = Field(..., description="Error message")


class SuccessResponse(BaseModel):
    """Success response schema."""
    
    message: str = Field(..., description="Success message")


class ProcessingOptions(BaseModel):
    """Processing options schema."""
    
    # OpenDroneMap options
    dsm: bool = Field(True, description="Generate DSM")
    dtm: bool = Field(True, description="Generate DTM")
    orthophoto_resolution: float = Field(5.0, description="Orthophoto resolution in cm/pixel")
    mesh_size: int = Field(200000, description="Mesh size (number of faces)")
    use_3dmesh: bool = Field(True, description="Generate 3D mesh")
    pc_quality: str = Field("medium", description="Point cloud quality")
    crop: float = Field(0.0, description="Crop amount")
    feature_quality: str = Field("high", description="Feature extraction quality")
    min_num_features: int = Field(8000, description="Minimum number of features")
    matcher_neighbors: int = Field(8, description="Matcher neighbors")
    texturing_data_term: str = Field("gmi", description="Texturing data term")
    texturing_outlier_removal_type: str = Field("gauss_damping", description="Texturing outlier removal type")
    texturing_skip_local_seam_leveling: bool = Field(False, description="Skip local seam leveling")
    texturing_skip_global_seam_leveling: bool = Field(False, description="Skip global seam leveling")
    texturing_tone_mapping: str = Field("none", description="Texturing tone mapping")
    dem_resolution: float = Field(5.0, description="DEM resolution in cm/pixel")
    orthophoto: bool = Field(True, description="Generate orthophoto")
    verbose: bool = Field(True, description="Verbose output")


class StartProcessingRequest(BaseModel):
    """Start processing request schema."""
    
    options: ProcessingOptions = Field(..., description="Processing options")
