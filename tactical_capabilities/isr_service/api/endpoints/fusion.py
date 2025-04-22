"""
API endpoints for data fusion.
"""

from typing import Dict, List, Any
from fastapi import APIRouter, Depends, HTTPException, status, Request
from pydantic import UUID4

from core.security import get_current_user, has_permission
from services.data_fusion_engine import DataFusionEngine

router = APIRouter()

@router.post("/observations", response_model=Dict[str, Any])
async def fuse_observations(
    observations: List[Dict[str, Any]],
    request: Request,
    current_user = Depends(has_permission("fusion:process"))
):
    """
    Fuse multiple observations into a single observation.
    
    Args:
        observations: List of observations to fuse
        request: FastAPI request
        current_user: Current authenticated user with required permission
        
    Returns:
        Fused observation
    """
    # Get data fusion engine from app state
    data_fusion_engine: DataFusionEngine = request.app.state.data_fusion_engine
    
    # Fuse observations
    fused_observation = await data_fusion_engine.fuse_observations(observations)
    
    return fused_observation

@router.post("/detections", response_model=Dict[str, Any])
async def fuse_detections(
    detections: List[Dict[str, Any]],
    request: Request,
    current_user = Depends(has_permission("fusion:process"))
):
    """
    Fuse multiple detections into a single detection.
    
    Args:
        detections: List of detections to fuse
        request: FastAPI request
        current_user: Current authenticated user with required permission
        
    Returns:
        Fused detection
    """
    # Get data fusion engine from app state
    data_fusion_engine: DataFusionEngine = request.app.state.data_fusion_engine
    
    # Fuse detections
    fused_detection = await data_fusion_engine.fuse_detections(detections)
    
    return fused_detection

@router.post("/correlate", response_model=List[List[Dict[str, Any]]])
async def correlate_detections(
    detections: List[Dict[str, Any]],
    request: Request,
    current_user = Depends(has_permission("fusion:process"))
):
    """
    Correlate detections across sensors.
    
    Args:
        detections: List of detections to correlate
        request: FastAPI request
        current_user: Current authenticated user with required permission
        
    Returns:
        List of correlated detection groups
    """
    # Get data fusion engine from app state
    data_fusion_engine: DataFusionEngine = request.app.state.data_fusion_engine
    
    # Correlate detections
    correlated_groups = await data_fusion_engine.correlate_detections(detections)
    
    return correlated_groups

@router.post("/intelligence", response_model=Dict[str, Any])
async def generate_intelligence_product(
    targets: List[Dict[str, Any]],
    request: Request,
    current_user = Depends(has_permission("fusion:process"))
):
    """
    Generate an intelligence product from fused data.
    
    Args:
        targets: List of targets
        request: FastAPI request
        current_user: Current authenticated user with required permission
        
    Returns:
        Intelligence product
    """
    # Get data fusion engine from app state
    data_fusion_engine: DataFusionEngine = request.app.state.data_fusion_engine
    
    # Generate intelligence product
    product = await data_fusion_engine.generate_intelligence_product(targets)
    
    # Publish intelligence product
    await request.app.state.event_publisher.publish_intelligence_product(product)
    
    return product
