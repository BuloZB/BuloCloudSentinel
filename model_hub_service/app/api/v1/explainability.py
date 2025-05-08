"""
API endpoints for model explainability.

This module provides API endpoints for explaining model predictions.
"""

import logging
from typing import Dict, List, Any, Optional
from datetime import datetime

from fastapi import APIRouter, Depends, HTTPException, status, Query
from fastapi.responses import Response, JSONResponse

from app.services.explainability_service import ExplainabilityService

# Setup logging
logger = logging.getLogger(__name__)

# Create router
router = APIRouter()

# Create explainability service
explainability_service = ExplainabilityService()

@router.post("/explain-prediction/{model_id}")
async def explain_prediction(
    model_id: str,
    inputs: Any,
    method: str = "shap",
    target_class: Optional[int] = None,
    num_samples: int = 100,
    format: str = "png",
):
    """
    Explain a model prediction.
    
    Args:
        model_id: ID of the model
        inputs: Input data
        method: Explainability method (shap, lime, captum)
        target_class: Target class for classification models
        num_samples: Number of samples for explainability methods
        format: Output format (png, svg, pdf, html)
        
    Returns:
        Explanation visualization
    """
    try:
        # Explain prediction
        explanation_data, content_type = await explainability_service.explain_prediction(
            model_id=model_id,
            inputs=inputs,
            method=method,
            target_class=target_class,
            num_samples=num_samples,
            format=format,
        )
        
        # Return explanation
        return Response(content=explanation_data, media_type=content_type)
    except ValueError as e:
        logger.error(f"Error explaining prediction: {e}")
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=str(e),
        )
    except Exception as e:
        logger.error(f"Error explaining prediction: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error explaining prediction: {str(e)}",
        )

@router.get("/explain-model/{model_id}")
async def explain_model(
    model_id: str,
    method: str = "shap",
    num_samples: int = 100,
    format: str = "png",
):
    """
    Explain a model globally.
    
    Args:
        model_id: ID of the model
        method: Explainability method (shap, lime, captum)
        num_samples: Number of samples for explainability methods
        format: Output format (png, svg, pdf, html)
        
    Returns:
        Explanation visualization
    """
    try:
        # Explain model
        explanation_data, content_type = await explainability_service.explain_model(
            model_id=model_id,
            method=method,
            num_samples=num_samples,
            format=format,
        )
        
        # Return explanation
        return Response(content=explanation_data, media_type=content_type)
    except ValueError as e:
        logger.error(f"Error explaining model: {e}")
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=str(e),
        )
    except Exception as e:
        logger.error(f"Error explaining model: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error explaining model: {str(e)}",
        )

@router.get("/methods")
async def list_explainability_methods():
    """
    List available explainability methods.
    
    Returns:
        List of available explainability methods
    """
    try:
        # Get available methods
        methods = []
        
        if explainability_service.shap_available:
            methods.append({
                "name": "shap",
                "description": "SHAP (SHapley Additive exPlanations)",
                "available": True,
            })
        else:
            methods.append({
                "name": "shap",
                "description": "SHAP (SHapley Additive exPlanations)",
                "available": False,
                "reason": "SHAP not installed",
            })
        
        if explainability_service.lime_available:
            methods.append({
                "name": "lime",
                "description": "LIME (Local Interpretable Model-agnostic Explanations)",
                "available": True,
            })
        else:
            methods.append({
                "name": "lime",
                "description": "LIME (Local Interpretable Model-agnostic Explanations)",
                "available": False,
                "reason": "LIME not installed",
            })
        
        if explainability_service.captum_available:
            methods.append({
                "name": "captum",
                "description": "Captum (PyTorch model interpretability)",
                "available": True,
            })
        else:
            methods.append({
                "name": "captum",
                "description": "Captum (PyTorch model interpretability)",
                "available": False,
                "reason": "Captum not installed",
            })
        
        return methods
    except Exception as e:
        logger.error(f"Error listing explainability methods: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error listing explainability methods: {str(e)}",
        )
