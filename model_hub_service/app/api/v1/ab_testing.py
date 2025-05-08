"""
API endpoints for A/B testing.

This module provides API endpoints for A/B testing model deployments.
"""

import logging
from typing import Dict, List, Any, Optional
from datetime import datetime

from fastapi import APIRouter, Depends, HTTPException, status, Query
from fastapi.responses import JSONResponse

from app.services.ab_testing_service import ABTestingService

# Setup logging
logger = logging.getLogger(__name__)

# Create router
router = APIRouter()

# Create A/B testing service
ab_testing_service = ABTestingService()

@router.post("/tests")
async def create_test(
    name: str,
    model_a_id: str,
    model_b_id: str,
    params: Optional[Dict[str, Any]] = None,
):
    """
    Create a new A/B test.
    
    Args:
        name: Name of the test
        model_a_id: ID of model A (control)
        model_b_id: ID of model B (variant)
        params: Test parameters
        
    Returns:
        Test information
    """
    try:
        # Create test
        test = await ab_testing_service.create_test(
            name=name,
            model_a_id=model_a_id,
            model_b_id=model_b_id,
            params=params,
        )
        
        return test
    except Exception as e:
        logger.error(f"Error creating A/B test: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error creating A/B test: {str(e)}",
        )

@router.post("/tests/{test_id}/start")
async def start_test(test_id: str):
    """
    Start an A/B test.
    
    Args:
        test_id: ID of the test
        
    Returns:
        Updated test information
    """
    try:
        # Start test
        test = await ab_testing_service.start_test(test_id)
        
        return test
    except ValueError as e:
        logger.error(f"Error starting A/B test: {e}")
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=str(e),
        )
    except Exception as e:
        logger.error(f"Error starting A/B test: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error starting A/B test: {str(e)}",
        )

@router.post("/tests/{test_id}/stop")
async def stop_test(test_id: str):
    """
    Stop an A/B test.
    
    Args:
        test_id: ID of the test
        
    Returns:
        Updated test information
    """
    try:
        # Stop test
        test = await ab_testing_service.stop_test(test_id)
        
        return test
    except ValueError as e:
        logger.error(f"Error stopping A/B test: {e}")
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=str(e),
        )
    except Exception as e:
        logger.error(f"Error stopping A/B test: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error stopping A/B test: {str(e)}",
        )

@router.get("/tests/{test_id}")
async def get_test(test_id: str):
    """
    Get information about an A/B test.
    
    Args:
        test_id: ID of the test
        
    Returns:
        Test information
    """
    try:
        # Get test
        test = await ab_testing_service.get_test(test_id)
        
        if not test:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail=f"A/B test with ID {test_id} not found",
            )
        
        return test
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error getting A/B test: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error getting A/B test: {str(e)}",
        )

@router.get("/tests")
async def list_tests(
    status: Optional[str] = None,
    limit: int = 100,
    offset: int = 0,
):
    """
    List A/B tests.
    
    Args:
        status: Filter by test status
        limit: Maximum number of tests to return
        offset: Offset for pagination
        
    Returns:
        List of tests
    """
    try:
        # List tests
        tests = await ab_testing_service.list_tests(
            status=status,
            limit=limit,
            offset=offset,
        )
        
        return tests
    except Exception as e:
        logger.error(f"Error listing A/B tests: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error listing A/B tests: {str(e)}",
        )

@router.post("/tests/{test_id}/samples")
async def record_sample(
    test_id: str,
    model_id: str,
    metrics: Dict[str, float],
):
    """
    Record a sample for an A/B test.
    
    Args:
        test_id: ID of the test
        model_id: ID of the model
        metrics: Metrics for the sample
        
    Returns:
        Updated test information
    """
    try:
        # Record sample
        test = await ab_testing_service.record_sample(
            test_id=test_id,
            model_id=model_id,
            metrics=metrics,
        )
        
        return test
    except ValueError as e:
        logger.error(f"Error recording sample for A/B test: {e}")
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=str(e),
        )
    except Exception as e:
        logger.error(f"Error recording sample for A/B test: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error recording sample for A/B test: {str(e)}",
        )

@router.get("/tests/{test_id}/select")
async def select_model(test_id: str):
    """
    Select a model for a request based on the A/B test.
    
    Args:
        test_id: ID of the test
        
    Returns:
        ID of the selected model
    """
    try:
        # Select model
        model_id = await ab_testing_service.select_model(test_id)
        
        return {"model_id": model_id}
    except ValueError as e:
        logger.error(f"Error selecting model for A/B test: {e}")
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=str(e),
        )
    except Exception as e:
        logger.error(f"Error selecting model for A/B test: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error selecting model for A/B test: {str(e)}",
        )
