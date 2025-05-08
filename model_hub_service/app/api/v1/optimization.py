"""
API endpoints for automated model optimization.

This module provides API endpoints for automated model optimization.
"""

import logging
from typing import Dict, List, Any, Optional
from datetime import datetime

from fastapi import APIRouter, Depends, HTTPException, status, Query
from fastapi.responses import JSONResponse

from app.services.optimization_service import OptimizationService

# Setup logging
logger = logging.getLogger(__name__)

# Create router
router = APIRouter()

# Create optimization service
optimization_service = OptimizationService()

@router.post("/jobs")
async def create_job(
    name: str,
    model_type: str,
    dataset_path: str,
    search_space: Dict[str, Any],
    config: Optional[Dict[str, Any]] = None,
):
    """
    Create a new optimization job.
    
    Args:
        name: Name of the job
        model_type: Type of the model to optimize
        dataset_path: Path to the dataset
        search_space: Search space for hyperparameters
        config: Job configuration
        
    Returns:
        Job information
    """
    try:
        # Create job
        job = await optimization_service.create_job(
            name=name,
            model_type=model_type,
            dataset_path=dataset_path,
            search_space=search_space,
            config=config,
        )
        
        return job
    except ValueError as e:
        logger.error(f"Error creating optimization job: {e}")
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=str(e),
        )
    except Exception as e:
        logger.error(f"Error creating optimization job: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error creating optimization job: {str(e)}",
        )

@router.post("/jobs/{job_id}/start")
async def start_job(job_id: str):
    """
    Start an optimization job.
    
    Args:
        job_id: ID of the job
        
    Returns:
        Updated job information
    """
    try:
        # Start job
        job = await optimization_service.start_job(job_id)
        
        return job
    except ValueError as e:
        logger.error(f"Error starting optimization job: {e}")
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=str(e),
        )
    except Exception as e:
        logger.error(f"Error starting optimization job: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error starting optimization job: {str(e)}",
        )

@router.get("/jobs/{job_id}")
async def get_job(job_id: str):
    """
    Get information about an optimization job.
    
    Args:
        job_id: ID of the job
        
    Returns:
        Job information
    """
    try:
        # Get job
        job = await optimization_service.get_job(job_id)
        
        if not job:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail=f"Optimization job with ID {job_id} not found",
            )
        
        return job
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error getting optimization job: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error getting optimization job: {str(e)}",
        )

@router.get("/jobs")
async def list_jobs(
    status: Optional[str] = None,
    limit: int = 100,
    offset: int = 0,
):
    """
    List optimization jobs.
    
    Args:
        status: Filter by job status
        limit: Maximum number of jobs to return
        offset: Offset for pagination
        
    Returns:
        List of jobs
    """
    try:
        # List jobs
        jobs = await optimization_service.list_jobs(
            status=status,
            limit=limit,
            offset=offset,
        )
        
        return jobs
    except Exception as e:
        logger.error(f"Error listing optimization jobs: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error listing optimization jobs: {str(e)}",
        )

@router.get("/jobs/{job_id}/trials/{trial_id}")
async def get_trial(
    job_id: str,
    trial_id: str,
):
    """
    Get information about a trial.
    
    Args:
        job_id: ID of the job
        trial_id: ID of the trial
        
    Returns:
        Trial information
    """
    try:
        # Get trial
        trial = await optimization_service.get_trial(
            job_id=job_id,
            trial_id=trial_id,
        )
        
        if not trial:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail=f"Trial with ID {trial_id} not found in job {job_id}",
            )
        
        return trial
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error getting trial: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error getting trial: {str(e)}",
        )

@router.get("/jobs/{job_id}/trials")
async def list_trials(
    job_id: str,
    limit: int = 100,
    offset: int = 0,
):
    """
    List trials for an optimization job.
    
    Args:
        job_id: ID of the job
        limit: Maximum number of trials to return
        offset: Offset for pagination
        
    Returns:
        List of trials
    """
    try:
        # List trials
        trials = await optimization_service.list_trials(
            job_id=job_id,
            limit=limit,
            offset=offset,
        )
        
        return trials
    except Exception as e:
        logger.error(f"Error listing trials: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error listing trials: {str(e)}",
        )

@router.get("/jobs/{job_id}/best-trial")
async def get_best_trial(job_id: str):
    """
    Get the best trial for an optimization job.
    
    Args:
        job_id: ID of the job
        
    Returns:
        Best trial information
    """
    try:
        # Get job
        job = await optimization_service.get_job(job_id)
        
        if not job:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail=f"Optimization job with ID {job_id} not found",
            )
        
        # Check if best trial exists
        if not job["best_trial"]:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail=f"No best trial found for job {job_id}",
            )
        
        return job["best_trial"]
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error getting best trial: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error getting best trial: {str(e)}",
        )

@router.get("/methods")
async def list_optimization_methods():
    """
    List available optimization methods.
    
    Returns:
        List of available optimization methods
    """
    try:
        # Get available methods
        methods = []
        
        if optimization_service.optuna_available:
            methods.append({
                "name": "bayesian",
                "description": "Bayesian optimization using Optuna",
                "available": True,
            })
        else:
            methods.append({
                "name": "bayesian",
                "description": "Bayesian optimization using Optuna",
                "available": False,
                "reason": "Optuna not installed",
            })
        
        if optimization_service.ray_tune_available:
            methods.append({
                "name": "hyperband",
                "description": "Hyperband optimization using Ray Tune",
                "available": True,
            })
        else:
            methods.append({
                "name": "hyperband",
                "description": "Hyperband optimization using Ray Tune",
                "available": False,
                "reason": "Ray Tune not installed",
            })
        
        if optimization_service.hyperopt_available:
            methods.append({
                "name": "tpe",
                "description": "Tree-structured Parzen Estimator using Hyperopt",
                "available": True,
            })
        else:
            methods.append({
                "name": "tpe",
                "description": "Tree-structured Parzen Estimator using Hyperopt",
                "available": False,
                "reason": "Hyperopt not installed",
            })
        
        return methods
    except Exception as e:
        logger.error(f"Error listing optimization methods: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error listing optimization methods: {str(e)}",
        )
