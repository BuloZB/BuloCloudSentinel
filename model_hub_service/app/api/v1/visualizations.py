"""
API endpoints for visualizations.

This module provides API endpoints for visualizing model performance metrics.
"""

import logging
from typing import Dict, List, Any, Optional
from datetime import datetime

from fastapi import APIRouter, Depends, HTTPException, status, Query
from fastapi.responses import Response, JSONResponse

from app.services.visualization_service import VisualizationService

# Setup logging
logger = logging.getLogger(__name__)

# Create router
router = APIRouter()

# Create visualization service
visualization_service = VisualizationService()

@router.post("/performance-chart")
async def create_performance_chart(
    metrics: Dict[str, List[float]],
    timestamps: Optional[List[str]] = None,
    title: str = "Model Performance",
    chart_type: str = "line",
    width: int = 800,
    height: int = 500,
    format: str = "png",
):
    """
    Create a performance chart.
    
    Args:
        metrics: Dictionary of metrics (metric_name -> list of values)
        timestamps: List of timestamps for the metrics
        title: Chart title
        chart_type: Type of chart (line, bar, scatter)
        width: Chart width
        height: Chart height
        format: Output format (png, svg, pdf, html)
        
    Returns:
        Chart image or HTML
    """
    try:
        # Create chart
        chart_data, content_type = await visualization_service.create_performance_chart(
            metrics=metrics,
            timestamps=timestamps,
            title=title,
            chart_type=chart_type,
            width=width,
            height=height,
            format=format,
        )
        
        # Return chart
        return Response(content=chart_data, media_type=content_type)
    except Exception as e:
        logger.error(f"Error creating performance chart: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error creating performance chart: {str(e)}",
        )

@router.post("/comparison-chart")
async def create_comparison_chart(
    metrics_a: Dict[str, List[float]],
    metrics_b: Dict[str, List[float]],
    model_a_name: str = "Model A",
    model_b_name: str = "Model B",
    title: str = "Model Comparison",
    chart_type: str = "bar",
    width: int = 800,
    height: int = 500,
    format: str = "png",
):
    """
    Create a comparison chart for two models.
    
    Args:
        metrics_a: Dictionary of metrics for model A (metric_name -> list of values)
        metrics_b: Dictionary of metrics for model B (metric_name -> list of values)
        model_a_name: Name of model A
        model_b_name: Name of model B
        title: Chart title
        chart_type: Type of chart (bar, radar)
        width: Chart width
        height: Chart height
        format: Output format (png, svg, pdf, html)
        
    Returns:
        Chart image or HTML
    """
    try:
        # Create chart
        chart_data, content_type = await visualization_service.create_comparison_chart(
            metrics_a=metrics_a,
            metrics_b=metrics_b,
            model_a_name=model_a_name,
            model_b_name=model_b_name,
            title=title,
            chart_type=chart_type,
            width=width,
            height=height,
            format=format,
        )
        
        # Return chart
        return Response(content=chart_data, media_type=content_type)
    except Exception as e:
        logger.error(f"Error creating comparison chart: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error creating comparison chart: {str(e)}",
        )

@router.post("/ab-test-chart/{test_id}")
async def create_ab_test_chart(
    test_id: str,
    title: str = "A/B Test Results",
    chart_type: str = "bar",
    width: int = 800,
    height: int = 500,
    format: str = "png",
):
    """
    Create a chart for A/B test results.
    
    Args:
        test_id: ID of the A/B test
        title: Chart title
        chart_type: Type of chart (bar, radar)
        width: Chart width
        height: Chart height
        format: Output format (png, svg, pdf, html)
        
    Returns:
        Chart image or HTML
    """
    try:
        # Get A/B test service
        from app.services.ab_testing_service import ABTestingService
        ab_testing_service = ABTestingService()
        
        # Get test
        test = await ab_testing_service.get_test(test_id)
        
        if not test:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail=f"A/B test with ID {test_id} not found",
            )
        
        # Get metrics
        metrics_a = {}
        metrics_b = {}
        
        for metric_name, values in test["results"]["model_a"]["metrics"].items():
            metrics_a[metric_name] = values
        
        for metric_name, values in test["results"]["model_b"]["metrics"].items():
            metrics_b[metric_name] = values
        
        # Create chart
        chart_data, content_type = await visualization_service.create_comparison_chart(
            metrics_a=metrics_a,
            metrics_b=metrics_b,
            model_a_name="Model A (Control)",
            model_b_name="Model B (Variant)",
            title=f"{title} - {test['name']}",
            chart_type=chart_type,
            width=width,
            height=height,
            format=format,
        )
        
        # Return chart
        return Response(content=chart_data, media_type=content_type)
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error creating A/B test chart: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error creating A/B test chart: {str(e)}",
        )

@router.post("/deployment-chart/{deployment_id}")
async def create_deployment_chart(
    deployment_id: str,
    metric: str = "fps",
    title: str = "Deployment Performance",
    chart_type: str = "line",
    width: int = 800,
    height: int = 500,
    format: str = "png",
):
    """
    Create a chart for deployment performance.
    
    Args:
        deployment_id: ID of the deployment
        metric: Metric to visualize
        title: Chart title
        chart_type: Type of chart (line, bar, scatter)
        width: Chart width
        height: Chart height
        format: Output format (png, svg, pdf, html)
        
    Returns:
        Chart image or HTML
    """
    try:
        # Get metrics service
        from app.services.metrics_service import MetricsService
        metrics_service = MetricsService()
        
        # Get metrics
        deployment_metrics = await metrics_service.get_metrics(deployment_id)
        
        if not deployment_metrics or not deployment_metrics.get("history"):
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail=f"Metrics for deployment {deployment_id} not found",
            )
        
        # Extract metrics
        metrics = {}
        timestamps = []
        
        for entry in deployment_metrics["history"]:
            if metric in entry:
                if metric not in metrics:
                    metrics[metric] = []
                
                metrics[metric].append(entry[metric])
                timestamps.append(entry.get("timestamp", ""))
        
        # Create chart
        chart_data, content_type = await visualization_service.create_performance_chart(
            metrics=metrics,
            timestamps=timestamps,
            title=f"{title} - {metric}",
            chart_type=chart_type,
            width=width,
            height=height,
            format=format,
        )
        
        # Return chart
        return Response(content=chart_data, media_type=content_type)
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error creating deployment chart: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error creating deployment chart: {str(e)}",
        )
