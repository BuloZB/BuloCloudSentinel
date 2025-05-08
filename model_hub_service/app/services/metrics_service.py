"""
Metrics service for the Model Hub.

This module provides a service for collecting and analyzing model performance metrics.
"""

import os
import logging
import json
import time
from typing import Dict, List, Any, Optional, Tuple, Union
from datetime import datetime, timedelta
import statistics

import numpy as np

# Setup logging
logger = logging.getLogger(__name__)

class MetricsService:
    """Service for collecting and analyzing model performance metrics."""
    
    def __init__(self):
        """Initialize the metrics service."""
        # Store metrics in memory
        self.metrics = {}
        
        # Define thresholds for automatic rollback
        self.thresholds = {
            "fps": 0.8,  # 20% degradation
            "map": 0.95,  # 5% degradation
            "latency_ms": 1.2,  # 20% increase
        }
        
        logger.info("Metrics service initialized")
    
    async def record_metrics(
        self,
        deployment_id: str,
        model_id: str,
        metrics: Dict[str, Any],
    ) -> Dict[str, Any]:
        """
        Record metrics for a deployment.
        
        Args:
            deployment_id: ID of the deployment
            model_id: ID of the model
            metrics: Metrics to record
            
        Returns:
            Recorded metrics with analysis
        """
        try:
            # Initialize metrics for deployment if not exists
            if deployment_id not in self.metrics:
                self.metrics[deployment_id] = {
                    "model_id": model_id,
                    "history": [],
                }
            
            # Add timestamp to metrics
            metrics["timestamp"] = datetime.utcnow().isoformat()
            
            # Add metrics to history
            self.metrics[deployment_id]["history"].append(metrics)
            
            # Limit history size
            max_history = 100
            if len(self.metrics[deployment_id]["history"]) > max_history:
                self.metrics[deployment_id]["history"] = self.metrics[deployment_id]["history"][-max_history:]
            
            # Analyze metrics
            analysis = await self.analyze_metrics(deployment_id)
            
            # Return metrics with analysis
            return {
                "deployment_id": deployment_id,
                "model_id": model_id,
                "metrics": metrics,
                "analysis": analysis,
            }
        except Exception as e:
            logger.error(f"Error recording metrics: {e}")
            raise
    
    async def analyze_metrics(self, deployment_id: str) -> Dict[str, Any]:
        """
        Analyze metrics for a deployment.
        
        Args:
            deployment_id: ID of the deployment
            
        Returns:
            Metrics analysis
        """
        try:
            # Check if deployment exists
            if deployment_id not in self.metrics:
                logger.warning(f"Deployment {deployment_id} not found in metrics")
                return {
                    "status": "unknown",
                    "reason": f"Deployment {deployment_id} not found in metrics",
                }
            
            # Get metrics history
            history = self.metrics[deployment_id]["history"]
            
            # Check if there are enough metrics
            if len(history) < 2:
                logger.info(f"Not enough metrics for deployment {deployment_id}")
                return {
                    "status": "insufficient_data",
                    "reason": "Not enough metrics to analyze",
                }
            
            # Get latest metrics
            latest = history[-1]
            
            # Calculate average of previous metrics
            previous = history[:-1]
            avg_fps = statistics.mean([m.get("fps", 0) for m in previous if "fps" in m])
            avg_map = statistics.mean([m.get("map", 0) for m in previous if "map" in m])
            avg_latency = statistics.mean([m.get("latency_ms", 0) for m in previous if "latency_ms" in m])
            
            # Calculate changes
            fps_change = latest.get("fps", 0) / avg_fps if avg_fps > 0 else 1.0
            map_change = latest.get("map", 0) / avg_map if avg_map > 0 else 1.0
            latency_change = avg_latency / latest.get("latency_ms", 1) if latest.get("latency_ms", 0) > 0 else 1.0
            
            # Check for degradation
            degradation = False
            degradation_reasons = []
            
            if fps_change < self.thresholds["fps"]:
                degradation = True
                degradation_reasons.append(f"FPS degraded by {(1 - fps_change) * 100:.1f}%")
            
            if map_change < self.thresholds["map"]:
                degradation = True
                degradation_reasons.append(f"mAP degraded by {(1 - map_change) * 100:.1f}%")
            
            if latency_change < self.thresholds["latency_ms"]:
                degradation = True
                degradation_reasons.append(f"Latency increased by {(1 / latency_change - 1) * 100:.1f}%")
            
            # Create analysis
            analysis = {
                "status": "degraded" if degradation else "healthy",
                "fps_change": fps_change,
                "map_change": map_change,
                "latency_change": latency_change,
                "avg_fps": avg_fps,
                "avg_map": avg_map,
                "avg_latency": avg_latency,
                "latest_fps": latest.get("fps", 0),
                "latest_map": latest.get("map", 0),
                "latest_latency": latest.get("latency_ms", 0),
                "degradation_reasons": degradation_reasons,
                "should_rollback": degradation,
            }
            
            return analysis
        except Exception as e:
            logger.error(f"Error analyzing metrics: {e}")
            raise
    
    async def get_metrics(
        self,
        deployment_id: str,
        start_time: Optional[datetime] = None,
        end_time: Optional[datetime] = None,
        limit: int = 100,
    ) -> Dict[str, Any]:
        """
        Get metrics for a deployment.
        
        Args:
            deployment_id: ID of the deployment
            start_time: Start time for metrics
            end_time: End time for metrics
            limit: Maximum number of metrics to return
            
        Returns:
            Metrics for the deployment
        """
        try:
            # Check if deployment exists
            if deployment_id not in self.metrics:
                logger.warning(f"Deployment {deployment_id} not found in metrics")
                return {
                    "deployment_id": deployment_id,
                    "history": [],
                }
            
            # Get metrics history
            history = self.metrics[deployment_id]["history"]
            
            # Filter by time range
            if start_time or end_time:
                filtered_history = []
                for metrics in history:
                    timestamp = datetime.fromisoformat(metrics["timestamp"])
                    if start_time and timestamp < start_time:
                        continue
                    if end_time and timestamp > end_time:
                        continue
                    filtered_history.append(metrics)
                history = filtered_history
            
            # Limit number of metrics
            if limit > 0 and len(history) > limit:
                history = history[-limit:]
            
            # Return metrics
            return {
                "deployment_id": deployment_id,
                "model_id": self.metrics[deployment_id]["model_id"],
                "history": history,
            }
        except Exception as e:
            logger.error(f"Error getting metrics: {e}")
            raise
    
    async def get_all_metrics(
        self,
        start_time: Optional[datetime] = None,
        end_time: Optional[datetime] = None,
        limit: int = 100,
    ) -> Dict[str, Any]:
        """
        Get metrics for all deployments.
        
        Args:
            start_time: Start time for metrics
            end_time: End time for metrics
            limit: Maximum number of metrics to return per deployment
            
        Returns:
            Metrics for all deployments
        """
        try:
            # Get metrics for each deployment
            all_metrics = {}
            for deployment_id in self.metrics:
                deployment_metrics = await self.get_metrics(
                    deployment_id, start_time, end_time, limit
                )
                all_metrics[deployment_id] = deployment_metrics
            
            return all_metrics
        except Exception as e:
            logger.error(f"Error getting all metrics: {e}")
            raise
    
    async def should_rollback(self, deployment_id: str) -> Tuple[bool, str]:
        """
        Check if a deployment should be rolled back.
        
        Args:
            deployment_id: ID of the deployment
            
        Returns:
            Tuple of (should_rollback, reason)
        """
        try:
            # Analyze metrics
            analysis = await self.analyze_metrics(deployment_id)
            
            # Check if should rollback
            if analysis.get("status") == "degraded":
                reasons = analysis.get("degradation_reasons", [])
                reason = ", ".join(reasons) if reasons else "Unknown reason"
                return True, reason
            
            return False, ""
        except Exception as e:
            logger.error(f"Error checking if should rollback: {e}")
            raise
    
    async def clear_metrics(self, deployment_id: str) -> None:
        """
        Clear metrics for a deployment.
        
        Args:
            deployment_id: ID of the deployment
        """
        try:
            # Check if deployment exists
            if deployment_id in self.metrics:
                del self.metrics[deployment_id]
                logger.info(f"Cleared metrics for deployment {deployment_id}")
        except Exception as e:
            logger.error(f"Error clearing metrics: {e}")
            raise
    
    async def set_threshold(self, metric: str, value: float) -> None:
        """
        Set threshold for a metric.
        
        Args:
            metric: Name of the metric
            value: Threshold value
        """
        try:
            # Check if metric exists
            if metric in self.thresholds:
                self.thresholds[metric] = value
                logger.info(f"Set threshold for {metric} to {value}")
            else:
                logger.warning(f"Metric {metric} not found in thresholds")
        except Exception as e:
            logger.error(f"Error setting threshold: {e}")
            raise
    
    async def get_thresholds(self) -> Dict[str, float]:
        """
        Get thresholds for all metrics.
        
        Returns:
            Thresholds for all metrics
        """
        return self.thresholds.copy()
