"""
A/B testing service for the Model Hub.

This module provides a service for A/B testing model deployments.
"""

import os
import logging
import json
import time
import random
from typing import Dict, List, Any, Optional, Tuple, Union
from datetime import datetime, timedelta
import statistics

# Setup logging
logger = logging.getLogger(__name__)

class ABTestingService:
    """Service for A/B testing model deployments."""
    
    def __init__(self):
        """Initialize the A/B testing service."""
        # Store tests in memory
        self.tests = {}
        
        # Define default test parameters
        self.default_params = {
            "duration_days": 7,
            "traffic_split": 0.5,  # 50% traffic to each variant
            "confidence_level": 0.95,
            "metrics": ["fps", "map", "latency_ms"],
            "primary_metric": "map",
            "success_criteria": {
                "map": ">= 0.05",  # 5% improvement
                "fps": ">= 0.1",  # 10% improvement
                "latency_ms": "<= 0.1",  # 10% improvement
            },
        }
        
        logger.info("A/B testing service initialized")
    
    async def create_test(
        self,
        name: str,
        model_a_id: str,
        model_b_id: str,
        params: Optional[Dict[str, Any]] = None,
    ) -> Dict[str, Any]:
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
            # Merge parameters
            test_params = self.default_params.copy()
            if params:
                test_params.update(params)
            
            # Create test ID
            test_id = f"test_{name}_{datetime.utcnow().strftime('%Y%m%d_%H%M%S')}"
            
            # Calculate end date
            start_date = datetime.utcnow()
            end_date = start_date + timedelta(days=test_params["duration_days"])
            
            # Create test
            test = {
                "id": test_id,
                "name": name,
                "model_a_id": model_a_id,
                "model_b_id": model_b_id,
                "params": test_params,
                "status": "created",
                "start_date": start_date.isoformat(),
                "end_date": end_date.isoformat(),
                "results": {
                    "model_a": {
                        "samples": 0,
                        "metrics": {},
                    },
                    "model_b": {
                        "samples": 0,
                        "metrics": {},
                    },
                    "winner": None,
                    "confidence": None,
                    "improvement": None,
                },
                "created_at": datetime.utcnow().isoformat(),
                "updated_at": datetime.utcnow().isoformat(),
            }
            
            # Store test
            self.tests[test_id] = test
            
            logger.info(f"Created A/B test: {test_id}")
            
            return test
        except Exception as e:
            logger.error(f"Error creating A/B test: {e}")
            raise
    
    async def start_test(self, test_id: str) -> Dict[str, Any]:
        """
        Start an A/B test.
        
        Args:
            test_id: ID of the test
            
        Returns:
            Updated test information
        """
        try:
            # Check if test exists
            if test_id not in self.tests:
                logger.error(f"A/B test {test_id} not found")
                raise ValueError(f"A/B test {test_id} not found")
            
            # Get test
            test = self.tests[test_id]
            
            # Check if test can be started
            if test["status"] != "created":
                logger.error(f"A/B test {test_id} cannot be started (status: {test['status']})")
                raise ValueError(f"A/B test {test_id} cannot be started (status: {test['status']})")
            
            # Update test status
            test["status"] = "running"
            test["updated_at"] = datetime.utcnow().isoformat()
            
            logger.info(f"Started A/B test: {test_id}")
            
            return test
        except Exception as e:
            logger.error(f"Error starting A/B test: {e}")
            raise
    
    async def stop_test(self, test_id: str) -> Dict[str, Any]:
        """
        Stop an A/B test.
        
        Args:
            test_id: ID of the test
            
        Returns:
            Updated test information
        """
        try:
            # Check if test exists
            if test_id not in self.tests:
                logger.error(f"A/B test {test_id} not found")
                raise ValueError(f"A/B test {test_id} not found")
            
            # Get test
            test = self.tests[test_id]
            
            # Check if test can be stopped
            if test["status"] != "running":
                logger.error(f"A/B test {test_id} cannot be stopped (status: {test['status']})")
                raise ValueError(f"A/B test {test_id} cannot be stopped (status: {test['status']})")
            
            # Update test status
            test["status"] = "stopped"
            test["updated_at"] = datetime.utcnow().isoformat()
            
            logger.info(f"Stopped A/B test: {test_id}")
            
            return test
        except Exception as e:
            logger.error(f"Error stopping A/B test: {e}")
            raise
    
    async def get_test(self, test_id: str) -> Optional[Dict[str, Any]]:
        """
        Get information about an A/B test.
        
        Args:
            test_id: ID of the test
            
        Returns:
            Test information or None if not found
        """
        try:
            # Check if test exists
            if test_id not in self.tests:
                logger.warning(f"A/B test {test_id} not found")
                return None
            
            # Get test
            test = self.tests[test_id]
            
            # Check if test has ended
            if test["status"] == "running":
                end_date = datetime.fromisoformat(test["end_date"])
                if datetime.utcnow() > end_date:
                    # Test has ended
                    test["status"] = "completed"
                    test["updated_at"] = datetime.utcnow().isoformat()
                    
                    # Analyze results
                    await self._analyze_results(test_id)
                    
                    logger.info(f"A/B test {test_id} completed")
            
            return test
        except Exception as e:
            logger.error(f"Error getting A/B test: {e}")
            raise
    
    async def list_tests(
        self,
        status: Optional[str] = None,
        limit: int = 100,
        offset: int = 0,
    ) -> List[Dict[str, Any]]:
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
            # Get tests
            tests = list(self.tests.values())
            
            # Filter by status
            if status:
                tests = [t for t in tests if t["status"] == status]
            
            # Sort by creation date (newest first)
            tests.sort(key=lambda t: t["created_at"], reverse=True)
            
            # Apply pagination
            tests = tests[offset:offset + limit]
            
            return tests
        except Exception as e:
            logger.error(f"Error listing A/B tests: {e}")
            raise
    
    async def record_sample(
        self,
        test_id: str,
        model_id: str,
        metrics: Dict[str, float],
    ) -> Dict[str, Any]:
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
            # Check if test exists
            if test_id not in self.tests:
                logger.error(f"A/B test {test_id} not found")
                raise ValueError(f"A/B test {test_id} not found")
            
            # Get test
            test = self.tests[test_id]
            
            # Check if test is running
            if test["status"] != "running":
                logger.error(f"A/B test {test_id} is not running (status: {test['status']})")
                raise ValueError(f"A/B test {test_id} is not running (status: {test['status']})")
            
            # Determine which model the sample is for
            if model_id == test["model_a_id"]:
                model_key = "model_a"
            elif model_id == test["model_b_id"]:
                model_key = "model_b"
            else:
                logger.error(f"Model {model_id} is not part of A/B test {test_id}")
                raise ValueError(f"Model {model_id} is not part of A/B test {test_id}")
            
            # Update sample count
            test["results"][model_key]["samples"] += 1
            
            # Update metrics
            for metric, value in metrics.items():
                if metric not in test["results"][model_key]["metrics"]:
                    test["results"][model_key]["metrics"][metric] = []
                
                test["results"][model_key]["metrics"][metric].append(value)
            
            # Update test
            test["updated_at"] = datetime.utcnow().isoformat()
            
            return test
        except Exception as e:
            logger.error(f"Error recording sample for A/B test: {e}")
            raise
    
    async def select_model(self, test_id: str) -> str:
        """
        Select a model for a request based on the A/B test.
        
        Args:
            test_id: ID of the test
            
        Returns:
            ID of the selected model
        """
        try:
            # Check if test exists
            if test_id not in self.tests:
                logger.error(f"A/B test {test_id} not found")
                raise ValueError(f"A/B test {test_id} not found")
            
            # Get test
            test = self.tests[test_id]
            
            # Check if test is running
            if test["status"] != "running":
                logger.error(f"A/B test {test_id} is not running (status: {test['status']})")
                raise ValueError(f"A/B test {test_id} is not running (status: {test['status']})")
            
            # Get traffic split
            traffic_split = test["params"]["traffic_split"]
            
            # Select model based on traffic split
            if random.random() < traffic_split:
                return test["model_b_id"]  # Variant
            else:
                return test["model_a_id"]  # Control
        except Exception as e:
            logger.error(f"Error selecting model for A/B test: {e}")
            raise
    
    async def _analyze_results(self, test_id: str) -> None:
        """
        Analyze the results of an A/B test.
        
        Args:
            test_id: ID of the test
        """
        try:
            # Check if test exists
            if test_id not in self.tests:
                logger.error(f"A/B test {test_id} not found")
                raise ValueError(f"A/B test {test_id} not found")
            
            # Get test
            test = self.tests[test_id]
            
            # Get results
            results = test["results"]
            
            # Get primary metric
            primary_metric = test["params"]["primary_metric"]
            
            # Check if there are enough samples
            if (results["model_a"]["samples"] < 10 or
                results["model_b"]["samples"] < 10 or
                primary_metric not in results["model_a"]["metrics"] or
                primary_metric not in results["model_b"]["metrics"]):
                logger.warning(f"Not enough samples for A/B test {test_id}")
                return
            
            # Calculate mean for primary metric
            mean_a = statistics.mean(results["model_a"]["metrics"][primary_metric])
            mean_b = statistics.mean(results["model_b"]["metrics"][primary_metric])
            
            # Calculate improvement
            if primary_metric == "latency_ms":
                # Lower is better for latency
                improvement = (mean_a - mean_b) / mean_a
            else:
                # Higher is better for other metrics
                improvement = (mean_b - mean_a) / mean_a
            
            # Determine winner
            success_criteria = test["params"]["success_criteria"].get(primary_metric, ">= 0.05")
            operator, threshold = success_criteria.split()
            threshold = float(threshold)
            
            if operator == ">=":
                is_winner = improvement >= threshold
            elif operator == ">":
                is_winner = improvement > threshold
            elif operator == "<=":
                is_winner = improvement <= threshold
            elif operator == "<":
                is_winner = improvement < threshold
            else:
                is_winner = False
            
            # Update results
            if is_winner:
                results["winner"] = "model_b"
            else:
                results["winner"] = "model_a"
            
            results["confidence"] = 0.95  # Placeholder for actual confidence calculation
            results["improvement"] = improvement
            
            # Update test
            test["updated_at"] = datetime.utcnow().isoformat()
            
            logger.info(f"Analyzed results for A/B test {test_id}: winner={results['winner']}, improvement={improvement:.2%}")
        except Exception as e:
            logger.error(f"Error analyzing results for A/B test {test_id}: {e}")
            raise
