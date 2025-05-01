"""
Inference service for the AI Analytics module.

This service provides a unified interface for running inference with different ML backends.
"""

import logging
import os
import sys
import time
import asyncio
from typing import Dict, Any, Optional, List, Tuple
import numpy as np
from pathlib import Path
import uuid
from datetime import datetime

# Add project root to path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))

# Import inference engine
from ai.inference import InferenceEngine

# Setup logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class InferenceService:
    """
    Inference service for the AI Analytics module.
    
    This service provides a unified interface for running inference with different ML backends.
    """
    
    def __init__(self, config: Dict[str, Any]):
        """
        Initialize the inference service.
        
        Args:
            config: Service configuration
        """
        self.config = config
        self.engines = {}
        self.active_tasks = {}
        
        # Get ML backend from environment or config
        self.ml_backend = os.environ.get("ML_BACKEND", config.get("ml_backend", "torch"))
        self.device = os.environ.get("DEVICE", config.get("device", "AUTO"))
        
        # Initialize engines
        self._initialize_engines()
    
    def _initialize_engines(self):
        """Initialize inference engines."""
        try:
            logger.info(f"Initializing inference engines with {self.ml_backend} backend on {self.device}")
            
            # Get model configurations from config
            model_configs = self.config.get("models", [])
            
            for model_config in model_configs:
                model_name = model_config.get("name")
                model_path = model_config.get("path")
                
                if model_name and model_path and os.path.exists(model_path):
                    # Initialize inference engine for the model
                    engine = InferenceEngine(
                        backend=self.ml_backend,
                        model_path=model_path,
                        device=self.device
                    )
                    
                    # Store engine
                    self.engines[model_name] = engine
                    
                    logger.info(f"Initialized model {model_name} with {self.ml_backend} backend")
                else:
                    logger.warning(f"Skipping model {model_name}: path {model_path} not found")
            
            logger.info(f"Initialized {len(self.engines)} inference engines")
        except Exception as e:
            logger.error(f"Error initializing inference engines: {str(e)}")
            raise
    
    async def run_inference(
        self,
        model_name: str,
        inputs: Dict[str, np.ndarray]
    ) -> Dict[str, Any]:
        """
        Run inference on a model.
        
        Args:
            model_name: Name of the model to use
            inputs: Dictionary mapping input names to numpy arrays
            
        Returns:
            Dictionary with inference results
        """
        try:
            # Check if model exists
            if model_name not in self.engines:
                logger.error(f"Model {model_name} not found")
                return {
                    "error": f"Model {model_name} not found",
                    "success": False
                }
            
            # Get engine
            engine = self.engines[model_name]
            
            # Run inference
            start_time = time.time()
            outputs = engine.predict(inputs)
            inference_time = (time.time() - start_time) * 1000  # Convert to ms
            
            # Return results
            return {
                "outputs": outputs,
                "inference_time": inference_time,
                "success": True
            }
        except Exception as e:
            logger.error(f"Error running inference: {str(e)}")
            return {
                "error": str(e),
                "success": False
            }
    
    async def start_inference_task(
        self,
        model_name: str,
        input_generator: Any,
        result_handler: Any,
        interval: float = 1.0
    ) -> str:
        """
        Start a continuous inference task.
        
        Args:
            model_name: Name of the model to use
            input_generator: Function that generates inputs for inference
            result_handler: Function that handles inference results
            interval: Inference interval in seconds
            
        Returns:
            Task ID
        """
        try:
            # Check if model exists
            if model_name not in self.engines:
                logger.error(f"Model {model_name} not found")
                raise ValueError(f"Model {model_name} not found")
            
            # Generate task ID
            task_id = str(uuid.uuid4())
            
            # Start inference task
            task = asyncio.create_task(
                self._inference_task(task_id, model_name, input_generator, result_handler, interval)
            )
            
            # Store task
            self.active_tasks[task_id] = {
                "task": task,
                "model_name": model_name,
                "interval": interval,
                "start_time": datetime.now()
            }
            
            logger.info(f"Started inference task {task_id} for model {model_name}")
            
            return task_id
        except Exception as e:
            logger.error(f"Error starting inference task: {str(e)}")
            raise
    
    async def stop_inference_task(self, task_id: str) -> bool:
        """
        Stop an inference task.
        
        Args:
            task_id: Task ID
            
        Returns:
            True if task was stopped, False otherwise
        """
        try:
            # Check if task exists
            if task_id not in self.active_tasks:
                logger.warning(f"Task {task_id} not found")
                return False
            
            # Get task
            task_info = self.active_tasks[task_id]
            task = task_info["task"]
            
            # Cancel task
            task.cancel()
            
            # Remove task
            del self.active_tasks[task_id]
            
            logger.info(f"Stopped inference task {task_id}")
            
            return True
        except Exception as e:
            logger.error(f"Error stopping inference task: {str(e)}")
            return False
    
    async def get_active_tasks(self) -> List[Dict[str, Any]]:
        """
        Get active inference tasks.
        
        Returns:
            List of active tasks
        """
        return [
            {
                "task_id": task_id,
                "model_name": task_info["model_name"],
                "interval": task_info["interval"],
                "start_time": task_info["start_time"].isoformat()
            }
            for task_id, task_info in self.active_tasks.items()
        ]
    
    async def _inference_task(
        self,
        task_id: str,
        model_name: str,
        input_generator: Any,
        result_handler: Any,
        interval: float
    ):
        """
        Inference task.
        
        Args:
            task_id: Task ID
            model_name: Name of the model to use
            input_generator: Function that generates inputs for inference
            result_handler: Function that handles inference results
            interval: Inference interval in seconds
        """
        try:
            logger.info(f"Starting inference task {task_id} for model {model_name}")
            
            while True:
                # Generate inputs
                inputs = await input_generator()
                
                if inputs is None:
                    logger.warning(f"Input generator returned None for task {task_id}")
                    await asyncio.sleep(interval)
                    continue
                
                # Run inference
                result = await self.run_inference(model_name, inputs)
                
                # Handle results
                await result_handler(result)
                
                # Wait for next interval
                await asyncio.sleep(interval)
        except asyncio.CancelledError:
            logger.info(f"Inference task {task_id} cancelled")
        except Exception as e:
            logger.error(f"Error in inference task {task_id}: {str(e)}")
    
    def get_available_backends(self) -> List[str]:
        """
        Get available ML backends.
        
        Returns:
            List of available backends
        """
        return InferenceEngine.get_available_backends()
    
    def get_backend_info(self) -> Dict[str, Any]:
        """
        Get information about the current backend.
        
        Returns:
            Dictionary with backend information
        """
        return {
            "backend": self.ml_backend,
            "device": self.device,
            "models": list(self.engines.keys())
        }
    
    def get_model_info(self, model_name: str) -> Optional[Dict[str, Any]]:
        """
        Get information about a model.
        
        Args:
            model_name: Name of the model
            
        Returns:
            Dictionary with model information
        """
        if model_name not in self.engines:
            return None
        
        engine = self.engines[model_name]
        
        return {
            "name": model_name,
            "backend": self.ml_backend,
            "device": self.device,
            "input_shapes": engine.get_input_shapes(),
            "output_shapes": engine.get_output_shapes()
        }
