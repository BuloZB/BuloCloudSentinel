"""
MLflow service for the Model Hub.

This module provides a service for interacting with MLflow.
"""

import os
import logging
import tempfile
import hashlib
from typing import Dict, List, Any, Optional, BinaryIO, Tuple
from datetime import datetime

import mlflow
from mlflow.tracking import MlflowClient
from mlflow.entities import Run
from mlflow.exceptions import MlflowException

# Setup logging
logger = logging.getLogger(__name__)

class MLflowService:
    """Service for interacting with MLflow."""
    
    def __init__(self):
        """Initialize the MLflow service."""
        # Get MLflow tracking URI from environment variable
        self.tracking_uri = os.environ.get("MLFLOW_TRACKING_URI", "http://localhost:5000")
        
        # Set MLflow tracking URI
        mlflow.set_tracking_uri(self.tracking_uri)
        
        # Create MLflow client
        self.client = MlflowClient()
        
        logger.info(f"MLflow service initialized with tracking URI: {self.tracking_uri}")
    
    async def create_experiment(self, name: str, tags: Optional[Dict[str, Any]] = None) -> str:
        """
        Create a new MLflow experiment.
        
        Args:
            name: Name of the experiment
            tags: Tags for the experiment
            
        Returns:
            Experiment ID
        """
        try:
            # Check if experiment already exists
            experiment = self.client.get_experiment_by_name(name)
            if experiment:
                logger.info(f"Experiment {name} already exists with ID {experiment.experiment_id}")
                return experiment.experiment_id
            
            # Create experiment
            experiment_id = self.client.create_experiment(name, tags=tags)
            logger.info(f"Created experiment {name} with ID {experiment_id}")
            
            return experiment_id
        except Exception as e:
            logger.error(f"Error creating experiment {name}: {e}")
            raise
    
    async def log_model(
        self,
        model_path: str,
        model_name: str,
        model_version: str,
        model_type: str,
        framework: str,
        tags: Optional[Dict[str, Any]] = None,
        metadata: Optional[Dict[str, Any]] = None,
    ) -> Tuple[str, str, str]:
        """
        Log a model to MLflow.
        
        Args:
            model_path: Path to the model file
            model_name: Name of the model
            model_version: Version of the model
            model_type: Type of the model (e.g., "yolov10", "sam")
            framework: Framework of the model (e.g., "pytorch", "onnx")
            tags: Tags for the model
            metadata: Additional metadata for the model
            
        Returns:
            Tuple of (run_id, experiment_id, model_hash)
        """
        try:
            # Create experiment if it doesn't exist
            experiment_id = await self.create_experiment(model_type)
            
            # Calculate model hash
            model_hash = self._calculate_file_hash(model_path)
            
            # Start MLflow run
            with mlflow.start_run(experiment_id=experiment_id) as run:
                run_id = run.info.run_id
                
                # Log model file as artifact
                mlflow.log_artifact(model_path, "model")
                
                # Log model info
                mlflow.log_param("model_name", model_name)
                mlflow.log_param("model_version", model_version)
                mlflow.log_param("model_type", model_type)
                mlflow.log_param("framework", framework)
                mlflow.log_param("model_hash", model_hash)
                
                # Log file size
                file_size = os.path.getsize(model_path)
                mlflow.log_metric("size_bytes", file_size)
                
                # Log additional metadata
                if metadata:
                    for key, value in metadata.items():
                        if isinstance(value, (int, float)):
                            mlflow.log_metric(key, value)
                        else:
                            mlflow.log_param(key, value)
                
                # Set tags
                if tags:
                    for key, value in tags.items():
                        mlflow.set_tag(key, value)
                
                # Set model version tag
                mlflow.set_tag("version", model_version)
                
                logger.info(f"Logged model {model_name} version {model_version} to MLflow run {run_id}")
                
                return run_id, experiment_id, model_hash
        except Exception as e:
            logger.error(f"Error logging model {model_name} to MLflow: {e}")
            raise
    
    async def get_model(self, run_id: str) -> Dict[str, Any]:
        """
        Get a model from MLflow.
        
        Args:
            run_id: MLflow run ID
            
        Returns:
            Model information
        """
        try:
            # Get run
            run = self.client.get_run(run_id)
            
            # Get model info
            model_info = {
                "run_id": run_id,
                "experiment_id": run.info.experiment_id,
                "model_name": run.data.params.get("model_name"),
                "model_version": run.data.params.get("model_version"),
                "model_type": run.data.params.get("model_type"),
                "framework": run.data.params.get("framework"),
                "model_hash": run.data.params.get("model_hash"),
                "size_bytes": run.data.metrics.get("size_bytes"),
                "tags": run.data.tags,
                "metrics": run.data.metrics,
                "params": run.data.params,
                "status": run.info.status,
                "start_time": datetime.fromtimestamp(run.info.start_time / 1000.0),
                "end_time": datetime.fromtimestamp(run.info.end_time / 1000.0) if run.info.end_time else None,
            }
            
            return model_info
        except MlflowException as e:
            if e.error_code == "RESOURCE_DOES_NOT_EXIST":
                logger.error(f"Model with run ID {run_id} not found")
                return None
            logger.error(f"Error getting model with run ID {run_id}: {e}")
            raise
        except Exception as e:
            logger.error(f"Error getting model with run ID {run_id}: {e}")
            raise
    
    async def download_model(self, run_id: str, output_path: str) -> str:
        """
        Download a model from MLflow.
        
        Args:
            run_id: MLflow run ID
            output_path: Path to save the model
            
        Returns:
            Path to the downloaded model
        """
        try:
            # Get artifact URI
            artifact_uri = self.client.get_run(run_id).info.artifact_uri
            
            # Download model
            model_path = os.path.join(artifact_uri, "model")
            local_path = mlflow.artifacts.download_artifacts(model_path, dst_path=output_path)
            
            logger.info(f"Downloaded model from run {run_id} to {local_path}")
            
            return local_path
        except Exception as e:
            logger.error(f"Error downloading model from run {run_id}: {e}")
            raise
    
    def _calculate_file_hash(self, file_path: str) -> str:
        """
        Calculate SHA-256 hash of a file.
        
        Args:
            file_path: Path to the file
            
        Returns:
            SHA-256 hash of the file
        """
        sha256_hash = hashlib.sha256()
        
        with open(file_path, "rb") as f:
            # Read and update hash in chunks
            for byte_block in iter(lambda: f.read(4096), b""):
                sha256_hash.update(byte_block)
        
        return sha256_hash.hexdigest()
