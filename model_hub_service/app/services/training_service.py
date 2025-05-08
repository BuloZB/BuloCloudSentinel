"""
Training service for the Model Hub.

This module provides a service for distributed training of models.
"""

import os
import logging
import json
import tempfile
import shutil
import subprocess
from typing import Dict, List, Any, Optional, Tuple, Union
from pathlib import Path
from datetime import datetime

import mlflow

# Setup logging
logger = logging.getLogger(__name__)

class TrainingService:
    """Service for distributed training of models."""
    
    def __init__(self):
        """Initialize the training service."""
        # Get MLflow tracking URI from environment variable
        self.tracking_uri = os.environ.get("MLFLOW_TRACKING_URI", "http://localhost:5000")
        
        # Set MLflow tracking URI
        mlflow.set_tracking_uri(self.tracking_uri)
        
        # Create MLflow client
        self.client = mlflow.tracking.MlflowClient()
        
        # Get training configuration from environment variables
        self.config = {
            "num_workers": int(os.environ.get("TRAINING_NUM_WORKERS", "2")),
            "backend": os.environ.get("TRAINING_BACKEND", "pytorch"),
            "distributed_backend": os.environ.get("TRAINING_DISTRIBUTED_BACKEND", "nccl"),
            "base_port": int(os.environ.get("TRAINING_BASE_PORT", "29500")),
            "max_epochs": int(os.environ.get("TRAINING_MAX_EPOCHS", "100")),
            "early_stopping_patience": int(os.environ.get("TRAINING_EARLY_STOPPING_PATIENCE", "10")),
            "batch_size": int(os.environ.get("TRAINING_BATCH_SIZE", "32")),
            "learning_rate": float(os.environ.get("TRAINING_LEARNING_RATE", "0.001")),
            "weight_decay": float(os.environ.get("TRAINING_WEIGHT_DECAY", "0.0001")),
            "optimizer": os.environ.get("TRAINING_OPTIMIZER", "adam"),
            "scheduler": os.environ.get("TRAINING_SCHEDULER", "cosine"),
            "mixed_precision": os.environ.get("TRAINING_MIXED_PRECISION", "true").lower() == "true",
            "gradient_accumulation_steps": int(os.environ.get("TRAINING_GRADIENT_ACCUMULATION_STEPS", "1")),
            "checkpoint_interval": int(os.environ.get("TRAINING_CHECKPOINT_INTERVAL", "10")),
        }
        
        logger.info(f"Training service initialized with config: {self.config}")
    
    async def create_training_job(
        self,
        name: str,
        model_type: str,
        dataset_path: str,
        config: Optional[Dict[str, Any]] = None,
    ) -> Dict[str, Any]:
        """
        Create a new training job.
        
        Args:
            name: Name of the training job
            model_type: Type of the model to train
            dataset_path: Path to the dataset
            config: Training configuration
            
        Returns:
            Training job information
        """
        try:
            # Merge configuration
            job_config = self.config.copy()
            if config:
                job_config.update(config)
            
            # Create MLflow experiment
            experiment_name = f"training_{name}"
            experiment = self.client.get_experiment_by_name(experiment_name)
            if experiment:
                experiment_id = experiment.experiment_id
            else:
                experiment_id = self.client.create_experiment(experiment_name)
            
            # Create training job
            job_id = f"training_{name}_{datetime.utcnow().strftime('%Y%m%d_%H%M%S')}"
            
            # Create training job information
            job_info = {
                "id": job_id,
                "name": name,
                "model_type": model_type,
                "dataset_path": dataset_path,
                "config": job_config,
                "experiment_id": experiment_id,
                "status": "created",
                "created_at": datetime.utcnow().isoformat(),
                "updated_at": datetime.utcnow().isoformat(),
            }
            
            logger.info(f"Created training job: {job_id}")
            
            return job_info
        except Exception as e:
            logger.error(f"Error creating training job: {e}")
            raise
    
    async def start_training_job(self, job_info: Dict[str, Any]) -> Dict[str, Any]:
        """
        Start a training job.
        
        Args:
            job_info: Training job information
            
        Returns:
            Updated training job information
        """
        try:
            # Update job status
            job_info["status"] = "running"
            job_info["updated_at"] = datetime.utcnow().isoformat()
            job_info["started_at"] = datetime.utcnow().isoformat()
            
            # Get job configuration
            job_id = job_info["id"]
            model_type = job_info["model_type"]
            dataset_path = job_info["dataset_path"]
            config = job_info["config"]
            experiment_id = job_info["experiment_id"]
            
            # Create temporary directory for training
            with tempfile.TemporaryDirectory() as temp_dir:
                # Create training script
                script_path = os.path.join(temp_dir, "train.py")
                await self._create_training_script(script_path, model_type, config)
                
                # Create distributed training command
                if config["backend"] == "pytorch":
                    command = await self._create_pytorch_distributed_command(
                        script_path, dataset_path, experiment_id, job_id, config
                    )
                elif config["backend"] == "tensorflow":
                    command = await self._create_tensorflow_distributed_command(
                        script_path, dataset_path, experiment_id, job_id, config
                    )
                else:
                    raise ValueError(f"Unsupported backend: {config['backend']}")
                
                # Start training process
                logger.info(f"Starting training job {job_id} with command: {command}")
                process = subprocess.Popen(
                    command,
                    shell=True,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    text=True,
                )
                
                # Store process ID
                job_info["process_id"] = process.pid
                
                # Wait for process to complete
                stdout, stderr = process.communicate()
                
                # Update job status
                if process.returncode == 0:
                    job_info["status"] = "completed"
                    logger.info(f"Training job {job_id} completed successfully")
                else:
                    job_info["status"] = "failed"
                    job_info["error"] = stderr
                    logger.error(f"Training job {job_id} failed: {stderr}")
                
                job_info["updated_at"] = datetime.utcnow().isoformat()
                job_info["completed_at"] = datetime.utcnow().isoformat()
                
                # Get MLflow run
                runs = self.client.search_runs(
                    experiment_ids=[experiment_id],
                    filter_string=f"tags.job_id = '{job_id}'",
                )
                
                if runs:
                    run = runs[0]
                    job_info["run_id"] = run.info.run_id
                    job_info["metrics"] = run.data.metrics
                    job_info["params"] = run.data.params
                    job_info["tags"] = run.data.tags
                    
                    # Get model artifact URI
                    job_info["model_uri"] = f"runs:/{run.info.run_id}/model"
                
                return job_info
        except Exception as e:
            logger.error(f"Error starting training job: {e}")
            job_info["status"] = "failed"
            job_info["error"] = str(e)
            job_info["updated_at"] = datetime.utcnow().isoformat()
            return job_info
    
    async def _create_training_script(
        self,
        script_path: str,
        model_type: str,
        config: Dict[str, Any],
    ) -> None:
        """
        Create a training script for distributed training.
        
        Args:
            script_path: Path to save the training script
            model_type: Type of the model to train
            config: Training configuration
        """
        try:
            # Create training script based on model type and backend
            if model_type.lower() in ["yolo", "yolov5", "yolov8", "yolov10"]:
                if config["backend"] == "pytorch":
                    script_content = self._create_pytorch_yolo_training_script(config)
                else:
                    raise ValueError(f"Unsupported backend for YOLO: {config['backend']}")
            elif model_type.lower() in ["sam", "segment-anything"]:
                if config["backend"] == "pytorch":
                    script_content = self._create_pytorch_sam_training_script(config)
                else:
                    raise ValueError(f"Unsupported backend for SAM: {config['backend']}")
            elif model_type.lower() in ["super-gradients", "yolo-nas"]:
                if config["backend"] == "pytorch":
                    script_content = self._create_pytorch_super_gradients_training_script(config)
                else:
                    raise ValueError(f"Unsupported backend for Super-Gradients: {config['backend']}")
            else:
                raise ValueError(f"Unsupported model type: {model_type}")
            
            # Write script to file
            with open(script_path, "w") as f:
                f.write(script_content)
            
            # Make script executable
            os.chmod(script_path, 0o755)
            
            logger.info(f"Created training script: {script_path}")
        except Exception as e:
            logger.error(f"Error creating training script: {e}")
            raise
    
    def _create_pytorch_yolo_training_script(self, config: Dict[str, Any]) -> str:
        """
        Create a PyTorch training script for YOLO models.
        
        Args:
            config: Training configuration
            
        Returns:
            Training script content
        """
        # This is a placeholder for the actual training script
        # In a real implementation, you would create a more sophisticated script
        return """#!/usr/bin/env python3
import os
import sys
import argparse
import torch
import torch.distributed as dist
import torch.multiprocessing as mp
from torch.nn.parallel import DistributedDataParallel as DDP
import mlflow

def setup(rank, world_size, backend, base_port):
    os.environ['MASTER_ADDR'] = 'localhost'
    os.environ['MASTER_PORT'] = str(base_port)
    dist.init_process_group(backend, rank=rank, world_size=world_size)

def cleanup():
    dist.destroy_process_group()

def train(rank, world_size, args):
    setup(rank, world_size, args.distributed_backend, args.base_port)
    
    # Set device
    device = torch.device(f"cuda:{rank}" if torch.cuda.is_available() else "cpu")
    
    # Start MLflow run
    with mlflow.start_run(experiment_id=args.experiment_id, run_name=args.job_id) as run:
        # Log parameters
        mlflow.log_params({
            "rank": rank,
            "world_size": world_size,
            "batch_size": args.batch_size,
            "learning_rate": args.learning_rate,
            "max_epochs": args.max_epochs,
            "model_type": "yolo",
            "dataset_path": args.dataset_path,
        })
        
        # Set tag for job ID
        mlflow.set_tag("job_id", args.job_id)
        
        # Load YOLO model
        try:
            from ultralytics import YOLO
            model = YOLO("yolov8n.pt")
        except ImportError:
            print("Ultralytics not available, using PyTorch")
            import torch
            model = torch.hub.load("ultralytics/yolov5", "yolov5s")
        
        # Move model to device
        model = model.to(device)
        
        # Wrap model with DDP
        if world_size > 1:
            model = DDP(model, device_ids=[rank])
        
        # Train model
        results = model.train(
            data=args.dataset_path,
            epochs=args.max_epochs,
            batch=args.batch_size,
            device=device,
        )
        
        # Log metrics
        mlflow.log_metrics({
            "mAP": results.results_dict.get("metrics/mAP50-95(B)", 0),
            "precision": results.results_dict.get("metrics/precision(B)", 0),
            "recall": results.results_dict.get("metrics/recall(B)", 0),
        })
        
        # Save model
        if rank == 0:
            model_path = f"model.pt"
            model.save(model_path)
            mlflow.log_artifact(model_path, "model")
    
    cleanup()

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset-path", type=str, required=True)
    parser.add_argument("--experiment-id", type=str, required=True)
    parser.add_argument("--job-id", type=str, required=True)
    parser.add_argument("--num-workers", type=int, default=2)
    parser.add_argument("--distributed-backend", type=str, default="nccl")
    parser.add_argument("--base-port", type=int, default=29500)
    parser.add_argument("--batch-size", type=int, default=32)
    parser.add_argument("--learning-rate", type=float, default=0.001)
    parser.add_argument("--max-epochs", type=int, default=100)
    args = parser.parse_args()
    
    # Set MLflow tracking URI
    mlflow.set_tracking_uri(os.environ.get("MLFLOW_TRACKING_URI", "http://localhost:5000"))
    
    # Get world size
    world_size = args.num_workers
    
    # Spawn processes
    mp.spawn(
        train,
        args=(world_size, args),
        nprocs=world_size,
        join=True,
    )

if __name__ == "__main__":
    main()
"""
    
    def _create_pytorch_sam_training_script(self, config: Dict[str, Any]) -> str:
        """
        Create a PyTorch training script for SAM models.
        
        Args:
            config: Training configuration
            
        Returns:
            Training script content
        """
        # This is a placeholder for the actual training script
        # In a real implementation, you would create a more sophisticated script
        return """#!/usr/bin/env python3
import os
import sys
import argparse
import torch
import torch.distributed as dist
import torch.multiprocessing as mp
from torch.nn.parallel import DistributedDataParallel as DDP
import mlflow

def setup(rank, world_size, backend, base_port):
    os.environ['MASTER_ADDR'] = 'localhost'
    os.environ['MASTER_PORT'] = str(base_port)
    dist.init_process_group(backend, rank=rank, world_size=world_size)

def cleanup():
    dist.destroy_process_group()

def train(rank, world_size, args):
    setup(rank, world_size, args.distributed_backend, args.base_port)
    
    # Set device
    device = torch.device(f"cuda:{rank}" if torch.cuda.is_available() else "cpu")
    
    # Start MLflow run
    with mlflow.start_run(experiment_id=args.experiment_id, run_name=args.job_id) as run:
        # Log parameters
        mlflow.log_params({
            "rank": rank,
            "world_size": world_size,
            "batch_size": args.batch_size,
            "learning_rate": args.learning_rate,
            "max_epochs": args.max_epochs,
            "model_type": "sam",
            "dataset_path": args.dataset_path,
        })
        
        # Set tag for job ID
        mlflow.set_tag("job_id", args.job_id)
        
        # Load SAM model
        try:
            from segment_anything import sam_model_registry
            model = sam_model_registry["vit_b"](checkpoint="sam_vit_b_01ec64.pth")
        except ImportError:
            print("Segment Anything not available, using PyTorch")
            import torch
            model = torch.hub.load("facebookresearch/segment-anything", "sam_vit_b")
        
        # Move model to device
        model = model.to(device)
        
        # Wrap model with DDP
        if world_size > 1:
            model = DDP(model, device_ids=[rank])
        
        # Train model (placeholder)
        # In a real implementation, you would implement the training loop
        
        # Log metrics (placeholder)
        mlflow.log_metrics({
            "loss": 0.1,
            "accuracy": 0.9,
        })
        
        # Save model
        if rank == 0:
            model_path = f"model.pt"
            torch.save(model.state_dict(), model_path)
            mlflow.log_artifact(model_path, "model")
    
    cleanup()

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset-path", type=str, required=True)
    parser.add_argument("--experiment-id", type=str, required=True)
    parser.add_argument("--job-id", type=str, required=True)
    parser.add_argument("--num-workers", type=int, default=2)
    parser.add_argument("--distributed-backend", type=str, default="nccl")
    parser.add_argument("--base-port", type=int, default=29500)
    parser.add_argument("--batch-size", type=int, default=32)
    parser.add_argument("--learning-rate", type=float, default=0.001)
    parser.add_argument("--max-epochs", type=int, default=100)
    args = parser.parse_args()
    
    # Set MLflow tracking URI
    mlflow.set_tracking_uri(os.environ.get("MLFLOW_TRACKING_URI", "http://localhost:5000"))
    
    # Get world size
    world_size = args.num_workers
    
    # Spawn processes
    mp.spawn(
        train,
        args=(world_size, args),
        nprocs=world_size,
        join=True,
    )

if __name__ == "__main__":
    main()
"""
    
    def _create_pytorch_super_gradients_training_script(self, config: Dict[str, Any]) -> str:
        """
        Create a PyTorch training script for Super-Gradients models.
        
        Args:
            config: Training configuration
            
        Returns:
            Training script content
        """
        # This is a placeholder for the actual training script
        # In a real implementation, you would create a more sophisticated script
        return """#!/usr/bin/env python3
import os
import sys
import argparse
import torch
import torch.distributed as dist
import torch.multiprocessing as mp
import mlflow

def setup(rank, world_size, backend, base_port):
    os.environ['MASTER_ADDR'] = 'localhost'
    os.environ['MASTER_PORT'] = str(base_port)
    dist.init_process_group(backend, rank=rank, world_size=world_size)

def cleanup():
    dist.destroy_process_group()

def train(rank, world_size, args):
    setup(rank, world_size, args.distributed_backend, args.base_port)
    
    # Set device
    device = torch.device(f"cuda:{rank}" if torch.cuda.is_available() else "cpu")
    
    # Start MLflow run
    with mlflow.start_run(experiment_id=args.experiment_id, run_name=args.job_id) as run:
        # Log parameters
        mlflow.log_params({
            "rank": rank,
            "world_size": world_size,
            "batch_size": args.batch_size,
            "learning_rate": args.learning_rate,
            "max_epochs": args.max_epochs,
            "model_type": "super-gradients",
            "dataset_path": args.dataset_path,
        })
        
        # Set tag for job ID
        mlflow.set_tag("job_id", args.job_id)
        
        # Load Super-Gradients model
        try:
            from super_gradients.training import models
            model = models.get("yolo_nas_l", pretrained_weights="coco")
        except ImportError:
            print("Super-Gradients not available, using PyTorch")
            import torch
            model = torch.hub.load("Deci-AI/super-gradients", "yolo_nas_l")
        
        # Move model to device
        model = model.to(device)
        
        # Train model (placeholder)
        # In a real implementation, you would implement the training loop
        
        # Log metrics (placeholder)
        mlflow.log_metrics({
            "mAP": 0.8,
            "precision": 0.9,
            "recall": 0.85,
        })
        
        # Save model
        if rank == 0:
            model_path = f"model.pt"
            torch.save(model.state_dict(), model_path)
            mlflow.log_artifact(model_path, "model")
    
    cleanup()

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset-path", type=str, required=True)
    parser.add_argument("--experiment-id", type=str, required=True)
    parser.add_argument("--job-id", type=str, required=True)
    parser.add_argument("--num-workers", type=int, default=2)
    parser.add_argument("--distributed-backend", type=str, default="nccl")
    parser.add_argument("--base-port", type=int, default=29500)
    parser.add_argument("--batch-size", type=int, default=32)
    parser.add_argument("--learning-rate", type=float, default=0.001)
    parser.add_argument("--max-epochs", type=int, default=100)
    args = parser.parse_args()
    
    # Set MLflow tracking URI
    mlflow.set_tracking_uri(os.environ.get("MLFLOW_TRACKING_URI", "http://localhost:5000"))
    
    # Get world size
    world_size = args.num_workers
    
    # Spawn processes
    mp.spawn(
        train,
        args=(world_size, args),
        nprocs=world_size,
        join=True,
    )

if __name__ == "__main__":
    main()
"""
    
    async def _create_pytorch_distributed_command(
        self,
        script_path: str,
        dataset_path: str,
        experiment_id: str,
        job_id: str,
        config: Dict[str, Any],
    ) -> str:
        """
        Create a command for distributed PyTorch training.
        
        Args:
            script_path: Path to the training script
            dataset_path: Path to the dataset
            experiment_id: MLflow experiment ID
            job_id: Training job ID
            config: Training configuration
            
        Returns:
            Command for distributed training
        """
        command = f"{script_path} \
            --dataset-path {dataset_path} \
            --experiment-id {experiment_id} \
            --job-id {job_id} \
            --num-workers {config['num_workers']} \
            --distributed-backend {config['distributed_backend']} \
            --base-port {config['base_port']} \
            --batch-size {config['batch_size']} \
            --learning-rate {config['learning_rate']} \
            --max-epochs {config['max_epochs']}"
        
        return command
    
    async def _create_tensorflow_distributed_command(
        self,
        script_path: str,
        dataset_path: str,
        experiment_id: str,
        job_id: str,
        config: Dict[str, Any],
    ) -> str:
        """
        Create a command for distributed TensorFlow training.
        
        Args:
            script_path: Path to the training script
            dataset_path: Path to the dataset
            experiment_id: MLflow experiment ID
            job_id: Training job ID
            config: Training configuration
            
        Returns:
            Command for distributed training
        """
        # This is a placeholder for the actual command
        # In a real implementation, you would create a more sophisticated command
        command = f"python {script_path} \
            --dataset-path {dataset_path} \
            --experiment-id {experiment_id} \
            --job-id {job_id} \
            --num-workers {config['num_workers']} \
            --batch-size {config['batch_size']} \
            --learning-rate {config['learning_rate']} \
            --max-epochs {config['max_epochs']}"
        
        return command
    
    async def get_training_job(self, job_id: str) -> Optional[Dict[str, Any]]:
        """
        Get information about a training job.
        
        Args:
            job_id: Training job ID
            
        Returns:
            Training job information or None if not found
        """
        # This is a placeholder for the actual implementation
        # In a real implementation, you would retrieve the job from a database
        return None
    
    async def list_training_jobs(
        self,
        status: Optional[str] = None,
        limit: int = 100,
        offset: int = 0,
    ) -> List[Dict[str, Any]]:
        """
        List training jobs.
        
        Args:
            status: Filter by job status
            limit: Maximum number of jobs to return
            offset: Offset for pagination
            
        Returns:
            List of training jobs
        """
        # This is a placeholder for the actual implementation
        # In a real implementation, you would retrieve jobs from a database
        return []
    
    async def cancel_training_job(self, job_id: str) -> Dict[str, Any]:
        """
        Cancel a training job.
        
        Args:
            job_id: Training job ID
            
        Returns:
            Updated training job information
        """
        # This is a placeholder for the actual implementation
        # In a real implementation, you would cancel the job and update its status
        return {}
