"""
Git LFS service for the Model Hub.

This module provides a service for versioning models with Git LFS.
"""

import os
import logging
import tempfile
import shutil
import subprocess
from typing import Dict, List, Any, Optional, Tuple, Union
from pathlib import Path
from datetime import datetime

# Setup logging
logger = logging.getLogger(__name__)

class GitLFSService:
    """Service for versioning models with Git LFS."""
    
    def __init__(self):
        """Initialize the Git LFS service."""
        # Get Git LFS configuration from environment variables
        self.repo_url = os.environ.get("GIT_LFS_REPO_URL", "")
        self.repo_branch = os.environ.get("GIT_LFS_REPO_BRANCH", "main")
        self.repo_username = os.environ.get("GIT_LFS_USERNAME", "")
        self.repo_password = os.environ.get("GIT_LFS_PASSWORD", "")
        self.repo_token = os.environ.get("GIT_LFS_TOKEN", "")
        self.repo_path = os.environ.get("GIT_LFS_REPO_PATH", "/tmp/model-hub-lfs")
        
        # Check if Git LFS is available
        self.git_lfs_available = self._check_git_lfs()
        
        # Initialize repository
        if self.git_lfs_available and self.repo_url:
            self._init_repo()
        
        logger.info(f"Git LFS service initialized with repo: {self.repo_url}")
    
    def _check_git_lfs(self) -> bool:
        """
        Check if Git LFS is available.
        
        Returns:
            True if Git LFS is available, False otherwise
        """
        try:
            result = subprocess.run(
                ["git", "lfs", "version"],
                capture_output=True,
                text=True,
                check=False,
            )
            
            if result.returncode == 0:
                logger.info(f"Git LFS is available: {result.stdout.strip()}")
                return True
            else:
                logger.warning(f"Git LFS command failed: {result.stderr.strip()}")
                return False
        except FileNotFoundError:
            logger.warning("Git LFS not found in PATH")
            return False
        except Exception as e:
            logger.warning(f"Error checking Git LFS: {e}")
            return False
    
    def _init_repo(self) -> bool:
        """
        Initialize the Git LFS repository.
        
        Returns:
            True if successful, False otherwise
        """
        try:
            # Check if repository directory exists
            repo_path = Path(self.repo_path)
            if repo_path.exists():
                # Pull latest changes
                return self._pull_repo()
            else:
                # Clone repository
                return self._clone_repo()
        except Exception as e:
            logger.error(f"Error initializing Git LFS repository: {e}")
            return False
    
    def _clone_repo(self) -> bool:
        """
        Clone the Git LFS repository.
        
        Returns:
            True if successful, False otherwise
        """
        try:
            # Create parent directory if it doesn't exist
            repo_path = Path(self.repo_path)
            repo_path.parent.mkdir(parents=True, exist_ok=True)
            
            # Create clone command
            command = ["git", "clone"]
            
            # Add authentication if provided
            repo_url = self.repo_url
            if self.repo_token:
                # Use token authentication
                repo_url = repo_url.replace("https://", f"https://{self.repo_token}@")
            elif self.repo_username and self.repo_password:
                # Use username/password authentication
                repo_url = repo_url.replace("https://", f"https://{self.repo_username}:{self.repo_password}@")
            
            # Add repository URL and path
            command.extend([repo_url, str(repo_path)])
            
            # Clone repository
            result = subprocess.run(
                command,
                capture_output=True,
                text=True,
                check=False,
            )
            
            if result.returncode == 0:
                logger.info(f"Cloned Git LFS repository to {repo_path}")
                
                # Initialize Git LFS
                self._init_git_lfs()
                
                return True
            else:
                logger.error(f"Error cloning Git LFS repository: {result.stderr.strip()}")
                return False
        except Exception as e:
            logger.error(f"Error cloning Git LFS repository: {e}")
            return False
    
    def _pull_repo(self) -> bool:
        """
        Pull the latest changes from the Git LFS repository.
        
        Returns:
            True if successful, False otherwise
        """
        try:
            # Change to repository directory
            cwd = os.getcwd()
            os.chdir(self.repo_path)
            
            try:
                # Pull latest changes
                result = subprocess.run(
                    ["git", "pull", "origin", self.repo_branch],
                    capture_output=True,
                    text=True,
                    check=False,
                )
                
                if result.returncode == 0:
                    logger.info(f"Pulled latest changes from Git LFS repository")
                    return True
                else:
                    logger.error(f"Error pulling Git LFS repository: {result.stderr.strip()}")
                    return False
            finally:
                # Change back to original directory
                os.chdir(cwd)
        except Exception as e:
            logger.error(f"Error pulling Git LFS repository: {e}")
            return False
    
    def _init_git_lfs(self) -> bool:
        """
        Initialize Git LFS in the repository.
        
        Returns:
            True if successful, False otherwise
        """
        try:
            # Change to repository directory
            cwd = os.getcwd()
            os.chdir(self.repo_path)
            
            try:
                # Initialize Git LFS
                result = subprocess.run(
                    ["git", "lfs", "install"],
                    capture_output=True,
                    text=True,
                    check=False,
                )
                
                if result.returncode == 0:
                    logger.info(f"Initialized Git LFS in repository")
                    
                    # Track model files
                    result = subprocess.run(
                        ["git", "lfs", "track", "*.pt", "*.pth", "*.onnx", "*.tflite", "*.h5"],
                        capture_output=True,
                        text=True,
                        check=False,
                    )
                    
                    if result.returncode == 0:
                        logger.info(f"Tracked model files with Git LFS")
                        
                        # Commit .gitattributes
                        result = subprocess.run(
                            ["git", "add", ".gitattributes"],
                            capture_output=True,
                            text=True,
                            check=False,
                        )
                        
                        if result.returncode == 0:
                            result = subprocess.run(
                                ["git", "commit", "-m", "Initialize Git LFS tracking"],
                                capture_output=True,
                                text=True,
                                check=False,
                            )
                            
                            if result.returncode == 0:
                                logger.info(f"Committed Git LFS tracking configuration")
                                
                                # Push changes
                                result = subprocess.run(
                                    ["git", "push", "origin", self.repo_branch],
                                    capture_output=True,
                                    text=True,
                                    check=False,
                                )
                                
                                if result.returncode == 0:
                                    logger.info(f"Pushed Git LFS tracking configuration")
                                    return True
                                else:
                                    logger.error(f"Error pushing Git LFS tracking configuration: {result.stderr.strip()}")
                            else:
                                logger.error(f"Error committing Git LFS tracking configuration: {result.stderr.strip()}")
                        else:
                            logger.error(f"Error adding .gitattributes: {result.stderr.strip()}")
                    else:
                        logger.error(f"Error tracking model files with Git LFS: {result.stderr.strip()}")
                else:
                    logger.error(f"Error initializing Git LFS: {result.stderr.strip()}")
                
                return False
            finally:
                # Change back to original directory
                os.chdir(cwd)
        except Exception as e:
            logger.error(f"Error initializing Git LFS: {e}")
            return False
    
    async def store_model(
        self,
        model_path: str,
        model_name: str,
        model_version: str,
        model_type: str,
        commit_message: Optional[str] = None,
    ) -> Tuple[bool, str]:
        """
        Store a model in the Git LFS repository.
        
        Args:
            model_path: Path to the model file
            model_name: Name of the model
            model_version: Version of the model
            model_type: Type of the model
            commit_message: Commit message
            
        Returns:
            Tuple of (success, model_url)
        """
        if not self.git_lfs_available or not self.repo_url:
            logger.warning("Git LFS is not available or repository URL is not configured")
            return False, ""
        
        try:
            # Pull latest changes
            if not self._pull_repo():
                logger.error("Error pulling latest changes from Git LFS repository")
                return False, ""
            
            # Create model directory
            model_dir = os.path.join(self.repo_path, "models", model_type, model_name)
            os.makedirs(model_dir, exist_ok=True)
            
            # Create model file path
            model_file = os.path.join(model_dir, f"{model_name}-{model_version}{os.path.splitext(model_path)[1]}")
            
            # Copy model file
            shutil.copy(model_path, model_file)
            
            # Create metadata file
            metadata_file = os.path.join(model_dir, f"{model_name}-{model_version}.json")
            with open(metadata_file, "w") as f:
                import json
                json.dump({
                    "name": model_name,
                    "version": model_version,
                    "type": model_type,
                    "created_at": datetime.utcnow().isoformat(),
                }, f, indent=2)
            
            # Change to repository directory
            cwd = os.getcwd()
            os.chdir(self.repo_path)
            
            try:
                # Add files
                result = subprocess.run(
                    ["git", "add", model_file, metadata_file],
                    capture_output=True,
                    text=True,
                    check=False,
                )
                
                if result.returncode != 0:
                    logger.error(f"Error adding files to Git LFS repository: {result.stderr.strip()}")
                    return False, ""
                
                # Commit changes
                if not commit_message:
                    commit_message = f"Add {model_name} version {model_version}"
                
                result = subprocess.run(
                    ["git", "commit", "-m", commit_message],
                    capture_output=True,
                    text=True,
                    check=False,
                )
                
                if result.returncode != 0:
                    logger.error(f"Error committing files to Git LFS repository: {result.stderr.strip()}")
                    return False, ""
                
                # Push changes
                result = subprocess.run(
                    ["git", "push", "origin", self.repo_branch],
                    capture_output=True,
                    text=True,
                    check=False,
                )
                
                if result.returncode != 0:
                    logger.error(f"Error pushing files to Git LFS repository: {result.stderr.strip()}")
                    return False, ""
                
                # Get model URL
                model_url = f"{self.repo_url.rstrip('/')}/blob/{self.repo_branch}/models/{model_type}/{model_name}/{os.path.basename(model_file)}"
                
                logger.info(f"Stored model {model_name} version {model_version} in Git LFS repository")
                
                return True, model_url
            finally:
                # Change back to original directory
                os.chdir(cwd)
        except Exception as e:
            logger.error(f"Error storing model in Git LFS repository: {e}")
            return False, ""
    
    async def get_model(
        self,
        model_name: str,
        model_version: str,
        model_type: str,
        output_path: str,
    ) -> bool:
        """
        Get a model from the Git LFS repository.
        
        Args:
            model_name: Name of the model
            model_version: Version of the model
            model_type: Type of the model
            output_path: Path to save the model
            
        Returns:
            True if successful, False otherwise
        """
        if not self.git_lfs_available or not self.repo_url:
            logger.warning("Git LFS is not available or repository URL is not configured")
            return False
        
        try:
            # Pull latest changes
            if not self._pull_repo():
                logger.error("Error pulling latest changes from Git LFS repository")
                return False
            
            # Find model file
            model_dir = os.path.join(self.repo_path, "models", model_type, model_name)
            if not os.path.exists(model_dir):
                logger.error(f"Model directory not found: {model_dir}")
                return False
            
            # Find model file
            model_files = [f for f in os.listdir(model_dir) if f.startswith(f"{model_name}-{model_version}") and not f.endswith(".json")]
            if not model_files:
                logger.error(f"Model file not found for {model_name} version {model_version}")
                return False
            
            # Get model file
            model_file = os.path.join(model_dir, model_files[0])
            
            # Copy model file
            shutil.copy(model_file, output_path)
            
            logger.info(f"Got model {model_name} version {model_version} from Git LFS repository")
            
            return True
        except Exception as e:
            logger.error(f"Error getting model from Git LFS repository: {e}")
            return False
    
    async def list_models(
        self,
        model_type: Optional[str] = None,
        model_name: Optional[str] = None,
    ) -> List[Dict[str, Any]]:
        """
        List models in the Git LFS repository.
        
        Args:
            model_type: Filter by model type
            model_name: Filter by model name
            
        Returns:
            List of models
        """
        if not self.git_lfs_available or not self.repo_url:
            logger.warning("Git LFS is not available or repository URL is not configured")
            return []
        
        try:
            # Pull latest changes
            if not self._pull_repo():
                logger.error("Error pulling latest changes from Git LFS repository")
                return []
            
            # Get models directory
            models_dir = os.path.join(self.repo_path, "models")
            if not os.path.exists(models_dir):
                logger.error(f"Models directory not found: {models_dir}")
                return []
            
            # List models
            models = []
            
            # Get model types
            model_types = os.listdir(models_dir) if os.path.exists(models_dir) else []
            if model_type:
                model_types = [t for t in model_types if t == model_type]
            
            # Iterate over model types
            for t in model_types:
                type_dir = os.path.join(models_dir, t)
                if not os.path.isdir(type_dir):
                    continue
                
                # Get model names
                model_names = os.listdir(type_dir)
                if model_name:
                    model_names = [n for n in model_names if n == model_name]
                
                # Iterate over model names
                for n in model_names:
                    name_dir = os.path.join(type_dir, n)
                    if not os.path.isdir(name_dir):
                        continue
                    
                    # Get model versions
                    model_files = [f for f in os.listdir(name_dir) if f.endswith(".json")]
                    
                    # Iterate over model versions
                    for f in model_files:
                        # Parse version from filename
                        version = f.replace(f"{n}-", "").replace(".json", "")
                        
                        # Read metadata
                        metadata_file = os.path.join(name_dir, f)
                        with open(metadata_file, "r") as mf:
                            import json
                            metadata = json.load(mf)
                        
                        # Add model to list
                        models.append({
                            "name": n,
                            "version": version,
                            "type": t,
                            "created_at": metadata.get("created_at"),
                            "url": f"{self.repo_url.rstrip('/')}/blob/{self.repo_branch}/models/{t}/{n}/{n}-{version}.json",
                        })
            
            return models
        except Exception as e:
            logger.error(f"Error listing models in Git LFS repository: {e}")
            return []
