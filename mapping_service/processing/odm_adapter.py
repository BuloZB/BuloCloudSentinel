"""
OpenDroneMap adapter for the Mapping Service.
"""

import asyncio
import json
import logging
import os
import shutil
import tempfile
from typing import Any, Dict, List, Optional, Tuple

import aiohttp

from mapping_service.core.config import settings

logger = logging.getLogger(__name__)


class OpenDroneMapAdapter:
    """Adapter for OpenDroneMap processing."""
    
    def __init__(self):
        """Initialize the OpenDroneMap adapter."""
        self.api_url = settings.ODM_API_URL
        self.temp_dir = settings.TEMP_DIRECTORY
        os.makedirs(self.temp_dir, exist_ok=True)
    
    async def create_project(self, name: str) -> Tuple[bool, str, Optional[str]]:
        """
        Create a new project in OpenDroneMap.
        
        Args:
            name: Project name
            
        Returns:
            Tuple[bool, str, Optional[str]]: (success, project_id, error_message)
        """
        try:
            async with aiohttp.ClientSession() as session:
                async with session.post(
                    f"{self.api_url}/projects/new",
                    json={"name": name}
                ) as response:
                    if response.status != 200:
                        error_text = await response.text()
                        return False, "", f"Failed to create project: {error_text}"
                    
                    data = await response.json()
                    return True, data["id"], None
        except Exception as e:
            logger.error(f"Error creating project: {str(e)}")
            return False, "", f"Error creating project: {str(e)}"
    
    async def upload_images(self, project_id: str, image_paths: List[str]) -> Tuple[bool, Optional[str]]:
        """
        Upload images to an OpenDroneMap project.
        
        Args:
            project_id: Project ID
            image_paths: List of image paths
            
        Returns:
            Tuple[bool, Optional[str]]: (success, error_message)
        """
        try:
            # Prepare form data with images
            data = aiohttp.FormData()
            for image_path in image_paths:
                data.add_field(
                    "images",
                    open(image_path, "rb"),
                    filename=os.path.basename(image_path),
                    content_type="image/jpeg"
                )
            
            async with aiohttp.ClientSession() as session:
                async with session.post(
                    f"{self.api_url}/projects/{project_id}/upload",
                    data=data
                ) as response:
                    if response.status != 200:
                        error_text = await response.text()
                        return False, f"Failed to upload images: {error_text}"
                    
                    return True, None
        except Exception as e:
            logger.error(f"Error uploading images: {str(e)}")
            return False, f"Error uploading images: {str(e)}"
    
    async def process_project(self, project_id: str, options: Dict[str, Any]) -> Tuple[bool, Optional[str]]:
        """
        Process an OpenDroneMap project.
        
        Args:
            project_id: Project ID
            options: Processing options
            
        Returns:
            Tuple[bool, Optional[str]]: (success, error_message)
        """
        try:
            # Set default options if not provided
            default_options = {
                "dsm": True,
                "dtm": True,
                "orthophoto-resolution": 5,
                "mesh-size": 200000,
                "use-3dmesh": True,
                "pc-quality": "medium",
                "crop": 0,
                "feature-quality": "high",
                "min-num-features": 8000,
                "matcher-neighbors": 8,
                "texturing-data-term": "gmi",
                "texturing-outlier-removal-type": "gauss_damping",
                "texturing-skip-local-seam-leveling": False,
                "texturing-skip-global-seam-leveling": False,
                "texturing-tone-mapping": "none",
                "dem-resolution": 5,
                "orthophoto": True,
                "verbose": True
            }
            
            # Merge default options with provided options
            merged_options = {**default_options, **options}
            
            async with aiohttp.ClientSession() as session:
                async with session.post(
                    f"{self.api_url}/projects/{project_id}/task",
                    json=merged_options
                ) as response:
                    if response.status != 200:
                        error_text = await response.text()
                        return False, f"Failed to start processing: {error_text}"
                    
                    return True, None
        except Exception as e:
            logger.error(f"Error starting processing: {str(e)}")
            return False, f"Error starting processing: {str(e)}"
    
    async def get_task_info(self, project_id: str) -> Tuple[bool, Dict[str, Any], Optional[str]]:
        """
        Get information about a processing task.
        
        Args:
            project_id: Project ID
            
        Returns:
            Tuple[bool, Dict[str, Any], Optional[str]]: (success, task_info, error_message)
        """
        try:
            async with aiohttp.ClientSession() as session:
                async with session.get(
                    f"{self.api_url}/projects/{project_id}/task"
                ) as response:
                    if response.status != 200:
                        error_text = await response.text()
                        return False, {}, f"Failed to get task info: {error_text}"
                    
                    data = await response.json()
                    return True, data, None
        except Exception as e:
            logger.error(f"Error getting task info: {str(e)}")
            return False, {}, f"Error getting task info: {str(e)}"
    
    async def download_result(self, project_id: str, asset_type: str, output_path: str) -> Tuple[bool, Optional[str]]:
        """
        Download a processing result.
        
        Args:
            project_id: Project ID
            asset_type: Asset type (orthophoto, dsm, dtm, model, etc.)
            output_path: Output path
            
        Returns:
            Tuple[bool, Optional[str]]: (success, error_message)
        """
        try:
            # Create output directory if it doesn't exist
            os.makedirs(os.path.dirname(output_path), exist_ok=True)
            
            async with aiohttp.ClientSession() as session:
                async with session.get(
                    f"{self.api_url}/projects/{project_id}/task/download/{asset_type}"
                ) as response:
                    if response.status != 200:
                        error_text = await response.text()
                        return False, f"Failed to download {asset_type}: {error_text}"
                    
                    # Save file
                    with open(output_path, "wb") as f:
                        while True:
                            chunk = await response.content.read(1024 * 1024)  # 1MB chunks
                            if not chunk:
                                break
                            f.write(chunk)
                    
                    return True, None
        except Exception as e:
            logger.error(f"Error downloading {asset_type}: {str(e)}")
            return False, f"Error downloading {asset_type}: {str(e)}"
    
    async def process_images(self, project_name: str, image_paths: List[str], options: Dict[str, Any]) -> Tuple[bool, Dict[str, str], Optional[str]]:
        """
        Process images using OpenDroneMap.
        
        Args:
            project_name: Project name
            image_paths: List of image paths
            options: Processing options
            
        Returns:
            Tuple[bool, Dict[str, str], Optional[str]]: (success, output_paths, error_message)
        """
        # Create project
        success, project_id, error = await self.create_project(project_name)
        if not success:
            return False, {}, error
        
        # Upload images
        success, error = await self.upload_images(project_id, image_paths)
        if not success:
            return False, {}, error
        
        # Process project
        success, error = await self.process_project(project_id, options)
        if not success:
            return False, {}, error
        
        # Wait for processing to complete
        while True:
            success, task_info, error = await self.get_task_info(project_id)
            if not success:
                return False, {}, error
            
            if task_info.get("status") == "completed":
                break
            elif task_info.get("status") == "failed":
                return False, {}, f"Processing failed: {task_info.get('error', 'Unknown error')}"
            
            # Wait before checking again
            await asyncio.sleep(10)
        
        # Create temporary directory for outputs
        output_dir = tempfile.mkdtemp(dir=self.temp_dir)
        
        # Download results
        outputs = {}
        for asset_type in ["orthophoto", "dsm", "dtm", "model", "point_cloud"]:
            output_path = os.path.join(output_dir, f"{asset_type}.tif" if asset_type in ["orthophoto", "dsm", "dtm"] else f"{asset_type}.zip")
            success, error = await self.download_result(project_id, asset_type, output_path)
            if success:
                outputs[asset_type] = output_path
        
        if not outputs:
            return False, {}, "No outputs were generated"
        
        return True, outputs, None
