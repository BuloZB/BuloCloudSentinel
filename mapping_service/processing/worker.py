"""
Processing worker for the Mapping Service.
"""

import asyncio
import logging
import os
import shutil
import tempfile
import uuid
from datetime import datetime, timezone
from typing import Any, Dict, List, Optional, Tuple

from sqlalchemy import select
from sqlalchemy.ext.asyncio import AsyncSession

from mapping_service.core.config import settings
from mapping_service.db.models import MappingAsset, MappingProject, ProcessingJob, SourceImage
from mapping_service.processing.cesium_converter import CesiumConverter
from mapping_service.processing.odm_adapter import OpenDroneMapAdapter
from mapping_service.services.storage_service import StorageService

logger = logging.getLogger(__name__)


class ProcessingWorker:
    """Worker for processing mapping jobs."""
    
    def __init__(self, db_session: AsyncSession):
        """
        Initialize the processing worker.
        
        Args:
            db_session: Database session
        """
        self.db_session = db_session
        self.odm_adapter = OpenDroneMapAdapter()
        self.cesium_converter = CesiumConverter()
        self.storage_service = StorageService()
        self.worker_id = str(uuid.uuid4())
        self.temp_dir = settings.TEMP_DIRECTORY
        os.makedirs(self.temp_dir, exist_ok=True)
    
    async def get_next_job(self) -> Optional[ProcessingJob]:
        """
        Get the next job from the queue.
        
        Returns:
            Optional[ProcessingJob]: Next job, or None if queue is empty
        """
        # Get next job
        query = select(ProcessingJob).where(
            ProcessingJob.status == "queued"
        ).order_by(
            ProcessingJob.created_at
        ).limit(1)
        
        result = await self.db_session.execute(query)
        job = result.scalar_one_or_none()
        
        if job is None:
            return None
        
        # Update job status
        job.status = "processing"
        job.started_at = datetime.now(timezone.utc)
        job.worker_id = self.worker_id
        
        await self.db_session.commit()
        await self.db_session.refresh(job)
        
        logger.info(f"Started processing job {job.id}")
        
        return job
    
    async def process_job(self, job: ProcessingJob) -> None:
        """
        Process a job.
        
        Args:
            job: Job to process
        """
        try:
            # Get project
            project = await self.db_session.get(MappingProject, job.project_id)
            if project is None:
                await self._fail_job(job, "Project not found")
                return
            
            # Update project status
            project.status = "processing"
            await self.db_session.commit()
            
            # Get source images
            query = select(SourceImage).where(SourceImage.project_id == job.project_id)
            result = await self.db_session.execute(query)
            source_images = result.scalars().all()
            
            if not source_images:
                await self._fail_job(job, "No source images found")
                return
            
            # Download source images
            image_paths = []
            temp_dir = tempfile.mkdtemp(dir=self.temp_dir)
            
            for image in source_images:
                local_path = os.path.join(temp_dir, image.filename)
                success = await self.storage_service.download_file(image.storage_path, local_path)
                if not success:
                    await self._fail_job(job, f"Failed to download image {image.filename}")
                    return
                
                image_paths.append(local_path)
            
            # Process images with OpenDroneMap
            success, outputs, error = await self.odm_adapter.process_images(
                project_name=project.name,
                image_paths=image_paths,
                options=job.parameters or {}
            )
            
            if not success:
                await self._fail_job(job, error or "Failed to process images")
                return
            
            # Convert outputs to Cesium 3D Tiles
            assets = []
            
            # Process orthomosaic
            if "orthophoto" in outputs:
                # Convert to tiles
                success, tiles_dir, error = await self.cesium_converter.convert_orthomosaic_to_tiles(
                    orthomosaic_path=outputs["orthophoto"]
                )
                
                if success and tiles_dir:
                    # Upload tiles to storage
                    tiles_path = f"{settings.TILES_PATH}/{project.id}/orthomosaic"
                    
                    # Upload each tile file
                    for root, _, files in os.walk(tiles_dir):
                        for file in files:
                            local_path = os.path.join(root, file)
                            relative_path = os.path.relpath(local_path, tiles_dir)
                            remote_path = f"{tiles_path}/{relative_path}"
                            
                            # Determine content type
                            content_type = None
                            if file.endswith(".png"):
                                content_type = "image/png"
                            elif file.endswith(".jpg") or file.endswith(".jpeg"):
                                content_type = "image/jpeg"
                            elif file.endswith(".xml"):
                                content_type = "application/xml"
                            elif file.endswith(".html"):
                                content_type = "text/html"
                            
                            await self.storage_service.upload_file(
                                local_path=local_path,
                                remote_path=remote_path,
                                content_type=content_type
                            )
                    
                    # Create asset record
                    assets.append({
                        "type": "orthomosaic_tiles",
                        "storage_path": tiles_path,
                        "metadata": {
                            "tile_format": "xyz",
                            "min_zoom": 12,
                            "max_zoom": 22
                        }
                    })
                
                # Upload original orthomosaic
                ortho_path = f"{settings.ORTHOMOSAICS_PATH}/{project.id}/orthomosaic.tif"
                success = await self.storage_service.upload_file(
                    local_path=outputs["orthophoto"],
                    remote_path=ortho_path,
                    content_type="image/tiff"
                )
                
                if success:
                    # Create asset record
                    assets.append({
                        "type": "orthomosaic",
                        "storage_path": ortho_path,
                        "file_size": os.path.getsize(outputs["orthophoto"]),
                        "metadata": {
                            "format": "geotiff"
                        }
                    })
            
            # Process 3D mesh
            if "model" in outputs:
                # Convert to 3D Tiles
                success, tiles_dir, error = await self.cesium_converter.convert_mesh_to_3d_tiles(
                    mesh_path=outputs["model"]
                )
                
                if success and tiles_dir:
                    # Upload tiles to storage
                    tiles_path = f"{settings.TILES_PATH}/{project.id}/3d_mesh"
                    
                    # Upload each tile file
                    for root, _, files in os.walk(tiles_dir):
                        for file in files:
                            local_path = os.path.join(root, file)
                            relative_path = os.path.relpath(local_path, tiles_dir)
                            remote_path = f"{tiles_path}/{relative_path}"
                            
                            # Determine content type
                            content_type = None
                            if file.endswith(".json"):
                                content_type = "application/json"
                            elif file.endswith(".b3dm"):
                                content_type = "application/octet-stream"
                            
                            await self.storage_service.upload_file(
                                local_path=local_path,
                                remote_path=remote_path,
                                content_type=content_type
                            )
                    
                    # Create asset record
                    assets.append({
                        "type": "3d_mesh_tiles",
                        "storage_path": tiles_path,
                        "metadata": {
                            "format": "3d_tiles",
                            "tileset": f"{tiles_path}/tileset.json"
                        }
                    })
            
            # Save assets to database
            for asset_data in assets:
                asset = MappingAsset(
                    project_id=project.id,
                    type=asset_data["type"],
                    storage_path=asset_data["storage_path"],
                    file_size=asset_data.get("file_size"),
                    metadata=asset_data.get("metadata"),
                    version=1,
                    is_current=True
                )
                
                self.db_session.add(asset)
            
            # Update project
            project.status = "completed"
            project.updated_at = datetime.now(timezone.utc)
            
            # Update job
            job.status = "completed"
            job.completed_at = datetime.now(timezone.utc)
            
            await self.db_session.commit()
            
            logger.info(f"Completed job {job.id}")
            
            # Clean up
            shutil.rmtree(temp_dir, ignore_errors=True)
        except Exception as e:
            logger.error(f"Error processing job {job.id}: {str(e)}")
            await self._fail_job(job, f"Error processing job: {str(e)}")
    
    async def _fail_job(self, job: ProcessingJob, error_message: str) -> None:
        """
        Mark a job as failed.
        
        Args:
            job: Job to mark as failed
            error_message: Error message
        """
        # Update job
        job.status = "failed"
        job.completed_at = datetime.now(timezone.utc)
        job.error_message = error_message
        
        # Update project
        project = await self.db_session.get(MappingProject, job.project_id)
        if project:
            project.status = "failed"
            project.updated_at = datetime.now(timezone.utc)
        
        await self.db_session.commit()
        
        logger.error(f"Job {job.id} failed: {error_message}")
    
    async def run(self) -> None:
        """Run the worker."""
        logger.info(f"Starting worker {self.worker_id}")
        
        while True:
            try:
                # Get next job
                job = await self.get_next_job()
                
                if job:
                    # Process job
                    await self.process_job(job)
                else:
                    # No jobs, wait before checking again
                    await asyncio.sleep(5)
            except Exception as e:
                logger.error(f"Error in worker loop: {str(e)}")
                await asyncio.sleep(10)  # Wait longer after an error
