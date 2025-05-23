"""
Mapping service for the Mapping Service.
"""

import logging
import os
import tempfile
import uuid
from datetime import datetime, timezone
from typing import Any, Dict, List, Optional, Tuple

import exifread
from geoalchemy2 import Geography
from geoalchemy2.shape import from_shape
from shapely.geometry import Point, Polygon
from sqlalchemy import select
from sqlalchemy.ext.asyncio import AsyncSession

from mapping_service.core.config import settings
from mapping_service.db.models import MappingAsset, MappingProject, ProcessingJob, SourceImage
from mapping_service.services.storage_service import StorageService

logger = logging.getLogger(__name__)


class MappingService:
    """Service for mapping operations."""
    
    def __init__(self, db_session: AsyncSession):
        """
        Initialize the mapping service.
        
        Args:
            db_session: Database session
        """
        self.db_session = db_session
        self.storage_service = StorageService()
    
    async def create_project(self, name: str, description: str, user_id: uuid.UUID) -> Tuple[bool, Optional[uuid.UUID], Optional[str]]:
        """
        Create a new mapping project.
        
        Args:
            name: Project name
            description: Project description
            user_id: User ID
            
        Returns:
            Tuple[bool, Optional[uuid.UUID], Optional[str]]: (success, project_id, error_message)
        """
        try:
            # Create project
            project = MappingProject(
                name=name,
                description=description,
                user_id=user_id,
                status="created"
            )
            
            # Save to database
            self.db_session.add(project)
            await self.db_session.commit()
            await self.db_session.refresh(project)
            
            logger.info(f"Created project {project.id}")
            
            return True, project.id, None
        except Exception as e:
            logger.error(f"Error creating project: {str(e)}")
            return False, None, f"Error creating project: {str(e)}"
    
    async def get_project(self, project_id: uuid.UUID) -> Optional[MappingProject]:
        """
        Get a mapping project.
        
        Args:
            project_id: Project ID
            
        Returns:
            Optional[MappingProject]: Project, or None if not found
        """
        return await self.db_session.get(MappingProject, project_id)
    
    async def list_projects(self, user_id: Optional[uuid.UUID] = None) -> List[MappingProject]:
        """
        List mapping projects.
        
        Args:
            user_id: Optional user ID to filter by
            
        Returns:
            List[MappingProject]: List of projects
        """
        query = select(MappingProject)
        
        if user_id:
            query = query.where(MappingProject.user_id == user_id)
        
        query = query.order_by(MappingProject.created_at.desc())
        
        result = await self.db_session.execute(query)
        return result.scalars().all()
    
    async def upload_images(self, project_id: uuid.UUID, files: List[Dict[str, Any]]) -> Tuple[bool, int, Optional[str]]:
        """
        Upload images to a project.
        
        Args:
            project_id: Project ID
            files: List of files (each with 'filename', 'content_type', and 'file' fields)
            
        Returns:
            Tuple[bool, int, Optional[str]]: (success, number of images uploaded, error_message)
        """
        try:
            # Get project
            project = await self.db_session.get(MappingProject, project_id)
            if project is None:
                return False, 0, "Project not found"
            
            # Process each file
            uploaded_count = 0
            for file_data in files:
                filename = file_data["filename"]
                file = file_data["file"]
                
                # Create temporary file
                with tempfile.NamedTemporaryFile(delete=False) as temp_file:
                    # Write file content
                    temp_file.write(file.read())
                    temp_file_path = temp_file.name
                
                try:
                    # Extract EXIF data
                    with open(temp_file_path, "rb") as f:
                        tags = exifread.process_file(f)
                    
                    # Extract GPS coordinates
                    location = None
                    altitude = None
                    captured_at = None
                    
                    if "GPS GPSLatitude" in tags and "GPS GPSLongitude" in tags:
                        lat = self._convert_to_degrees(tags["GPS GPSLatitude"].values)
                        lon = self._convert_to_degrees(tags["GPS GPSLongitude"].values)
                        
                        # Check for reference (N/S, E/W)
                        if "GPS GPSLatitudeRef" in tags and tags["GPS GPSLatitudeRef"].values == "S":
                            lat = -lat
                        if "GPS GPSLongitudeRef" in tags and tags["GPS GPSLongitudeRef"].values == "W":
                            lon = -lon
                        
                        # Create point
                        point = Point(lon, lat)
                        location = from_shape(point, srid=4326)
                    
                    # Extract altitude
                    if "GPS GPSAltitude" in tags:
                        altitude_value = tags["GPS GPSAltitude"].values[0]
                        altitude = float(altitude_value.num) / float(altitude_value.den)
                        
                        # Check for reference (above/below sea level)
                        if "GPS GPSAltitudeRef" in tags and tags["GPS GPSAltitudeRef"].values == 1:
                            altitude = -altitude
                    
                    # Extract timestamp
                    if "EXIF DateTimeOriginal" in tags:
                        date_str = str(tags["EXIF DateTimeOriginal"])
                        try:
                            captured_at = datetime.strptime(date_str, "%Y:%m:%d %H:%M:%S")
                        except ValueError:
                            captured_at = None
                    
                    # Upload to storage
                    storage_path = f"{settings.IMAGES_PATH}/{project_id}/{filename}"
                    success = await self.storage_service.upload_file(
                        local_path=temp_file_path,
                        remote_path=storage_path,
                        content_type=file_data["content_type"]
                    )
                    
                    if not success:
                        return False, uploaded_count, f"Failed to upload image {filename}"
                    
                    # Create source image record
                    source_image = SourceImage(
                        project_id=project_id,
                        filename=filename,
                        storage_path=storage_path,
                        captured_at=captured_at,
                        location=location,
                        altitude=altitude,
                        metadata={
                            "content_type": file_data["content_type"],
                            "size": os.path.getsize(temp_file_path)
                        }
                    )
                    
                    self.db_session.add(source_image)
                    uploaded_count += 1
                finally:
                    # Clean up temporary file
                    os.unlink(temp_file_path)
            
            # Update project
            project.image_count += uploaded_count
            project.updated_at = datetime.now(timezone.utc)
            
            # If this is the first upload, update status
            if project.status == "created":
                project.status = "uploaded"
            
            await self.db_session.commit()
            
            logger.info(f"Uploaded {uploaded_count} images to project {project_id}")
            
            return True, uploaded_count, None
        except Exception as e:
            logger.error(f"Error uploading images: {str(e)}")
            return False, 0, f"Error uploading images: {str(e)}"
    
    async def start_processing(self, project_id: uuid.UUID, parameters: Dict[str, Any]) -> Tuple[bool, Optional[uuid.UUID], Optional[str]]:
        """
        Start processing a project.
        
        Args:
            project_id: Project ID
            parameters: Processing parameters
            
        Returns:
            Tuple[bool, Optional[uuid.UUID], Optional[str]]: (success, job_id, error_message)
        """
        try:
            # Get project
            project = await self.db_session.get(MappingProject, project_id)
            if project is None:
                return False, None, "Project not found"
            
            # Check if project has images
            if project.image_count == 0:
                return False, None, "Project has no images"
            
            # Create processing job
            job = ProcessingJob(
                project_id=project_id,
                status="queued",
                parameters=parameters
            )
            
            # Save to database
            self.db_session.add(job)
            
            # Update project status
            project.status = "queued"
            project.updated_at = datetime.now(timezone.utc)
            
            await self.db_session.commit()
            await self.db_session.refresh(job)
            
            logger.info(f"Created processing job {job.id} for project {project_id}")
            
            return True, job.id, None
        except Exception as e:
            logger.error(f"Error starting processing: {str(e)}")
            return False, None, f"Error starting processing: {str(e)}"
    
    async def get_project_assets(self, project_id: uuid.UUID) -> List[MappingAsset]:
        """
        Get assets for a project.
        
        Args:
            project_id: Project ID
            
        Returns:
            List[MappingAsset]: List of assets
        """
        query = select(MappingAsset).where(
            MappingAsset.project_id == project_id
        ).order_by(
            MappingAsset.created_at.desc()
        )
        
        result = await self.db_session.execute(query)
        return result.scalars().all()
    
    async def get_project_source_images(self, project_id: uuid.UUID) -> List[SourceImage]:
        """
        Get source images for a project.
        
        Args:
            project_id: Project ID
            
        Returns:
            List[SourceImage]: List of source images
        """
        query = select(SourceImage).where(
            SourceImage.project_id == project_id
        ).order_by(
            SourceImage.filename
        )
        
        result = await self.db_session.execute(query)
        return result.scalars().all()
    
    async def get_project_processing_jobs(self, project_id: uuid.UUID) -> List[ProcessingJob]:
        """
        Get processing jobs for a project.
        
        Args:
            project_id: Project ID
            
        Returns:
            List[ProcessingJob]: List of processing jobs
        """
        query = select(ProcessingJob).where(
            ProcessingJob.project_id == project_id
        ).order_by(
            ProcessingJob.created_at.desc()
        )
        
        result = await self.db_session.execute(query)
        return result.scalars().all()
    
    def _convert_to_degrees(self, value):
        """
        Convert GPS coordinates to degrees.
        
        Args:
            value: GPS coordinate value
            
        Returns:
            float: Coordinate in degrees
        """
        d = float(value[0].num) / float(value[0].den)
        m = float(value[1].num) / float(value[1].den)
        s = float(value[2].num) / float(value[2].den)
        
        return d + (m / 60.0) + (s / 3600.0)
