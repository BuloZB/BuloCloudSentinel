"""
Storage manager service for the SIGINT service.

This service is responsible for managing signal data storage, including:
- Storing signal recordings
- Retrieving signal recordings
- Managing storage lifecycle
"""

import logging
import io
import uuid
from typing import Dict, List, Optional, Any, BinaryIO
from datetime import datetime, timedelta

from minio import Minio
from minio.error import S3Error

logger = logging.getLogger(__name__)

class StorageManager:
    """Storage manager service."""
    
    def __init__(self, endpoint: str, access_key: str, secret_key: str, bucket_name: str, secure: bool = True):
        """
        Initialize the storage manager.
        
        Args:
            endpoint: MinIO endpoint
            access_key: MinIO access key
            secret_key: MinIO secret key
            bucket_name: MinIO bucket name
            secure: Whether to use HTTPS
        """
        self.endpoint = endpoint
        self.access_key = access_key
        self.secret_key = secret_key
        self.bucket_name = bucket_name
        self.secure = secure
        self.client = None
    
    async def connect(self):
        """Connect to the storage service."""
        logger.info(f"Connecting to MinIO at {self.endpoint}")
        
        try:
            # Create MinIO client
            self.client = Minio(
                self.endpoint,
                access_key=self.access_key,
                secret_key=self.secret_key,
                secure=self.secure
            )
            
            # Check if bucket exists
            if not self.client.bucket_exists(self.bucket_name):
                # Create bucket
                self.client.make_bucket(self.bucket_name)
                logger.info(f"Created bucket {self.bucket_name}")
            
            logger.info("Connected to MinIO")
        except Exception as e:
            logger.error(f"Failed to connect to MinIO: {e}")
            # In a real implementation, we would retry with backoff
            raise
    
    async def disconnect(self):
        """Disconnect from the storage service."""
        logger.info("Disconnecting from MinIO")
        
        # MinIO client doesn't need explicit disconnection
        self.client = None
    
    async def store_recording(self, data: BinaryIO, metadata: Dict[str, str]) -> Dict[str, Any]:
        """
        Store a signal recording.
        
        Args:
            data: Recording data
            metadata: Recording metadata
            
        Returns:
            Storage information
        """
        if not self.client:
            logger.warning("Not connected to MinIO, reconnecting")
            await self.connect()
        
        try:
            # Generate object name
            object_name = f"recordings/{uuid.uuid4()}.{metadata.get('format', 'dat').lower()}"
            
            # Get data size
            data.seek(0, io.SEEK_END)
            size = data.tell()
            data.seek(0)
            
            # Upload data
            self.client.put_object(
                bucket_name=self.bucket_name,
                object_name=object_name,
                data=data,
                length=size,
                content_type=self._get_content_type(metadata.get("format", "dat")),
                metadata=metadata
            )
            
            # Generate URL
            url = f"minio://{self.bucket_name}/{object_name}"
            
            logger.info(f"Stored recording at {url}")
            
            # Return storage information
            return {
                "url": url,
                "size": size,
                "metadata": metadata
            }
        except Exception as e:
            logger.error(f"Failed to store recording: {e}")
            raise
    
    async def retrieve_recording(self, url: str) -> Tuple[BinaryIO, Dict[str, str]]:
        """
        Retrieve a signal recording.
        
        Args:
            url: Recording URL
            
        Returns:
            Tuple of (recording data, metadata)
        """
        if not self.client:
            logger.warning("Not connected to MinIO, reconnecting")
            await self.connect()
        
        try:
            # Parse URL
            if not url.startswith(f"minio://{self.bucket_name}/"):
                raise ValueError(f"Invalid URL: {url}")
            
            object_name = url[len(f"minio://{self.bucket_name}/"):]
            
            # Get object
            response = self.client.get_object(
                bucket_name=self.bucket_name,
                object_name=object_name
            )
            
            # Get metadata
            stat = self.client.stat_object(
                bucket_name=self.bucket_name,
                object_name=object_name
            )
            
            # Read data
            data = io.BytesIO(response.read())
            
            # Close response
            response.close()
            response.release_conn()
            
            logger.info(f"Retrieved recording from {url}")
            
            # Return data and metadata
            return data, stat.metadata
        except Exception as e:
            logger.error(f"Failed to retrieve recording: {e}")
            raise
    
    async def delete_recording(self, url: str):
        """
        Delete a signal recording.
        
        Args:
            url: Recording URL
        """
        if not self.client:
            logger.warning("Not connected to MinIO, reconnecting")
            await self.connect()
        
        try:
            # Parse URL
            if not url.startswith(f"minio://{self.bucket_name}/"):
                raise ValueError(f"Invalid URL: {url}")
            
            object_name = url[len(f"minio://{self.bucket_name}/"):]
            
            # Remove object
            self.client.remove_object(
                bucket_name=self.bucket_name,
                object_name=object_name
            )
            
            logger.info(f"Deleted recording at {url}")
        except Exception as e:
            logger.error(f"Failed to delete recording: {e}")
            raise
    
    async def list_recordings(self, prefix: str = "recordings/", max_keys: int = 1000) -> List[Dict[str, Any]]:
        """
        List signal recordings.
        
        Args:
            prefix: Object name prefix
            max_keys: Maximum number of keys to return
            
        Returns:
            List of recording information
        """
        if not self.client:
            logger.warning("Not connected to MinIO, reconnecting")
            await self.connect()
        
        try:
            # List objects
            objects = self.client.list_objects(
                bucket_name=self.bucket_name,
                prefix=prefix,
                recursive=True
            )
            
            # Convert to list
            recordings = []
            for obj in objects:
                # Get object metadata
                stat = self.client.stat_object(
                    bucket_name=self.bucket_name,
                    object_name=obj.object_name
                )
                
                # Generate URL
                url = f"minio://{self.bucket_name}/{obj.object_name}"
                
                # Add to list
                recordings.append({
                    "url": url,
                    "size": obj.size,
                    "last_modified": obj.last_modified,
                    "metadata": stat.metadata
                })
                
                # Check if we've reached the maximum
                if len(recordings) >= max_keys:
                    break
            
            logger.info(f"Listed {len(recordings)} recordings")
            
            return recordings
        except Exception as e:
            logger.error(f"Failed to list recordings: {e}")
            raise
    
    async def cleanup_old_recordings(self, max_age_days: int = 30):
        """
        Clean up old recordings.
        
        Args:
            max_age_days: Maximum age of recordings in days
        """
        if not self.client:
            logger.warning("Not connected to MinIO, reconnecting")
            await self.connect()
        
        try:
            # Calculate cutoff date
            cutoff_date = datetime.utcnow() - timedelta(days=max_age_days)
            
            # List objects
            objects = self.client.list_objects(
                bucket_name=self.bucket_name,
                prefix="recordings/",
                recursive=True
            )
            
            # Delete old objects
            deleted_count = 0
            for obj in objects:
                if obj.last_modified < cutoff_date:
                    # Remove object
                    self.client.remove_object(
                        bucket_name=self.bucket_name,
                        object_name=obj.object_name
                    )
                    
                    deleted_count += 1
            
            logger.info(f"Cleaned up {deleted_count} old recordings")
        except Exception as e:
            logger.error(f"Failed to clean up old recordings: {e}")
            raise
    
    def _get_content_type(self, format: str) -> str:
        """
        Get content type for a recording format.
        
        Args:
            format: Recording format
            
        Returns:
            Content type
        """
        format = format.lower()
        
        if format == "wav":
            return "audio/wav"
        elif format == "iq":
            return "application/octet-stream"
        elif format == "json":
            return "application/json"
        else:
            return "application/octet-stream"
