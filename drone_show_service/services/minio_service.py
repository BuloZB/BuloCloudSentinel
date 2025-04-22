"""
MinIO service for the Drone Show microservice.

This module provides services for storing and retrieving data from MinIO.
"""

import logging
import json
import io
from typing import Any, Dict, List, Optional
import aioboto3
from botocore.exceptions import ClientError

from drone_show_service.core.config import settings

logger = logging.getLogger(__name__)


class MinioService:
    """
    Service for storing and retrieving data from MinIO.
    
    This service provides methods for storing and retrieving JSON data,
    files, and other objects from MinIO.
    """
    
    def __init__(self):
        """Initialize the MinIO service."""
        self.session = aioboto3.Session()
        self.endpoint_url = f"http://{settings.MINIO_URL}"
        self.access_key = settings.MINIO_ACCESS_KEY
        self.secret_key = settings.MINIO_SECRET_KEY
        self.bucket = settings.MINIO_BUCKET
    
    async def _get_client(self):
        """
        Get an S3 client.
        
        Returns:
            S3 client
        """
        return self.session.client(
            "s3",
            endpoint_url=self.endpoint_url,
            aws_access_key_id=self.access_key,
            aws_secret_access_key=self.secret_key,
        )
    
    async def ensure_bucket_exists(self):
        """
        Ensure that the bucket exists.
        
        Creates the bucket if it doesn't exist.
        """
        async with await self._get_client() as client:
            try:
                await client.head_bucket(Bucket=self.bucket)
            except ClientError:
                await client.create_bucket(Bucket=self.bucket)
    
    async def put_json(self, key: str, data: Any) -> bool:
        """
        Store JSON data in MinIO.
        
        Args:
            key: Object key
            data: JSON-serializable data
            
        Returns:
            True if successful, False otherwise
        """
        try:
            # Ensure bucket exists
            await self.ensure_bucket_exists()
            
            # Convert data to JSON
            json_data = json.dumps(data)
            
            # Store in MinIO
            async with await self._get_client() as client:
                await client.put_object(
                    Bucket=self.bucket,
                    Key=key,
                    Body=json_data,
                    ContentType="application/json",
                )
            
            return True
        except Exception as e:
            logger.error(f"Error storing JSON data in MinIO: {str(e)}")
            return False
    
    async def get_json(self, key: str) -> Any:
        """
        Retrieve JSON data from MinIO.
        
        Args:
            key: Object key
            
        Returns:
            JSON-deserialized data
        """
        try:
            # Retrieve from MinIO
            async with await self._get_client() as client:
                response = await client.get_object(Bucket=self.bucket, Key=key)
                data = await response["Body"].read()
            
            # Parse JSON
            return json.loads(data)
        except Exception as e:
            logger.error(f"Error retrieving JSON data from MinIO: {str(e)}")
            return None
    
    async def put_file(self, key: str, file_path: str) -> bool:
        """
        Store a file in MinIO.
        
        Args:
            key: Object key
            file_path: Path to file
            
        Returns:
            True if successful, False otherwise
        """
        try:
            # Ensure bucket exists
            await self.ensure_bucket_exists()
            
            # Store in MinIO
            async with await self._get_client() as client:
                await client.upload_file(file_path, self.bucket, key)
            
            return True
        except Exception as e:
            logger.error(f"Error storing file in MinIO: {str(e)}")
            return False
    
    async def get_file(self, key: str, file_path: str) -> bool:
        """
        Retrieve a file from MinIO.
        
        Args:
            key: Object key
            file_path: Path to save file
            
        Returns:
            True if successful, False otherwise
        """
        try:
            # Retrieve from MinIO
            async with await self._get_client() as client:
                await client.download_file(self.bucket, key, file_path)
            
            return True
        except Exception as e:
            logger.error(f"Error retrieving file from MinIO: {str(e)}")
            return False
    
    async def delete(self, key: str) -> bool:
        """
        Delete an object from MinIO.
        
        Args:
            key: Object key
            
        Returns:
            True if successful, False otherwise
        """
        try:
            # Delete from MinIO
            async with await self._get_client() as client:
                await client.delete_object(Bucket=self.bucket, Key=key)
            
            return True
        except Exception as e:
            logger.error(f"Error deleting object from MinIO: {str(e)}")
            return False
    
    async def list_objects(self, prefix: str = "") -> List[str]:
        """
        List objects in MinIO.
        
        Args:
            prefix: Object key prefix
            
        Returns:
            List of object keys
        """
        try:
            # List objects in MinIO
            async with await self._get_client() as client:
                response = await client.list_objects_v2(Bucket=self.bucket, Prefix=prefix)
            
            # Extract keys
            if "Contents" in response:
                return [obj["Key"] for obj in response["Contents"]]
            
            return []
        except Exception as e:
            logger.error(f"Error listing objects in MinIO: {str(e)}")
            return []
