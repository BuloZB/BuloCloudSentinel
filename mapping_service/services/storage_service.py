"""
Storage service for the Mapping Service.
"""

import json
import logging
import os
from typing import Any, Dict, List, Optional

import aioboto3

from mapping_service.core.config import settings

logger = logging.getLogger(__name__)


class StorageService:
    """Service for storing and retrieving data from MinIO."""
    
    def __init__(self):
        """Initialize the storage service."""
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
    
    async def ensure_bucket_exists(self) -> bool:
        """
        Ensure the bucket exists.
        
        Returns:
            bool: True if successful, False otherwise
        """
        try:
            async with await self._get_client() as client:
                # Check if bucket exists
                try:
                    await client.head_bucket(Bucket=self.bucket)
                except Exception:
                    # Create bucket
                    await client.create_bucket(Bucket=self.bucket)
                    
                    # Set bucket policy for public access to tiles
                    policy = {
                        "Version": "2012-10-17",
                        "Statement": [
                            {
                                "Effect": "Allow",
                                "Principal": "*",
                                "Action": ["s3:GetObject"],
                                "Resource": [
                                    f"arn:aws:s3:::{self.bucket}/{settings.TILES_PATH}/*",
                                    f"arn:aws:s3:::{self.bucket}/{settings.ORTHOMOSAICS_PATH}/*"
                                ]
                            }
                        ]
                    }
                    
                    await client.put_bucket_policy(
                        Bucket=self.bucket,
                        Policy=json.dumps(policy)
                    )
            
            return True
        except Exception as e:
            logger.error(f"Error ensuring bucket exists: {str(e)}")
            return False
    
    async def upload_file(self, local_path: str, remote_path: str, content_type: Optional[str] = None) -> bool:
        """
        Upload a file to storage.
        
        Args:
            local_path: Local file path
            remote_path: Remote file path
            content_type: Content type
            
        Returns:
            bool: True if successful, False otherwise
        """
        try:
            # Ensure bucket exists
            await self.ensure_bucket_exists()
            
            # Upload file
            async with await self._get_client() as client:
                extra_args = {}
                if content_type:
                    extra_args["ContentType"] = content_type
                
                await client.upload_file(
                    local_path,
                    self.bucket,
                    remote_path,
                    ExtraArgs=extra_args
                )
            
            logger.info(f"Uploaded {local_path} to {remote_path}")
            return True
        except Exception as e:
            logger.error(f"Error uploading file: {str(e)}")
            return False
    
    async def download_file(self, remote_path: str, local_path: str) -> bool:
        """
        Download a file from storage.
        
        Args:
            remote_path: Remote file path
            local_path: Local file path
            
        Returns:
            bool: True if successful, False otherwise
        """
        try:
            # Create directory if it doesn't exist
            os.makedirs(os.path.dirname(local_path), exist_ok=True)
            
            # Download file
            async with await self._get_client() as client:
                await client.download_file(
                    self.bucket,
                    remote_path,
                    local_path
                )
            
            logger.info(f"Downloaded {remote_path} to {local_path}")
            return True
        except Exception as e:
            logger.error(f"Error downloading file: {str(e)}")
            return False
    
    async def list_files(self, prefix: str) -> List[Dict[str, Any]]:
        """
        List files in storage.
        
        Args:
            prefix: Prefix to filter by
            
        Returns:
            List[Dict[str, Any]]: List of files
        """
        try:
            async with await self._get_client() as client:
                response = await client.list_objects_v2(
                    Bucket=self.bucket,
                    Prefix=prefix
                )
                
                if "Contents" not in response:
                    return []
                
                return [
                    {
                        "key": item["Key"],
                        "size": item["Size"],
                        "last_modified": item["LastModified"],
                        "etag": item["ETag"].strip('"'),
                        "url": f"{self.endpoint_url}/{self.bucket}/{item['Key']}"
                    }
                    for item in response["Contents"]
                ]
        except Exception as e:
            logger.error(f"Error listing files: {str(e)}")
            return []
    
    async def delete_file(self, remote_path: str) -> bool:
        """
        Delete a file from storage.
        
        Args:
            remote_path: Remote file path
            
        Returns:
            bool: True if successful, False otherwise
        """
        try:
            async with await self._get_client() as client:
                await client.delete_object(
                    Bucket=self.bucket,
                    Key=remote_path
                )
            
            logger.info(f"Deleted {remote_path}")
            return True
        except Exception as e:
            logger.error(f"Error deleting file: {str(e)}")
            return False
    
    def get_public_url(self, remote_path: str) -> str:
        """
        Get the public URL for a file.
        
        Args:
            remote_path: Remote file path
            
        Returns:
            str: Public URL
        """
        return f"{self.endpoint_url}/{self.bucket}/{remote_path}"
