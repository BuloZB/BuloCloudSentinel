"""
Storage manager service for the EW service.

This service is responsible for managing file storage, including:
- Uploading files
- Downloading files
- Deleting files
- Listing files
"""

import logging
import os
import aiofiles
import aiohttp
import boto3
import botocore
from typing import Dict, List, Optional, Any, BinaryIO
from urllib.parse import urlparse

from core.config import settings

logger = logging.getLogger(__name__)

class StorageManager:
    """Storage manager service."""
    
    def __init__(self):
        """Initialize the storage manager."""
        self.storage_type = settings.STORAGE_TYPE
        self.local_storage_path = settings.LOCAL_STORAGE_PATH
        self.s3_bucket = settings.S3_BUCKET
        self.s3_region = settings.S3_REGION
        self.s3_endpoint = settings.S3_ENDPOINT
        self.s3_access_key = settings.S3_ACCESS_KEY
        self.s3_secret_key = settings.S3_SECRET_KEY
        self.s3_client = None
        
        # Initialize storage
        if self.storage_type == "s3":
            self._init_s3()
        elif self.storage_type == "local":
            self._init_local()
    
    def _init_s3(self):
        """Initialize S3 storage."""
        try:
            # Create S3 client
            self.s3_client = boto3.client(
                "s3",
                region_name=self.s3_region,
                endpoint_url=self.s3_endpoint,
                aws_access_key_id=self.s3_access_key,
                aws_secret_access_key=self.s3_secret_key
            )
            
            # Check if bucket exists
            try:
                self.s3_client.head_bucket(Bucket=self.s3_bucket)
                logger.info(f"S3 bucket {self.s3_bucket} exists")
            except botocore.exceptions.ClientError as e:
                error_code = e.response["Error"]["Code"]
                if error_code == "404":
                    # Bucket doesn't exist, create it
                    logger.info(f"Creating S3 bucket {self.s3_bucket}")
                    self.s3_client.create_bucket(Bucket=self.s3_bucket)
                else:
                    logger.error(f"Failed to check S3 bucket: {e}")
                    raise
        
        except Exception as e:
            logger.error(f"Failed to initialize S3 storage: {e}")
            raise
    
    def _init_local(self):
        """Initialize local storage."""
        try:
            # Create local storage directory if it doesn't exist
            os.makedirs(self.local_storage_path, exist_ok=True)
            logger.info(f"Local storage initialized at {self.local_storage_path}")
        
        except Exception as e:
            logger.error(f"Failed to initialize local storage: {e}")
            raise
    
    async def upload_file(
        self,
        local_path: str,
        remote_path: str,
        content_type: Optional[str] = None
    ) -> Optional[str]:
        """
        Upload a file to storage.
        
        Args:
            local_path: Local file path
            remote_path: Remote file path
            content_type: Optional content type
            
        Returns:
            File URL or None if upload failed
        """
        try:
            if self.storage_type == "s3":
                return await self._upload_to_s3(local_path, remote_path, content_type)
            elif self.storage_type == "local":
                return await self._upload_to_local(local_path, remote_path)
            else:
                logger.error(f"Unknown storage type: {self.storage_type}")
                return None
        
        except Exception as e:
            logger.error(f"Failed to upload file: {e}")
            return None
    
    async def download_file(self, remote_path: str, local_path: str) -> bool:
        """
        Download a file from storage.
        
        Args:
            remote_path: Remote file path
            local_path: Local file path
            
        Returns:
            True if successful, False otherwise
        """
        try:
            if self.storage_type == "s3":
                return await self._download_from_s3(remote_path, local_path)
            elif self.storage_type == "local":
                return await self._download_from_local(remote_path, local_path)
            else:
                logger.error(f"Unknown storage type: {self.storage_type}")
                return False
        
        except Exception as e:
            logger.error(f"Failed to download file: {e}")
            return False
    
    async def delete_file(self, remote_path: str) -> bool:
        """
        Delete a file from storage.
        
        Args:
            remote_path: Remote file path
            
        Returns:
            True if successful, False otherwise
        """
        try:
            if self.storage_type == "s3":
                return await self._delete_from_s3(remote_path)
            elif self.storage_type == "local":
                return await self._delete_from_local(remote_path)
            else:
                logger.error(f"Unknown storage type: {self.storage_type}")
                return False
        
        except Exception as e:
            logger.error(f"Failed to delete file: {e}")
            return False
    
    async def list_files(self, prefix: str = "") -> List[Dict[str, Any]]:
        """
        List files in storage.
        
        Args:
            prefix: Optional prefix to filter files
            
        Returns:
            List of file information
        """
        try:
            if self.storage_type == "s3":
                return await self._list_s3_files(prefix)
            elif self.storage_type == "local":
                return await self._list_local_files(prefix)
            else:
                logger.error(f"Unknown storage type: {self.storage_type}")
                return []
        
        except Exception as e:
            logger.error(f"Failed to list files: {e}")
            return []
    
    async def _upload_to_s3(
        self,
        local_path: str,
        remote_path: str,
        content_type: Optional[str] = None
    ) -> Optional[str]:
        """
        Upload a file to S3.
        
        Args:
            local_path: Local file path
            remote_path: Remote file path
            content_type: Optional content type
            
        Returns:
            File URL or None if upload failed
        """
        try:
            # Set extra args
            extra_args = {}
            if content_type:
                extra_args["ContentType"] = content_type
            
            # Upload file
            self.s3_client.upload_file(
                local_path,
                self.s3_bucket,
                remote_path,
                ExtraArgs=extra_args
            )
            
            # Generate URL
            if self.s3_endpoint:
                # Custom endpoint
                url = f"{self.s3_endpoint}/{self.s3_bucket}/{remote_path}"
                
                # Remove double slashes in path
                parsed = urlparse(url)
                path = parsed.path.replace("//", "/")
                url = f"{parsed.scheme}://{parsed.netloc}{path}"
            else:
                # AWS S3
                url = f"https://{self.s3_bucket}.s3.{self.s3_region}.amazonaws.com/{remote_path}"
            
            logger.info(f"Uploaded file to S3: {url}")
            
            return url
        
        except Exception as e:
            logger.error(f"Failed to upload file to S3: {e}")
            return None
    
    async def _upload_to_local(self, local_path: str, remote_path: str) -> Optional[str]:
        """
        Upload a file to local storage.
        
        Args:
            local_path: Local file path
            remote_path: Remote file path
            
        Returns:
            File URL or None if upload failed
        """
        try:
            # Create destination directory if it doesn't exist
            dest_path = os.path.join(self.local_storage_path, remote_path)
            os.makedirs(os.path.dirname(dest_path), exist_ok=True)
            
            # Copy file
            async with aiofiles.open(local_path, "rb") as src_file:
                content = await src_file.read()
                
                async with aiofiles.open(dest_path, "wb") as dest_file:
                    await dest_file.write(content)
            
            # Generate URL
            url = f"file://{dest_path}"
            
            logger.info(f"Uploaded file to local storage: {url}")
            
            return url
        
        except Exception as e:
            logger.error(f"Failed to upload file to local storage: {e}")
            return None
    
    async def _download_from_s3(self, remote_path: str, local_path: str) -> bool:
        """
        Download a file from S3.
        
        Args:
            remote_path: Remote file path
            local_path: Local file path
            
        Returns:
            True if successful, False otherwise
        """
        try:
            # Create destination directory if it doesn't exist
            os.makedirs(os.path.dirname(local_path), exist_ok=True)
            
            # Download file
            self.s3_client.download_file(
                self.s3_bucket,
                remote_path,
                local_path
            )
            
            logger.info(f"Downloaded file from S3: {remote_path}")
            
            return True
        
        except botocore.exceptions.ClientError as e:
            if e.response["Error"]["Code"] == "404":
                logger.warning(f"File not found in S3: {remote_path}")
            else:
                logger.error(f"Failed to download file from S3: {e}")
            return False
        
        except Exception as e:
            logger.error(f"Failed to download file from S3: {e}")
            return False
    
    async def _download_from_local(self, remote_path: str, local_path: str) -> bool:
        """
        Download a file from local storage.
        
        Args:
            remote_path: Remote file path
            local_path: Local file path
            
        Returns:
            True if successful, False otherwise
        """
        try:
            # Get source path
            src_path = os.path.join(self.local_storage_path, remote_path)
            
            # Check if file exists
            if not os.path.isfile(src_path):
                logger.warning(f"File not found in local storage: {remote_path}")
                return False
            
            # Create destination directory if it doesn't exist
            os.makedirs(os.path.dirname(local_path), exist_ok=True)
            
            # Copy file
            async with aiofiles.open(src_path, "rb") as src_file:
                content = await src_file.read()
                
                async with aiofiles.open(local_path, "wb") as dest_file:
                    await dest_file.write(content)
            
            logger.info(f"Downloaded file from local storage: {remote_path}")
            
            return True
        
        except Exception as e:
            logger.error(f"Failed to download file from local storage: {e}")
            return False
    
    async def _delete_from_s3(self, remote_path: str) -> bool:
        """
        Delete a file from S3.
        
        Args:
            remote_path: Remote file path
            
        Returns:
            True if successful, False otherwise
        """
        try:
            # Delete file
            self.s3_client.delete_object(
                Bucket=self.s3_bucket,
                Key=remote_path
            )
            
            logger.info(f"Deleted file from S3: {remote_path}")
            
            return True
        
        except Exception as e:
            logger.error(f"Failed to delete file from S3: {e}")
            return False
    
    async def _delete_from_local(self, remote_path: str) -> bool:
        """
        Delete a file from local storage.
        
        Args:
            remote_path: Remote file path
            
        Returns:
            True if successful, False otherwise
        """
        try:
            # Get file path
            file_path = os.path.join(self.local_storage_path, remote_path)
            
            # Check if file exists
            if not os.path.isfile(file_path):
                logger.warning(f"File not found in local storage: {remote_path}")
                return False
            
            # Delete file
            os.remove(file_path)
            
            logger.info(f"Deleted file from local storage: {remote_path}")
            
            return True
        
        except Exception as e:
            logger.error(f"Failed to delete file from local storage: {e}")
            return False
    
    async def _list_s3_files(self, prefix: str = "") -> List[Dict[str, Any]]:
        """
        List files in S3.
        
        Args:
            prefix: Optional prefix to filter files
            
        Returns:
            List of file information
        """
        try:
            # List objects
            response = self.s3_client.list_objects_v2(
                Bucket=self.s3_bucket,
                Prefix=prefix
            )
            
            # Extract file information
            files = []
            for obj in response.get("Contents", []):
                # Generate URL
                if self.s3_endpoint:
                    # Custom endpoint
                    url = f"{self.s3_endpoint}/{self.s3_bucket}/{obj['Key']}"
                    
                    # Remove double slashes in path
                    parsed = urlparse(url)
                    path = parsed.path.replace("//", "/")
                    url = f"{parsed.scheme}://{parsed.netloc}{path}"
                else:
                    # AWS S3
                    url = f"https://{self.s3_bucket}.s3.{self.s3_region}.amazonaws.com/{obj['Key']}"
                
                files.append({
                    "key": obj["Key"],
                    "size": obj["Size"],
                    "last_modified": obj["LastModified"].isoformat(),
                    "url": url
                })
            
            return files
        
        except Exception as e:
            logger.error(f"Failed to list files in S3: {e}")
            return []
    
    async def _list_local_files(self, prefix: str = "") -> List[Dict[str, Any]]:
        """
        List files in local storage.
        
        Args:
            prefix: Optional prefix to filter files
            
        Returns:
            List of file information
        """
        try:
            # Get base directory
            base_dir = os.path.join(self.local_storage_path, prefix)
            
            # Check if directory exists
            if not os.path.isdir(base_dir):
                return []
            
            # List files
            files = []
            for root, _, filenames in os.walk(base_dir):
                for filename in filenames:
                    # Get file path
                    file_path = os.path.join(root, filename)
                    
                    # Get relative path
                    rel_path = os.path.relpath(file_path, self.local_storage_path)
                    
                    # Get file stats
                    stats = os.stat(file_path)
                    
                    # Generate URL
                    url = f"file://{file_path}"
                    
                    files.append({
                        "key": rel_path,
                        "size": stats.st_size,
                        "last_modified": datetime.fromtimestamp(stats.st_mtime).isoformat(),
                        "url": url
                    })
            
            return files
        
        except Exception as e:
            logger.error(f"Failed to list files in local storage: {e}")
            return []
