"""
File validation utilities for secure file uploads.

This module provides utilities for validating file uploads to prevent
security vulnerabilities such as path traversal, malicious file uploads,
and content type spoofing.
"""

import os
import re
import logging
import magic
import hashlib
from typing import List, Optional, Set, Tuple, Union
from fastapi import UploadFile, HTTPException, status

# Set up logging
log = logging.getLogger(__name__)

# Default allowed file extensions
DEFAULT_ALLOWED_EXTENSIONS = {
    # Images
    "jpg", "jpeg", "png", "gif", "bmp", "svg", "webp",
    # Documents
    "pdf", "doc", "docx", "xls", "xlsx", "ppt", "pptx", "txt", "csv",
    # Archives
    "zip", "tar", "gz", "7z", "rar",
    # Audio
    "mp3", "wav", "ogg", "flac",
    # Video
    "mp4", "avi", "mov", "wmv", "webm",
}

# Default allowed MIME types
DEFAULT_ALLOWED_MIME_TYPES = {
    # Images
    "image/jpeg", "image/png", "image/gif", "image/bmp", "image/svg+xml", "image/webp",
    # Documents
    "application/pdf", 
    "application/msword", "application/vnd.openxmlformats-officedocument.wordprocessingml.document",
    "application/vnd.ms-excel", "application/vnd.openxmlformats-officedocument.spreadsheetml.sheet",
    "application/vnd.ms-powerpoint", "application/vnd.openxmlformats-officedocument.presentationml.presentation",
    "text/plain", "text/csv",
    # Archives
    "application/zip", "application/x-tar", "application/gzip", "application/x-7z-compressed", "application/x-rar-compressed",
    # Audio
    "audio/mpeg", "audio/wav", "audio/ogg", "audio/flac",
    # Video
    "video/mp4", "video/x-msvideo", "video/quicktime", "video/x-ms-wmv", "video/webm",
}

# High-risk file extensions that should be blocked
HIGH_RISK_EXTENSIONS = {
    # Executable
    "exe", "dll", "so", "bat", "cmd", "sh", "ps1", "vbs", "js", "msi", "com",
    # Scripts
    "php", "py", "rb", "pl", "cgi", "asp", "aspx", "jsp", "htaccess",
}

class FileValidator:
    """
    Validator for file uploads.
    
    This class provides methods for validating file uploads to prevent
    security vulnerabilities.
    """
    
    def __init__(
        self,
        allowed_extensions: Optional[Set[str]] = None,
        allowed_mime_types: Optional[Set[str]] = None,
        max_file_size: int = 10 * 1024 * 1024,  # 10 MB
        validate_content: bool = True,
        block_high_risk: bool = True,
    ):
        """
        Initialize the file validator.
        
        Args:
            allowed_extensions: Set of allowed file extensions
            allowed_mime_types: Set of allowed MIME types
            max_file_size: Maximum file size in bytes
            validate_content: Whether to validate file content
            block_high_risk: Whether to block high-risk file types
        """
        self.allowed_extensions = allowed_extensions or DEFAULT_ALLOWED_EXTENSIONS
        self.allowed_mime_types = allowed_mime_types or DEFAULT_ALLOWED_MIME_TYPES
        self.max_file_size = max_file_size
        self.validate_content = validate_content
        self.block_high_risk = block_high_risk
        
    async def validate(self, file: UploadFile) -> Tuple[bool, Optional[str]]:
        """
        Validate a file upload.
        
        Args:
            file: The uploaded file
            
        Returns:
            Tuple of (is_valid, error_message)
        """
        # Validate filename
        if not self._validate_filename(file.filename):
            return False, "Invalid filename"
            
        # Validate file extension
        extension = self._get_file_extension(file.filename)
        if not extension:
            return False, "Missing file extension"
            
        if extension.lower() not in self.allowed_extensions:
            return False, f"File extension '{extension}' not allowed"
            
        if self.block_high_risk and extension.lower() in HIGH_RISK_EXTENSIONS:
            return False, f"High-risk file extension '{extension}' not allowed"
            
        # Validate file size
        file_size = await self._get_file_size(file)
        if file_size > self.max_file_size:
            return False, f"File size exceeds maximum allowed size ({self.max_file_size} bytes)"
            
        # Validate file content
        if self.validate_content:
            # Read the first chunk of the file to determine its MIME type
            file.file.seek(0)
            chunk = await file.read(2048)  # Read first 2KB
            file.file.seek(0)  # Reset file position
            
            # Get MIME type using python-magic
            mime_type = magic.from_buffer(chunk, mime=True)
            
            if mime_type not in self.allowed_mime_types:
                return False, f"File content type '{mime_type}' not allowed"
                
            # Check if the MIME type matches the file extension
            if not self._validate_mime_type_matches_extension(mime_type, extension):
                return False, f"File content type '{mime_type}' does not match extension '{extension}'"
                
        return True, None
        
    def _validate_filename(self, filename: Optional[str]) -> bool:
        """
        Validate a filename.
        
        Args:
            filename: The filename to validate
            
        Returns:
            True if the filename is valid, False otherwise
        """
        if not filename:
            return False
            
        # Check for path traversal attempts
        if os.path.basename(filename) != filename:
            return False
            
        # Check for invalid characters
        if re.search(r'[<>:"/\\|?*\x00-\x1F]', filename):
            return False
            
        return True
        
    def _get_file_extension(self, filename: Optional[str]) -> Optional[str]:
        """
        Get the file extension from a filename.
        
        Args:
            filename: The filename
            
        Returns:
            The file extension, or None if there is no extension
        """
        if not filename:
            return None
            
        parts = filename.rsplit(".", 1)
        if len(parts) < 2:
            return None
            
        return parts[1].lower()
        
    async def _get_file_size(self, file: UploadFile) -> int:
        """
        Get the size of an uploaded file.
        
        Args:
            file: The uploaded file
            
        Returns:
            The file size in bytes
        """
        file.file.seek(0, os.SEEK_END)
        size = file.file.tell()
        file.file.seek(0)
        return size
        
    def _validate_mime_type_matches_extension(self, mime_type: str, extension: str) -> bool:
        """
        Validate that a MIME type matches a file extension.
        
        Args:
            mime_type: The MIME type
            extension: The file extension
            
        Returns:
            True if the MIME type matches the extension, False otherwise
        """
        # Map of file extensions to expected MIME types
        extension_mime_map = {
            # Images
            "jpg": ["image/jpeg"],
            "jpeg": ["image/jpeg"],
            "png": ["image/png"],
            "gif": ["image/gif"],
            "bmp": ["image/bmp"],
            "svg": ["image/svg+xml"],
            "webp": ["image/webp"],
            # Documents
            "pdf": ["application/pdf"],
            "doc": ["application/msword"],
            "docx": ["application/vnd.openxmlformats-officedocument.wordprocessingml.document"],
            "xls": ["application/vnd.ms-excel"],
            "xlsx": ["application/vnd.openxmlformats-officedocument.spreadsheetml.sheet"],
            "ppt": ["application/vnd.ms-powerpoint"],
            "pptx": ["application/vnd.openxmlformats-officedocument.presentationml.presentation"],
            "txt": ["text/plain"],
            "csv": ["text/csv", "text/plain"],
            # Archives
            "zip": ["application/zip"],
            "tar": ["application/x-tar"],
            "gz": ["application/gzip"],
            "7z": ["application/x-7z-compressed"],
            "rar": ["application/x-rar-compressed", "application/vnd.rar"],
            # Audio
            "mp3": ["audio/mpeg"],
            "wav": ["audio/wav", "audio/x-wav"],
            "ogg": ["audio/ogg"],
            "flac": ["audio/flac"],
            # Video
            "mp4": ["video/mp4"],
            "avi": ["video/x-msvideo"],
            "mov": ["video/quicktime"],
            "wmv": ["video/x-ms-wmv"],
            "webm": ["video/webm"],
        }
        
        # Get expected MIME types for the extension
        expected_mime_types = extension_mime_map.get(extension.lower(), [])
        
        # If we don't have expected MIME types for this extension, allow it
        if not expected_mime_types:
            return True
            
        return mime_type in expected_mime_types
        
# Create a default file validator instance
default_file_validator = FileValidator()

async def validate_file_upload(
    file: UploadFile,
    allowed_extensions: Optional[Set[str]] = None,
    allowed_mime_types: Optional[Set[str]] = None,
    max_file_size: int = 10 * 1024 * 1024,  # 10 MB
    validate_content: bool = True,
) -> UploadFile:
    """
    Validate a file upload.
    
    This function can be used as a dependency in FastAPI routes to validate
    file uploads.
    
    Args:
        file: The uploaded file
        allowed_extensions: Set of allowed file extensions
        allowed_mime_types: Set of allowed MIME types
        max_file_size: Maximum file size in bytes
        validate_content: Whether to validate file content
        
    Returns:
        The validated file
        
    Raises:
        HTTPException: If the file is invalid
    """
    validator = FileValidator(
        allowed_extensions=allowed_extensions,
        allowed_mime_types=allowed_mime_types,
        max_file_size=max_file_size,
        validate_content=validate_content,
    )
    
    is_valid, error_message = await validator.validate(file)
    if not is_valid:
        log.warning(f"Invalid file upload: {error_message}")
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=error_message,
        )
        
    return file

async def compute_file_hash(file: UploadFile, algorithm: str = "sha256") -> str:
    """
    Compute a hash of a file.
    
    Args:
        file: The file to hash
        algorithm: The hash algorithm to use
        
    Returns:
        The file hash as a hexadecimal string
    """
    hash_obj = None
    if algorithm == "md5":
        hash_obj = hashlib.md5()
    elif algorithm == "sha1":
        hash_obj = hashlib.sha1()
    elif algorithm == "sha256":
        hash_obj = hashlib.sha256()
    elif algorithm == "sha512":
        hash_obj = hashlib.sha512()
    else:
        raise ValueError(f"Unsupported hash algorithm: {algorithm}")
        
    file.file.seek(0)
    
    # Read the file in chunks to avoid loading the entire file into memory
    chunk_size = 8192  # 8KB chunks
    while True:
        chunk = await file.read(chunk_size)
        if not chunk:
            break
        hash_obj.update(chunk)
        
    file.file.seek(0)
    
    return hash_obj.hexdigest()

def sanitize_filename(filename: str) -> str:
    """
    Sanitize a filename to make it safe for storage.
    
    Args:
        filename: The filename to sanitize
        
    Returns:
        The sanitized filename
    """
    # Remove path components
    filename = os.path.basename(filename)
    
    # Replace invalid characters with underscores
    filename = re.sub(r'[<>:"/\\|?*\x00-\x1F]', "_", filename)
    
    # Limit length
    if len(filename) > 255:
        name, ext = os.path.splitext(filename)
        name = name[:255 - len(ext)]
        filename = name + ext
        
    return filename
