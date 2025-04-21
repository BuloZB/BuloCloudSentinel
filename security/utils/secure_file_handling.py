"""
Secure file handling utilities.

This module provides functions for securely handling files, including:
- Sanitizing filenames
- Validating file types
- Verifying file content
- Computing file hashes
"""

import os
import re
import hashlib
import mimetypes
from typing import List, Optional, Tuple

# Initialize mimetypes
mimetypes.init()

# Define safe file types
SAFE_CONTENT_TYPES = {
    # Images
    'image/jpeg': ['.jpg', '.jpeg'],
    'image/png': ['.png'],
    'image/gif': ['.gif'],
    'image/bmp': ['.bmp'],
    'image/tiff': ['.tiff', '.tif'],
    'image/webp': ['.webp'],
    
    # Videos
    'video/mp4': ['.mp4'],
    'video/mpeg': ['.mpeg', '.mpg'],
    'video/quicktime': ['.mov'],
    'video/x-msvideo': ['.avi'],
    'video/webm': ['.webm'],
    
    # Documents
    'application/pdf': ['.pdf'],
    'application/json': ['.json'],
    'application/xml': ['.xml'],
    'text/plain': ['.txt'],
    'text/csv': ['.csv'],
    'application/vnd.openxmlformats-officedocument.wordprocessingml.document': ['.docx'],
    'application/vnd.openxmlformats-officedocument.spreadsheetml.sheet': ['.xlsx'],
    'application/vnd.openxmlformats-officedocument.presentationml.presentation': ['.pptx'],
}

# Maximum file size (10MB)
MAX_FILE_SIZE = 10 * 1024 * 1024

def sanitize_filename(filename: str) -> str:
    """
    Sanitize a filename to prevent path traversal attacks.
    
    Args:
        filename: The original filename
        
    Returns:
        A sanitized filename
    """
    # Remove any directory components
    sanitized = os.path.basename(filename)
    
    # Remove any potentially dangerous characters
    sanitized = re.sub(r'[^\w\.-]', '_', sanitized)
    
    # Ensure the filename isn't too long
    if len(sanitized) > 255:
        sanitized = sanitized[:255]
        
    return sanitized

def is_safe_file_type(content_type: str, filename: str) -> bool:
    """
    Check if the file type is safe based on content type and extension.
    
    Args:
        content_type: The MIME type of the file
        filename: The filename
        
    Returns:
        True if the file type is safe, False otherwise
    """
    # Normalize content type
    content_type = content_type.lower()
    
    # Check if content type is in our safe list
    if content_type not in SAFE_CONTENT_TYPES:
        return False
        
    # Check file extension
    ext = os.path.splitext(filename.lower())[1]
    if ext not in SAFE_CONTENT_TYPES[content_type]:
        return False
        
    return True

def verify_file_content(file_path: str, claimed_content_type: str) -> bool:
    """
    Verify that the file content matches the claimed content type.
    
    Args:
        file_path: Path to the file
        claimed_content_type: The claimed MIME type
        
    Returns:
        True if the file content matches the claimed type, False otherwise
    """
    try:
        # Get file size
        file_size = os.path.getsize(file_path)
        if file_size > MAX_FILE_SIZE:
            return False
            
        # Check file signature (magic bytes)
        with open(file_path, 'rb') as f:
            header = f.read(8)  # Read first 8 bytes
            
        # Check common file signatures
        if claimed_content_type == 'image/jpeg' and not header.startswith(b'\xFF\xD8\xFF'):
            return False
        elif claimed_content_type == 'image/png' and not header.startswith(b'\x89PNG\r\n\x1A\n'):
            return False
        elif claimed_content_type == 'image/gif' and not (header.startswith(b'GIF87a') or header.startswith(b'GIF89a')):
            return False
        elif claimed_content_type == 'application/pdf' and not header.startswith(b'%PDF'):
            return False
            
        return True
    except Exception:
        # If there's any error in detection, fail closed
        return False

def compute_file_hash(file_path: str) -> str:
    """
    Compute SHA-256 hash of a file.
    
    Args:
        file_path: Path to the file
        
    Returns:
        SHA-256 hash of the file
    """
    sha256_hash = hashlib.sha256()
    with open(file_path, "rb") as f:
        # Read and update hash in chunks of 4K
        for byte_block in iter(lambda: f.read(4096), b""):
            sha256_hash.update(byte_block)
    return sha256_hash.hexdigest()

def validate_file(file_path: str, content_type: str, original_filename: str) -> Tuple[bool, Optional[str]]:
    """
    Validate a file for security.
    
    Args:
        file_path: Path to the file
        content_type: The claimed MIME type
        original_filename: The original filename
        
    Returns:
        Tuple of (is_valid, error_message)
    """
    # Check file size
    try:
        file_size = os.path.getsize(file_path)
        if file_size > MAX_FILE_SIZE:
            return False, f"File exceeds maximum size of {MAX_FILE_SIZE/1024/1024}MB"
    except Exception as e:
        return False, f"Error checking file size: {str(e)}"
        
    # Check file type
    if not is_safe_file_type(content_type, original_filename):
        return False, "Unsupported file type"
        
    # Verify file content
    if not verify_file_content(file_path, content_type):
        return False, "File content does not match claimed type"
        
    return True, None
