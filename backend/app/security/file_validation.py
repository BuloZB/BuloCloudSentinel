"""
File validation utilities for the Bulo.Cloud Sentinel backend.

This module provides utilities for validating file uploads to prevent security vulnerabilities.
"""

import os
import magic
import hashlib
from typing import List, Set, Dict, Any, Optional, Tuple
import logging

logger = logging.getLogger(__name__)

# Allowed MIME types for file uploads
ALLOWED_MIME_TYPES = {
    # Images
    "image/jpeg",
    "image/png",
    "image/gif",
    "image/webp",
    "image/svg+xml",
    
    # Documents
    "application/pdf",
    "application/msword",
    "application/vnd.openxmlformats-officedocument.wordprocessingml.document",
    "application/vnd.ms-excel",
    "application/vnd.openxmlformats-officedocument.spreadsheetml.sheet",
    "application/vnd.ms-powerpoint",
    "application/vnd.openxmlformats-officedocument.presentationml.presentation",
    "text/plain",
    "text/csv",
    
    # Archives
    "application/zip",
    "application/x-tar",
    "application/x-gzip",
    
    # Others
    "application/json",
    "application/xml",
    "text/xml",
}

# Maximum file size in bytes (50 MB)
MAX_FILE_SIZE = 50 * 1024 * 1024

# Dangerous file extensions
DANGEROUS_EXTENSIONS = {
    ".exe", ".dll", ".bat", ".cmd", ".sh", ".php", ".phtml", ".asp", ".aspx",
    ".js", ".vbs", ".py", ".pl", ".rb", ".jar", ".war", ".jsp", ".jspx",
}


def validate_file_type(file_content: bytes, filename: str) -> Tuple[bool, str]:
    """
    Validate the file type using magic numbers.
    
    Args:
        file_content: The file content.
        filename: The file name.
        
    Returns:
        Tuple[bool, str]: A tuple containing a boolean indicating whether the file is valid
                          and a string containing the detected MIME type.
    """
    # Check file size
    if len(file_content) > MAX_FILE_SIZE:
        return False, f"File size exceeds maximum allowed size of {MAX_FILE_SIZE // (1024 * 1024)} MB"
    
    # Check file extension
    _, ext = os.path.splitext(filename.lower())
    if ext in DANGEROUS_EXTENSIONS:
        return False, f"File extension {ext} is not allowed"
    
    # Check MIME type using python-magic
    mime = magic.Magic(mime=True)
    detected_mime = mime.from_buffer(file_content)
    
    if detected_mime not in ALLOWED_MIME_TYPES:
        return False, f"File type {detected_mime} is not allowed"
    
    return True, detected_mime


def sanitize_filename(filename: str) -> str:
    """
    Sanitize a filename to prevent path traversal attacks.
    
    Args:
        filename: The filename to sanitize.
        
    Returns:
        str: The sanitized filename.
    """
    # Remove any directory components
    filename = os.path.basename(filename)
    
    # Replace potentially dangerous characters
    filename = filename.replace("\\", "").replace("/", "").replace(":", "").replace("*", "")
    filename = filename.replace("?", "").replace("\"", "").replace("<", "").replace(">", "")
    filename = filename.replace("|", "").replace("'", "").replace("`", "")
    
    # Ensure the filename is not empty
    if not filename:
        filename = "unnamed_file"
    
    return filename


def compute_file_hash(file_content: bytes) -> str:
    """
    Compute the SHA-256 hash of a file.
    
    Args:
        file_content: The file content.
        
    Returns:
        str: The SHA-256 hash of the file.
    """
    return hashlib.sha256(file_content).hexdigest()


def scan_file_for_malware(file_content: bytes) -> Tuple[bool, str]:
    """
    Scan a file for malware.
    
    This is a placeholder function. In a real implementation, this would
    integrate with an antivirus or malware scanning service.
    
    Args:
        file_content: The file content.
        
    Returns:
        Tuple[bool, str]: A tuple containing a boolean indicating whether the file is safe
                          and a string containing any error message.
    """
    # This is a placeholder. In a real implementation, this would
    # integrate with an antivirus or malware scanning service.
    return True, ""


def validate_file(file_content: bytes, filename: str) -> Tuple[bool, str, Dict[str, Any]]:
    """
    Validate a file for security.
    
    Args:
        file_content: The file content.
        filename: The file name.
        
    Returns:
        Tuple[bool, str, Dict[str, Any]]: A tuple containing a boolean indicating whether the file is valid,
                                          a string containing any error message, and a dictionary containing
                                          metadata about the file.
    """
    # Sanitize filename
    sanitized_filename = sanitize_filename(filename)
    
    # Validate file type
    is_valid_type, mime_type_or_error = validate_file_type(file_content, sanitized_filename)
    if not is_valid_type:
        return False, mime_type_or_error, {}
    
    # Scan for malware
    is_safe, malware_error = scan_file_for_malware(file_content)
    if not is_safe:
        return False, malware_error, {}
    
    # Compute file hash
    file_hash = compute_file_hash(file_content)
    
    # Return metadata
    metadata = {
        "filename": sanitized_filename,
        "mime_type": mime_type_or_error,
        "size": len(file_content),
        "hash": file_hash,
    }
    
    return True, "", metadata
