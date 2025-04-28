"""
Subresource Integrity (SRI) helper for Bulo.Cloud Sentinel.

This module provides utilities for implementing Subresource Integrity (SRI)
for external resources like JavaScript and CSS files.
"""

import hashlib
import base64
import requests
from typing import Dict, List, Optional, Tuple, Union
import logging

# Configure logging
logger = logging.getLogger(__name__)

def generate_sri_hash(content: Union[str, bytes], algorithms: List[str] = None) -> str:
    """
    Generate SRI hash for content.
    
    Args:
        content: Content to hash (string or bytes)
        algorithms: Hash algorithms to use (default: ["sha384"])
        
    Returns:
        SRI hash string
    """
    if algorithms is None:
        algorithms = ["sha384"]
    
    # Convert string to bytes if needed
    if isinstance(content, str):
        content = content.encode("utf-8")
    
    # Generate hashes for each algorithm
    hashes = []
    for algorithm in algorithms:
        if algorithm not in ["sha256", "sha384", "sha512"]:
            raise ValueError(f"Unsupported hash algorithm: {algorithm}")
        
        hash_obj = getattr(hashlib, algorithm)(content)
        hash_base64 = base64.b64encode(hash_obj.digest()).decode("utf-8")
        hashes.append(f"{algorithm}-{hash_base64}")
    
    return " ".join(hashes)

def fetch_and_generate_sri(url: str, algorithms: List[str] = None) -> Tuple[str, str]:
    """
    Fetch resource from URL and generate SRI hash.
    
    Args:
        url: URL to fetch
        algorithms: Hash algorithms to use (default: ["sha384"])
        
    Returns:
        Tuple of (content, SRI hash)
    """
    try:
        response = requests.get(url, timeout=10)
        response.raise_for_status()
        
        content = response.content
        sri_hash = generate_sri_hash(content, algorithms)
        
        return content.decode("utf-8") if isinstance(content, bytes) else content, sri_hash
    except Exception as e:
        logger.error(f"Error fetching resource from {url}: {str(e)}")
        raise

def add_sri_to_html(html: str, resources: Dict[str, str]) -> str:
    """
    Add SRI attributes to HTML resources.
    
    Args:
        html: HTML content
        resources: Dictionary mapping resource URLs to SRI hashes
        
    Returns:
        HTML with SRI attributes added
    """
    for url, sri_hash in resources.items():
        # For script tags
        html = html.replace(
            f'<script src="{url}"',
            f'<script src="{url}" integrity="{sri_hash}" crossorigin="anonymous"'
        )
        
        # For link tags (CSS)
        html = html.replace(
            f'<link rel="stylesheet" href="{url}"',
            f'<link rel="stylesheet" href="{url}" integrity="{sri_hash}" crossorigin="anonymous"'
        )
    
    return html

def verify_sri(content: Union[str, bytes], sri_hash: str) -> bool:
    """
    Verify content against SRI hash.
    
    Args:
        content: Content to verify
        sri_hash: SRI hash to verify against
        
    Returns:
        True if content matches SRI hash, False otherwise
    """
    # Parse SRI hash
    hash_parts = sri_hash.split(" ")
    
    for hash_part in hash_parts:
        algorithm, expected_hash = hash_part.split("-", 1)
        
        # Generate hash for content
        if isinstance(content, str):
            content = content.encode("utf-8")
        
        hash_obj = getattr(hashlib, algorithm)(content)
        actual_hash = base64.b64encode(hash_obj.digest()).decode("utf-8")
        
        # Compare hashes
        if actual_hash == expected_hash:
            return True
    
    return False