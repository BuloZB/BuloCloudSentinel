"""
HTML validation and sanitization utilities for Bulo.Cloud Sentinel.

This module provides functions for safely handling HTML content
to prevent XSS attacks and other security issues.
"""

import re
import html
from typing import List, Optional, Set
from fastapi import HTTPException, status


# Default allowed HTML tags and attributes
DEFAULT_ALLOWED_TAGS = {
    "a", "abbr", "acronym", "b", "blockquote", "br", "code", "div", "em",
    "h1", "h2", "h3", "h4", "h5", "h6", "hr", "i", "li", "ol", "p", "pre",
    "span", "strong", "table", "tbody", "td", "th", "thead", "tr", "ul"
}

DEFAULT_ALLOWED_ATTRIBUTES = {
    "a": {"href", "title", "target", "rel"},
    "abbr": {"title"},
    "acronym": {"title"},
    "blockquote": {"cite"},
    "div": {"class", "id"},
    "h1": {"class", "id"},
    "h2": {"class", "id"},
    "h3": {"class", "id"},
    "h4": {"class", "id"},
    "h5": {"class", "id"},
    "h6": {"class", "id"},
    "p": {"class", "id"},
    "pre": {"class", "id"},
    "span": {"class", "id"},
    "table": {"class", "id", "border", "cellpadding", "cellspacing"},
    "td": {"class", "id", "colspan", "rowspan"},
    "th": {"class", "id", "colspan", "rowspan", "scope"},
    "tr": {"class", "id"},
    "ul": {"class", "id"},
    "ol": {"class", "id"},
    "li": {"class", "id"}
}

# Regular expressions for HTML parsing
TAG_REGEX = re.compile(r'<(/?)([a-zA-Z][a-zA-Z0-9]*)((?:\s+[a-zA-Z][a-zA-Z0-9]*(?:\s*=\s*(?:"[^"]*"|\'[^\']*\'|[^\s>]*))?)*)\s*(/?)>', re.DOTALL)
ATTR_REGEX = re.compile(r'([a-zA-Z][a-zA-Z0-9]*)\s*(?:=\s*(?:"([^"]*)"|\'([^\']*)\'|([^\s>]*)))?')
COMMENT_REGEX = re.compile(r'<!--.*?-->', re.DOTALL)
SCRIPT_REGEX = re.compile(r'<script.*?>.*?</script>', re.DOTALL | re.IGNORECASE)
STYLE_REGEX = re.compile(r'<style.*?>.*?</style>', re.DOTALL | re.IGNORECASE)
EVENT_HANDLER_REGEX = re.compile(r'\s+on[a-zA-Z]+\s*=', re.IGNORECASE)
DATA_ATTR_REGEX = re.compile(r'\s+data-[a-zA-Z0-9-]+\s*=', re.IGNORECASE)
JAVASCRIPT_URL_REGEX = re.compile(r'javascript:', re.IGNORECASE)


def strip_all_tags(html_content: str) -> str:
    """
    Strip all HTML tags from a string.
    
    Args:
        html_content: The HTML content to strip
        
    Returns:
        The plain text content
    """
    if not html_content or not isinstance(html_content, str):
        return ""
    
    # Remove comments
    content = COMMENT_REGEX.sub('', html_content)
    
    # Remove script and style tags with their content
    content = SCRIPT_REGEX.sub('', content)
    content = STYLE_REGEX.sub('', content)
    
    # Remove all tags
    content = TAG_REGEX.sub('', content)
    
    # Decode HTML entities
    content = html.unescape(content)
    
    return content.strip()


def sanitize_html(
    html_content: str,
    allowed_tags: Optional[Set[str]] = None,
    allowed_attributes: Optional[dict] = None
) -> str:
    """
    Sanitize HTML content by removing disallowed tags and attributes.
    
    Args:
        html_content: The HTML content to sanitize
        allowed_tags: Set of allowed HTML tags
        allowed_attributes: Dictionary of allowed attributes for each tag
        
    Returns:
        The sanitized HTML content
    """
    if not html_content or not isinstance(html_content, str):
        return ""
    
    # Use default allowed tags and attributes if not provided
    if allowed_tags is None:
        allowed_tags = DEFAULT_ALLOWED_TAGS
    
    if allowed_attributes is None:
        allowed_attributes = DEFAULT_ALLOWED_ATTRIBUTES
    
    # Remove comments
    content = COMMENT_REGEX.sub('', html_content)
    
    # Remove script and style tags with their content
    content = SCRIPT_REGEX.sub('', content)
    content = STYLE_REGEX.sub('', content)
    
    # Process all tags
    result = []
    last_pos = 0
    
    for match in TAG_REGEX.finditer(content):
        # Add text before the tag
        result.append(content[last_pos:match.start()])
        
        closing, tag_name, attrs_str, self_closing = match.groups()
        tag_name = tag_name.lower()
        
        # Check if the tag is allowed
        if tag_name in allowed_tags:
            # Start building the sanitized tag
            sanitized_tag = f"<{closing}{tag_name}"
            
            # Process attributes
            if attrs_str and not closing:
                allowed_attrs = allowed_attributes.get(tag_name, set())
                
                for attr_match in ATTR_REGEX.finditer(attrs_str):
                    attr_name, val1, val2, val3 = attr_match.groups()
                    attr_name = attr_name.lower()
                    
                    # Use the first non-None value
                    attr_value = next((v for v in (val1, val2, val3) if v is not None), "")
                    
                    # Check if the attribute is allowed
                    if attr_name in allowed_attrs:
                        # Check for javascript: URLs
                        if attr_name == "href" and JAVASCRIPT_URL_REGEX.search(attr_value):
                            continue
                        
                        # Add the sanitized attribute
                        sanitized_tag += f' {attr_name}="{html.escape(attr_value)}"'
            
            # Close the tag
            sanitized_tag += f"{self_closing}>"
            result.append(sanitized_tag)
        
        last_pos = match.end()
    
    # Add the remaining text
    result.append(content[last_pos:])
    
    return "".join(result)


def detect_xss(content: str) -> bool:
    """
    Detect potential XSS attacks in content.
    
    Args:
        content: The content to check
        
    Returns:
        True if potential XSS is detected, False otherwise
    """
    if not content or not isinstance(content, str):
        return False
    
    # Check for script tags
    if SCRIPT_REGEX.search(content):
        return True
    
    # Check for event handlers
    if EVENT_HANDLER_REGEX.search(content):
        return True
    
    # Check for javascript: URLs
    if JAVASCRIPT_URL_REGEX.search(content):
        return True
    
    # Check for data attributes (potential for XSS in some contexts)
    if DATA_ATTR_REGEX.search(content):
        return True
    
    return False


def validate_html_content(
    html_content: str,
    max_length: Optional[int] = None,
    allowed_tags: Optional[Set[str]] = None,
    allowed_attributes: Optional[dict] = None
) -> str:
    """
    Validate and sanitize HTML content.
    
    Args:
        html_content: The HTML content to validate
        max_length: Maximum allowed length
        allowed_tags: Set of allowed HTML tags
        allowed_attributes: Dictionary of allowed attributes for each tag
        
    Returns:
        The validated and sanitized HTML content
        
    Raises:
        HTTPException: If validation fails
    """
    if not html_content or not isinstance(html_content, str):
        return ""
    
    # Check length
    if max_length and len(html_content) > max_length:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=f"HTML content exceeds maximum length of {max_length} characters"
        )
    
    # Detect potential XSS
    if detect_xss(html_content):
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Potential XSS attack detected in HTML content"
        )
    
    # Sanitize HTML
    return sanitize_html(html_content, allowed_tags, allowed_attributes)
