#!/usr/bin/env python3
"""
Script to automatically add SRI hashes to HTML templates.

This script scans HTML files for script and link tags, fetches the resources,
generates SRI hashes, and adds them to the tags.
"""

import os
import sys
import argparse
import re
from pathlib import Path
from typing import Dict, List, Set, Tuple

# Add project root to path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from security.utils.sri_helper import fetch_and_generate_sri, add_sri_to_html

def find_html_files(directory: str) -> List[str]:
    """
    Find all HTML files in a directory.
    
    Args:
        directory: Directory to search
        
    Returns:
        List of HTML file paths
    """
    html_files = []
    for root, _, files in os.walk(directory):
        for file in files:
            if file.endswith('.html'):
                html_files.append(os.path.join(root, file))
    return html_files

def extract_resources(html_content: str) -> Set[str]:
    """
    Extract resource URLs from HTML content.
    
    Args:
        html_content: HTML content
        
    Returns:
        Set of resource URLs
    """
    resources = set()
    
    # Extract script src
    script_pattern = re.compile(r'<script[^>]+src="([^"]+)"[^>]*></script>')
    for match in script_pattern.finditer(html_content):
        url = match.group(1)
        if url.startswith(('http://', 'https://')):
            resources.add(url)
    
    # Extract link href
    link_pattern = re.compile(r'<link[^>]+href="([^"]+)"[^>]*>')
    for match in link_pattern.finditer(html_content):
        url = match.group(1)
        if url.startswith(('http://', 'https://')) and 'stylesheet' in match.group(0):
            resources.add(url)
    
    return resources

def process_html_file(file_path: str, dry_run: bool = False) -> None:
    """
    Process an HTML file to add SRI hashes.
    
    Args:
        file_path: Path to HTML file
        dry_run: If True, don't modify the file
    """
    print(f"Processing {file_path}...")
    
    # Read HTML content
    with open(file_path, 'r', encoding='utf-8') as f:
        html_content = f.read()
    
    # Extract resources
    resources = extract_resources(html_content)
    print(f"Found {len(resources)} external resources")
    
    # Generate SRI hashes
    resource_hashes = {}
    for url in resources:
        try:
            print(f"Fetching {url}...")
            _, sri_hash = fetch_and_generate_sri(url)
            resource_hashes[url] = sri_hash
            print(f"Generated SRI hash: {sri_hash}")
        except Exception as e:
            print(f"Error fetching {url}: {str(e)}")
    
    # Add SRI hashes to HTML
    new_html_content = add_sri_to_html(html_content, resource_hashes)
    
    # Write modified HTML
    if not dry_run and new_html_content != html_content:
        with open(file_path, 'w', encoding='utf-8') as f:
            f.write(new_html_content)
        print(f"Updated {file_path}")
    elif new_html_content != html_content:
        print(f"Would update {file_path} (dry run)")
    else:
        print(f"No changes needed for {file_path}")

def main():
    """Main function."""
    parser = argparse.ArgumentParser(description='Add SRI hashes to HTML templates')
    parser.add_argument('path', help='Path to HTML file or directory')
    parser.add_argument('--dry-run', action='store_true', help='Don\'t modify files')
    args = parser.parse_args()
    
    path = Path(args.path)
    
    if path.is_file() and path.suffix == '.html':
        # Process single file
        process_html_file(str(path), args.dry_run)
    elif path.is_dir():
        # Process directory
        html_files = find_html_files(str(path))
        print(f"Found {len(html_files)} HTML files")
        
        for file_path in html_files:
            process_html_file(file_path, args.dry_run)
    else:
        print(f"Error: {args.path} is not an HTML file or directory")
        sys.exit(1)

if __name__ == '__main__':
    main()