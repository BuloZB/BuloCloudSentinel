#!/usr/bin/env python3
"""
Bulk update critical dependencies across all requirements files.
"""

import os
import re
import glob
from pathlib import Path

# Critical dependency updates
CRITICAL_UPDATES = {
    'python-multipart==0.0.18': 'python-multipart==0.0.20',
    'pyjwt==2.8.0': 'pyjwt==2.10.1',
    'pyjwt==2.9.0': 'pyjwt==2.10.1',
}

def update_requirements_file(file_path):
    """Update a single requirements file."""
    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()
        
        original_content = content
        updated = False
        
        # Apply critical updates
        for old_dep, new_dep in CRITICAL_UPDATES.items():
            if old_dep in content:
                content = content.replace(old_dep, new_dep)
                updated = True
                print(f"  Updated {old_dep} -> {new_dep}")
        
        # Additional pattern-based updates
        patterns = [
            (r'python-multipart==0\.0\.1[0-8]', 'python-multipart==0.0.20'),
            (r'pyjwt==2\.[0-9]\.0', 'pyjwt==2.10.1'),
        ]
        
        for pattern, replacement in patterns:
            if re.search(pattern, content):
                content = re.sub(pattern, replacement, content)
                updated = True
                print(f"  Applied pattern update: {pattern} -> {replacement}")
        
        if updated:
            with open(file_path, 'w', encoding='utf-8') as f:
                f.write(content)
            print(f"✅ Updated: {file_path}")
            return True
        else:
            print(f"⏭️  No updates needed: {file_path}")
            return False
            
    except Exception as e:
        print(f"❌ Error updating {file_path}: {e}")
        return False

def main():
    """Main function."""
    print("🔄 Starting bulk dependency update...")
    
    # Find all requirements files
    requirements_files = []
    
    # Common patterns for requirements files
    patterns = [
        '**/requirements.txt',
        '**/requirements-*.txt',
        'requirements*.txt'
    ]
    
    for pattern in patterns:
        files = glob.glob(pattern, recursive=True)
        requirements_files.extend(files)
    
    # Remove duplicates and sort
    requirements_files = sorted(list(set(requirements_files)))
    
    print(f"📋 Found {len(requirements_files)} requirements files:")
    for file in requirements_files:
        print(f"  - {file}")
    
    print("\n🔧 Updating files...")
    updated_count = 0
    
    for file_path in requirements_files:
        print(f"\n📄 Processing: {file_path}")
        if update_requirements_file(file_path):
            updated_count += 1
    
    print(f"\n✅ Bulk update completed!")
    print(f"📊 Updated {updated_count} out of {len(requirements_files)} files")
    
    if updated_count > 0:
        print("\n🚀 Next steps:")
        print("1. Review the changes")
        print("2. Commit and push to GitHub")
        print("3. Wait for GitHub to rescan dependencies")

if __name__ == "__main__":
    main()
