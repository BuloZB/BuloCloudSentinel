#!/usr/bin/env python3
"""
Install Git hooks for Bulo.Cloud Sentinel.

This script installs Git hooks to ensure code quality and security.
"""

import os
import shutil
import stat
from pathlib import Path

def main():
    """Install Git hooks."""
    # Get the repository root directory
    repo_root = Path(__file__).parent.parent
    
    # Source and destination paths for hooks
    hooks_dir = repo_root / ".git" / "hooks"
    
    # Ensure hooks directory exists
    hooks_dir.mkdir(exist_ok=True)
    
    # Install pre-commit hook
    pre_commit_src = repo_root / ".git" / "hooks" / "pre-commit"
    pre_commit_dst = hooks_dir / "pre-commit"
    
    if pre_commit_src.exists():
        # Copy the hook
        shutil.copy2(pre_commit_src, pre_commit_dst)
        
        # Make it executable
        st = os.stat(pre_commit_dst)
        os.chmod(pre_commit_dst, st.st_mode | stat.S_IEXEC)
        
        print(f"Installed pre-commit hook to {pre_commit_dst}")
    else:
        print(f"Pre-commit hook not found at {pre_commit_src}")
        return 1
    
    print("Git hooks installed successfully!")
    return 0

if __name__ == "__main__":
    exit(main())
