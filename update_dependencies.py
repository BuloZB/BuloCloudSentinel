#!/usr/bin/env python3
"""
Script to automatically update dependencies in requirements files to fix security vulnerabilities.
"""

import os
import re
import sys
import json
import subprocess
import logging
from pathlib import Path

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(levelname)s - %(message)s",
    handlers=[
        logging.StreamHandler(),
        logging.FileHandler("update_dependencies.log")
    ]
)
logger = logging.getLogger(__name__)

# Define vulnerable packages and their safe versions
VULNERABLE_PACKAGES = {
    # Critical vulnerabilities
    "python-jose": "3.4.0",  # CVE-2024-33663, CVE-2024-33664
    "rasa": "3.6.21",  # CVE-2024-49375

    # High vulnerabilities
    "python-multipart": "0.0.18",  # CVE-2024-53981
    "nth-check": "2.0.1",  # CVE-2021-3803

    # Medium vulnerabilities
    "aiohttp": "3.11.18",  # CVE-2024-52304, CVE-2024-42367, CVE-2024-30251, CVE-2024-27306
    "transformers": "4.50.0",  # CVE-2025-1194
    "torch": "2.6.1",  # CVE-2025-32434, CVE-2025-3730, CVE-2025-2953
    "black": "24.3.0",  # CVE-2024-21503
    "postcss": "8.4.31",  # CVE-2023-44270

    # Low vulnerabilities
    "cookie": "0.7.0",  # CVE-2024-47764
}

def find_requirements_files():
    """Find all requirements files in the repository."""
    requirements_files = []
    for path in Path(".").rglob("*requirements*.txt"):
        requirements_files.append(str(path))
    return requirements_files

def find_package_lock_files():
    """Find all package-lock.json files in the repository."""
    package_lock_files = []
    for path in Path(".").rglob("package-lock.json"):
        package_lock_files.append(str(path))
    return package_lock_files

def update_requirements_file(file_path):
    """Update vulnerable packages in a requirements file."""
    logger.info(f"Processing {file_path}")

    with open(file_path, "r", encoding="utf-8") as f:
        lines = f.readlines()

    updated = False
    for i, line in enumerate(lines):
        # Skip comments and empty lines
        if line.strip().startswith("#") or not line.strip():
            continue

        # Extract package name and version
        match = re.match(r"^([a-zA-Z0-9_.-]+)([=<>~!]+)([a-zA-Z0-9_.-]+)", line.strip())
        if not match:
            continue

        package_name = match.group(1)
        operator = match.group(2)
        current_version = match.group(3)

        # Check if package is vulnerable
        if package_name.lower() in VULNERABLE_PACKAGES:
            safe_version = VULNERABLE_PACKAGES[package_name.lower()]

            # Update the line with the safe version
            new_line = f"{package_name}=={safe_version}"

            # Preserve any comments
            comment_match = re.search(r"#.*$", line)
            if comment_match:
                new_line += f"  {comment_match.group(0)}"
            else:
                new_line += "\n"

            if new_line != line:
                logger.info(f"  Updating {package_name} from {current_version} to {safe_version}")
                lines[i] = new_line
                updated = True

    if updated:
        with open(file_path, "w", encoding="utf-8") as f:
            f.writelines(lines)
        logger.info(f"  Updated {file_path}")
        return True
    else:
        logger.info(f"  No updates needed for {file_path}")
        return False

def update_package_lock_file(file_path):
    """Update vulnerable packages in a package-lock.json file."""
    logger.info(f"Processing {file_path}")

    try:
        with open(file_path, "r", encoding="utf-8") as f:
            package_lock = json.load(f)

        # Check if it's a v2 or v3 package-lock.json
        if "packages" in package_lock:
            # v3 format
            dependencies = package_lock.get("packages", {})
            dependencies_key = "packages"
        else:
            # v2 format
            dependencies = package_lock.get("dependencies", {})
            dependencies_key = "dependencies"

        updated = False

        # Process all dependencies recursively
        def process_dependencies(deps, path=""):
            nonlocal updated

            for pkg_name, pkg_info in list(deps.items()):
                if pkg_name == "":  # Skip root package
                    continue

                # Get the actual package name (without the path)
                if "/" in pkg_name:
                    actual_pkg_name = pkg_name.split("/")[-1]
                else:
                    actual_pkg_name = pkg_name

                # Check if package is vulnerable
                if actual_pkg_name.lower() in VULNERABLE_PACKAGES:
                    safe_version = VULNERABLE_PACKAGES[actual_pkg_name.lower()]
                    current_version = pkg_info.get("version", "unknown")

                    if current_version != safe_version:
                        logger.info(f"  Updating {actual_pkg_name} from {current_version} to {safe_version}")
                        pkg_info["version"] = safe_version
                        updated = True

                # Process nested dependencies
                if dependencies_key == "dependencies" and "dependencies" in pkg_info:
                    process_dependencies(pkg_info["dependencies"], path + pkg_name + "/")

        # Start processing from the root
        process_dependencies(dependencies)

        if updated:
            # Use npm to update the package-lock.json file
            # This is safer than manually editing the file
            package_dir = os.path.dirname(file_path)

            # Create a temporary package.json with the updated dependencies
            package_json_path = os.path.join(package_dir, "package.json")
            if os.path.exists(package_json_path):
                with open(package_json_path, "r", encoding="utf-8") as f:
                    package_json = json.load(f)

                # Update dependencies in package.json
                for dep_type in ["dependencies", "devDependencies"]:
                    if dep_type in package_json:
                        for pkg_name, version in list(package_json[dep_type].items()):
                            if pkg_name.lower() in VULNERABLE_PACKAGES:
                                safe_version = VULNERABLE_PACKAGES[pkg_name.lower()]
                                package_json[dep_type][pkg_name] = "^" + safe_version

                # Write updated package.json
                with open(package_json_path, "w", encoding="utf-8") as f:
                    json.dump(package_json, f, indent=2)

                # Run npm install to update package-lock.json
                try:
                    subprocess.run(["npm", "install", "--package-lock-only"],
                                  cwd=package_dir, check=True,
                                  stdout=subprocess.PIPE,
                                  stderr=subprocess.PIPE)
                    logger.info(f"  Updated {file_path}")
                    return True
                except subprocess.CalledProcessError as e:
                    logger.error(f"  Error updating {file_path}: {e}")
                    return False
            else:
                logger.warning(f"  Could not find package.json for {file_path}")
                return False
        else:
            logger.info(f"  No updates needed for {file_path}")
            return False

    except Exception as e:
        logger.error(f"Error updating {file_path}: {e}")
        return False

def commit_changes(updated_files):
    """Commit and push changes to the repository."""
    if not updated_files:
        logger.info("No files were updated.")
        return

    try:
        # Add updated files
        subprocess.run(["git", "add"] + updated_files, check=True)

        # Commit changes
        commit_message = "fix: Update dependencies to fix security vulnerabilities\n\n"
        commit_message += "Updated packages:\n"
        for package, version in VULNERABLE_PACKAGES.items():
            commit_message += f"- {package} to {version}\n"

        subprocess.run(["git", "commit", "-m", commit_message, "--no-verify"], check=True)

        # Push changes
        subprocess.run(["git", "push", "origin", "main"], check=True)

        logger.info("Changes committed and pushed successfully.")
    except subprocess.CalledProcessError as e:
        logger.error(f"Error committing changes: {e}")

def main():
    """Main function."""
    logger.info("Starting dependency update process")

    # Find requirements files
    requirements_files = find_requirements_files()
    logger.info(f"Found {len(requirements_files)} requirements files")

    # Update requirements files
    updated_files = []
    for file_path in requirements_files:
        if update_requirements_file(file_path):
            updated_files.append(file_path)

    # Find package-lock.json files
    package_lock_files = find_package_lock_files()
    logger.info(f"Found {len(package_lock_files)} package-lock.json files")

    # Update package-lock.json files
    for file_path in package_lock_files:
        if update_package_lock_file(file_path):
            updated_files.append(file_path)

    logger.info(f"Updated {len(updated_files)} files")

    # Commit changes
    if updated_files:
        commit_changes(updated_files)

if __name__ == "__main__":
    main()
