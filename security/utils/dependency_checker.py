"""
Dependency checker for Bulo.Cloud Sentinel Security Module.

This module provides utilities for checking dependencies for vulnerabilities.
"""

import os
import subprocess
import sys
import json
from typing import Dict, List, Optional, Any, Tuple

import pkg_resources
import requests
from packaging.version import parse as parse_version

# NVD API URL
NVD_API_URL = "https://services.nvd.nist.gov/rest/json/cves/2.0"

def get_installed_packages() -> Dict[str, str]:
    """
    Get installed Python packages.

    Returns:
        Dictionary of package name to version
    """
    return {pkg.key: pkg.version for pkg in pkg_resources.working_set}

def check_package_vulnerabilities(
    package_name: str,
    package_version: str
) -> List[Dict[str, Any]]:
    """
    Check a package for known vulnerabilities.

    Args:
        package_name: Package name
        package_version: Package version

    Returns:
        List of vulnerabilities
    """
    try:
        # Query NVD API
        params = {
            "keywordSearch": package_name,
            "keywordExactMatch": True
        }

        response = requests.get(NVD_API_URL, params=params, timeout=30)
        data = response.json()

        vulnerabilities = []

        # Process results
        if "vulnerabilities" in data:
            for vuln in data["vulnerabilities"]:
                cve = vuln["cve"]

                # Check if this vulnerability affects the package
                affected = False

                if "configurations" in cve:
                    for config in cve["configurations"]:
                        for node in config.get("nodes", []):
                            for cpe_match in node.get("cpeMatch", []):
                                cpe = cpe_match.get("criteria", "")

                                if package_name.lower() in cpe.lower():
                                    # Check version range
                                    if "versionStartIncluding" in cpe_match and "versionEndIncluding" in cpe_match:
                                        start_version = parse_version(cpe_match["versionStartIncluding"])
                                        end_version = parse_version(cpe_match["versionEndIncluding"])
                                        current_version = parse_version(package_version)

                                        if start_version <= current_version <= end_version:
                                            affected = True
                                    elif "versionStartIncluding" in cpe_match and "versionEndExcluding" in cpe_match:
                                        start_version = parse_version(cpe_match["versionStartIncluding"])
                                        end_version = parse_version(cpe_match["versionEndExcluding"])
                                        current_version = parse_version(package_version)

                                        if start_version <= current_version < end_version:
                                            affected = True
                                    elif "versionStartExcluding" in cpe_match and "versionEndIncluding" in cpe_match:
                                        start_version = parse_version(cpe_match["versionStartExcluding"])
                                        end_version = parse_version(cpe_match["versionEndIncluding"])
                                        current_version = parse_version(package_version)

                                        if start_version < current_version <= end_version:
                                            affected = True
                                    elif "versionStartExcluding" in cpe_match and "versionEndExcluding" in cpe_match:
                                        start_version = parse_version(cpe_match["versionStartExcluding"])
                                        end_version = parse_version(cpe_match["versionEndExcluding"])
                                        current_version = parse_version(package_version)

                                        if start_version < current_version < end_version:
                                            affected = True
                                    elif "versionStartIncluding" in cpe_match:
                                        start_version = parse_version(cpe_match["versionStartIncluding"])
                                        current_version = parse_version(package_version)

                                        if start_version <= current_version:
                                            affected = True
                                    elif "versionStartExcluding" in cpe_match:
                                        start_version = parse_version(cpe_match["versionStartExcluding"])
                                        current_version = parse_version(package_version)

                                        if start_version < current_version:
                                            affected = True
                                    elif "versionEndIncluding" in cpe_match:
                                        end_version = parse_version(cpe_match["versionEndIncluding"])
                                        current_version = parse_version(package_version)

                                        if current_version <= end_version:
                                            affected = True
                                    elif "versionEndExcluding" in cpe_match:
                                        end_version = parse_version(cpe_match["versionEndExcluding"])
                                        current_version = parse_version(package_version)

                                        if current_version < end_version:
                                            affected = True

                if affected:
                    # Get vulnerability details
                    vuln_id = cve["id"]
                    description = cve.get("descriptions", [{}])[0].get("value", "No description")

                    # Get severity
                    severity = "unknown"
                    cvss_score = 0.0

                    if "metrics" in cve:
                        if "cvssMetricV31" in cve["metrics"]:
                            cvss_data = cve["metrics"]["cvssMetricV31"][0]["cvssData"]
                            severity = cvss_data.get("baseSeverity", "unknown").lower()
                            cvss_score = cvss_data.get("baseScore", 0.0)
                        elif "cvssMetricV30" in cve["metrics"]:
                            cvss_data = cve["metrics"]["cvssMetricV30"][0]["cvssData"]
                            severity = cvss_data.get("baseSeverity", "unknown").lower()
                            cvss_score = cvss_data.get("baseScore", 0.0)
                        elif "cvssMetricV2" in cve["metrics"]:
                            cvss_data = cve["metrics"]["cvssMetricV2"][0]["cvssData"]
                            severity = cvss_data.get("baseSeverity", "unknown").lower()
                            cvss_score = cvss_data.get("baseScore", 0.0)

                    # Add to vulnerabilities
                    vulnerabilities.append({
                        "id": vuln_id,
                        "package": package_name,
                        "version": package_version,
                        "description": description,
                        "severity": severity,
                        "cvss_score": cvss_score
                    })

        return vulnerabilities

    except Exception as e:
        print(f"Error checking vulnerabilities for {package_name}: {str(e)}")
        return []

def check_all_packages() -> Dict[str, List[Dict[str, Any]]]:
    """
    Check all installed packages for vulnerabilities.

    Returns:
        Dictionary of package name to list of vulnerabilities
    """
    packages = get_installed_packages()
    vulnerabilities = {}

    for package_name, package_version in packages.items():
        package_vulns = check_package_vulnerabilities(package_name, package_version)

        if package_vulns:
            vulnerabilities[package_name] = package_vulns

    return vulnerabilities

def run_safety_check() -> Tuple[bool, List[Dict[str, Any]]]:
    """
    Run safety check using the safety package.

    Returns:
        Tuple of (success, vulnerabilities)
    """
    try:
        # Check if safety is installed
        try:
            import safety
        except ImportError:
            print("Safety package not installed. Installing...")
            subprocess.check_call([sys.executable, "-m", "pip", "install", "safety"])

        # Run safety check
        result = subprocess.run(
            [sys.executable, "-m", "safety", "check", "--json"],
            capture_output=True,
            text=True
        )

        # Parse output
        if result.returncode == 0:
            return True, []

        vulnerabilities = json.loads(result.stdout)
        return False, vulnerabilities

    except Exception as e:
        print(f"Error running safety check: {str(e)}")
        return False, []

def run_bandit_scan(
    directory: str,
    report_file: Optional[str] = None
) -> Tuple[bool, Dict[str, Any]]:
    """
    Run Bandit security scan on Python code.

    Args:
        directory: Directory to scan
        report_file: Optional file to save report

    Returns:
        Tuple of (success, results)
    """
    try:
        # Check if bandit is installed
        try:
            import bandit
        except ImportError:
            print("Bandit package not installed. Installing...")
            subprocess.check_call([sys.executable, "-m", "pip", "install", "bandit"])

        # Run bandit scan
        cmd = [sys.executable, "-m", "bandit", "-r", directory, "-f", "json"]

        if report_file:
            cmd.extend(["-o", report_file])

        result = subprocess.run(cmd, capture_output=True, text=True)

        # Parse output
        try:
            scan_results = json.loads(result.stdout)
            success = result.returncode == 0
            return success, scan_results
        except json.JSONDecodeError:
            print(f"Error parsing Bandit output: {result.stdout}")
            return False, {}

    except Exception as e:
        print(f"Error running Bandit scan: {str(e)}")
        return False, {}

def check_npm_packages(
    package_json_path: str
) -> Dict[str, List[Dict[str, Any]]]:
    """
    Check NPM packages for vulnerabilities.

    Args:
        package_json_path: Path to package.json file

    Returns:
        Dictionary of package name to list of vulnerabilities
    """
    try:
        # Check if npm is installed
        try:
            subprocess.run(["npm", "--version"], capture_output=True, check=True)
        except (subprocess.SubprocessError, FileNotFoundError):
            print("npm not found. Please install Node.js and npm.")
            return {}

        # Get directory containing package.json
        directory = os.path.dirname(os.path.abspath(package_json_path))

        # Run npm audit
        result = subprocess.run(
            ["npm", "audit", "--json"],
            cwd=directory,
            capture_output=True,
            text=True
        )

        # Parse output
        try:
            audit_results = json.loads(result.stdout)

            vulnerabilities = {}

            if "vulnerabilities" in audit_results:
                for package_name, vuln_info in audit_results["vulnerabilities"].items():
                    package_vulns = []

                    for via in vuln_info.get("via", []):
                        if isinstance(via, dict):
                            vuln_id = via.get("url", "").split("/")[-1]
                            description = via.get("title", "No description")
                            severity = via.get("severity", "unknown").lower()

                            package_vulns.append({
                                "id": vuln_id,
                                "package": package_name,
                                "version": vuln_info.get("version", "unknown"),
                                "description": description,
                                "severity": severity,
                                "fix": vuln_info.get("fixAvailable", False)
                            })

                    if package_vulns:
                        vulnerabilities[package_name] = package_vulns

            return vulnerabilities

        except json.JSONDecodeError:
            print(f"Error parsing npm audit output: {result.stdout}")
            return {}

    except Exception as e:
        print(f"Error checking NPM packages: {str(e)}")
        return {}
