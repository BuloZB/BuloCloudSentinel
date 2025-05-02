"""
Security scanner for Bulo.Cloud Sentinel.

This module provides security scanning of the codebase and dependencies.
"""

import os
import sys
import logging
import subprocess
import tempfile
import json
import re
from typing import Dict, Any, Optional, List, Tuple
from pathlib import Path

# Add project root to path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))

# Import local modules
from ai.inference.config import ConfigManager
from ai.inference.monitoring import structured_logger, alert_manager
from ai.inference.audit import audit_logger

# Setup logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Create configuration manager
config = ConfigManager()


class SecurityScanner:
    """
    Security scanner for Bulo.Cloud Sentinel.
    
    This class provides security scanning of the codebase and dependencies.
    """
    
    def __init__(
        self,
        scan_dir: Optional[str] = None,
        report_dir: Optional[str] = None,
        scan_interval: int = 86400,  # 1 day
        max_reports: int = 10
    ):
        """
        Initialize the security scanner.
        
        Args:
            scan_dir: Directory to scan
            report_dir: Directory for scan reports
            scan_interval: Scan interval in seconds
            max_reports: Maximum number of reports to keep
        """
        # Set scan directory
        self.scan_dir = scan_dir or os.path.abspath(
            os.path.join(os.path.dirname(__file__), '../..')
        )
        
        # Set report directory
        self.report_dir = report_dir or os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            "../../reports/security"
        )
        
        # Create report directory if it doesn't exist
        os.makedirs(self.report_dir, exist_ok=True)
        
        # Set scan parameters
        self.scan_interval = scan_interval
        self.max_reports = max_reports
    
    def scan_dependencies(self) -> Dict[str, Any]:
        """
        Scan dependencies for vulnerabilities.
        
        Returns:
            Scan results
        """
        try:
            # Create temporary directory
            with tempfile.TemporaryDirectory() as temp_dir:
                # Create requirements file path
                requirements_path = os.path.join(self.scan_dir, "requirements.txt")
                
                # Check if requirements file exists
                if not os.path.exists(requirements_path):
                    structured_logger.warning(
                        "Requirements file not found",
                        logger_name="security_scanner",
                        requirements_path=requirements_path
                    )
                    
                    return {
                        "success": False,
                        "message": "Requirements file not found",
                        "vulnerabilities": []
                    }
                
                # Create output file path
                output_path = os.path.join(temp_dir, "safety_report.json")
                
                # Run safety check
                try:
                    subprocess.run([
                        "safety", "check",
                        "-r", requirements_path,
                        "--json",
                        "-o", output_path
                    ], check=True)
                except subprocess.CalledProcessError:
                    # Safety check found vulnerabilities
                    pass
                
                # Check if output file exists
                if not os.path.exists(output_path):
                    structured_logger.warning(
                        "Safety check failed",
                        logger_name="security_scanner"
                    )
                    
                    return {
                        "success": False,
                        "message": "Safety check failed",
                        "vulnerabilities": []
                    }
                
                # Load safety report
                with open(output_path, "r") as f:
                    safety_report = json.load(f)
                
                # Process vulnerabilities
                vulnerabilities = []
                for vulnerability in safety_report:
                    # Get vulnerability data
                    package_name = vulnerability[0]
                    affected_version = vulnerability[1]
                    fixed_version = vulnerability[2]
                    vulnerability_id = vulnerability[3]
                    vulnerability_info = vulnerability[4]
                    
                    # Add vulnerability to list
                    vulnerabilities.append({
                        "package_name": package_name,
                        "affected_version": affected_version,
                        "fixed_version": fixed_version,
                        "vulnerability_id": vulnerability_id,
                        "vulnerability_info": vulnerability_info
                    })
                
                # Create scan report
                scan_report = {
                    "success": True,
                    "message": f"Found {len(vulnerabilities)} vulnerabilities",
                    "vulnerabilities": vulnerabilities
                }
                
                # Save scan report
                self._save_report("dependencies", scan_report)
                
                # Log scan results
                structured_logger.info(
                    "Dependency scan completed",
                    logger_name="security_scanner",
                    vulnerabilities_count=len(vulnerabilities)
                )
                
                # Create alerts for vulnerabilities
                for vulnerability in vulnerabilities:
                    alert_manager.create_alert(
                        level="warning",
                        message=f"Vulnerability found in {vulnerability['package_name']}: {vulnerability['vulnerability_info']}",
                        source="security_scanner",
                        data=vulnerability
                    )
                
                # Log audit event
                audit_logger.log_security_event(
                    event_name="dependency_scan",
                    severity="info" if not vulnerabilities else "warning",
                    source="security_scanner",
                    details={
                        "vulnerabilities_count": len(vulnerabilities),
                        "scan_report": scan_report
                    }
                )
                
                return scan_report
        
        except Exception as e:
            # Log error
            structured_logger.error(
                "Error scanning dependencies",
                logger_name="security_scanner",
                error=str(e)
            )
            
            # Create alert
            alert_manager.create_alert(
                level="error",
                message=f"Error scanning dependencies: {str(e)}",
                source="security_scanner"
            )
            
            # Log audit event
            audit_logger.log_security_event(
                event_name="dependency_scan_error",
                severity="error",
                source="security_scanner",
                details={
                    "error": str(e)
                }
            )
            
            return {
                "success": False,
                "message": f"Error scanning dependencies: {str(e)}",
                "vulnerabilities": []
            }
    
    def scan_code(self) -> Dict[str, Any]:
        """
        Scan code for vulnerabilities.
        
        Returns:
            Scan results
        """
        try:
            # Create temporary directory
            with tempfile.TemporaryDirectory() as temp_dir:
                # Create output file path
                output_path = os.path.join(temp_dir, "bandit_report.json")
                
                # Run bandit scan
                try:
                    subprocess.run([
                        "bandit",
                        "-r", self.scan_dir,
                        "-f", "json",
                        "-o", output_path,
                        "--exclude", ".git,venv,env,node_modules"
                    ], check=True)
                except subprocess.CalledProcessError:
                    # Bandit scan found vulnerabilities
                    pass
                
                # Check if output file exists
                if not os.path.exists(output_path):
                    structured_logger.warning(
                        "Bandit scan failed",
                        logger_name="security_scanner"
                    )
                    
                    return {
                        "success": False,
                        "message": "Bandit scan failed",
                        "vulnerabilities": []
                    }
                
                # Load bandit report
                with open(output_path, "r") as f:
                    bandit_report = json.load(f)
                
                # Process vulnerabilities
                vulnerabilities = []
                for result in bandit_report.get("results", []):
                    # Get vulnerability data
                    filename = result.get("filename", "")
                    line_number = result.get("line_number", 0)
                    issue_text = result.get("issue_text", "")
                    issue_severity = result.get("issue_severity", "")
                    issue_confidence = result.get("issue_confidence", "")
                    issue_cwe = result.get("issue_cwe", {}).get("id", "")
                    
                    # Add vulnerability to list
                    vulnerabilities.append({
                        "filename": filename,
                        "line_number": line_number,
                        "issue_text": issue_text,
                        "issue_severity": issue_severity,
                        "issue_confidence": issue_confidence,
                        "issue_cwe": issue_cwe
                    })
                
                # Create scan report
                scan_report = {
                    "success": True,
                    "message": f"Found {len(vulnerabilities)} vulnerabilities",
                    "vulnerabilities": vulnerabilities,
                    "metrics": bandit_report.get("metrics", {})
                }
                
                # Save scan report
                self._save_report("code", scan_report)
                
                # Log scan results
                structured_logger.info(
                    "Code scan completed",
                    logger_name="security_scanner",
                    vulnerabilities_count=len(vulnerabilities)
                )
                
                # Create alerts for high severity vulnerabilities
                for vulnerability in vulnerabilities:
                    if vulnerability["issue_severity"] == "HIGH":
                        alert_manager.create_alert(
                            level="warning",
                            message=f"High severity vulnerability found in {vulnerability['filename']}: {vulnerability['issue_text']}",
                            source="security_scanner",
                            data=vulnerability
                        )
                
                # Log audit event
                audit_logger.log_security_event(
                    event_name="code_scan",
                    severity="info" if not vulnerabilities else "warning",
                    source="security_scanner",
                    details={
                        "vulnerabilities_count": len(vulnerabilities),
                        "scan_report": scan_report
                    }
                )
                
                return scan_report
        
        except Exception as e:
            # Log error
            structured_logger.error(
                "Error scanning code",
                logger_name="security_scanner",
                error=str(e)
            )
            
            # Create alert
            alert_manager.create_alert(
                level="error",
                message=f"Error scanning code: {str(e)}",
                source="security_scanner"
            )
            
            # Log audit event
            audit_logger.log_security_event(
                event_name="code_scan_error",
                severity="error",
                source="security_scanner",
                details={
                    "error": str(e)
                }
            )
            
            return {
                "success": False,
                "message": f"Error scanning code: {str(e)}",
                "vulnerabilities": []
            }
    
    def scan_secrets(self) -> Dict[str, Any]:
        """
        Scan code for secrets.
        
        Returns:
            Scan results
        """
        try:
            # Create temporary directory
            with tempfile.TemporaryDirectory() as temp_dir:
                # Create output file path
                output_path = os.path.join(temp_dir, "detect_secrets_report.json")
                
                # Run detect-secrets scan
                try:
                    subprocess.run([
                        "detect-secrets", "scan",
                        self.scan_dir,
                        "--all-files",
                        "--exclude-files", ".git/|venv/|env/|node_modules/",
                        "-f", "json",
                        "-o", output_path
                    ], check=True)
                except subprocess.CalledProcessError:
                    # Detect-secrets scan failed
                    structured_logger.warning(
                        "Detect-secrets scan failed",
                        logger_name="security_scanner"
                    )
                    
                    return {
                        "success": False,
                        "message": "Detect-secrets scan failed",
                        "secrets": []
                    }
                
                # Check if output file exists
                if not os.path.exists(output_path):
                    structured_logger.warning(
                        "Detect-secrets scan failed",
                        logger_name="security_scanner"
                    )
                    
                    return {
                        "success": False,
                        "message": "Detect-secrets scan failed",
                        "secrets": []
                    }
                
                # Load detect-secrets report
                with open(output_path, "r") as f:
                    secrets_report = json.load(f)
                
                # Process secrets
                secrets = []
                for filename, file_results in secrets_report.get("results", {}).items():
                    for secret in file_results:
                        # Get secret data
                        line_number = secret.get("line_number", 0)
                        type = secret.get("type", "")
                        
                        # Add secret to list
                        secrets.append({
                            "filename": filename,
                            "line_number": line_number,
                            "type": type
                        })
                
                # Create scan report
                scan_report = {
                    "success": True,
                    "message": f"Found {len(secrets)} potential secrets",
                    "secrets": secrets
                }
                
                # Save scan report
                self._save_report("secrets", scan_report)
                
                # Log scan results
                structured_logger.info(
                    "Secrets scan completed",
                    logger_name="security_scanner",
                    secrets_count=len(secrets)
                )
                
                # Create alerts for secrets
                if secrets:
                    alert_manager.create_alert(
                        level="warning",
                        message=f"Found {len(secrets)} potential secrets in the codebase",
                        source="security_scanner",
                        data={"secrets_count": len(secrets)}
                    )
                
                # Log audit event
                audit_logger.log_security_event(
                    event_name="secrets_scan",
                    severity="info" if not secrets else "warning",
                    source="security_scanner",
                    details={
                        "secrets_count": len(secrets),
                        "scan_report": scan_report
                    }
                )
                
                return scan_report
        
        except Exception as e:
            # Log error
            structured_logger.error(
                "Error scanning for secrets",
                logger_name="security_scanner",
                error=str(e)
            )
            
            # Create alert
            alert_manager.create_alert(
                level="error",
                message=f"Error scanning for secrets: {str(e)}",
                source="security_scanner"
            )
            
            # Log audit event
            audit_logger.log_security_event(
                event_name="secrets_scan_error",
                severity="error",
                source="security_scanner",
                details={
                    "error": str(e)
                }
            )
            
            return {
                "success": False,
                "message": f"Error scanning for secrets: {str(e)}",
                "secrets": []
            }
    
    def scan_all(self) -> Dict[str, Any]:
        """
        Run all security scans.
        
        Returns:
            Scan results
        """
        try:
            # Log scan start
            structured_logger.info(
                "Starting security scan",
                logger_name="security_scanner"
            )
            
            # Run dependency scan
            dependency_scan = self.scan_dependencies()
            
            # Run code scan
            code_scan = self.scan_code()
            
            # Run secrets scan
            secrets_scan = self.scan_secrets()
            
            # Create scan report
            scan_report = {
                "success": True,
                "message": "Security scan completed",
                "dependency_scan": dependency_scan,
                "code_scan": code_scan,
                "secrets_scan": secrets_scan
            }
            
            # Save scan report
            self._save_report("all", scan_report)
            
            # Log scan results
            structured_logger.info(
                "Security scan completed",
                logger_name="security_scanner",
                dependency_vulnerabilities=len(dependency_scan.get("vulnerabilities", [])),
                code_vulnerabilities=len(code_scan.get("vulnerabilities", [])),
                secrets=len(secrets_scan.get("secrets", []))
            )
            
            # Log audit event
            audit_logger.log_security_event(
                event_name="security_scan",
                severity="info",
                source="security_scanner",
                details={
                    "dependency_vulnerabilities": len(dependency_scan.get("vulnerabilities", [])),
                    "code_vulnerabilities": len(code_scan.get("vulnerabilities", [])),
                    "secrets": len(secrets_scan.get("secrets", []))
                }
            )
            
            return scan_report
        
        except Exception as e:
            # Log error
            structured_logger.error(
                "Error running security scan",
                logger_name="security_scanner",
                error=str(e)
            )
            
            # Create alert
            alert_manager.create_alert(
                level="error",
                message=f"Error running security scan: {str(e)}",
                source="security_scanner"
            )
            
            # Log audit event
            audit_logger.log_security_event(
                event_name="security_scan_error",
                severity="error",
                source="security_scanner",
                details={
                    "error": str(e)
                }
            )
            
            return {
                "success": False,
                "message": f"Error running security scan: {str(e)}"
            }
    
    def _save_report(self, scan_type: str, report: Dict[str, Any]):
        """
        Save a scan report.
        
        Args:
            scan_type: Type of scan
            report: Scan report
        """
        try:
            # Create report file path
            import datetime
            
            report_file = os.path.join(
                self.report_dir,
                f"{scan_type}_scan_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
            )
            
            # Save report
            with open(report_file, "w") as f:
                json.dump(report, f, indent=2)
            
            # Set secure permissions for report file
            os.chmod(report_file, 0o600)
            
            # Clean up old reports
            self._clean_reports(scan_type)
        
        except Exception as e:
            # Log error
            structured_logger.error(
                "Error saving scan report",
                logger_name="security_scanner",
                error=str(e)
            )
    
    def _clean_reports(self, scan_type: str):
        """
        Clean up old scan reports.
        
        Args:
            scan_type: Type of scan
        """
        try:
            # Get scan reports
            report_files = []
            for file_path in Path(self.report_dir).glob(f"{scan_type}_scan_*.json"):
                # Get file modification time
                mod_time = file_path.stat().st_mtime
                
                # Add file to list
                report_files.append((file_path, mod_time))
            
            # Sort files by modification time (newest first)
            report_files.sort(key=lambda x: x[1], reverse=True)
            
            # Delete old files
            for file_path, _ in report_files[self.max_reports:]:
                try:
                    # Delete file
                    os.remove(file_path)
                    structured_logger.debug(
                        "Deleted old scan report",
                        logger_name="security_scanner",
                        file_path=str(file_path)
                    )
                except Exception as e:
                    structured_logger.error(
                        "Error deleting scan report",
                        logger_name="security_scanner",
                        error=str(e),
                        file_path=str(file_path)
                    )
        
        except Exception as e:
            # Log error
            structured_logger.error(
                "Error cleaning scan reports",
                logger_name="security_scanner",
                error=str(e)
            )


# Create global security scanner
security_scanner = SecurityScanner()
