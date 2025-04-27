"""
Security scanner for Bulo.Cloud Sentinel.

This module provides functionality for scanning the codebase for security issues.
"""

import os
import logging
import subprocess
import json
import re
import time
from typing import Dict, List, Optional, Any, Union, Tuple
from pathlib import Path
from datetime import datetime

# Configure logging
logger = logging.getLogger(__name__)


class SecurityIssue:
    """
    Security issue found by a scanner.
    
    This class represents a security issue found by a scanner.
    """
    
    def __init__(
        self,
        scanner: str,
        issue_type: str,
        severity: str,
        file_path: str,
        line_number: Optional[int] = None,
        message: str = "",
        code: Optional[str] = None,
        confidence: Optional[str] = None,
        cwe: Optional[Union[str, int]] = None,
        fix: Optional[str] = None,
        references: Optional[List[str]] = None
    ):
        """
        Initialize security issue.
        
        Args:
            scanner: Scanner that found the issue
            issue_type: Issue type
            severity: Issue severity
            file_path: File path
            line_number: Line number
            message: Issue message
            code: Code snippet
            confidence: Confidence level
            cwe: CWE identifier
            fix: Suggested fix
            references: References
        """
        self.scanner = scanner
        self.issue_type = issue_type
        self.severity = severity
        self.file_path = file_path
        self.line_number = line_number
        self.message = message
        self.code = code
        self.confidence = confidence
        self.cwe = cwe
        self.fix = fix
        self.references = references or []
        self.timestamp = time.time()
    
    def to_dict(self) -> Dict[str, Any]:
        """
        Convert issue to dictionary.
        
        Returns:
            Issue as dictionary
        """
        return {
            "scanner": self.scanner,
            "issue_type": self.issue_type,
            "severity": self.severity,
            "file_path": self.file_path,
            "line_number": self.line_number,
            "message": self.message,
            "code": self.code,
            "confidence": self.confidence,
            "cwe": self.cwe,
            "fix": self.fix,
            "references": self.references,
            "timestamp": self.timestamp,
            "timestamp_iso": datetime.fromtimestamp(self.timestamp).isoformat()
        }
    
    def to_json(self) -> str:
        """
        Convert issue to JSON.
        
        Returns:
            Issue as JSON string
        """
        return json.dumps(self.to_dict(), indent=2)
    
    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "SecurityIssue":
        """
        Create issue from dictionary.
        
        Args:
            data: Issue data
            
        Returns:
            Security issue
        """
        issue = cls(
            scanner=data["scanner"],
            issue_type=data["issue_type"],
            severity=data["severity"],
            file_path=data["file_path"],
            line_number=data.get("line_number"),
            message=data.get("message", ""),
            code=data.get("code"),
            confidence=data.get("confidence"),
            cwe=data.get("cwe"),
            fix=data.get("fix"),
            references=data.get("references", [])
        )
        
        if "timestamp" in data:
            issue.timestamp = data["timestamp"]
        
        return issue
    
    @classmethod
    def from_json(cls, json_str: str) -> "SecurityIssue":
        """
        Create issue from JSON.
        
        Args:
            json_str: Issue as JSON string
            
        Returns:
            Security issue
        """
        data = json.loads(json_str)
        return cls.from_dict(data)


class SecurityScanResult:
    """
    Result of a security scan.
    
    This class represents the result of a security scan.
    """
    
    def __init__(
        self,
        scanner: str,
        issues: Optional[List[SecurityIssue]] = None,
        start_time: Optional[float] = None,
        end_time: Optional[float] = None,
        scan_id: Optional[str] = None,
        scan_options: Optional[Dict[str, Any]] = None
    ):
        """
        Initialize security scan result.
        
        Args:
            scanner: Scanner name
            issues: Security issues
            start_time: Scan start time
            end_time: Scan end time
            scan_id: Scan ID
            scan_options: Scan options
        """
        self.scanner = scanner
        self.issues = issues or []
        self.start_time = start_time or time.time()
        self.end_time = end_time
        self.scan_id = scan_id or f"scan-{int(self.start_time)}"
        self.scan_options = scan_options or {}
    
    def add_issue(self, issue: SecurityIssue):
        """
        Add a security issue.
        
        Args:
            issue: Security issue
        """
        self.issues.append(issue)
    
    def complete(self):
        """Complete the scan."""
        self.end_time = time.time()
    
    @property
    def duration(self) -> float:
        """
        Get scan duration.
        
        Returns:
            Scan duration in seconds
        """
        if self.end_time:
            return self.end_time - self.start_time
        return 0
    
    @property
    def is_complete(self) -> bool:
        """
        Check if scan is complete.
        
        Returns:
            True if scan is complete, False otherwise
        """
        return self.end_time is not None
    
    @property
    def issue_count(self) -> int:
        """
        Get issue count.
        
        Returns:
            Number of issues
        """
        return len(self.issues)
    
    @property
    def severity_counts(self) -> Dict[str, int]:
        """
        Get severity counts.
        
        Returns:
            Dictionary mapping severity to count
        """
        counts = {}
        for issue in self.issues:
            severity = issue.severity.lower()
            counts[severity] = counts.get(severity, 0) + 1
        return counts
    
    def to_dict(self) -> Dict[str, Any]:
        """
        Convert scan result to dictionary.
        
        Returns:
            Scan result as dictionary
        """
        return {
            "scanner": self.scanner,
            "scan_id": self.scan_id,
            "start_time": self.start_time,
            "start_time_iso": datetime.fromtimestamp(self.start_time).isoformat(),
            "end_time": self.end_time,
            "end_time_iso": datetime.fromtimestamp(self.end_time).isoformat() if self.end_time else None,
            "duration": self.duration,
            "is_complete": self.is_complete,
            "scan_options": self.scan_options,
            "issue_count": self.issue_count,
            "severity_counts": self.severity_counts,
            "issues": [issue.to_dict() for issue in self.issues]
        }
    
    def to_json(self) -> str:
        """
        Convert scan result to JSON.
        
        Returns:
            Scan result as JSON string
        """
        return json.dumps(self.to_dict(), indent=2)
    
    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "SecurityScanResult":
        """
        Create scan result from dictionary.
        
        Args:
            data: Scan result data
            
        Returns:
            Security scan result
        """
        result = cls(
            scanner=data["scanner"],
            start_time=data["start_time"],
            end_time=data.get("end_time"),
            scan_id=data["scan_id"],
            scan_options=data.get("scan_options", {})
        )
        
        for issue_data in data.get("issues", []):
            result.add_issue(SecurityIssue.from_dict(issue_data))
        
        return result
    
    @classmethod
    def from_json(cls, json_str: str) -> "SecurityScanResult":
        """
        Create scan result from JSON.
        
        Args:
            json_str: Scan result as JSON string
            
        Returns:
            Security scan result
        """
        data = json.loads(json_str)
        return cls.from_dict(data)


class SecurityScanner:
    """
    Base class for security scanners.
    
    This class provides a base for security scanners.
    """
    
    def __init__(self, name: str):
        """
        Initialize security scanner.
        
        Args:
            name: Scanner name
        """
        self.name = name
        
        logger.info(f"Initialized security scanner: {name}")
    
    def scan(
        self,
        target: str,
        options: Optional[Dict[str, Any]] = None
    ) -> SecurityScanResult:
        """
        Scan a target.
        
        Args:
            target: Target to scan
            options: Scan options
            
        Returns:
            Scan result
        """
        # Create scan result
        result = SecurityScanResult(
            scanner=self.name,
            scan_options=options
        )
        
        try:
            # Perform scan
            self._scan(target, result, options or {})
        except Exception as e:
            logger.error(f"Error scanning {target} with {self.name}: {str(e)}")
        finally:
            # Complete scan
            result.complete()
        
        return result
    
    def _scan(
        self,
        target: str,
        result: SecurityScanResult,
        options: Dict[str, Any]
    ):
        """
        Perform scan (to be implemented by subclasses).
        
        Args:
            target: Target to scan
            result: Scan result
            options: Scan options
        """
        raise NotImplementedError("Subclasses must implement _scan")


class BanditScanner(SecurityScanner):
    """Security scanner that uses Bandit."""
    
    def __init__(self):
        """Initialize Bandit scanner."""
        super().__init__("bandit")
    
    def _scan(
        self,
        target: str,
        result: SecurityScanResult,
        options: Dict[str, Any]
    ):
        """
        Perform scan using Bandit.
        
        Args:
            target: Target to scan
            result: Scan result
            options: Scan options
        """
        # Check if Bandit is installed
        try:
            subprocess.run(
                ["bandit", "--version"],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                check=True
            )
        except (subprocess.SubprocessError, FileNotFoundError):
            logger.error("Bandit is not installed")
            return
        
        # Build command
        cmd = ["bandit", "-r", target, "-f", "json"]
        
        # Add options
        if "confidence" in options:
            cmd.extend(["--confidence", options["confidence"]])
        
        if "severity" in options:
            cmd.extend(["--severity", options["severity"]])
        
        if "skip" in options:
            cmd.extend(["--skip", options["skip"]])
        
        if "exclude" in options:
            for exclude in options["exclude"]:
                cmd.extend(["--exclude", exclude])
        
        # Run Bandit
        try:
            process = subprocess.run(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                check=False
            )
            
            # Parse output
            if process.returncode == 0:
                logger.info("No security issues found by Bandit")
                return
            
            try:
                output = json.loads(process.stdout)
                
                # Add issues
                for result_item in output.get("results", []):
                    issue = SecurityIssue(
                        scanner=self.name,
                        issue_type=result_item.get("test_id", ""),
                        severity=result_item.get("issue_severity", "").lower(),
                        file_path=result_item.get("filename", ""),
                        line_number=result_item.get("line_number"),
                        message=result_item.get("issue_text", ""),
                        code=result_item.get("code", ""),
                        confidence=result_item.get("issue_confidence", "").lower(),
                        cwe=None,
                        fix=None,
                        references=[result_item.get("test_name", "")]
                    )
                    
                    result.add_issue(issue)
                
                logger.info(f"Found {len(result.issues)} security issues with Bandit")
            
            except json.JSONDecodeError:
                logger.error("Failed to parse Bandit output")
        
        except Exception as e:
            logger.error(f"Error running Bandit: {str(e)}")


class SafetyScanner(SecurityScanner):
    """Security scanner that uses Safety."""
    
    def __init__(self):
        """Initialize Safety scanner."""
        super().__init__("safety")
    
    def _scan(
        self,
        target: str,
        result: SecurityScanResult,
        options: Dict[str, Any]
    ):
        """
        Perform scan using Safety.
        
        Args:
            target: Target to scan
            result: Scan result
            options: Scan options
        """
        # Check if Safety is installed
        try:
            subprocess.run(
                ["safety", "--version"],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                check=True
            )
        except (subprocess.SubprocessError, FileNotFoundError):
            logger.error("Safety is not installed")
            return
        
        # Build command
        cmd = ["safety", "check", "--json"]
        
        # Add options
        if os.path.isfile(target):
            cmd.extend(["-r", target])
        else:
            cmd.extend(["--full-report"])
        
        # Run Safety
        try:
            process = subprocess.run(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                check=False
            )
            
            # Parse output
            try:
                output = json.loads(process.stdout)
                
                # Add issues
                for vulnerability in output.get("vulnerabilities", []):
                    package_name = vulnerability.get("package_name", "")
                    affected_versions = vulnerability.get("vulnerable_spec", "")
                    installed_version = vulnerability.get("analyzed_version", "")
                    
                    issue = SecurityIssue(
                        scanner=self.name,
                        issue_type="vulnerable_dependency",
                        severity=vulnerability.get("severity", "").lower(),
                        file_path=target,
                        line_number=None,
                        message=f"{package_name} {installed_version} is vulnerable: {vulnerability.get('advisory', '')}",
                        code=None,
                        confidence="high",
                        cwe=None,
                        fix=f"Update {package_name} to a secure version",
                        references=[vulnerability.get("advisory_link", "")]
                    )
                    
                    result.add_issue(issue)
                
                logger.info(f"Found {len(result.issues)} security issues with Safety")
            
            except json.JSONDecodeError:
                logger.error("Failed to parse Safety output")
        
        except Exception as e:
            logger.error(f"Error running Safety: {str(e)}")


class SemgrepScanner(SecurityScanner):
    """Security scanner that uses Semgrep."""
    
    def __init__(self):
        """Initialize Semgrep scanner."""
        super().__init__("semgrep")
    
    def _scan(
        self,
        target: str,
        result: SecurityScanResult,
        options: Dict[str, Any]
    ):
        """
        Perform scan using Semgrep.
        
        Args:
            target: Target to scan
            result: Scan result
            options: Scan options
        """
        # Check if Semgrep is installed
        try:
            subprocess.run(
                ["semgrep", "--version"],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                check=True
            )
        except (subprocess.SubprocessError, FileNotFoundError):
            logger.error("Semgrep is not installed")
            return
        
        # Build command
        cmd = ["semgrep", "--json", "--config", "p/security-audit"]
        
        # Add target
        cmd.append(target)
        
        # Add options
        if "exclude" in options:
            for exclude in options["exclude"]:
                cmd.extend(["--exclude", exclude])
        
        # Run Semgrep
        try:
            process = subprocess.run(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                check=False
            )
            
            # Parse output
            try:
                output = json.loads(process.stdout)
                
                # Add issues
                for finding in output.get("results", []):
                    issue = SecurityIssue(
                        scanner=self.name,
                        issue_type=finding.get("check_id", ""),
                        severity=finding.get("severity", "").lower(),
                        file_path=finding.get("path", ""),
                        line_number=finding.get("start", {}).get("line"),
                        message=finding.get("extra", {}).get("message", ""),
                        code=finding.get("extra", {}).get("lines", ""),
                        confidence=None,
                        cwe=finding.get("extra", {}).get("metadata", {}).get("cwe", []),
                        fix=finding.get("extra", {}).get("fix", ""),
                        references=[finding.get("extra", {}).get("metadata", {}).get("references", [])]
                    )
                    
                    result.add_issue(issue)
                
                logger.info(f"Found {len(result.issues)} security issues with Semgrep")
            
            except json.JSONDecodeError:
                logger.error("Failed to parse Semgrep output")
        
        except Exception as e:
            logger.error(f"Error running Semgrep: {str(e)}")


class SecurityScanManager:
    """
    Manager for security scanners.
    
    This class manages security scanners and scan results.
    """
    
    def __init__(self):
        """Initialize security scan manager."""
        self.scanners: Dict[str, SecurityScanner] = {}
        self.results: Dict[str, SecurityScanResult] = {}
        
        logger.info("Initialized security scan manager")
    
    def add_scanner(self, scanner: SecurityScanner):
        """
        Add a security scanner.
        
        Args:
            scanner: Security scanner
        """
        self.scanners[scanner.name] = scanner
        logger.info(f"Added security scanner: {scanner.name}")
    
    def remove_scanner(self, name: str):
        """
        Remove a security scanner.
        
        Args:
            name: Scanner name
        """
        if name in self.scanners:
            del self.scanners[name]
            logger.info(f"Removed security scanner: {name}")
        else:
            logger.warning(f"Scanner not found: {name}")
    
    def get_scanner(self, name: str) -> Optional[SecurityScanner]:
        """
        Get a security scanner.
        
        Args:
            name: Scanner name
            
        Returns:
            Security scanner or None if not found
        """
        return self.scanners.get(name)
    
    def scan(
        self,
        target: str,
        scanner_name: Optional[str] = None,
        options: Optional[Dict[str, Any]] = None
    ) -> Optional[SecurityScanResult]:
        """
        Scan a target.
        
        Args:
            target: Target to scan
            scanner_name: Scanner name (None = all scanners)
            options: Scan options
            
        Returns:
            Scan result or None if scanner not found
        """
        if scanner_name:
            # Get scanner
            scanner = self.get_scanner(scanner_name)
            if not scanner:
                logger.warning(f"Scanner not found: {scanner_name}")
                return None
            
            # Perform scan
            result = scanner.scan(target, options)
            
            # Store result
            self.results[result.scan_id] = result
            
            return result
        else:
            # Scan with all scanners
            results = []
            
            for scanner in self.scanners.values():
                result = scanner.scan(target, options)
                self.results[result.scan_id] = result
                results.append(result)
            
            # Combine results
            combined_result = SecurityScanResult(
                scanner="combined",
                scan_options=options
            )
            
            for result in results:
                for issue in result.issues:
                    combined_result.add_issue(issue)
            
            combined_result.complete()
            
            # Store result
            self.results[combined_result.scan_id] = combined_result
            
            return combined_result
    
    def get_result(self, scan_id: str) -> Optional[SecurityScanResult]:
        """
        Get a scan result.
        
        Args:
            scan_id: Scan ID
            
        Returns:
            Scan result or None if not found
        """
        return self.results.get(scan_id)
    
    def get_results(self) -> List[SecurityScanResult]:
        """
        Get all scan results.
        
        Returns:
            List of scan results
        """
        return list(self.results.values())
    
    def save_result(self, scan_id: str, file_path: str):
        """
        Save a scan result to a file.
        
        Args:
            scan_id: Scan ID
            file_path: File path
        """
        result = self.get_result(scan_id)
        if not result:
            logger.warning(f"Scan result not found: {scan_id}")
            return
        
        with open(file_path, "w") as f:
            f.write(result.to_json())
        
        logger.info(f"Saved scan result {scan_id} to {file_path}")
    
    def load_result(self, file_path: str) -> Optional[SecurityScanResult]:
        """
        Load a scan result from a file.
        
        Args:
            file_path: File path
            
        Returns:
            Scan result or None if file not found
        """
        try:
            with open(file_path, "r") as f:
                result = SecurityScanResult.from_json(f.read())
            
            self.results[result.scan_id] = result
            
            logger.info(f"Loaded scan result {result.scan_id} from {file_path}")
            
            return result
        
        except (FileNotFoundError, json.JSONDecodeError) as e:
            logger.error(f"Error loading scan result from {file_path}: {str(e)}")
            return None


# Create global security scan manager
security_scan_manager = SecurityScanManager()

# Add default scanners
security_scan_manager.add_scanner(BanditScanner())
security_scan_manager.add_scanner(SafetyScanner())
security_scan_manager.add_scanner(SemgrepScanner())


def scan(
    target: str,
    scanner_name: Optional[str] = None,
    options: Optional[Dict[str, Any]] = None
) -> Optional[SecurityScanResult]:
    """
    Scan a target.
    
    Args:
        target: Target to scan
        scanner_name: Scanner name (None = all scanners)
        options: Scan options
        
    Returns:
        Scan result or None if scanner not found
    """
    return security_scan_manager.scan(target, scanner_name, options)


def get_result(scan_id: str) -> Optional[SecurityScanResult]:
    """
    Get a scan result.
    
    Args:
        scan_id: Scan ID
        
    Returns:
        Scan result or None if not found
    """
    return security_scan_manager.get_result(scan_id)


def get_results() -> List[SecurityScanResult]:
    """
    Get all scan results.
    
    Returns:
        List of scan results
    """
    return security_scan_manager.get_results()


def save_result(scan_id: str, file_path: str):
    """
    Save a scan result to a file.
    
    Args:
        scan_id: Scan ID
        file_path: File path
    """
    security_scan_manager.save_result(scan_id, file_path)


def load_result(file_path: str) -> Optional[SecurityScanResult]:
    """
    Load a scan result from a file.
    
    Args:
        file_path: File path
        
    Returns:
        Scan result or None if file not found
    """
    return security_scan_manager.load_result(file_path)


def add_scanner(scanner: SecurityScanner):
    """
    Add a security scanner.
    
    Args:
        scanner: Security scanner
    """
    security_scan_manager.add_scanner(scanner)


def remove_scanner(name: str):
    """
    Remove a security scanner.
    
    Args:
        name: Scanner name
    """
    security_scan_manager.remove_scanner(name)


def get_scanner(name: str) -> Optional[SecurityScanner]:
    """
    Get a security scanner.
    
    Args:
        name: Scanner name
        
    Returns:
        Security scanner or None if not found
    """
    return security_scan_manager.get_scanner(name)
