"""
OWASP ZAP security testing framework for Bulo.Cloud Sentinel.

This module provides automated security testing using OWASP ZAP (Zed Attack Proxy)
to identify security vulnerabilities in the application.
"""

import os
import time
import json
import logging
import subprocess
import requests
from datetime import datetime
from typing import Dict, List, Optional, Set, Union, Any

# Set up logging
log = logging.getLogger(__name__)

class ZAPSecurityTester:
    """
    Security testing framework using OWASP ZAP.
    
    This class provides methods for running automated security tests against
    the application using OWASP ZAP.
    """
    
    def __init__(
        self,
        target_url: str,
        zap_path: Optional[str] = None,
        api_key: Optional[str] = None,
        report_dir: str = "security/testing/reports",
        zap_port: int = 8090,
    ):
        """
        Initialize the security tester.
        
        Args:
            target_url: URL of the target application
            zap_path: Path to ZAP installation (optional)
            api_key: ZAP API key (optional)
            report_dir: Directory to store reports
            zap_port: Port for ZAP API
        """
        self.target_url = target_url
        self.zap_path = zap_path or self._find_zap_path()
        self.api_key = api_key
        self.report_dir = report_dir
        self.zap_port = zap_port
        self.zap_process = None
        
        # Create report directory if it doesn't exist
        os.makedirs(self.report_dir, exist_ok=True)
        
        log.info(f"Initialized ZAP security tester for {target_url}")
        
    def _find_zap_path(self) -> str:
        """
        Find the ZAP installation path.
        
        Returns:
            Path to ZAP installation
        """
        # Common installation paths
        common_paths = [
            r"C:\Program Files\OWASP\Zed Attack Proxy",
            r"C:\Program Files (x86)\OWASP\Zed Attack Proxy",
            "/usr/share/zaproxy",
            "/opt/zaproxy",
            "/Applications/OWASP ZAP.app/Contents/Java",
        ]
        
        # Check environment variable
        if "ZAP_PATH" in os.environ:
            return os.environ["ZAP_PATH"]
            
        # Check common paths
        for path in common_paths:
            if os.path.exists(path):
                return path
                
        # Default to assuming ZAP is in PATH
        return ""
        
    def start_zap(self) -> bool:
        """
        Start the ZAP daemon.
        
        Returns:
            True if ZAP was started successfully, False otherwise
        """
        try:
            # Determine ZAP executable
            if os.name == "nt":  # Windows
                zap_exe = os.path.join(self.zap_path, "zap.bat")
            else:  # Linux/Mac
                zap_exe = os.path.join(self.zap_path, "zap.sh")
                
            if not os.path.exists(zap_exe):
                log.error(f"ZAP executable not found at {zap_exe}")
                return False
                
            # Build command
            cmd = [
                zap_exe,
                "-daemon",
                "-port", str(self.zap_port),
                "-config", "api.disablekey=false",
            ]
            
            if self.api_key:
                cmd.extend(["-config", f"api.key={self.api_key}"])
                
            # Start ZAP
            log.info(f"Starting ZAP daemon on port {self.zap_port}")
            self.zap_process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                universal_newlines=True,
            )
            
            # Wait for ZAP to start
            time.sleep(10)
            
            # Check if ZAP is running
            try:
                response = requests.get(f"http://localhost:{self.zap_port}/JSON/core/view/version")
                if response.status_code == 200:
                    log.info(f"ZAP started successfully: {response.json()}")
                    return True
            except requests.RequestException as e:
                log.error(f"Failed to connect to ZAP: {str(e)}")
                
            return False
        except Exception as e:
            log.error(f"Failed to start ZAP: {str(e)}")
            return False
            
    def stop_zap(self) -> bool:
        """
        Stop the ZAP daemon.
        
        Returns:
            True if ZAP was stopped successfully, False otherwise
        """
        try:
            if self.zap_process:
                # Try to shutdown gracefully via API
                try:
                    headers = {"X-ZAP-API-Key": self.api_key} if self.api_key else {}
                    requests.get(
                        f"http://localhost:{self.zap_port}/JSON/core/action/shutdown",
                        headers=headers,
                    )
                    time.sleep(5)
                except requests.RequestException:
                    pass
                    
                # Terminate process if still running
                if self.zap_process.poll() is None:
                    self.zap_process.terminate()
                    self.zap_process.wait(timeout=10)
                    
                log.info("ZAP stopped successfully")
                return True
            return False
        except Exception as e:
            log.error(f"Failed to stop ZAP: {str(e)}")
            return False
            
    def run_spider(self, max_depth: int = 5) -> bool:
        """
        Run the ZAP spider to discover URLs.
        
        Args:
            max_depth: Maximum depth to spider
            
        Returns:
            True if spider completed successfully, False otherwise
        """
        try:
            log.info(f"Starting ZAP spider on {self.target_url}")
            
            # Start spider
            headers = {"X-ZAP-API-Key": self.api_key} if self.api_key else {}
            response = requests.get(
                f"http://localhost:{self.zap_port}/JSON/spider/action/scan",
                params={
                    "url": self.target_url,
                    "maxChildren": str(max_depth),
                    "recurse": "true",
                    "contextName": "",
                    "subtreeOnly": "false",
                },
                headers=headers,
            )
            
            if response.status_code != 200:
                log.error(f"Failed to start spider: {response.text}")
                return False
                
            # Get scan ID
            scan_id = response.json()["scan"]
            
            # Wait for spider to complete
            while True:
                response = requests.get(
                    f"http://localhost:{self.zap_port}/JSON/spider/view/status",
                    params={"scanId": scan_id},
                    headers=headers,
                )
                
                status = response.json()["status"]
                log.info(f"Spider progress: {status}%")
                
                if status == "100":
                    break
                    
                time.sleep(5)
                
            log.info("Spider completed successfully")
            return True
        except Exception as e:
            log.error(f"Spider failed: {str(e)}")
            return False
            
    def run_active_scan(self) -> bool:
        """
        Run the ZAP active scanner to find vulnerabilities.
        
        Returns:
            True if scan completed successfully, False otherwise
        """
        try:
            log.info(f"Starting ZAP active scan on {self.target_url}")
            
            # Start active scan
            headers = {"X-ZAP-API-Key": self.api_key} if self.api_key else {}
            response = requests.get(
                f"http://localhost:{self.zap_port}/JSON/ascan/action/scan",
                params={
                    "url": self.target_url,
                    "recurse": "true",
                    "inScopeOnly": "false",
                    "scanPolicyName": "",
                    "method": "",
                    "postData": "",
                },
                headers=headers,
            )
            
            if response.status_code != 200:
                log.error(f"Failed to start active scan: {response.text}")
                return False
                
            # Get scan ID
            scan_id = response.json()["scan"]
            
            # Wait for scan to complete
            while True:
                response = requests.get(
                    f"http://localhost:{self.zap_port}/JSON/ascan/view/status",
                    params={"scanId": scan_id},
                    headers=headers,
                )
                
                status = response.json()["status"]
                log.info(f"Active scan progress: {status}%")
                
                if status == "100":
                    break
                    
                time.sleep(10)
                
            log.info("Active scan completed successfully")
            return True
        except Exception as e:
            log.error(f"Active scan failed: {str(e)}")
            return False
            
    def generate_report(self, report_type: str = "html") -> str:
        """
        Generate a security report.
        
        Args:
            report_type: Type of report (html, xml, json, md)
            
        Returns:
            Path to the generated report
        """
        try:
            log.info(f"Generating {report_type} report")
            
            # Generate timestamp
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            
            # Set report path
            report_path = os.path.join(
                self.report_dir,
                f"zap_report_{timestamp}.{report_type}"
            )
            
            # Generate report
            headers = {"X-ZAP-API-Key": self.api_key} if self.api_key else {}
            
            if report_type == "html":
                response = requests.get(
                    f"http://localhost:{self.zap_port}/OTHER/core/other/htmlreport",
                    headers=headers,
                )
            elif report_type == "xml":
                response = requests.get(
                    f"http://localhost:{self.zap_port}/OTHER/core/other/xmlreport",
                    headers=headers,
                )
            elif report_type == "json":
                response = requests.get(
                    f"http://localhost:{self.zap_port}/JSON/core/view/alerts",
                    params={"baseurl": self.target_url},
                    headers=headers,
                )
            elif report_type == "md":
                response = requests.get(
                    f"http://localhost:{self.zap_port}/OTHER/core/other/mdreport",
                    headers=headers,
                )
            else:
                log.error(f"Unsupported report type: {report_type}")
                return ""
                
            if response.status_code != 200:
                log.error(f"Failed to generate report: {response.text}")
                return ""
                
            # Save report
            with open(report_path, "wb") as f:
                if report_type == "json":
                    f.write(json.dumps(response.json(), indent=4).encode("utf-8"))
                else:
                    f.write(response.content)
                
            log.info(f"Report saved to {report_path}")
            return report_path
        except Exception as e:
            log.error(f"Failed to generate report: {str(e)}")
            return ""
            
    def get_alerts(self, risk_level: Optional[str] = None) -> List[Dict[str, Any]]:
        """
        Get alerts from ZAP.
        
        Args:
            risk_level: Risk level filter (high, medium, low, informational)
            
        Returns:
            List of alerts
        """
        try:
            log.info("Getting alerts from ZAP")
            
            # Get alerts
            headers = {"X-ZAP-API-Key": self.api_key} if self.api_key else {}
            response = requests.get(
                f"http://localhost:{self.zap_port}/JSON/core/view/alerts",
                params={"baseurl": self.target_url},
                headers=headers,
            )
            
            if response.status_code != 200:
                log.error(f"Failed to get alerts: {response.text}")
                return []
                
            # Parse alerts
            alerts = response.json()["alerts"]
            
            # Filter by risk level if specified
            if risk_level:
                risk_level = risk_level.lower()
                alerts = [a for a in alerts if a["risk"].lower() == risk_level]
                
            return alerts
        except Exception as e:
            log.error(f"Failed to get alerts: {str(e)}")
            return []
            
    def run_full_scan(self) -> Dict[str, Any]:
        """
        Run a full security scan (spider + active scan).
        
        Returns:
            Dictionary with scan results
        """
        try:
            results = {
                "success": False,
                "alerts": {
                    "high": 0,
                    "medium": 0,
                    "low": 0,
                    "informational": 0,
                },
                "reports": {},
            }
            
            # Start ZAP
            if not self.start_zap():
                log.error("Failed to start ZAP")
                return results
                
            try:
                # Run spider
                if not self.run_spider():
                    log.error("Spider failed")
                    return results
                    
                # Run active scan
                if not self.run_active_scan():
                    log.error("Active scan failed")
                    return results
                    
                # Get alerts
                alerts = self.get_alerts()
                
                # Count alerts by risk level
                for alert in alerts:
                    risk = alert["risk"].lower()
                    if risk in results["alerts"]:
                        results["alerts"][risk] += 1
                        
                # Generate reports
                for report_type in ["html", "json", "md"]:
                    report_path = self.generate_report(report_type)
                    if report_path:
                        results["reports"][report_type] = report_path
                        
                results["success"] = True
                
                return results
            finally:
                # Stop ZAP
                self.stop_zap()
        except Exception as e:
            log.error(f"Full scan failed: {str(e)}")
            return {"success": False, "error": str(e)}


def run_security_scan(target_url: str) -> Dict[str, Any]:
    """
    Run a security scan on the target URL.
    
    Args:
        target_url: URL of the target application
        
    Returns:
        Dictionary with scan results
    """
    tester = ZAPSecurityTester(target_url)
    return tester.run_full_scan()


if __name__ == "__main__":
    # Configure logging
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(message)s",
    )
    
    # Get target URL from command line or use default
    import sys
    target_url = sys.argv[1] if len(sys.argv) > 1 else "http://localhost:8000"
    
    # Run security scan
    results = run_security_scan(target_url)
    
    # Print results
    print("\nSecurity Scan Results:")
    print(f"Success: {results['success']}")
    
    if results["success"]:
        print("\nAlerts:")
        for level, count in results["alerts"].items():
            print(f"  {level.capitalize()}: {count}")
            
        print("\nReports:")
        for report_type, path in results["reports"].items():
            print(f"  {report_type.upper()}: {path}")
    else:
        print(f"Error: {results.get('error', 'Unknown error')}")
