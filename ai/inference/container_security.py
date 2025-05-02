"""
Container security for Bulo.Cloud Sentinel.

This module provides security measures for containerized deployments.
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


class ContainerSecurityManager:
    """
    Container security manager for Bulo.Cloud Sentinel.
    
    This class provides security measures for containerized deployments.
    """
    
    def __init__(
        self,
        docker_dir: Optional[str] = None,
        report_dir: Optional[str] = None,
        max_reports: int = 10
    ):
        """
        Initialize the container security manager.
        
        Args:
            docker_dir: Directory containing Dockerfiles
            report_dir: Directory for scan reports
            max_reports: Maximum number of reports to keep
        """
        # Set Docker directory
        self.docker_dir = docker_dir or os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            "../../docker"
        )
        
        # Set report directory
        self.report_dir = report_dir or os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            "../../reports/container"
        )
        
        # Create report directory if it doesn't exist
        os.makedirs(self.report_dir, exist_ok=True)
        
        # Set scan parameters
        self.max_reports = max_reports
    
    def scan_dockerfile(self, dockerfile_path: str) -> Dict[str, Any]:
        """
        Scan a Dockerfile for security issues.
        
        Args:
            dockerfile_path: Path to the Dockerfile
            
        Returns:
            Scan results
        """
        try:
            # Check if Dockerfile exists
            if not os.path.exists(dockerfile_path):
                structured_logger.warning(
                    "Dockerfile not found",
                    logger_name="container_security",
                    dockerfile_path=dockerfile_path
                )
                
                return {
                    "success": False,
                    "message": "Dockerfile not found",
                    "issues": []
                }
            
            # Create temporary directory
            with tempfile.TemporaryDirectory() as temp_dir:
                # Create output file path
                output_path = os.path.join(temp_dir, "hadolint_report.json")
                
                # Run hadolint scan
                try:
                    subprocess.run([
                        "hadolint",
                        "--format", "json",
                        dockerfile_path,
                        "-o", output_path
                    ], check=True)
                except subprocess.CalledProcessError:
                    # Hadolint scan found issues
                    pass
                
                # Check if output file exists
                if not os.path.exists(output_path):
                    structured_logger.warning(
                        "Hadolint scan failed",
                        logger_name="container_security",
                        dockerfile_path=dockerfile_path
                    )
                    
                    return {
                        "success": False,
                        "message": "Hadolint scan failed",
                        "issues": []
                    }
                
                # Load hadolint report
                with open(output_path, "r") as f:
                    hadolint_report = json.load(f)
                
                # Process issues
                issues = []
                for issue in hadolint_report:
                    # Get issue data
                    line = issue.get("line", 0)
                    code = issue.get("code", "")
                    message = issue.get("message", "")
                    level = issue.get("level", "")
                    
                    # Add issue to list
                    issues.append({
                        "line": line,
                        "code": code,
                        "message": message,
                        "level": level
                    })
                
                # Create scan report
                scan_report = {
                    "success": True,
                    "message": f"Found {len(issues)} issues",
                    "issues": issues,
                    "dockerfile": os.path.basename(dockerfile_path)
                }
                
                # Save scan report
                self._save_report("dockerfile", scan_report)
                
                # Log scan results
                structured_logger.info(
                    "Dockerfile scan completed",
                    logger_name="container_security",
                    dockerfile=os.path.basename(dockerfile_path),
                    issues_count=len(issues)
                )
                
                # Create alerts for error level issues
                for issue in issues:
                    if issue["level"] == "error":
                        alert_manager.create_alert(
                            level="warning",
                            message=f"Error in Dockerfile {os.path.basename(dockerfile_path)}: {issue['message']}",
                            source="container_security",
                            data=issue
                        )
                
                # Log audit event
                audit_logger.log_security_event(
                    event_name="dockerfile_scan",
                    severity="info" if not issues else "warning",
                    source="container_security",
                    details={
                        "dockerfile": os.path.basename(dockerfile_path),
                        "issues_count": len(issues),
                        "scan_report": scan_report
                    }
                )
                
                return scan_report
        
        except Exception as e:
            # Log error
            structured_logger.error(
                "Error scanning Dockerfile",
                logger_name="container_security",
                error=str(e),
                dockerfile_path=dockerfile_path
            )
            
            # Create alert
            alert_manager.create_alert(
                level="error",
                message=f"Error scanning Dockerfile {os.path.basename(dockerfile_path)}: {str(e)}",
                source="container_security"
            )
            
            # Log audit event
            audit_logger.log_security_event(
                event_name="dockerfile_scan_error",
                severity="error",
                source="container_security",
                details={
                    "dockerfile": os.path.basename(dockerfile_path),
                    "error": str(e)
                }
            )
            
            return {
                "success": False,
                "message": f"Error scanning Dockerfile: {str(e)}",
                "issues": []
            }
    
    def scan_container_image(self, image_name: str) -> Dict[str, Any]:
        """
        Scan a container image for vulnerabilities.
        
        Args:
            image_name: Name of the container image
            
        Returns:
            Scan results
        """
        try:
            # Create temporary directory
            with tempfile.TemporaryDirectory() as temp_dir:
                # Create output file path
                output_path = os.path.join(temp_dir, "trivy_report.json")
                
                # Run trivy scan
                try:
                    subprocess.run([
                        "trivy", "image",
                        "--format", "json",
                        "--output", output_path,
                        image_name
                    ], check=True)
                except subprocess.CalledProcessError:
                    # Trivy scan found vulnerabilities
                    pass
                
                # Check if output file exists
                if not os.path.exists(output_path):
                    structured_logger.warning(
                        "Trivy scan failed",
                        logger_name="container_security",
                        image_name=image_name
                    )
                    
                    return {
                        "success": False,
                        "message": "Trivy scan failed",
                        "vulnerabilities": []
                    }
                
                # Load trivy report
                with open(output_path, "r") as f:
                    trivy_report = json.load(f)
                
                # Process vulnerabilities
                vulnerabilities = []
                for result in trivy_report.get("Results", []):
                    for vulnerability in result.get("Vulnerabilities", []):
                        # Get vulnerability data
                        vuln_id = vulnerability.get("VulnerabilityID", "")
                        pkg_name = vulnerability.get("PkgName", "")
                        installed_version = vulnerability.get("InstalledVersion", "")
                        fixed_version = vulnerability.get("FixedVersion", "")
                        severity = vulnerability.get("Severity", "")
                        description = vulnerability.get("Description", "")
                        
                        # Add vulnerability to list
                        vulnerabilities.append({
                            "vuln_id": vuln_id,
                            "pkg_name": pkg_name,
                            "installed_version": installed_version,
                            "fixed_version": fixed_version,
                            "severity": severity,
                            "description": description
                        })
                
                # Create scan report
                scan_report = {
                    "success": True,
                    "message": f"Found {len(vulnerabilities)} vulnerabilities",
                    "vulnerabilities": vulnerabilities,
                    "image_name": image_name
                }
                
                # Save scan report
                self._save_report("container_image", scan_report)
                
                # Log scan results
                structured_logger.info(
                    "Container image scan completed",
                    logger_name="container_security",
                    image_name=image_name,
                    vulnerabilities_count=len(vulnerabilities)
                )
                
                # Create alerts for high severity vulnerabilities
                high_severity_vulns = [v for v in vulnerabilities if v["severity"] in ["HIGH", "CRITICAL"]]
                if high_severity_vulns:
                    alert_manager.create_alert(
                        level="warning",
                        message=f"Found {len(high_severity_vulns)} high severity vulnerabilities in {image_name}",
                        source="container_security",
                        data={"high_severity_count": len(high_severity_vulns)}
                    )
                
                # Log audit event
                audit_logger.log_security_event(
                    event_name="container_image_scan",
                    severity="info" if not vulnerabilities else "warning",
                    source="container_security",
                    details={
                        "image_name": image_name,
                        "vulnerabilities_count": len(vulnerabilities),
                        "high_severity_count": len(high_severity_vulns)
                    }
                )
                
                return scan_report
        
        except Exception as e:
            # Log error
            structured_logger.error(
                "Error scanning container image",
                logger_name="container_security",
                error=str(e),
                image_name=image_name
            )
            
            # Create alert
            alert_manager.create_alert(
                level="error",
                message=f"Error scanning container image {image_name}: {str(e)}",
                source="container_security"
            )
            
            # Log audit event
            audit_logger.log_security_event(
                event_name="container_image_scan_error",
                severity="error",
                source="container_security",
                details={
                    "image_name": image_name,
                    "error": str(e)
                }
            )
            
            return {
                "success": False,
                "message": f"Error scanning container image: {str(e)}",
                "vulnerabilities": []
            }
    
    def scan_all_dockerfiles(self) -> Dict[str, Any]:
        """
        Scan all Dockerfiles in the Docker directory.
        
        Returns:
            Scan results
        """
        try:
            # Check if Docker directory exists
            if not os.path.exists(self.docker_dir):
                structured_logger.warning(
                    "Docker directory not found",
                    logger_name="container_security",
                    docker_dir=self.docker_dir
                )
                
                return {
                    "success": False,
                    "message": "Docker directory not found",
                    "results": []
                }
            
            # Find all Dockerfiles
            dockerfiles = []
            for root, _, files in os.walk(self.docker_dir):
                for file in files:
                    if file == "Dockerfile" or file.startswith("Dockerfile."):
                        dockerfiles.append(os.path.join(root, file))
            
            # Scan each Dockerfile
            results = []
            for dockerfile in dockerfiles:
                # Scan Dockerfile
                result = self.scan_dockerfile(dockerfile)
                
                # Add result to list
                results.append({
                    "dockerfile": os.path.relpath(dockerfile, self.docker_dir),
                    "result": result
                })
            
            # Create scan report
            scan_report = {
                "success": True,
                "message": f"Scanned {len(dockerfiles)} Dockerfiles",
                "results": results
            }
            
            # Save scan report
            self._save_report("all_dockerfiles", scan_report)
            
            # Log scan results
            structured_logger.info(
                "All Dockerfiles scan completed",
                logger_name="container_security",
                dockerfiles_count=len(dockerfiles)
            )
            
            # Log audit event
            audit_logger.log_security_event(
                event_name="all_dockerfiles_scan",
                severity="info",
                source="container_security",
                details={
                    "dockerfiles_count": len(dockerfiles)
                }
            )
            
            return scan_report
        
        except Exception as e:
            # Log error
            structured_logger.error(
                "Error scanning all Dockerfiles",
                logger_name="container_security",
                error=str(e)
            )
            
            # Create alert
            alert_manager.create_alert(
                level="error",
                message=f"Error scanning all Dockerfiles: {str(e)}",
                source="container_security"
            )
            
            # Log audit event
            audit_logger.log_security_event(
                event_name="all_dockerfiles_scan_error",
                severity="error",
                source="container_security",
                details={
                    "error": str(e)
                }
            )
            
            return {
                "success": False,
                "message": f"Error scanning all Dockerfiles: {str(e)}",
                "results": []
            }
    
    def generate_secure_dockerfile(
        self,
        base_image: str,
        app_dir: str = "/app",
        user: str = "appuser",
        uid: int = 1000,
        gid: int = 1000
    ) -> str:
        """
        Generate a secure Dockerfile.
        
        Args:
            base_image: Base image to use
            app_dir: Application directory
            user: User to run the application as
            uid: User ID
            gid: Group ID
            
        Returns:
            Secure Dockerfile content
        """
        try:
            # Create Dockerfile content
            dockerfile = f"""# Secure Dockerfile for Bulo.Cloud Sentinel
FROM {base_image}

# Set working directory
WORKDIR {app_dir}

# Create non-root user
RUN groupadd -g {gid} {user} && \\
    useradd -u {uid} -g {user} -s /bin/bash -m {user} && \\
    mkdir -p {app_dir} && \\
    chown -R {user}:{user} {app_dir}

# Copy requirements file
COPY --chown={user}:{user} requirements.txt .

# Install dependencies
RUN pip install --no-cache-dir -r requirements.txt

# Copy application code
COPY --chown={user}:{user} . .

# Set secure permissions
RUN chmod -R 755 {app_dir} && \\
    find {app_dir} -type f -exec chmod 644 {{}} \\; && \\
    find {app_dir} -type d -exec chmod 755 {{}} \\;

# Set environment variables
ENV PYTHONDONTWRITEBYTECODE=1 \\
    PYTHONUNBUFFERED=1 \\
    PYTHONHASHSEED=random \\
    PIP_NO_CACHE_DIR=off \\
    PIP_DISABLE_PIP_VERSION_CHECK=on \\
    PIP_DEFAULT_TIMEOUT=100

# Switch to non-root user
USER {user}

# Expose port
EXPOSE 8000

# Run application
CMD ["python", "app.py"]
"""
            
            # Log Dockerfile generation
            structured_logger.info(
                "Secure Dockerfile generated",
                logger_name="container_security",
                base_image=base_image,
                user=user
            )
            
            return dockerfile
        
        except Exception as e:
            # Log error
            structured_logger.error(
                "Error generating secure Dockerfile",
                logger_name="container_security",
                error=str(e)
            )
            
            # Create alert
            alert_manager.create_alert(
                level="error",
                message=f"Error generating secure Dockerfile: {str(e)}",
                source="container_security"
            )
            
            return ""
    
    def generate_docker_compose(
        self,
        services: List[Dict[str, Any]],
        network_name: str = "sentinel-network",
        use_volumes: bool = True
    ) -> str:
        """
        Generate a secure Docker Compose file.
        
        Args:
            services: List of services
            network_name: Name of the network
            use_volumes: Whether to use volumes
            
        Returns:
            Secure Docker Compose content
        """
        try:
            # Create Docker Compose content
            docker_compose = """version: '3.8'

services:
"""
            
            # Add services
            for service in services:
                # Get service data
                name = service.get("name", "app")
                image = service.get("image", "sentinel:latest")
                ports = service.get("ports", [])
                environment = service.get("environment", {})
                volumes = service.get("volumes", [])
                depends_on = service.get("depends_on", [])
                
                # Add service
                docker_compose += f"  {name}:\n"
                docker_compose += f"    image: {image}\n"
                docker_compose += "    restart: unless-stopped\n"
                
                # Add ports
                if ports:
                    docker_compose += "    ports:\n"
                    for port in ports:
                        docker_compose += f"      - \"{port}\"\n"
                
                # Add environment variables
                if environment:
                    docker_compose += "    environment:\n"
                    for key, value in environment.items():
                        docker_compose += f"      - {key}={value}\n"
                
                # Add volumes
                if use_volumes and volumes:
                    docker_compose += "    volumes:\n"
                    for volume in volumes:
                        docker_compose += f"      - {volume}\n"
                
                # Add dependencies
                if depends_on:
                    docker_compose += "    depends_on:\n"
                    for dependency in depends_on:
                        docker_compose += f"      - {dependency}\n"
                
                # Add security options
                docker_compose += "    security_opt:\n"
                docker_compose += "      - no-new-privileges:true\n"
                
                # Add resource limits
                docker_compose += "    deploy:\n"
                docker_compose += "      resources:\n"
                docker_compose += "        limits:\n"
                docker_compose += "          cpus: '1.0'\n"
                docker_compose += "          memory: 1G\n"
                
                # Add network
                docker_compose += "    networks:\n"
                docker_compose += f"      - {network_name}\n"
                
                # Add healthcheck
                docker_compose += "    healthcheck:\n"
                docker_compose += "      test: [\"CMD\", \"curl\", \"-f\", \"http://localhost:8000/health\"]\n"
                docker_compose += "      interval: 30s\n"
                docker_compose += "      timeout: 10s\n"
                docker_compose += "      retries: 3\n"
                docker_compose += "      start_period: 40s\n"
                
                # Add logging
                docker_compose += "    logging:\n"
                docker_compose += "      driver: \"json-file\"\n"
                docker_compose += "      options:\n"
                docker_compose += "        max-size: \"10m\"\n"
                docker_compose += "        max-file: \"3\"\n"
            
            # Add network
            docker_compose += "\nnetworks:\n"
            docker_compose += f"  {network_name}:\n"
            docker_compose += "    driver: bridge\n"
            
            # Add volumes
            if use_volumes:
                docker_compose += "\nvolumes:\n"
                for service in services:
                    for volume in service.get("volumes", []):
                        if ":" in volume:
                            volume_name = volume.split(":")[0]
                            if not volume_name.startswith("./") and not volume_name.startswith("/"):
                                docker_compose += f"  {volume_name}:\n"
            
            # Log Docker Compose generation
            structured_logger.info(
                "Secure Docker Compose generated",
                logger_name="container_security",
                services_count=len(services),
                network_name=network_name
            )
            
            return docker_compose
        
        except Exception as e:
            # Log error
            structured_logger.error(
                "Error generating secure Docker Compose",
                logger_name="container_security",
                error=str(e)
            )
            
            # Create alert
            alert_manager.create_alert(
                level="error",
                message=f"Error generating secure Docker Compose: {str(e)}",
                source="container_security"
            )
            
            return ""
    
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
                logger_name="container_security",
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
                        logger_name="container_security",
                        file_path=str(file_path)
                    )
                except Exception as e:
                    structured_logger.error(
                        "Error deleting scan report",
                        logger_name="container_security",
                        error=str(e),
                        file_path=str(file_path)
                    )
        
        except Exception as e:
            # Log error
            structured_logger.error(
                "Error cleaning scan reports",
                logger_name="container_security",
                error=str(e)
            )


# Create global container security manager
container_security_manager = ContainerSecurityManager()
