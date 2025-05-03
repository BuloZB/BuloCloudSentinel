"""
Security test runner for Bulo.Cloud Sentinel.

This module provides functionality to run security tests against the Bulo.Cloud Sentinel
platform, including vulnerability scanning, penetration testing, and security checks.
"""

import os
import sys
import json
import logging
import argparse
import subprocess
import requests
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Set, Union, Any

# Set up logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger(__name__)

# Define the root directory
ROOT_DIR = Path(__file__).parent.parent.parent
REPORTS_DIR = ROOT_DIR / "reports" / "security"

class SecurityTestRunner:
    """
    Runner for security tests.
    """

    def __init__(
        self,
        target_url: str = "http://localhost:8000",
        reports_dir: Path = REPORTS_DIR,
        verbose: bool = False
    ):
        """
        Initialize the security test runner.

        Args:
            target_url: URL of the target application
            reports_dir: Directory to save reports
            verbose: Whether to enable verbose output
        """
        self.target_url = target_url
        self.reports_dir = reports_dir
        self.verbose = verbose

        # Create reports directory if it doesn't exist
        self.reports_dir.mkdir(parents=True, exist_ok=True)

        # Set up timestamp for reports
        self.timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

        # Initialize results
        self.results = {
            "timestamp": self.timestamp,
            "target_url": self.target_url,
            "tests": {}
        }

        logger.info(f"Initialized security test runner for {self.target_url}")
        logger.info(f"Reports will be saved to {self.reports_dir}")

    def run_dependency_check(self) -> Dict[str, Any]:
        """
        Run dependency check to identify vulnerabilities in dependencies.

        Returns:
            Dictionary with test results
        """
        logger.info("Running dependency check")

        report_file = self.reports_dir / f"dependency_check_{self.timestamp}.json"

        try:
            # Check if safety is installed
            try:
                subprocess.run(
                    ["safety", "--version"],
                    check=True,
                    capture_output=True,
                    text=True
                )
            except (subprocess.CalledProcessError, FileNotFoundError):
                logger.warning("Safety is not installed. Installing...")
                subprocess.run(
                    [sys.executable, "-m", "pip", "install", "safety"],
                    check=True,
                    capture_output=True,
                    text=True
                )

            # Run safety check
            result = subprocess.run(
                [sys.executable, "-m", "safety", "check", "--json", "-r", str(ROOT_DIR / "requirements.txt")],
                check=False,  # Don't raise an exception if vulnerabilities are found
                capture_output=True,
                text=True
            )

            # Parse the output
            try:
                vulnerabilities = json.loads(result.stdout)
            except json.JSONDecodeError:
                vulnerabilities = []

            # Save the report
            with open(report_file, "w") as f:
                json.dump(vulnerabilities, f, indent=2)

            # Count vulnerabilities by severity
            severity_counts = {
                "critical": 0,
                "high": 0,
                "medium": 0,
                "low": 0
            }

            for vuln in vulnerabilities:
                severity = vuln.get("severity", "").lower()
                if severity in severity_counts:
                    severity_counts[severity] += 1

            # Create test results
            test_results = {
                "status": "completed",
                "vulnerabilities_found": len(vulnerabilities),
                "severity_counts": severity_counts,
                "report_file": str(report_file)
            }

            logger.info(f"Dependency check completed. Found {len(vulnerabilities)} vulnerabilities.")
            logger.info(f"Severity counts: {severity_counts}")

            return test_results
        except Exception as e:
            logger.error(f"Error running dependency check: {str(e)}")

            return {
                "status": "error",
                "error": str(e)
            }

    def run_bandit_scan(self) -> Dict[str, Any]:
        """
        Run Bandit scan to identify security issues in Python code.

        Returns:
            Dictionary with test results
        """
        logger.info("Running Bandit scan")

        report_file = self.reports_dir / f"bandit_scan_{self.timestamp}.json"

        try:
            # Check if bandit is installed
            try:
                subprocess.run(
                    ["bandit", "--version"],
                    check=True,
                    capture_output=True,
                    text=True
                )
            except (subprocess.CalledProcessError, FileNotFoundError):
                logger.warning("Bandit is not installed. Installing...")
                subprocess.run(
                    [sys.executable, "-m", "pip", "install", "bandit"],
                    check=True,
                    capture_output=True,
                    text=True
                )

            # Run bandit scan
            result = subprocess.run(
                [
                    sys.executable, "-m", "bandit",
                    "-r", str(ROOT_DIR),
                    "-f", "json",
                    "-o", str(report_file),
                    "-x", "tests,venv,env,.venv,.env,node_modules"
                ],
                check=False,  # Don't raise an exception if issues are found
                capture_output=True,
                text=True
            )

            # Check if the report file was created
            if not report_file.exists():
                logger.error("Bandit scan failed to create report file")
                return {
                    "status": "error",
                    "error": "Failed to create report file"
                }

            # Parse the report
            with open(report_file, "r") as f:
                report_data = json.load(f)

            # Count issues by severity
            severity_counts = {
                "high": 0,
                "medium": 0,
                "low": 0
            }

            for result in report_data.get("results", []):
                severity = result.get("issue_severity", "").lower()
                if severity in severity_counts:
                    severity_counts[severity] += 1

            # Create test results
            test_results = {
                "status": "completed",
                "issues_found": len(report_data.get("results", [])),
                "severity_counts": severity_counts,
                "report_file": str(report_file)
            }

            logger.info(f"Bandit scan completed. Found {test_results['issues_found']} issues.")
            logger.info(f"Severity counts: {severity_counts}")

            return test_results
        except Exception as e:
            logger.error(f"Error running Bandit scan: {str(e)}")

            return {
                "status": "error",
                "error": str(e)
            }

    def run_api_security_scan(self) -> Dict[str, Any]:
        """
        Run API security scan to identify security issues in the API.

        Returns:
            Dictionary with test results
        """
        logger.info(f"Running API security scan against {self.target_url}")

        report_file = self.reports_dir / f"api_security_scan_{self.timestamp}.json"

        try:
            # Check if the API is accessible
            try:
                response = requests.get(self.target_url, timeout=5)
                response.raise_for_status()
            except requests.exceptions.RequestException as e:
                logger.error(f"API is not accessible: {str(e)}")
                return {
                    "status": "error",
                    "error": f"API is not accessible: {str(e)}"
                }

            # Define security checks
            security_checks = [
                self._check_security_headers,
                self._check_cors_configuration,
                self._check_content_type,
                self._check_error_handling,
                self._check_rate_limiting
            ]

            # Run security checks
            check_results = {}
            for check in security_checks:
                check_name = check.__name__.replace("_check_", "")
                try:
                    check_result = check()
                    check_results[check_name] = check_result
                except Exception as e:
                    logger.error(f"Error running {check_name} check: {str(e)}")
                    check_results[check_name] = {
                        "status": "error",
                        "error": str(e)
                    }

            # Save the report
            with open(report_file, "w") as f:
                json.dump(check_results, f, indent=2)

            # Count issues
            issues_found = 0
            for check_name, check_result in check_results.items():
                if check_result.get("status") == "failed":
                    issues_found += 1

            # Create test results
            test_results = {
                "status": "completed",
                "issues_found": issues_found,
                "checks_performed": len(security_checks),
                "report_file": str(report_file)
            }

            logger.info(f"API security scan completed. Found {issues_found} issues.")

            return test_results
        except Exception as e:
            logger.error(f"Error running API security scan: {str(e)}")

            return {
                "status": "error",
                "error": str(e)
            }

    def _check_security_headers(self) -> Dict[str, Any]:
        """
        Check security headers in the API response.

        Returns:
            Dictionary with check results
        """
        logger.info("Checking security headers")

        try:
            # Send a request to the API
            response = requests.get(self.target_url, timeout=5)

            # Define required security headers
            required_headers = {
                "X-Content-Type-Options": "nosniff",
                "X-Frame-Options": ["DENY", "SAMEORIGIN"],
                "X-XSS-Protection": "1; mode=block",
                "Content-Security-Policy": None,
                "Strict-Transport-Security": None,
                "Referrer-Policy": None
            }

            # Check headers
            missing_headers = []
            incorrect_headers = []

            for header, expected_value in required_headers.items():
                if header not in response.headers:
                    missing_headers.append(header)
                elif expected_value is not None:
                    if isinstance(expected_value, list):
                        if response.headers[header] not in expected_value:
                            incorrect_headers.append({
                                "header": header,
                                "expected": expected_value,
                                "actual": response.headers[header]
                            })
                    elif response.headers[header] != expected_value:
                        incorrect_headers.append({
                            "header": header,
                            "expected": expected_value,
                            "actual": response.headers[header]
                        })

            # Create check results
            if missing_headers or incorrect_headers:
                return {
                    "status": "failed",
                    "missing_headers": missing_headers,
                    "incorrect_headers": incorrect_headers
                }
            else:
                return {
                    "status": "passed"
                }
        except Exception as e:
            logger.error(f"Error checking security headers: {str(e)}")

            return {
                "status": "error",
                "error": str(e)
            }

    def _check_cors_configuration(self) -> Dict[str, Any]:
        """
        Check CORS configuration in the API.

        Returns:
            Dictionary with check results
        """
        logger.info("Checking CORS configuration")

        try:
            # Send a preflight request to the API
            headers = {
                "Origin": "https://example.com",
                "Access-Control-Request-Method": "GET",
                "Access-Control-Request-Headers": "Content-Type"
            }

            response = requests.options(self.target_url, headers=headers, timeout=5)

            # Check CORS headers
            cors_headers = [
                "Access-Control-Allow-Origin",
                "Access-Control-Allow-Methods",
                "Access-Control-Allow-Headers",
                "Access-Control-Max-Age"
            ]

            missing_headers = []
            insecure_headers = []

            for header in cors_headers:
                if header not in response.headers:
                    missing_headers.append(header)

            # Check for insecure CORS configuration
            if "Access-Control-Allow-Origin" in response.headers:
                if response.headers["Access-Control-Allow-Origin"] == "*":
                    insecure_headers.append({
                        "header": "Access-Control-Allow-Origin",
                        "value": "*",
                        "issue": "Wildcard origin is insecure"
                    })

            # Create check results
            if missing_headers or insecure_headers:
                return {
                    "status": "failed",
                    "missing_headers": missing_headers,
                    "insecure_headers": insecure_headers
                }
            else:
                return {
                    "status": "passed"
                }
        except Exception as e:
            logger.error(f"Error checking CORS configuration: {str(e)}")

            return {
                "status": "error",
                "error": str(e)
            }

    def _check_content_type(self) -> Dict[str, Any]:
        """
        Check content type headers in the API response.

        Returns:
            Dictionary with check results
        """
        logger.info("Checking content type headers")

        try:
            # Send a request to the API
            response = requests.get(self.target_url, timeout=5)

            # Check content type header
            if "Content-Type" not in response.headers:
                return {
                    "status": "failed",
                    "issue": "Content-Type header is missing"
                }

            # Check for proper content type
            content_type = response.headers["Content-Type"]
            if not content_type.startswith("application/json"):
                return {
                    "status": "failed",
                    "issue": f"Content-Type is not application/json: {content_type}"
                }

            return {
                "status": "passed"
            }
        except Exception as e:
            logger.error(f"Error checking content type: {str(e)}")

            return {
                "status": "error",
                "error": str(e)
            }

    def _check_error_handling(self) -> Dict[str, Any]:
        """
        Check error handling in the API.

        Returns:
            Dictionary with check results
        """
        logger.info("Checking error handling")

        try:
            # Send a request to a non-existent endpoint
            response = requests.get(f"{self.target_url}/non-existent-endpoint", timeout=5)

            # Check status code
            if response.status_code != 404:
                return {
                    "status": "failed",
                    "issue": f"Expected 404 status code, got {response.status_code}"
                }

            # Check content type
            if "Content-Type" not in response.headers:
                return {
                    "status": "failed",
                    "issue": "Content-Type header is missing in error response"
                }

            # Check for proper error format
            try:
                error_data = response.json()

                if "error" not in error_data:
                    return {
                        "status": "failed",
                        "issue": "Error response does not contain 'error' field"
                    }

                if "message" not in error_data["error"]:
                    return {
                        "status": "failed",
                        "issue": "Error response does not contain 'message' field"
                    }

                # Check for sensitive information in error message
                error_message = error_data["error"]["message"]
                sensitive_patterns = [
                    "exception",
                    "traceback",
                    "stack",
                    "at line",
                    "syntax error",
                    "sql",
                    "database",
                    "password",
                    "token",
                    "secret"
                ]

                for pattern in sensitive_patterns:
                    if pattern.lower() in error_message.lower():
                        return {
                            "status": "failed",
                            "issue": f"Error message contains sensitive information: '{pattern}'"
                        }

                return {
                    "status": "passed"
                }
            except (json.JSONDecodeError, KeyError):
                return {
                    "status": "failed",
                    "issue": "Error response is not valid JSON or does not have the expected format"
                }
        except Exception as e:
            logger.error(f"Error checking error handling: {str(e)}")

            return {
                "status": "error",
                "error": str(e)
            }

    def _check_rate_limiting(self) -> Dict[str, Any]:
        """
        Check rate limiting in the API.

        Returns:
            Dictionary with check results
        """
        logger.info("Checking rate limiting")

        try:
            # Send multiple requests to the API
            num_requests = 20
            rate_limit_headers = [
                "X-RateLimit-Limit",
                "X-RateLimit-Remaining",
                "X-RateLimit-Reset"
            ]

            rate_limited = False
            has_rate_limit_headers = False

            for i in range(num_requests):
                response = requests.get(self.target_url, timeout=5)

                # Check for rate limit headers
                if any(header in response.headers for header in rate_limit_headers):
                    has_rate_limit_headers = True

                # Check if rate limited
                if response.status_code == 429:
                    rate_limited = True
                    break

            # Create check results
            if not has_rate_limit_headers:
                return {
                    "status": "failed",
                    "issue": "Rate limit headers are missing"
                }

            return {
                "status": "passed",
                "rate_limited": rate_limited
            }
        except Exception as e:
            logger.error(f"Error checking rate limiting: {str(e)}")

            return {
                "status": "error",
                "error": str(e)
            }

    def run_unit_tests(self) -> Dict[str, Any]:
        """
        Run unit tests for security components.

        Returns:
            Dictionary with test results
        """
        logger.info("Running security unit tests")

        report_file = self.reports_dir / f"security_unit_tests_{self.timestamp}.json"

        try:
            # Check if pytest is installed
            try:
                subprocess.run(
                    ["pytest", "--version"],
                    check=True,
                    capture_output=True,
                    text=True
                )
            except (subprocess.CalledProcessError, FileNotFoundError):
                logger.warning("Pytest is not installed. Installing...")
                subprocess.run(
                    [sys.executable, "-m", "pip", "install", "pytest"],
                    check=True,
                    capture_output=True,
                    text=True
                )

            # Run pytest for security tests
            result = subprocess.run(
                [
                    "pytest",
                    str(ROOT_DIR / "security" / "testing" / "test_*.py"),
                    "-v",
                    "--junitxml", str(self.reports_dir / f"security_unit_tests_{self.timestamp}.xml")
                ],
                check=False,
                capture_output=True,
                text=True
            )

            # Parse the output
            test_output = result.stdout

            # Count passed and failed tests
            passed_count = test_output.count("PASSED")
            failed_count = test_output.count("FAILED")
            error_count = test_output.count("ERROR")
            skipped_count = test_output.count("SKIPPED")

            # Save the report
            with open(report_file, "w") as f:
                json.dump({
                    "output": test_output,
                    "passed": passed_count,
                    "failed": failed_count,
                    "errors": error_count,
                    "skipped": skipped_count,
                    "return_code": result.returncode
                }, f, indent=2)

            # Create test results
            test_results = {
                "status": "completed",
                "passed": passed_count,
                "failed": failed_count,
                "errors": error_count,
                "skipped": skipped_count,
                "total": passed_count + failed_count + error_count + skipped_count,
                "report_file": str(report_file)
            }

            logger.info(f"Security unit tests completed. Passed: {passed_count}, Failed: {failed_count}, Errors: {error_count}, Skipped: {skipped_count}")

            return test_results
        except Exception as e:
            logger.error(f"Error running security unit tests: {str(e)}")

            return {
                "status": "error",
                "error": str(e)
            }

    def run_all_tests(self) -> Dict[str, Any]:
        """
        Run all security tests.

        Returns:
            Dictionary with all test results
        """
        logger.info("Running all security tests")

        # Run dependency check
        self.results["tests"]["dependency_check"] = self.run_dependency_check()

        # Run bandit scan
        self.results["tests"]["bandit_scan"] = self.run_bandit_scan()

        # Run API security scan
        self.results["tests"]["api_security_scan"] = self.run_api_security_scan()

        # Run unit tests
        self.results["tests"]["unit_tests"] = self.run_unit_tests()

        # Save results
        self.save_results()

        return self.results

    def save_results(self) -> str:
        """
        Save test results to a file.

        Returns:
            Path to the results file
        """
        results_file = self.reports_dir / f"security_test_results_{self.timestamp}.json"

        with open(results_file, "w") as f:
            json.dump(self.results, f, indent=2)

        logger.info(f"Test results saved to {results_file}")

        return str(results_file)

    def generate_report(self) -> str:
        """
        Generate a human-readable report from the test results.

        Returns:
            Path to the report file
        """
        report_file = self.reports_dir / f"security_test_report_{self.timestamp}.md"

        with open(report_file, "w") as f:
            f.write("# Security Test Report\n\n")
            f.write(f"**Date:** {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write(f"**Target:** {self.target_url}\n\n")

            # Write summary
            f.write("## Summary\n\n")

            total_issues = 0
            for test_name, test_results in self.results["tests"].items():
                if test_results.get("status") == "completed":
                    if "vulnerabilities_found" in test_results:
                        total_issues += test_results["vulnerabilities_found"]
                    elif "issues_found" in test_results:
                        total_issues += test_results["issues_found"]

            f.write(f"**Total Issues Found:** {total_issues}\n\n")

            # Write test results
            for test_name, test_results in self.results["tests"].items():
                f.write(f"## {test_name.replace('_', ' ').title()}\n\n")

                if test_results.get("status") == "completed":
                    if "vulnerabilities_found" in test_results:
                        f.write(f"**Vulnerabilities Found:** {test_results['vulnerabilities_found']}\n\n")

                        if "severity_counts" in test_results:
                            f.write("**Severity Counts:**\n\n")
                            for severity, count in test_results["severity_counts"].items():
                                f.write(f"- {severity.title()}: {count}\n")
                            f.write("\n")

                    elif "issues_found" in test_results:
                        f.write(f"**Issues Found:** {test_results['issues_found']}\n\n")

                        if "severity_counts" in test_results:
                            f.write("**Severity Counts:**\n\n")
                            for severity, count in test_results["severity_counts"].items():
                                f.write(f"- {severity.title()}: {count}\n")
                            f.write("\n")

                    elif "passed" in test_results:
                        # Unit test results
                        f.write(f"**Tests Summary:**\n\n")
                        f.write(f"- **Passed:** {test_results['passed']}\n")
                        f.write(f"- **Failed:** {test_results['failed']}\n")
                        f.write(f"- **Errors:** {test_results['errors']}\n")
                        f.write(f"- **Skipped:** {test_results['skipped']}\n")
                        f.write(f"- **Total:** {test_results['total']}\n\n")

                    if "report_file" in test_results:
                        f.write(f"**Detailed Report:** {test_results['report_file']}\n\n")

                elif test_results.get("status") == "error":
                    f.write(f"**Error:** {test_results.get('error', 'Unknown error')}\n\n")

            # Write recommendations
            f.write("## Recommendations\n\n")

            f.write("1. **Fix identified vulnerabilities** - Address all vulnerabilities found in the dependency check.\n")
            f.write("2. **Fix identified code issues** - Address all issues found in the Bandit scan.\n")
            f.write("3. **Improve API security** - Address all issues found in the API security scan.\n")
            f.write("4. **Implement regular security testing** - Run security tests regularly to identify and fix issues.\n")
            f.write("5. **Keep dependencies up to date** - Regularly update dependencies to address security vulnerabilities.\n")

        logger.info(f"Report generated at {report_file}")

        return str(report_file)

def main():
    """
    Main function.
    """
    parser = argparse.ArgumentParser(description="Run security tests for Bulo.Cloud Sentinel")
    parser.add_argument("--target", default="http://localhost:8000", help="Target URL")
    parser.add_argument("--reports-dir", default=str(REPORTS_DIR), help="Directory to save reports")
    parser.add_argument("--verbose", action="store_true", help="Enable verbose output")
    parser.add_argument("--output", default="json", choices=["json", "markdown"], help="Output format")

    args = parser.parse_args()

    # Create security test runner
    runner = SecurityTestRunner(
        target_url=args.target,
        reports_dir=Path(args.reports_dir),
        verbose=args.verbose
    )

    # Run all tests
    results = runner.run_all_tests()

    # Generate report if requested
    if args.output == "markdown":
        report_file = runner.generate_report()
        logger.info(f"Report generated at {report_file}")
    else:
        logger.info(f"Results saved to {runner.save_results()}")

    return 0

if __name__ == "__main__":
    sys.exit(main())
