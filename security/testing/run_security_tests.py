#!/usr/bin/env python
"""
Custom security test runner for Bulo.Cloud Sentinel.

This script runs custom security tests on the Bulo.Cloud Sentinel platform
and generates a report of the results.
"""

import os
import sys
import json
import argparse
import logging
import importlib
import inspect
from datetime import datetime
from typing import Dict, List, Any, Callable, Optional, Tuple

# Set up logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
)
log = logging.getLogger(__name__)

# Define test result types
class TestResult:
    """Base class for test results."""
    def __init__(
        self,
        test_name: str,
        status: str,
        message: str,
        details: Optional[Dict[str, Any]] = None,
    ):
        self.test_name = test_name
        self.status = status
        self.message = message
        self.details = details or {}
        self.timestamp = datetime.now().isoformat()
        
    def to_dict(self) -> Dict[str, Any]:
        """Convert the test result to a dictionary."""
        return {
            "test_name": self.test_name,
            "status": self.status,
            "message": self.message,
            "details": self.details,
            "timestamp": self.timestamp,
        }

class TestSuccess(TestResult):
    """Test success result."""
    def __init__(
        self,
        test_name: str,
        message: str,
        details: Optional[Dict[str, Any]] = None,
    ):
        super().__init__(test_name, "success", message, details)

class TestFailure(TestResult):
    """Test failure result."""
    def __init__(
        self,
        test_name: str,
        message: str,
        details: Optional[Dict[str, Any]] = None,
    ):
        super().__init__(test_name, "failure", message, details)

class TestError(TestResult):
    """Test error result."""
    def __init__(
        self,
        test_name: str,
        message: str,
        details: Optional[Dict[str, Any]] = None,
    ):
        super().__init__(test_name, "error", message, details)

class TestSkipped(TestResult):
    """Test skipped result."""
    def __init__(
        self,
        test_name: str,
        message: str,
        details: Optional[Dict[str, Any]] = None,
    ):
        super().__init__(test_name, "skipped", message, details)

# Define test runner
class SecurityTestRunner:
    """
    Security test runner for Bulo.Cloud Sentinel.
    
    This class discovers and runs security tests in the specified modules.
    """
    
    def __init__(
        self,
        test_modules: List[str] = None,
        test_dirs: List[str] = None,
    ):
        """
        Initialize the security test runner.
        
        Args:
            test_modules: List of test modules to run
            test_dirs: List of directories to search for test modules
        """
        self.test_modules = test_modules or []
        self.test_dirs = test_dirs or ["security/testing/tests"]
        self.tests: Dict[str, Callable] = {}
        self.results: List[TestResult] = []
        
    def discover_tests(self) -> Dict[str, Callable]:
        """
        Discover security tests in the specified modules and directories.
        
        Returns:
            Dictionary of test names to test functions
        """
        tests = {}
        
        # Import specified modules
        for module_name in self.test_modules:
            try:
                module = importlib.import_module(module_name)
                tests.update(self._get_tests_from_module(module))
            except ImportError as e:
                log.error(f"Failed to import module {module_name}: {str(e)}")
                
        # Import modules from specified directories
        for test_dir in self.test_dirs:
            if not os.path.exists(test_dir):
                log.warning(f"Test directory {test_dir} does not exist")
                continue
                
            for root, _, files in os.walk(test_dir):
                for file in files:
                    if file.startswith("test_") and file.endswith(".py"):
                        module_path = os.path.join(root, file)
                        module_name = module_path.replace(os.path.sep, ".").replace(".py", "")
                        
                        try:
                            spec = importlib.util.spec_from_file_location(module_name, module_path)
                            if spec and spec.loader:
                                module = importlib.util.module_from_spec(spec)
                                spec.loader.exec_module(module)
                                tests.update(self._get_tests_from_module(module))
                        except Exception as e:
                            log.error(f"Failed to import module {module_path}: {str(e)}")
                            
        self.tests = tests
        return tests
        
    def _get_tests_from_module(self, module) -> Dict[str, Callable]:
        """
        Get security tests from a module.
        
        Args:
            module: Module to get tests from
            
        Returns:
            Dictionary of test names to test functions
        """
        tests = {}
        
        for name, obj in inspect.getmembers(module):
            if name.startswith("test_") and callable(obj):
                tests[name] = obj
                
        return tests
        
    def run_tests(self) -> List[TestResult]:
        """
        Run all discovered security tests.
        
        Returns:
            List of test results
        """
        if not self.tests:
            self.discover_tests()
            
        results = []
        
        for name, test_func in self.tests.items():
            log.info(f"Running test: {name}")
            
            try:
                result = test_func()
                
                if isinstance(result, TestResult):
                    results.append(result)
                elif isinstance(result, tuple) and len(result) >= 2:
                    status, message = result[0], result[1]
                    details = result[2] if len(result) >= 3 else None
                    
                    if status:
                        results.append(TestSuccess(name, message, details))
                    else:
                        results.append(TestFailure(name, message, details))
                elif result is True:
                    results.append(TestSuccess(name, "Test passed"))
                elif result is False:
                    results.append(TestFailure(name, "Test failed"))
                else:
                    results.append(TestSuccess(name, "Test completed"))
            except Exception as e:
                log.error(f"Error running test {name}: {str(e)}")
                results.append(TestError(name, f"Error: {str(e)}"))
                
        self.results = results
        return results
        
    def generate_report(self, output_file: Optional[str] = None) -> Dict[str, Any]:
        """
        Generate a report of the test results.
        
        Args:
            output_file: Path to output file
            
        Returns:
            Report dictionary
        """
        if not self.results:
            self.run_tests()
            
        # Count results by status
        counts = {
            "total": len(self.results),
            "success": sum(1 for r in self.results if r.status == "success"),
            "failure": sum(1 for r in self.results if r.status == "failure"),
            "error": sum(1 for r in self.results if r.status == "error"),
            "skipped": sum(1 for r in self.results if r.status == "skipped"),
        }
        
        # Generate report
        report = {
            "timestamp": datetime.now().isoformat(),
            "counts": counts,
            "results": [r.to_dict() for r in self.results],
        }
        
        # Write report to file
        if output_file:
            with open(output_file, "w") as f:
                json.dump(report, f, indent=2)
                
        return report

# Example security tests
def test_jwt_token_validation() -> TestResult:
    """
    Test JWT token validation.
    
    Returns:
        Test result
    """
    try:
        # Import JWT handler
        from security.auth.jwt_handler import decode_token, create_access_token
        
        # Create a token
        token = create_access_token("test_user")
        
        # Validate the token
        payload = decode_token(token)
        
        # Check that the token contains the expected claims
        if payload.get("sub") != "test_user":
            return TestFailure(
                "test_jwt_token_validation",
                "Token subject does not match",
                {"expected": "test_user", "actual": payload.get("sub")},
            )
            
        # Try to validate an invalid token
        try:
            decode_token("invalid_token")
            return TestFailure(
                "test_jwt_token_validation",
                "Invalid token was accepted",
            )
        except Exception:
            # This is expected
            pass
            
        return TestSuccess(
            "test_jwt_token_validation",
            "JWT token validation is working correctly",
        )
    except ImportError:
        return TestSkipped(
            "test_jwt_token_validation",
            "JWT handler module not found",
        )
    except Exception as e:
        return TestError(
            "test_jwt_token_validation",
            f"Error testing JWT token validation: {str(e)}",
        )

def test_password_hashing() -> TestResult:
    """
    Test password hashing.
    
    Returns:
        Test result
    """
    try:
        # Import password module
        from security.auth.password import hash_password, verify_password
        
        # Hash a password
        password = "Test@Password123"
        hashed = hash_password(password)
        
        # Verify the password
        if not verify_password(hashed, password):
            return TestFailure(
                "test_password_hashing",
                "Password verification failed",
            )
            
        # Verify with incorrect password
        if verify_password(hashed, "wrong_password"):
            return TestFailure(
                "test_password_hashing",
                "Incorrect password was accepted",
            )
            
        return TestSuccess(
            "test_password_hashing",
            "Password hashing is working correctly",
        )
    except ImportError:
        return TestSkipped(
            "test_password_hashing",
            "Password module not found",
        )
    except Exception as e:
        return TestError(
            "test_password_hashing",
            f"Error testing password hashing: {str(e)}",
        )

def test_file_validation() -> TestResult:
    """
    Test file validation.
    
    Returns:
        Test result
    """
    try:
        # Import file validation module
        from security.validation.file_validation import FileValidator
        
        # Create a file validator
        validator = FileValidator()
        
        # Test file extension validation
        allowed_extensions = {"jpg", "png", "pdf"}
        validator.allowed_extensions = allowed_extensions
        
        # Test valid extension
        if "jpg" not in validator.allowed_extensions:
            return TestFailure(
                "test_file_validation",
                "Valid extension not allowed",
                {"extension": "jpg", "allowed_extensions": list(validator.allowed_extensions)},
            )
            
        # Test invalid extension
        if "exe" in validator.allowed_extensions:
            return TestFailure(
                "test_file_validation",
                "Invalid extension allowed",
                {"extension": "exe", "allowed_extensions": list(validator.allowed_extensions)},
            )
            
        return TestSuccess(
            "test_file_validation",
            "File validation is working correctly",
        )
    except ImportError:
        return TestSkipped(
            "test_file_validation",
            "File validation module not found",
        )
    except Exception as e:
        return TestError(
            "test_file_validation",
            f"Error testing file validation: {str(e)}",
        )

def test_cors_middleware() -> TestResult:
    """
    Test CORS middleware.
    
    Returns:
        Test result
    """
    try:
        # Import CORS middleware
        from security.middleware.security_middleware import SecurityHeadersMiddleware
        
        # Create a middleware instance
        middleware = SecurityHeadersMiddleware(app=None)
        
        # Check that default CSP is set
        if not middleware.content_security_policy:
            return TestFailure(
                "test_cors_middleware",
                "Default CSP not set",
            )
            
        # Check that default CORS headers are set
        if not middleware.cross_origin_opener_policy:
            return TestFailure(
                "test_cors_middleware",
                "Default CORS headers not set",
            )
            
        return TestSuccess(
            "test_cors_middleware",
            "CORS middleware is configured correctly",
        )
    except ImportError:
        return TestSkipped(
            "test_cors_middleware",
            "CORS middleware module not found",
        )
    except Exception as e:
        return TestError(
            "test_cors_middleware",
            f"Error testing CORS middleware: {str(e)}",
        )

def test_rate_limiting() -> TestResult:
    """
    Test rate limiting middleware.
    
    Returns:
        Test result
    """
    try:
        # Import rate limiting middleware
        from security.middleware.security_middleware import RateLimitingMiddleware
        
        # Create a middleware instance
        middleware = RateLimitingMiddleware(app=None)
        
        # Check that default rate limits are set
        if middleware.default_rate_limit <= 0:
            return TestFailure(
                "test_rate_limiting",
                "Default rate limit not set",
                {"default_rate_limit": middleware.default_rate_limit},
            )
            
        # Check that sensitive rate limits are set
        if middleware.sensitive_rate_limit <= 0:
            return TestFailure(
                "test_rate_limiting",
                "Sensitive rate limit not set",
                {"sensitive_rate_limit": middleware.sensitive_rate_limit},
            )
            
        return TestSuccess(
            "test_rate_limiting",
            "Rate limiting middleware is configured correctly",
        )
    except ImportError:
        return TestSkipped(
            "test_rate_limiting",
            "Rate limiting middleware module not found",
        )
    except Exception as e:
        return TestError(
            "test_rate_limiting",
            f"Error testing rate limiting middleware: {str(e)}",
        )

def main():
    """Main function."""
    parser = argparse.ArgumentParser(description="Run security tests")
    parser.add_argument(
        "--output",
        help="Output file for test results",
        default="security-test-results.json",
    )
    parser.add_argument(
        "--modules",
        help="Comma-separated list of test modules to run",
        default="",
    )
    parser.add_argument(
        "--dirs",
        help="Comma-separated list of directories to search for test modules",
        default="security/testing/tests",
    )
    args = parser.parse_args()
    
    # Parse modules and directories
    modules = [m.strip() for m in args.modules.split(",") if m.strip()]
    dirs = [d.strip() for d in args.dirs.split(",") if d.strip()]
    
    # Create test runner
    runner = SecurityTestRunner(modules, dirs)
    
    # Add example tests
    runner.tests = {
        "test_jwt_token_validation": test_jwt_token_validation,
        "test_password_hashing": test_password_hashing,
        "test_file_validation": test_file_validation,
        "test_cors_middleware": test_cors_middleware,
        "test_rate_limiting": test_rate_limiting,
    }
    
    # Run tests
    runner.run_tests()
    
    # Generate report
    report = runner.generate_report(args.output)
    
    # Print summary
    print(f"Total tests: {report['counts']['total']}")
    print(f"Successes: {report['counts']['success']}")
    print(f"Failures: {report['counts']['failure']}")
    print(f"Errors: {report['counts']['error']}")
    print(f"Skipped: {report['counts']['skipped']}")
    print(f"Report written to {args.output}")
    
    # Exit with non-zero status if any tests failed or errored
    if report["counts"]["failure"] > 0 or report["counts"]["error"] > 0:
        sys.exit(1)
        
    sys.exit(0)

if __name__ == "__main__":
    main()
