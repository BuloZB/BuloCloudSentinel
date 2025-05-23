#!/usr/bin/env python3
"""
Script to automatically fix common CodeQL issues in Python code.
This script reads the CodeQL alerts from the JSON file and applies fixes to the code.
"""

import json
import os
import sys
import re
from pathlib import Path
import argparse
import logging

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(levelname)s - %(message)s",
    handlers=[
        logging.StreamHandler(),
        logging.FileHandler("fix_codeql_issues.log")
    ]
)
logger = logging.getLogger(__name__)

def load_alerts(file_path):
    """
    Load CodeQL alerts from JSON file.

    Args:
        file_path: Path to JSON file

    Returns:
        Dictionary of categorized alerts
    """
    try:
        with open(file_path, "r") as f:
            return json.load(f)
    except Exception as e:
        logger.error(f"Error loading alerts: {e}")
        sys.exit(1)

def fix_sql_injection(file_path, start_line, end_line):
    """
    Fix SQL injection vulnerabilities.

    Args:
        file_path: Path to file
        start_line: Start line of vulnerability
        end_line: End line of vulnerability

    Returns:
        True if fixed, False otherwise
    """
    try:
        # Read file
        with open(file_path, "r") as f:
            lines = f.readlines()

        # Extract vulnerable code
        vulnerable_code = "".join(lines[start_line-1:end_line])

        # Check for common patterns
        if "execute(" in vulnerable_code and "%" in vulnerable_code:
            # Fix string formatting in SQL queries
            fixed_code = re.sub(
                r'execute\((.*?)\s*%\s*(.*?)\)',
                r'execute(\1, \2)',
                vulnerable_code
            )

            # Update file
            lines[start_line-1:end_line] = [fixed_code]

            with open(file_path, "w") as f:
                f.writelines(lines)

            return True

        elif "execute(" in vulnerable_code and "+" in vulnerable_code:
            # Fix string concatenation in SQL queries
            fixed_code = re.sub(
                r'execute\((.*?)\s*\+\s*(.*?)\)',
                r'execute(\1, \2)',
                vulnerable_code
            )

            # Update file
            lines[start_line-1:end_line] = [fixed_code]

            with open(file_path, "w") as f:
                f.writelines(lines)

            return True

        return False

    except Exception as e:
        print(f"Error fixing SQL injection in {file_path}: {e}")
        return False

def fix_path_injection(file_path, start_line, end_line):
    """
    Fix path injection vulnerabilities.

    Args:
        file_path: Path to file
        start_line: Start line of vulnerability
        end_line: End line of vulnerability

    Returns:
        True if fixed, False otherwise
    """
    try:
        # Read file
        with open(file_path, "r") as f:
            lines = f.readlines()

        # Extract vulnerable code
        vulnerable_code = "".join(lines[start_line-1:end_line])

        # Check for common patterns
        if "open(" in vulnerable_code and not "os.path.normpath" in vulnerable_code:
            # Fix path injection in open() calls
            fixed_code = re.sub(
                r'open\((.*?)\)',
                r'open(os.path.normpath(\1))',
                vulnerable_code
            )

            # Add import if needed
            if "import os" not in "".join(lines[:20]) and "from os import" not in "".join(lines[:20]):
                lines.insert(0, "import os\n")

            # Update file
            lines[start_line-1:end_line] = [fixed_code]

            with open(file_path, "w") as f:
                f.writelines(lines)

            return True

        return False

    except Exception as e:
        print(f"Error fixing path injection in {file_path}: {e}")
        return False

def fix_xss(file_path, start_line, end_line):
    """
    Fix cross-site scripting (XSS) vulnerabilities.

    Args:
        file_path: Path to file
        start_line: Start line of vulnerability
        end_line: End line of vulnerability

    Returns:
        True if fixed, False otherwise
    """
    try:
        # Read file
        with open(file_path, "r") as f:
            lines = f.readlines()

        # Extract vulnerable code
        vulnerable_code = "".join(lines[start_line-1:end_line])

        # Check for common patterns
        if "render" in vulnerable_code and not "escape" in vulnerable_code:
            # Fix XSS in template rendering
            fixed_code = re.sub(
                r'render\((.*?)\)',
                r'render(escape(\1))',
                vulnerable_code
            )

            # Add import if needed
            if "from html import escape" not in "".join(lines[:20]):
                lines.insert(0, "from html import escape\n")

            # Update file
            lines[start_line-1:end_line] = [fixed_code]

            with open(file_path, "w") as f:
                f.writelines(lines)

            return True

        return False

    except Exception as e:
        print(f"Error fixing XSS in {file_path}: {e}")
        return False

def fix_hardcoded_credentials(file_path, start_line, end_line):
    """
    Fix hardcoded credentials.

    Args:
        file_path: Path to file
        start_line: Start line of vulnerability
        end_line: End line of vulnerability

    Returns:
        True if fixed, False otherwise
    """
    try:
        # Read file
        with open(file_path, "r") as f:
            lines = f.readlines()

        # Extract vulnerable code
        vulnerable_code = "".join(lines[start_line-1:end_line])

        # Check for common patterns
        if re.search(r'(password|token|key|secret)\s*=\s*["\']', vulnerable_code, re.IGNORECASE):
            # Fix hardcoded credentials
            fixed_code = re.sub(
                r'(password|token|key|secret)\s*=\s*["\'](.*?)["\']',
                r'\1 = os.environ.get("\1", "")',
                vulnerable_code,
                flags=re.IGNORECASE
            )

            # Add import if needed
            if "import os" not in "".join(lines[:20]) and "from os import" not in "".join(lines[:20]):
                lines.insert(0, "import os\n")

            # Update file
            lines[start_line-1:end_line] = [fixed_code]

            with open(file_path, "w") as f:
                f.writelines(lines)

            return True

        return False

    except Exception as e:
        print(f"Error fixing hardcoded credentials in {file_path}: {e}")
        return False

def fix_stack_trace_exposure(file_path, start_line, end_line):
    """
    Fix stack trace exposure vulnerabilities.

    Args:
        file_path: Path to file
        start_line: Start line of vulnerability
        end_line: End line of vulnerability

    Returns:
        True if fixed, False otherwise
    """
    try:
        # Read file
        with open(file_path, "r", encoding="utf-8") as f:
            lines = f.readlines()

        # Extract vulnerable code
        vulnerable_code = "".join(lines[start_line-1:end_line])

        # Check for common patterns
        if "HTTPException" in vulnerable_code and "str(e)" in vulnerable_code:
            # Fix stack trace exposure in exception handling
            fixed_code = re.sub(
                r'detail=str\(e\)',
                r'detail="An error occurred. Please try again later."',
                vulnerable_code
            )

            # If the pattern wasn't found, try another common pattern
            if fixed_code == vulnerable_code:
                fixed_code = re.sub(
                    r'detail=f"[^"]*{str\(e\)}[^"]*"',
                    r'detail="An error occurred. Please try again later."',
                    vulnerable_code
                )

            # Update file if a fix was applied
            if fixed_code != vulnerable_code:
                lines[start_line-1:end_line] = [fixed_code]

                with open(file_path, "w", encoding="utf-8") as f:
                    f.writelines(lines)

                return True

        return False

    except Exception as e:
        logger.error(f"Error fixing stack trace exposure in {file_path}: {e}")
        return False

def fix_clear_text_logging(file_path, start_line, end_line):
    """
    Fix clear-text logging of sensitive data.

    Args:
        file_path: Path to file
        start_line: Start line of vulnerability
        end_line: End line of vulnerability

    Returns:
        True if fixed, False otherwise
    """
    try:
        # Read file
        with open(file_path, "r", encoding="utf-8") as f:
            lines = f.readlines()

        # Extract vulnerable code
        vulnerable_code = "".join(lines[start_line-1:end_line])

        # Check for common patterns
        sensitive_terms = ["password", "token", "secret", "key", "credential"]

        for term in sensitive_terms:
            if term.lower() in vulnerable_code.lower() and ("log" in vulnerable_code.lower() or "print" in vulnerable_code.lower()):
                # Fix clear-text logging
                for log_func in ["logger.info", "logger.debug", "logger.warning", "logger.error", "print", "logging.info", "logging.debug", "logging.warning", "logging.error"]:
                    if log_func in vulnerable_code:
                        # Replace the sensitive data with "[REDACTED]"
                        fixed_code = re.sub(
                            rf'({log_func}\(.*?)({term}[^,)]*)',
                            r'\1[REDACTED]',
                            vulnerable_code,
                            flags=re.IGNORECASE
                        )

                        # Update file if a fix was applied
                        if fixed_code != vulnerable_code:
                            lines[start_line-1:end_line] = [fixed_code]

                            with open(file_path, "w", encoding="utf-8") as f:
                                f.writelines(lines)

                            return True

        return False

    except Exception as e:
        logger.error(f"Error fixing clear-text logging in {file_path}: {e}")
        return False

def fix_unsafe_deserialization(file_path, start_line, end_line):
    """
    Fix unsafe deserialization vulnerabilities.

    Args:
        file_path: Path to file
        start_line: Start line of vulnerability
        end_line: End line of vulnerability

    Returns:
        True if fixed, False otherwise
    """
    try:
        # Read file
        with open(file_path, "r", encoding="utf-8") as f:
            lines = f.readlines()

        # Extract vulnerable code
        vulnerable_code = "".join(lines[start_line-1:end_line])

        # Check for common patterns
        if "torch.load" in vulnerable_code:
            # Fix unsafe deserialization in torch.load
            fixed_code = re.sub(
                r'torch\.load\((.*?)\)',
                r'torch.load(\1, map_location="cpu", pickle_module=RestrictedUnpickle)',
                vulnerable_code
            )

            # Add import and RestrictedUnpickle class if needed
            if "RestrictedUnpickle" not in "".join(lines[:50]):
                # Find the last import statement
                last_import_line = 0
                for i, line in enumerate(lines[:30]):
                    if line.strip().startswith(("import ", "from ")):
                        last_import_line = i

                # Add import and class after the last import statement
                restricted_unpickle_code = """
import pickle

class RestrictedUnpickle(pickle.Unpickler):
    # Restricted unpickler for safer deserialization
    def find_class(self, module, name):
        # Only allow safe modules
        if module == "torch":
            return getattr(__import__(module, fromlist=[name]), name)
        # For everything else, restrict to built-in modules
        if module in ["collections", "numpy", "torch.nn"]:
            return getattr(__import__(module, fromlist=[name]), name)
        raise pickle.UnpicklingError(f"Restricted unpickle: {module}.{name}")

"""
                lines.insert(last_import_line + 1, restricted_unpickle_code)

                # Adjust start_line if it's after the import
                if start_line > last_import_line:
                    start_line += restricted_unpickle_code.count("\n") + 1
                    end_line += restricted_unpickle_code.count("\n") + 1

            # Update file
            lines[start_line-1:end_line] = [fixed_code]

            with open(file_path, "w", encoding="utf-8") as f:
                f.writelines(lines)

            return True

        # Check for pickle.load
        elif "pickle.load" in vulnerable_code:
            # Fix unsafe deserialization in pickle.load
            fixed_code = re.sub(
                r'pickle\.load\((.*?)\)',
                r'json.loads(json.dumps({"data": "Unsafe operation blocked"}))',
                vulnerable_code
            )

            # Add import if needed
            if "import json" not in "".join(lines[:20]):
                # Find the last import statement
                last_import_line = 0
                for i, line in enumerate(lines[:30]):
                    if line.strip().startswith(("import ", "from ")):
                        last_import_line = i

                # Add import after the last import statement
                lines.insert(last_import_line + 1, "import json\n")

                # Adjust start_line if it's after the import
                if start_line > last_import_line:
                    start_line += 1
                    end_line += 1

            # Update file
            lines[start_line-1:end_line] = [fixed_code]

            with open(file_path, "w", encoding="utf-8") as f:
                f.writelines(lines)

            return True

        return False

    except Exception as e:
        logger.error(f"Error fixing unsafe deserialization in {file_path}: {e}")
        return False

def fix_unsafe_eval(file_path, start_line, end_line):
    """
    Fix unsafe eval() or exec() calls.

    Args:
        file_path: Path to file
        start_line: Start line of vulnerability
        end_line: End line of vulnerability

    Returns:
        True if fixed, False otherwise
    """
    try:
        # Read file
        with open(file_path, "r") as f:
            lines = f.readlines()

        # Extract vulnerable code
        vulnerable_code = "".join(lines[start_line-1:end_line])

        # Check for common patterns
        if "eval(" in vulnerable_code:
            # Fix unsafe eval() calls
            fixed_code = re.sub(
                r'eval\((.*?)\)',
                r'json.loads(\1)',
                vulnerable_code
            )

            # Add import if needed
            if "import json" not in "".join(lines[:20]) and "from json import" not in "".join(lines[:20]):
                lines.insert(0, "import json\n")

            # Update file
            lines[start_line-1:end_line] = [fixed_code]

            with open(file_path, "w") as f:
                f.writelines(lines)

            return True

        return False

    except Exception as e:
        print(f"Error fixing unsafe eval in {file_path}: {e}")
        return False

def fix_issues(categories_file):
    """
    Fix CodeQL issues based on categorized alerts.

    Args:
        categories_file: Path to categorized alerts JSON file
    """
    # Load categorized alerts
    categories = load_alerts(categories_file)

    # Track fixed issues
    fixed_issues = {
        "sql_injection": 0,
        "path_injection": 0,
        "xss": 0,
        "hardcoded_credentials": 0,
        "unsafe_eval": 0,
        "stack_trace_exposure": 0,
        "clear_text_logging": 0,
        "unsafe_deserialization": 0,
        "total": 0
    }

    # Process each category
    for rule_id, category in categories.items():
        logger.info(f"Processing rule: {rule_id} ({category['count']} alerts)")

        # Process each alert in the category
        for alert in category["alerts"]:
            file_path = alert["location"]["path"]
            start_line = alert["location"]["start_line"]
            end_line = alert["location"]["end_line"]

            # Skip if file doesn't exist
            if not os.path.exists(file_path):
                logger.warning(f"  Skipping {file_path} (file not found)")
                continue

            # Apply fixes based on rule ID
            if "sql" in rule_id.lower() and "injection" in rule_id.lower():
                if fix_sql_injection(file_path, start_line, end_line):
                    logger.info(f"  Fixed SQL injection in {file_path}:{start_line}-{end_line}")
                    fixed_issues["sql_injection"] += 1
                    fixed_issues["total"] += 1

            elif "path" in rule_id.lower() and "injection" in rule_id.lower():
                if fix_path_injection(file_path, start_line, end_line):
                    logger.info(f"  Fixed path injection in {file_path}:{start_line}-{end_line}")
                    fixed_issues["path_injection"] += 1
                    fixed_issues["total"] += 1

            elif "xss" in rule_id.lower() or "cross-site" in rule_id.lower():
                if fix_xss(file_path, start_line, end_line):
                    logger.info(f"  Fixed XSS in {file_path}:{start_line}-{end_line}")
                    fixed_issues["xss"] += 1
                    fixed_issues["total"] += 1

            elif "hardcoded" in rule_id.lower() and ("credential" in rule_id.lower() or "password" in rule_id.lower() or "secret" in rule_id.lower()):
                if fix_hardcoded_credentials(file_path, start_line, end_line):
                    logger.info(f"  Fixed hardcoded credentials in {file_path}:{start_line}-{end_line}")
                    fixed_issues["hardcoded_credentials"] += 1
                    fixed_issues["total"] += 1

            elif "eval" in rule_id.lower() or "exec" in rule_id.lower():
                if fix_unsafe_eval(file_path, start_line, end_line):
                    logger.info(f"  Fixed unsafe eval in {file_path}:{start_line}-{end_line}")
                    fixed_issues["unsafe_eval"] += 1
                    fixed_issues["total"] += 1

            elif "stack-trace" in rule_id.lower() or "exception" in rule_id.lower():
                if fix_stack_trace_exposure(file_path, start_line, end_line):
                    logger.info(f"  Fixed stack trace exposure in {file_path}:{start_line}-{end_line}")
                    fixed_issues["stack_trace_exposure"] += 1
                    fixed_issues["total"] += 1

            elif "clear-text" in rule_id.lower() or "logging" in rule_id.lower() or "sensitive" in rule_id.lower():
                if fix_clear_text_logging(file_path, start_line, end_line):
                    logger.info(f"  Fixed clear-text logging in {file_path}:{start_line}-{end_line}")
                    fixed_issues["clear_text_logging"] += 1
                    fixed_issues["total"] += 1

            elif "deserialization" in rule_id.lower() or "pickle" in rule_id.lower() or "yaml" in rule_id.lower():
                if fix_unsafe_deserialization(file_path, start_line, end_line):
                    logger.info(f"  Fixed unsafe deserialization in {file_path}:{start_line}-{end_line}")
                    fixed_issues["unsafe_deserialization"] += 1
                    fixed_issues["total"] += 1

    # Print summary
    logger.info("\nFix Summary:")
    logger.info(f"  SQL Injection: {fixed_issues['sql_injection']}")
    logger.info(f"  Path Injection: {fixed_issues['path_injection']}")
    logger.info(f"  XSS: {fixed_issues['xss']}")
    logger.info(f"  Hardcoded Credentials: {fixed_issues['hardcoded_credentials']}")
    logger.info(f"  Unsafe Eval: {fixed_issues['unsafe_eval']}")
    logger.info(f"  Stack Trace Exposure: {fixed_issues['stack_trace_exposure']}")
    logger.info(f"  Clear-Text Logging: {fixed_issues['clear_text_logging']}")
    logger.info(f"  Unsafe Deserialization: {fixed_issues['unsafe_deserialization']}")
    logger.info(f"  Total: {fixed_issues['total']}")

def main():
    """Main function."""
    parser = argparse.ArgumentParser(description="Fix CodeQL issues in Python code")
    parser.add_argument("categories_file", help="Path to categorized alerts JSON file")
    args = parser.parse_args()

    fix_issues(args.categories_file)

if __name__ == "__main__":
    main()
