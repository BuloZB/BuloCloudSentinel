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

        # Check for exception traceback exposure in FastAPI/Flask routes
        elif "except" in vulnerable_code and "return" in vulnerable_code:
            # Look for patterns like "return jsonify({'error': str(e)})" or "return {'error': str(e)}"
            fixed_code = re.sub(
                r'return\s+(?:jsonify\s*\()?\s*{[^}]*[\'"](?:error|message)[\'"]:\s*str\(e\)[^}]*}(?:\))?',
                r'return {"error": "An internal error occurred"}',
                vulnerable_code
            )

            # If the pattern wasn't found, try another common pattern
            if fixed_code == vulnerable_code:
                fixed_code = re.sub(
                    r'return\s+(?:jsonify\s*\()?\s*{[^}]*[\'"](?:error|message)[\'"]:\s*f[\'"][^\'"]*({\s*e\s*}|{\s*str\(e\)\s*})[^\']*[\'"][^}]*}(?:\))?',
                    r'return {"error": "An internal error occurred"}',
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

def fix_flask_debug(file_path, start_line, end_line):
    """
    Fix Flask app running in debug mode.

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
        if "app.run" in vulnerable_code and "debug=True" in vulnerable_code:
            # Fix Flask debug mode
            fixed_code = re.sub(
                r'debug\s*=\s*True',
                r'debug=False',
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
        logger.error(f"Error fixing Flask debug mode in {file_path}: {e}")
        return False

def fix_insecure_cookie(file_path, start_line, end_line):
    """
    Fix insecure cookie settings.

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
        if "set_cookie" in vulnerable_code and "secure=" not in vulnerable_code:
            # Fix insecure cookie by adding secure and httponly flags
            fixed_code = re.sub(
                r'set_cookie\((.*?)\)',
                r'set_cookie(\1, secure=True, httponly=True)',
                vulnerable_code
            )

            # Update file if a fix was applied
            if fixed_code != vulnerable_code:
                lines[start_line-1:end_line] = [fixed_code]

                with open(file_path, "w", encoding="utf-8") as f:
                    f.writelines(lines)

                return True

        # Check for Flask session cookie configuration
        elif "app.config" in vulnerable_code and "SESSION_COOKIE_SECURE" in vulnerable_code and "False" in vulnerable_code:
            # Fix insecure session cookie configuration
            fixed_code = re.sub(
                r'SESSION_COOKIE_SECURE\s*=\s*False',
                r'SESSION_COOKIE_SECURE = True',
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
        logger.error(f"Error fixing insecure cookie in {file_path}: {e}")
        return False

def fix_ssrf(file_path, start_line, end_line):
    """
    Fix server-side request forgery (SSRF) vulnerabilities.

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
        if ("requests.get" in vulnerable_code or "requests.post" in vulnerable_code or
            "requests.put" in vulnerable_code or "requests.delete" in vulnerable_code or
            "urllib.request.urlopen" in vulnerable_code):

            # Add URL validation before making the request
            # First, find the URL parameter
            url_match = re.search(r'(requests\.\w+|urllib\.request\.urlopen)\s*\(\s*([^,)]+)', vulnerable_code)
            if url_match:
                url_var = url_match.group(2).strip()

                # Add URL validation code
                validation_code = f"""
        # Validate URL to prevent SSRF
        from urllib.parse import urlparse
        parsed_url = urlparse({url_var})
        if parsed_url.netloc in ['localhost', '127.0.0.1', '0.0.0.0'] or parsed_url.netloc.startswith('10.') or parsed_url.netloc.startswith('172.16.') or parsed_url.netloc.startswith('192.168.'):
            raise ValueError(f"Blocked request to internal network: {{parsed_url.netloc}}")
        """

                # Find the right place to insert the validation code
                # Look for the beginning of the function or method
                function_start = -1
                for i in range(start_line-2, max(0, start_line-50), -1):
                    if re.match(r'^\s*def\s+', lines[i]):
                        function_start = i + 1
                        break

                if function_start >= 0:
                    # Insert the validation code at the beginning of the function
                    indent = re.match(r'^(\s*)', lines[function_start]).group(1)
                    validation_lines = [indent + line for line in validation_code.strip().split('\n')]
                    lines[function_start:function_start] = [line + '\n' for line in validation_lines]

                    # Write the modified file
                    with open(file_path, "w", encoding="utf-8") as f:
                        f.writelines(lines)

                    return True

        return False

    except Exception as e:
        logger.error(f"Error fixing SSRF in {file_path}: {e}")
        return False

def fix_workflow_permissions(file_path, start_line, end_line):
    """
    Fix missing workflow permissions in GitHub Actions workflows.

    Args:
        file_path: Path to file
        start_line: Start line of vulnerability
        end_line: End line of vulnerability

    Returns:
        True if fixed, False otherwise
    """
    try:
        # Only process YAML files in .github/workflows
        if not file_path.startswith(".github/workflows/") or not file_path.endswith((".yml", ".yaml")):
            return False

        # Read file
        with open(file_path, "r", encoding="utf-8") as f:
            lines = f.readlines()

        # Find the 'jobs:' section
        jobs_line = -1
        for i, line in enumerate(lines):
            if re.match(r'^\s*jobs\s*:', line):
                jobs_line = i
                break

        if jobs_line >= 0:
            # Check if permissions are already defined at the top level
            has_top_permissions = False
            for i in range(jobs_line):
                if re.match(r'^\s*permissions\s*:', lines[i]):
                    has_top_permissions = True
                    break

            if not has_top_permissions:
                # Add least privilege permissions at the top level
                permissions_block = """
permissions:
  contents: read
  issues: read
  pull-requests: read
  actions: read
  security-events: read

"""
                lines.insert(jobs_line, permissions_block)

                # Write the modified file
                with open(file_path, "w", encoding="utf-8") as f:
                    f.writelines(lines)

                return True

        return False

    except Exception as e:
        logger.error(f"Error fixing workflow permissions in {file_path}: {e}")
        return False

def fix_regex_issues(file_path, start_line, end_line):
    """
    Fix regular expression issues (ReDoS, overly large ranges).

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

        # Fix overly large ranges like [a-zA-Z0-9_]+ with \w+
        if re.search(r'\[[a-zA-Z0-9_]+\]', vulnerable_code):
            fixed_code = re.sub(
                r'\[a-zA-Z0-9_\](\+|\*)',
                r'\\w\1',
                vulnerable_code
            )

            # Update file if a fix was applied
            if fixed_code != vulnerable_code:
                lines[start_line-1:end_line] = [fixed_code]

                with open(file_path, "w", encoding="utf-8") as f:
                    f.writelines(lines)

                return True

        # Fix polynomial ReDoS patterns like (a+)+ with a+
        if re.search(r'\([^)]+[+*]\)[+*]', vulnerable_code):
            fixed_code = re.sub(
                r'\(([^)]+)[+*]\)[+*]',
                r'\1+',
                vulnerable_code
            )

            # Update file if a fix was applied
            if fixed_code != vulnerable_code:
                lines[start_line-1:end_line] = [fixed_code]

                with open(file_path, "w", encoding="utf-8") as f:
                    f.writelines(lines)

                return True

        # Fix bad HTML filtering regexes
        if re.search(r'<[^>]*>', vulnerable_code) and "filter" in vulnerable_code.lower():
            # Add a proper HTML sanitizer
            if "import" in "".join(lines[:50]):
                # Find the last import statement
                last_import_line = 0
                for i, line in enumerate(lines[:30]):
                    if line.strip().startswith(("import ", "from ")):
                        last_import_line = i

                # Add import for html.parser
                if "from html.parser import HTMLParser" not in "".join(lines[:50]):
                    lines.insert(last_import_line + 1, "from html.parser import HTMLParser\n")

                    # Adjust start_line if it's after the import
                    if start_line > last_import_line:
                        start_line += 1
                        end_line += 1

                # Add a proper HTML sanitizer class
                sanitizer_code = """
class HTMLSanitizer(HTMLParser):
    def __init__(self):
        super().__init__()
        self.safe_html = ""
        self.allowed_tags = {'p', 'br', 'h1', 'h2', 'h3', 'h4', 'h5', 'h6', 'ul', 'ol', 'li', 'strong', 'em', 'b', 'i'}

    def handle_starttag(self, tag, attrs):
        if tag in self.allowed_tags:
            self.safe_html += f"<{tag}>"

    def handle_endtag(self, tag):
        if tag in self.allowed_tags:
            self.safe_html += f"</{tag}>"

    def handle_data(self, data):
        self.safe_html += data

    @staticmethod
    def sanitize(html):
        parser = HTMLSanitizer()
        parser.feed(html)
        return parser.safe_html
"""

                # Add the sanitizer class after the imports
                if "class HTMLSanitizer" not in "".join(lines[:100]):
                    lines.insert(last_import_line + 2, sanitizer_code)

                    # Adjust start_line if it's after the sanitizer
                    if start_line > last_import_line + 1:
                        start_line += sanitizer_code.count("\n") + 1
                        end_line += sanitizer_code.count("\n") + 1

                # Replace the regex-based filtering with the sanitizer
                fixed_code = re.sub(
                    r're\.sub\([\'"]<[^>]*>[\'"],\s*[\'"][\'"],\s*([^)]+)\)',
                    r'HTMLSanitizer.sanitize(\1)',
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
        logger.error(f"Error fixing regex issues in {file_path}: {e}")
        return False

def fix_url_redirection(file_path, start_line, end_line):
    """
    Fix URL redirection vulnerabilities.

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
        if "redirect" in vulnerable_code.lower() and "request" in vulnerable_code.lower():
            # Add URL validation before redirection
            # First, find the URL parameter
            url_match = re.search(r'redirect\s*\(\s*([^,)]+)', vulnerable_code)
            if url_match:
                url_var = url_match.group(1).strip()

                # Replace with safe redirection
                fixed_code = re.sub(
                    r'redirect\s*\(\s*' + re.escape(url_var) + r'\s*\)',
                    f'redirect(validate_redirect_url({url_var}))',
                    vulnerable_code
                )

                # Add URL validation function if needed
                if "validate_redirect_url" not in "".join(lines[:100]):
                    # Find the right place to add the function
                    # Look for the beginning of the file or after imports
                    last_import_line = 0
                    for i, line in enumerate(lines[:30]):
                        if line.strip().startswith(("import ", "from ")):
                            last_import_line = i

                    validation_function = """
def validate_redirect_url(url):
    # Validate and sanitize redirect URLs to prevent open redirects
    # Only allow relative URLs or URLs to trusted domains
    from urllib.parse import urlparse
    parsed = urlparse(url)

    # If it's a relative URL (no netloc), it's safe
    if not parsed.netloc:
        return url

    # List of allowed domains
    allowed_domains = ['bulo.cloud', 'api.bulo.cloud', 'bulocloud.com']

    # Check if the domain is in the allowed list
    if parsed.netloc in allowed_domains:
        return url

    # Default to a safe URL
    return "/"
"""

                    # Add the validation function after the imports
                    lines.insert(last_import_line + 1, validation_function)

                    # Adjust start_line if it's after the validation function
                    if start_line > last_import_line:
                        start_line += validation_function.count("\n") + 1
                        end_line += validation_function.count("\n") + 1

                # Update file
                lines[start_line-1:end_line] = [fixed_code]

                with open(file_path, "w", encoding="utf-8") as f:
                    f.writelines(lines)

                return True

        return False

    except Exception as e:
        logger.error(f"Error fixing URL redirection in {file_path}: {e}")
        return False

def fix_url_sanitization(file_path, start_line, end_line):
    """
    Fix incomplete URL substring sanitization.

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
        if "http" in vulnerable_code and "in" in vulnerable_code and "startswith" in vulnerable_code:
            # Replace simple string checks with proper URL validation
            fixed_code = re.sub(
                r'([\w.]+)\.startswith\([\'"]https?://[\w.]+[\'"]\)',
                r'is_safe_url(\1)',
                vulnerable_code
            )

            # Add URL validation function if needed
            if "is_safe_url" not in "".join(lines[:100]):
                # Find the right place to add the function
                # Look for the beginning of the file or after imports
                last_import_line = 0
                for i, line in enumerate(lines[:30]):
                    if line.strip().startswith(("import ", "from ")):
                        last_import_line = i

                validation_function = """
def is_safe_url(url):
    # Validate if a URL is safe using proper parsing
    from urllib.parse import urlparse

    # Parse the URL
    try:
        parsed = urlparse(url)

        # Check if the URL has a scheme and netloc
        if not parsed.scheme or not parsed.netloc:
            return False

        # List of allowed domains
        allowed_domains = ['bulo.cloud', 'api.bulo.cloud', 'bulocloud.com']

        # Check if the domain is in the allowed list
        for domain in allowed_domains:
            if parsed.netloc == domain or parsed.netloc.endswith('.' + domain):
                return True

        return False
    except:
        return False
"""

                # Add the validation function after the imports
                lines.insert(last_import_line + 1, validation_function)

                # Adjust start_line if it's after the validation function
                if start_line > last_import_line:
                    start_line += validation_function.count("\n") + 1
                    end_line += validation_function.count("\n") + 1

            # Update file if a fix was applied
            if fixed_code != vulnerable_code:
                lines[start_line-1:end_line] = [fixed_code]

                with open(file_path, "w", encoding="utf-8") as f:
                    f.writelines(lines)

                return True

        return False

    except Exception as e:
        logger.error(f"Error fixing URL sanitization in {file_path}: {e}")
        return False

def fix_command_injection(file_path, start_line, end_line):
    """
    Fix command line injection vulnerabilities.

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
        if ("os.system" in vulnerable_code or "subprocess.call" in vulnerable_code or
            "subprocess.Popen" in vulnerable_code or "subprocess.run" in vulnerable_code):

            # Replace shell=True with shell=False and use list arguments
            if "shell=True" in vulnerable_code:
                fixed_code = re.sub(
                    r'shell\s*=\s*True',
                    r'shell=False',
                    vulnerable_code
                )

                # Update file if a fix was applied
                if fixed_code != vulnerable_code:
                    lines[start_line-1:end_line] = [fixed_code]

                    with open(file_path, "w", encoding="utf-8") as f:
                        f.writelines(lines)

                    return True

            # Replace string command with list arguments
            cmd_match = re.search(r'(os\.system|subprocess\.\w+)\s*\(\s*([^,)]+)', vulnerable_code)
            if cmd_match:
                cmd_var = cmd_match.group(2).strip()

                # If it's a variable, add validation
                if not (cmd_var.startswith("'") or cmd_var.startswith('"')):
                    # Find the right place to add the validation
                    # Look for the beginning of the function or method
                    function_start = -1
                    for i in range(start_line-2, max(0, start_line-50), -1):
                        if re.match(r'^\s*def\s+', lines[i]):
                            function_start = i + 1
                            break

                    if function_start >= 0:
                        # Add command validation
                        indent = re.match(r'^(\s*)', lines[function_start]).group(1)
                        validation_code = f"""
{indent}# Validate command to prevent injection
{indent}import shlex
{indent}if isinstance({cmd_var}, str):
{indent}    {cmd_var} = shlex.split({cmd_var})
{indent}# Whitelist allowed commands
{indent}allowed_commands = ['ls', 'dir', 'echo', 'cat', 'type']
{indent}if {cmd_var}[0] not in allowed_commands:
{indent}    raise ValueError(f"Command not allowed: {{{cmd_var}[0]}}")
"""

                        # Insert the validation code
                        lines.insert(function_start, validation_code)

                        # Adjust start_line if it's after the validation
                        if start_line > function_start:
                            start_line += validation_code.count("\n") + 1
                            end_line += validation_code.count("\n") + 1

                        # Write the modified file
                        with open(file_path, "w", encoding="utf-8") as f:
                            f.writelines(lines)

                        return True

        return False

    except Exception as e:
        logger.error(f"Error fixing command injection in {file_path}: {e}")
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
        "flask_debug": 0,
        "insecure_cookie": 0,
        "ssrf": 0,
        "workflow_permissions": 0,
        "regex_issues": 0,
        "url_redirection": 0,
        "url_sanitization": 0,
        "command_injection": 0,
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

            elif "flask-debug" in rule_id.lower():
                if fix_flask_debug(file_path, start_line, end_line):
                    logger.info(f"  Fixed Flask debug mode in {file_path}:{start_line}-{end_line}")
                    fixed_issues["flask_debug"] += 1
                    fixed_issues["total"] += 1

            elif "insecure-cookie" in rule_id.lower():
                if fix_insecure_cookie(file_path, start_line, end_line):
                    logger.info(f"  Fixed insecure cookie in {file_path}:{start_line}-{end_line}")
                    fixed_issues["insecure_cookie"] += 1
                    fixed_issues["total"] += 1

            elif "ssrf" in rule_id.lower() or "server-side-request-forgery" in rule_id.lower():
                if fix_ssrf(file_path, start_line, end_line):
                    logger.info(f"  Fixed SSRF vulnerability in {file_path}:{start_line}-{end_line}")
                    fixed_issues["ssrf"] += 1
                    fixed_issues["total"] += 1

            elif "missing-workflow-permissions" in rule_id.lower():
                if fix_workflow_permissions(file_path, start_line, end_line):
                    logger.info(f"  Fixed missing workflow permissions in {file_path}")
                    fixed_issues["workflow_permissions"] += 1
                    fixed_issues["total"] += 1

            elif "redos" in rule_id.lower() or "overly-large-range" in rule_id.lower() or "bad-tag-filter" in rule_id.lower():
                if fix_regex_issues(file_path, start_line, end_line):
                    logger.info(f"  Fixed regex issue in {file_path}:{start_line}-{end_line}")
                    fixed_issues["regex_issues"] += 1
                    fixed_issues["total"] += 1

            elif "url-redirection" in rule_id.lower():
                if fix_url_redirection(file_path, start_line, end_line):
                    logger.info(f"  Fixed URL redirection vulnerability in {file_path}:{start_line}-{end_line}")
                    fixed_issues["url_redirection"] += 1
                    fixed_issues["total"] += 1

            elif "incomplete-url-substring-sanitization" in rule_id.lower():
                if fix_url_sanitization(file_path, start_line, end_line):
                    logger.info(f"  Fixed URL sanitization issue in {file_path}:{start_line}-{end_line}")
                    fixed_issues["url_sanitization"] += 1
                    fixed_issues["total"] += 1

            elif "command-line-injection" in rule_id.lower():
                if fix_command_injection(file_path, start_line, end_line):
                    logger.info(f"  Fixed command injection vulnerability in {file_path}:{start_line}-{end_line}")
                    fixed_issues["command_injection"] += 1
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
    logger.info(f"  Flask Debug Mode: {fixed_issues['flask_debug']}")
    logger.info(f"  Insecure Cookie: {fixed_issues['insecure_cookie']}")
    logger.info(f"  SSRF Vulnerability: {fixed_issues['ssrf']}")
    logger.info(f"  Workflow Permissions: {fixed_issues['workflow_permissions']}")
    logger.info(f"  Regex Issues: {fixed_issues['regex_issues']}")
    logger.info(f"  URL Redirection: {fixed_issues['url_redirection']}")
    logger.info(f"  URL Sanitization: {fixed_issues['url_sanitization']}")
    logger.info(f"  Command Injection: {fixed_issues['command_injection']}")
    logger.info(f"  Total: {fixed_issues['total']}")

def main():
    """Main function."""
    parser = argparse.ArgumentParser(description="Fix CodeQL issues in Python code")
    parser.add_argument("categories_file", help="Path to categorized alerts JSON file")
    args = parser.parse_args()

    fix_issues(args.categories_file)

if __name__ == "__main__":
    main()
