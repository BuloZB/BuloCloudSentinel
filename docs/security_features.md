# Bulo.Cloud Sentinel Security Features

This document provides comprehensive documentation for the security features implemented in the Bulo.Cloud Sentinel system.

## Table of Contents

1. [Authentication](#authentication)
2. [Authorization](#authorization)
3. [Token Management](#token-management)
4. [Password Security](#password-security)
5. [Input Validation](#input-validation)
6. [CSRF Protection](#csrf-protection)
7. [Content Security Policy](#content-security-policy)
8. [Rate Limiting](#rate-limiting)
9. [Secure Cookies](#secure-cookies)
10. [Error Handling](#error-handling)
11. [Logging](#logging)
12. [Subresource Integrity](#subresource-integrity)

## Authentication

### JWT Authentication

The system uses JSON Web Tokens (JWT) for authentication. JWTs are signed using a secure algorithm (HS256) and include the following claims:

- `sub`: Subject (user ID)
- `exp`: Expiration time
- `iat`: Issued at time
- `jti`: JWT ID (unique identifier for the token)
- `type`: Token type (access or refresh)
- `roles`: User roles
- `permissions`: User permissions

### Token Types

The system uses two types of tokens:

1. **Access Tokens**: Short-lived tokens (30 minutes) used for API access
2. **Refresh Tokens**: Long-lived tokens (7 days) used to obtain new access tokens

### Implementation

The JWT authentication is implemented in the following files:

- `security/auth/jwt_handler.py`: Core JWT functionality
- `backend/api/dependencies.py`: FastAPI dependencies for JWT validation
- `backend/api/login.py`: Login and token refresh endpoints

### Usage Example

```python
# Protect an endpoint with JWT authentication
@router.get("/protected")
async def protected_endpoint(token_data: TokenData = Depends(verify_jwt_token)):
    return {"message": f"Hello, {token_data.username}!"}
```

## Authorization

### Role-Based Access Control

The system implements role-based access control (RBAC) to restrict access to resources based on user roles.

### Permission-Based Access Control

In addition to roles, the system supports fine-grained permission-based access control.

### Implementation

The authorization system is implemented in the following files:

- `security/auth/jwt_handler.py`: Role and permission checking functions
- `backend/api/dependencies.py`: FastAPI dependencies for role and permission validation

### Usage Example

```python
# Require admin role for an endpoint
@router.get("/admin")
async def admin_endpoint(token_data: TokenData = Depends(require_role("admin"))):
    return {"message": "Admin access granted"}

# Require specific permission for an endpoint
@router.get("/users")
async def users_endpoint(token_data: TokenData = Depends(require_permission("read:users"))):
    return {"message": "User list access granted"}
```

## Token Management

### Token Blacklisting

The system implements token blacklisting to invalidate tokens before their expiration time. This is used for logout and security incident response.

### Redis-Backed Persistence

Token blacklisting is implemented using Redis for persistence, with a fallback to in-memory storage if Redis is unavailable.

### Implementation

The token blacklisting system is implemented in the following files:

- `security/auth/token_blacklist.py`: Token blacklist implementation
- `security/auth/memory_blacklist.py`: Fallback in-memory implementation

### Usage Example

```python
# Blacklist a token
from security.auth.token_blacklist import blacklist_token

blacklist_token(jti, exp)

# Check if a token is blacklisted
from security.auth.token_blacklist import is_token_blacklisted

if is_token_blacklisted(jti):
    # Token is blacklisted
    pass
```

## Password Security

### Argon2id Hashing

The system uses Argon2id for password hashing, which is the recommended algorithm for password hashing according to OWASP.

### Secure Parameters

The Argon2id implementation uses secure parameters:

- Time cost: 3 iterations
- Memory cost: 64 MB
- Parallelism: 4 threads
- Hash length: 32 bytes
- Salt length: 16 bytes

### Automatic Rehashing

The system automatically rehashes passwords using the current algorithm and parameters when a user logs in with a password hashed using an older algorithm or parameters.

### Implementation

The password security system is implemented in the following files:

- `backend/application/services/auth_service.py`: Password hashing and verification
- `security/auth/password.py`: Password validation and strength checking

### Usage Example

```python
# Hash a password
from backend.application.services.auth_service import AuthService

auth_service = AuthService()
hashed_password = auth_service.hash_password("secure_password")

# Verify a password
is_valid = auth_service.verify_password("secure_password", hashed_password)
```

## Input Validation

### SQL Injection Protection

The system implements SQL injection protection by validating input against common SQL injection patterns.

### Command Injection Protection

The system implements command injection protection by validating input against common command injection patterns.

### Path Traversal Protection

The system implements path traversal protection by validating input against common path traversal patterns.

### XSS Protection

The system implements XSS protection by sanitizing HTML input.

### Implementation

The input validation system is implemented in the following files:

- `security/utils/input_sanitizer.py`: Input sanitization functions
- `security/middleware/input_sanitization.py`: Input sanitization middleware
- `security/middleware/sql_injection_protection.py`: SQL injection protection middleware
- `security/middleware/path_traversal_protection.py`: Path traversal protection middleware
- `security/middleware/xss_protection.py`: XSS protection middleware

### Usage Example

```python
# Sanitize HTML input
from security.utils.input_sanitizer import sanitize_html_input

safe_html = sanitize_html_input(user_input)

# Check for SQL injection
from security.utils.input_sanitizer import is_sql_injection

if is_sql_injection(user_input):
    # Input contains SQL injection
    pass
```

## CSRF Protection

### Double Submit Cookie Pattern

The system implements the Double Submit Cookie pattern for CSRF protection for regular web routes.

### Token-Based CSRF Protection for API Routes

The system implements token-based CSRF protection for API routes, which is more suitable for API clients.

### Implementation

The CSRF protection system is implemented in the following files:

- `security/middleware/csrf.py`: CSRF middleware for regular web routes
- `security/middleware/api_csrf.py`: CSRF middleware for API routes

### Usage Example

```html
<!-- Include CSRF token in forms -->
<form method="POST" action="/api/submit">
    <input type="hidden" name="csrf_token" value="{{ csrf_token }}">
    <!-- Form fields -->
    <button type="submit">Submit</button>
</form>

<!-- Include CSRF token in AJAX requests -->
<script>
    fetch('/api/data', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json',
            'X-CSRF-Token': '{{ csrf_token }}'
        },
        body: JSON.stringify({ data: 'example' })
    });
</script>
```

## Content Security Policy

### CSP Configuration

The system implements a Content Security Policy (CSP) to prevent XSS and other code injection attacks.

### CSP Reporting

The system implements CSP reporting to collect violation reports for security monitoring.

### Implementation

The CSP system is implemented in the following files:

- `security/middleware/csp.py`: CSP middleware
- `security/api/csp_report.py`: CSP reporting endpoints

### Usage Example

```python
# Configure CSP
from security.middleware.csp import ContentSecurityPolicyMiddleware, get_default_csp_policy

csp_policy = get_default_csp_policy()
app.add_middleware(
    ContentSecurityPolicyMiddleware,
    policy=csp_policy,
    report_uri="/api/security/csp-report"
)
```

## Rate Limiting

### IP-Based Rate Limiting

The system implements IP-based rate limiting to prevent brute force attacks and abuse.

### Progressive Rate Limiting

The system implements progressive rate limiting for authentication endpoints, with increasing delays for repeated failures.

### Implementation

The rate limiting system is implemented in the following files:

- `security/api/rate_limiter.py`: Rate limiting implementation
- `security/middleware/rate_limiting.py`: Rate limiting middleware

### Usage Example

```python
# Apply rate limiting to an endpoint
from security.api.rate_limiter import rate_limit

@router.post("/login")
@rate_limit("10/minute")
async def login(request: Request):
    # Login logic
    pass

# Apply authentication rate limiting
from security.api.rate_limiter import auth_rate_limit

@router.post("/login")
@auth_rate_limit()
async def login(request: Request):
    # Login logic
    pass
```

## Secure Cookies

### Secure Cookie Configuration

The system implements secure cookie configuration with the following flags:

- `Secure`: Cookies are only sent over HTTPS
- `HttpOnly`: Cookies are not accessible via JavaScript
- `SameSite`: Cookies are only sent in same-site requests
- `Max-Age`: Cookies have a limited lifetime

### Implementation

The secure cookie system is implemented in the following files:

- `security/auth/cookie_handler.py`: Secure cookie handling

### Usage Example

```python
# Set secure cookies
from security.auth.cookie_handler import set_jwt_cookies

response = Response(content={"message": "Success"})
set_jwt_cookies(response, access_token, refresh_token)

# Unset secure cookies
from security.auth.cookie_handler import unset_jwt_cookies

response = Response(content={"message": "Logged out"})
unset_jwt_cookies(response)
```

## Error Handling

### Secure Error Responses

The system implements secure error responses that do not leak sensitive information.

### Error Reference IDs

The system generates unique error reference IDs for tracking errors without exposing sensitive information.

### Implementation

The error handling system is implemented in the following files:

- `security/logging/error_handling.py`: Secure error handling

### Usage Example

```python
# Handle errors securely
from security.logging.error_handling import ErrorHandler

error_handler = ErrorHandler()
app.add_exception_handler(Exception, error_handler.generic_exception_handler)
```

## Logging

### Secure Logging

The system implements secure logging that sanitizes sensitive information.

### Structured Logging

The system uses structured logging for better analysis and monitoring.

### Implementation

The logging system is implemented in the following files:

- `security/logging/secure_logging.py`: Secure logging implementation

### Usage Example

```python
# Use secure logging
from security.logging.secure_logging import get_secure_logger

logger = get_secure_logger("auth")
logger.info(
    "User logged in",
    {
        "username": user.username,
        "user_id": str(user.id),
        "ip_address": client_ip
    }
)
```

## Subresource Integrity

### SRI Implementation

The system implements Subresource Integrity (SRI) for external resources to prevent unauthorized modifications.

### SRI Verification

The system verifies SRI hashes for external resources to ensure integrity.

### Implementation

The SRI system is implemented in the following files:

- `security/utils/sri_helper.py`: SRI implementation

### Usage Example

```python
# Generate SRI hash for content
from security.utils.sri_helper import generate_sri_hash

content = "console.log('Hello, world!');"
sri_hash = generate_sri_hash(content)

# Add SRI attributes to HTML
from security.utils.sri_helper import add_sri_to_html

html = """
<!DOCTYPE html>
<html>
<head>
    <script src="https://example.com/script.js"></script>
</head>
<body>
    <h1>Hello, world!</h1>
</body>
</html>
"""

resources = {
    "https://example.com/script.js": "sha384-oqVuAfXRKap7fdgcCY5uykM6+R9GqQ8K/uxy9rx7HNQlGYl1kPzQho1wx4JwY8wC"
}

html_with_sri = add_sri_to_html(html, resources)