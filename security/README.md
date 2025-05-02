# 🔒 Bulo.Cloud Sentinel Security Module

The Security Module provides comprehensive security features for the Bulo.Cloud Sentinel platform, ensuring data protection, secure communications, and protection against common vulnerabilities.

## 🚀 Features

- **🔐 Authentication & Authorization**: Advanced authentication mechanisms with JWT, OAuth2, and MFA
- **🛡️ API Security**: Protection against common API vulnerabilities (OWASP Top 10)
- **🔒 Data Encryption**: End-to-end encryption for sensitive data
- **🔍 Security Monitoring**: Real-time monitoring and alerting for security events
- **🧪 Vulnerability Scanning**: Automated scanning for known vulnerabilities
- **📝 Audit Logging**: Comprehensive logging of security-relevant events
- **🚫 Rate Limiting**: Protection against brute force and DoS attacks
- **🔄 Secure Communications**: TLS/SSL implementation for all communications

## 🏗️ Architecture

The Security Module is designed as a cross-cutting concern that integrates with all components of the Bulo.Cloud Sentinel platform:

```
security/
├── auth/                  # Authentication and authorization
│   ├── jwt_handler.py     # JWT token handling
│   ├── oauth2.py          # OAuth2 integration
│   ├── mfa.py             # Multi-factor authentication
│   └── rbac.py            # Role-based access control
├── encryption/            # Data encryption utilities
│   ├── aes.py             # AES encryption
│   ├── rsa.py             # RSA encryption
│   └── key_management.py  # Key management
├── monitoring/            # Security monitoring
│   ├── event_monitor.py   # Security event monitoring
│   ├── alerts.py          # Security alerts
│   └── dashboard.py       # Security dashboard
├── scanning/              # Vulnerability scanning
│   ├── dependency_scan.py # Dependency vulnerability scanning
│   ├── code_scan.py       # Code vulnerability scanning
│   └── network_scan.py    # Network vulnerability scanning
├── logging/               # Audit logging
│   ├── audit_logger.py    # Audit logging
│   ├── log_analyzer.py    # Log analysis
│   └── log_storage.py     # Secure log storage
├── api/                   # API security
│   ├── rate_limiter.py    # API rate limiting
│   ├── input_validation.py # Input validation
│   └── csrf_protection.py # CSRF protection
├── network/               # Network security
│   ├── tls.py             # TLS/SSL configuration
│   ├── firewall.py        # Application firewall
│   └── proxy.py           # Secure proxy
└── utils/                 # Security utilities
    ├── password_policy.py # Password policy enforcement
    ├── secrets.py         # Secure secrets management
    └── random.py          # Cryptographically secure random
```

## 🔄 Integration with Bulo.Cloud Sentinel

The Security Module integrates with all components of the Bulo.Cloud Sentinel platform:

1. **Backend Services**: Authentication, authorization, and API security
2. **Frontend Applications**: CSRF protection, secure communications
3. **Database**: Data encryption, secure connections
4. **Microservices**: Service-to-service authentication, secure communications
5. **APIs**: Rate limiting, input validation, authentication

## 🛠️ Implementation

The Security Module is implemented using industry-standard security libraries and best practices:

- **Authentication**: JWT with short expiration, refresh tokens, and secure storage
- **Encryption**: AES-256 for symmetric encryption, RSA-2048 for asymmetric encryption
- **Hashing**: Argon2id for password hashing
- **TLS/SSL**: TLS 1.3 with strong cipher suites
- **API Security**: Input validation, output encoding, and proper error handling
- **Logging**: Structured logging with sensitive data masking

## 🔄 Recent Security Updates

The following security updates have been implemented:

1. **Replaced python-jose with PyJWT**: Removed python-jose due to vulnerabilities (CVE-2024-33664 and CVE-2024-33663) and replaced it with PyJWT.
2. **Standardized authentication**: Implemented a unified authentication module using Argon2id for password hashing.
3. **Fixed dependency vulnerabilities**: Updated vulnerable dependencies to secure versions.
4. **Implemented comprehensive input validation**: Created a unified input validation module.

## 🛠️ Using the Unified Security Modules

### Authentication

Use the unified authentication module for all authentication needs:

```python
from security.auth.unified_auth import (
    hash_password,
    verify_password,
    create_access_token,
    create_refresh_token,
    decode_token,
    get_current_token_data,
    get_current_user_id,
    has_role,
    has_permission,
    require_role,
    require_permission,
)

# Hash a password
hashed_password = hash_password("secure_password")

# Verify a password
is_valid = verify_password(hashed_password, "secure_password")

# Create a JWT token
token = create_access_token(
    subject="user_id",
    roles=["admin"],
    permissions=["read", "write"]
)

# Protect an endpoint with role-based access control
@app.get("/admin")
async def admin_endpoint(token_data = Depends(require_role("admin"))):
    return {"message": "Admin access granted"}
```

### Input Validation

Use the unified validation module for all input validation needs:

```python
from security.validation.unified_validation import (
    validate_email,
    validate_username,
    validate_name,
    validate_uuid,
    validate_url,
    sanitize_string,
    sanitize_html,
    check_sql_injection,
    input_validator,
    form_validator,
    request_validator,
)

# Validate an email
is_valid = validate_email("user@example.com")

# Sanitize HTML
safe_html = sanitize_html("<script>alert('XSS')</script>")

# Validate a form
validation_schema = {
    "username": {"type": "string", "min_length": 3, "max_length": 32},
    "email": {"type": "email", "required": True},
    "age": {"type": "integer", "min_value": 18, "max_value": 120},
}

try:
    validated_data = form_validator.validate_form(form_data, validation_schema)
except HTTPException as e:
    # Handle validation error
    pass
```

## 📋 Security Best Practices

The Security Module enforces the following security best practices:

1. **Defense in Depth**: Multiple layers of security controls
2. **Principle of Least Privilege**: Minimal access rights for users and services
3. **Secure by Default**: Secure default configurations
4. **Fail Secure**: Systems fail in a secure state
5. **Keep It Simple**: Simple security designs are easier to verify
6. **Regular Updates**: Keeping dependencies up-to-date
7. **Security Testing**: Regular security testing and code reviews
8. **Always use the unified modules**: Use the unified authentication and validation modules instead of implementing your own
9. **Validate all inputs**: Validate all user inputs using the unified validation module
10. **Implement proper error handling**: Don't expose sensitive information in error messages

## 🚀 Running Security Updates

To run all security updates, use the provided script:

```bash
python scripts/run_security_updates.py
```

This will:
1. Update all dependencies to secure versions
2. Update all authentication implementations to use the unified module
3. Update all input validation implementations to use the unified module
