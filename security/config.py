"""
Security configuration for Bulo.Cloud Sentinel.

This module provides centralized security configuration settings for the application.
"""

import os
import secrets
import logging
from typing import Dict, List, Optional, Set, Union

# Set up logging
log = logging.getLogger(__name__)

# JWT Configuration
JWT_SECRET_KEY = os.getenv("JWT_SECRET_KEY", "")
JWT_ALGORITHM = "HS256"
JWT_ACCESS_TOKEN_EXPIRE_MINUTES = int(os.getenv("JWT_ACCESS_TOKEN_EXPIRE_MINUTES", "30"))
JWT_REFRESH_TOKEN_EXPIRE_DAYS = int(os.getenv("JWT_REFRESH_TOKEN_EXPIRE_DAYS", "7"))
JWT_ISSUER = os.getenv("JWT_ISSUER", "bulo.cloud.sentinel")
JWT_AUDIENCE = os.getenv("JWT_AUDIENCE", "bulo.cloud.sentinel.api")
JWT_CLOCK_SKEW_SECONDS = 30  # Allow for small clock differences

# Generate a secure random key if none is provided
if not JWT_SECRET_KEY:
    JWT_SECRET_KEY = secrets.token_hex(32)
    log.warning("Generated random JWT secret key. This will change on restart!")

# Password Policy
PASSWORD_MIN_LENGTH = int(os.getenv("PASSWORD_MIN_LENGTH", "12"))
PASSWORD_REQUIRE_UPPERCASE = os.getenv("PASSWORD_REQUIRE_UPPERCASE", "true").lower() == "true"
PASSWORD_REQUIRE_LOWERCASE = os.getenv("PASSWORD_REQUIRE_LOWERCASE", "true").lower() == "true"
PASSWORD_REQUIRE_DIGITS = os.getenv("PASSWORD_REQUIRE_DIGITS", "true").lower() == "true"
PASSWORD_REQUIRE_SPECIAL = os.getenv("PASSWORD_REQUIRE_SPECIAL", "true").lower() == "true"
PASSWORD_MAX_LENGTH = int(os.getenv("PASSWORD_MAX_LENGTH", "128"))
PASSWORD_HISTORY_SIZE = int(os.getenv("PASSWORD_HISTORY_SIZE", "5"))

# CORS Configuration
CORS_ALLOW_ORIGINS = os.getenv("CORS_ALLOW_ORIGINS", "*").split(",")
CORS_ALLOW_METHODS = os.getenv("CORS_ALLOW_METHODS", "GET,POST,PUT,DELETE,OPTIONS").split(",")
CORS_ALLOW_HEADERS = os.getenv("CORS_ALLOW_HEADERS", "Authorization,Content-Type,Accept,Origin,User-Agent").split(",")
CORS_ALLOW_CREDENTIALS = os.getenv("CORS_ALLOW_CREDENTIALS", "true").lower() == "true"
CORS_MAX_AGE = int(os.getenv("CORS_MAX_AGE", "3600"))

# Rate Limiting
RATE_LIMIT_DEFAULT = int(os.getenv("RATE_LIMIT_DEFAULT", "100"))  # requests per minute
RATE_LIMIT_AUTH = int(os.getenv("RATE_LIMIT_AUTH", "10"))  # requests per minute for auth endpoints
RATE_LIMIT_WINDOW = int(os.getenv("RATE_LIMIT_WINDOW", "60"))  # seconds

# Security Headers
SECURITY_HEADERS_CSP = os.getenv(
    "SECURITY_HEADERS_CSP",
    "default-src 'self'; script-src 'self' 'unsafe-inline'; style-src 'self' 'unsafe-inline'; "
    "img-src 'self' data:; font-src 'self'; connect-src 'self'; frame-ancestors 'none'; "
    "form-action 'self'; base-uri 'self'; object-src 'none'"
)
SECURITY_HEADERS_HSTS = os.getenv(
    "SECURITY_HEADERS_HSTS",
    "max-age=31536000; includeSubDomains; preload"
)
SECURITY_HEADERS_PERMISSIONS_POLICY = os.getenv(
    "SECURITY_HEADERS_PERMISSIONS_POLICY",
    "accelerometer=(), camera=(), geolocation=(), gyroscope=(), magnetometer=(), microphone=(), payment=(), usb=()"
)

# File Upload
FILE_UPLOAD_MAX_SIZE = int(os.getenv("FILE_UPLOAD_MAX_SIZE", str(10 * 1024 * 1024)))  # 10 MB
FILE_UPLOAD_ALLOWED_EXTENSIONS = set(os.getenv(
    "FILE_UPLOAD_ALLOWED_EXTENSIONS",
    "jpg,jpeg,png,gif,pdf,doc,docx,xls,xlsx,ppt,pptx,txt,csv,zip,tar,gz"
).split(","))
FILE_UPLOAD_VALIDATE_CONTENT = os.getenv("FILE_UPLOAD_VALIDATE_CONTENT", "true").lower() == "true"

# Redis Configuration for Token Blacklist
REDIS_URL = os.getenv("REDIS_URL", "redis://localhost:6379/0")
TOKEN_BLACKLIST_CLEANUP_INTERVAL = int(os.getenv("TOKEN_BLACKLIST_CLEANUP_INTERVAL", "3600"))  # 1 hour

# Session Configuration
SESSION_COOKIE_NAME = os.getenv("SESSION_COOKIE_NAME", "bulo_session")
SESSION_COOKIE_SECURE = os.getenv("SESSION_COOKIE_SECURE", "true").lower() == "true"
SESSION_COOKIE_HTTPONLY = os.getenv("SESSION_COOKIE_HTTPONLY", "true").lower() == "true"
SESSION_COOKIE_SAMESITE = os.getenv("SESSION_COOKIE_SAMESITE", "strict")
SESSION_COOKIE_MAX_AGE = int(os.getenv("SESSION_COOKIE_MAX_AGE", str(60 * 60)))  # 1 hour
SESSION_SECRET_KEY = os.getenv("SESSION_SECRET_KEY", "")

# Generate a secure random key if none is provided
if not SESSION_SECRET_KEY:
    SESSION_SECRET_KEY = secrets.token_hex(32)
    log.warning("Generated random session secret key. This will change on restart!")

# Authentication Configuration
AUTH_ENABLE_SIGNUP = os.getenv("AUTH_ENABLE_SIGNUP", "true").lower() == "true"
AUTH_ENABLE_EMAIL_VERIFICATION = os.getenv("AUTH_ENABLE_EMAIL_VERIFICATION", "true").lower() == "true"
AUTH_ENABLE_PASSWORD_RESET = os.getenv("AUTH_ENABLE_PASSWORD_RESET", "true").lower() == "true"
AUTH_ENABLE_MFA = os.getenv("AUTH_ENABLE_MFA", "true").lower() == "true"
AUTH_MAX_LOGIN_ATTEMPTS = int(os.getenv("AUTH_MAX_LOGIN_ATTEMPTS", "5"))
AUTH_LOCKOUT_DURATION = int(os.getenv("AUTH_LOCKOUT_DURATION", "300"))  # 5 minutes

# Default Roles and Permissions
DEFAULT_ROLES = {
    "admin": [
        "users:read", "users:write", "users:delete",
        "roles:read", "roles:write", "roles:delete",
        "permissions:read", "permissions:write", "permissions:delete",
        "settings:read", "settings:write",
        "logs:read",
        "drones:read", "drones:write", "drones:delete", "drones:control",
        "missions:read", "missions:write", "missions:delete", "missions:execute",
        "analytics:read", "analytics:write",
    ],
    "operator": [
        "users:read",
        "drones:read", "drones:write", "drones:control",
        "missions:read", "missions:write", "missions:execute",
        "analytics:read",
    ],
    "observer": [
        "drones:read",
        "missions:read",
        "analytics:read",
    ],
}

# Log sensitive actions
def log_security_event(event_type: str, user_id: Optional[str], details: Dict[str, any]):
    """
    Log a security event.
    
    Args:
        event_type: Type of security event
        user_id: User ID (if available)
        details: Additional details about the event
    """
    log.warning(
        f"SECURITY EVENT: {event_type} | User: {user_id or 'anonymous'} | "
        f"Details: {details}"
    )
