"""
Bulo.Cloud Sentinel Security Module.

This module provides comprehensive security features for the Bulo.Cloud Sentinel platform.
"""

__version__ = "0.3.0"

# Authentication and authorization
from .auth.jwt_handler import (
    create_access_token,
    create_refresh_token,
    create_token_response,
    decode_token,
    get_current_token_data,
    get_current_user_id,
    has_role,
    has_permission,
)

from .auth.password import (
    hash_password,
    verify_password,
    check_password_needs_rehash,
    validate_password,
    generate_password,
)

# Encryption and key management
from .encryption.encryption import (
    generate_aes_key,
    aes_encrypt,
    aes_decrypt,
    generate_fernet_key,
    derive_key_from_password,
    fernet_encrypt,
    fernet_decrypt,
    generate_rsa_key_pair,
    rsa_encrypt,
    rsa_decrypt,
    hybrid_encrypt,
    hybrid_decrypt,
)

from .encryption.key_management import (
    KeyStore,
    KeyRotationManager,
    create_key_store,
    generate_key_pair,
)

# API security
from .api.rate_limiting import (
    RateLimiter,
    RateLimitMiddleware,
    rate_limit,
)

from .api.cors import (
    configure_cors,
    configure_secure_cors,
)

from .api.security_headers import (
    SecurityHeadersMiddleware,
    add_security_headers,
    get_csp_header,
    get_permissions_policy_header,
)

from .api.csrf_protection import (
    CSRFMiddleware,
    add_csrf_protection,
    csrf_protect,
    get_csrf_token,
    set_csrf_token,
)

from .api.unified_security import (
    configure_security,
)

# Error handling
from .error_handling.secure_error_handler import (
    ErrorType,
    ErrorResponse,
    create_error_response,
    configure_error_handlers,
    configure_custom_exception_handlers,
    SecurityException,
    AuthenticationException,
    AuthorizationException,
    RateLimitException,
    ValidationException,
)

# Input validation
from .validation.input_validation import (
    validate_email,
    validate_username,
    validate_name,
    validate_uuid,
    validate_coordinates,
    validate_phone,
    validate_url,
    validate_filename,
    validate_path,
    sanitize_string,
    sanitize_html,
    check_sql_injection,
    validate_input,
)

from .validation.sql_validation import (
    validate_sql_identifier,
    safe_table_name,
    safe_column_name,
    safe_order_direction,
    build_safe_select_query,
    execute_safe_query,
)

from .validation.html_validation import (
    strip_all_tags,
    sanitize_html,
    detect_xss,
    validate_html_content,
)

# Logging and error handling
from .logging.secure_logging import (
    SecureLogger,
    AuditLogger,
    get_secure_logger,
    get_audit_logger,
)

from .logging.error_handling import (
    ErrorHandler,
    exception_handler,
    async_exception_handler,
)

from .logging.security_logger import (
    SecurityLogger,
    SecurityEventType,
    SecuritySeverity,
    security_logger,
)

# Security monitoring
from .monitoring.anomaly_detection import (
    AnomalyType,
    AnomalyDetector,
    StatisticalAnomalyDetector,
    RateAnomalyDetector,
    PatternAnomalyDetector,
    AnomalyDetectionManager,
    anomaly_detection_manager,
)

# Security monitoring
from .monitoring.anomaly_detection import (
    AnomalyType,
    AnomalyDetector,
    StatisticalAnomalyDetector,
    RateAnomalyDetector,
    PatternAnomalyDetector,
    AnomalyDetectionManager,
    anomaly_detection_manager,
)