"""
Bulo.Cloud Sentinel Security Module.

This module provides comprehensive security features for the Bulo.Cloud Sentinel platform.
"""

__version__ = "0.1.0"

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

from .api.rate_limiter import (
    rate_limit,
    auth_rate_limit,
    sensitive_rate_limit,
    admin_rate_limit,
    RateLimitExceeded,
    RateLimiterConfig,
    configure_rate_limiter,
)

from .api.input_validation import (
    validate_email,
    validate_username,
    validate_name,
    validate_url,
    validate_phone,
    validate_alphanumeric,
    validate_numeric,
    validate_alpha,
    sanitize_html,
    validate_and_sanitize,
    is_sql_injection,
    prevent_sql_injection,
    is_nosql_injection,
    prevent_nosql_injection,
    is_command_injection,
    prevent_command_injection,
)

from .logging.audit_logger import (
    log_audit_event,
    log_auth_success,
    log_auth_failure,
    log_logout,
    log_password_change,
    log_password_reset,
    log_user_create,
    log_user_update,
    log_user_delete,
    log_role_change,
    log_data_access,
    log_security_alert,
    log_api_access,
    EventType,
    SeverityLevel,
)

from .monitoring.security_monitor import (
    security_monitor,
    SecurityAlert,
    SecurityRule,
    AlertType,
)

from .network.tls import (
    TLSConfig,
    generate_self_signed_cert,
    check_cert_expiration,
    get_cert_fingerprint,
)
