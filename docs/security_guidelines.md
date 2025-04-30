# Security Guidelines for Developers

This document provides security guidelines for developers working on the Bulo.Cloud Sentinel project.

## Table of Contents

1. [Authentication and Authorization](#authentication-and-authorization)
2. [Input Validation](#input-validation)
3. [Database Security](#database-security)
4. [API Security](#api-security)
5. [Cryptography](#cryptography)
6. [Error Handling and Logging](#error-handling-and-logging)
7. [File Operations](#file-operations)
8. [Dependency Management](#dependency-management)
9. [Frontend Security](#frontend-security)
10. [Security Testing](#security-testing)

## Authentication and Authorization

### Authentication

- **Use the provided authentication framework**: Always use the built-in authentication framework (`security/auth`) instead of implementing your own.
- **Implement multi-factor authentication**: Use the MFA utilities (`security/auth/mfa.py`) for sensitive operations.
- **Secure password handling**: Use the password utilities (`security/auth/password.py`) for hashing and verifying passwords with Argon2id.
- **Token validation**: Always validate JWT tokens using the provided utilities (`security/auth/jwt_handler.py`), ensuring full validation of signature, expiration, and issuer.
- **Session management**: Implement proper session timeouts (max 1 hour) and invalidation mechanisms.
- **Avoid using python-jose**: Use PyJWT instead of python-jose due to security vulnerabilities.
- **Implement token revocation**: Use a token blacklist or revocation mechanism for logout and password changes.
- **Secure cookie settings**: Always set cookies with `HttpOnly`, `Secure`, and `SameSite=strict` flags.

Example of proper authentication:

```python
from security.auth import verify_password, hash_password, mfa_manager

# Hashing a password
hashed_password = hash_password(plain_password)

# Verifying a password
is_valid = verify_password(plain_password, hashed_password)

# MFA verification
is_valid = mfa_manager.verify_mfa(code, totp_secret, hashed_backup_codes)
```

### Authorization

- **Use role-based access control**: Implement RBAC using the provided utilities.
- **Principle of least privilege**: Grant only the minimum necessary permissions.
- **Check permissions for every action**: Always verify that the user has the required permissions.
- **Audit access control decisions**: Log access control decisions, especially denials.

Example of proper authorization:

```python
from security.auth import has_permission, has_role

# Check if user has a specific permission
if has_permission(user_id, "resource:action"):
    # Perform action
else:
    # Deny access

# Check if user has a specific role
if has_role(user_id, "admin"):
    # Perform admin action
else:
    # Deny access
```

## Input Validation

- **Validate all inputs**: Use the validation utilities (`security/validation`) to validate all inputs.
- **Sanitize user inputs**: Sanitize user inputs to prevent injection attacks.
- **Use parameterized queries**: Always use parameterized queries for database operations.
- **Validate file uploads**: Validate file uploads to prevent malicious file uploads.

Example of proper input validation:

```python
from security.validation import validate_input, sanitize_string, validate_email

# Validate an email address
if validate_email(email):
    # Process email
else:
    # Invalid email

# Sanitize user input
safe_input = sanitize_string(user_input)

# Validate input with custom rules
try:
    validated_input = validate_input(
        value=user_input,
        input_type="string",
        required=True,
        min_length=3,
        max_length=50
    )
except Exception as e:
    # Handle validation error
```

## Database Security

- **Use ORM**: Use SQLAlchemy ORM to prevent SQL injection.
- **Parameterized queries**: If you need to write raw SQL, use parameterized queries.
- **Least privilege**: Use database accounts with the minimum necessary privileges.
- **Sanitize inputs**: Sanitize inputs before using them in database queries.
- **Validate database outputs**: Validate data retrieved from the database before using it.

Example of proper database access:

```python
from security.validation import execute_safe_query, build_safe_select_query

# Using ORM (preferred)
user = session.query(User).filter(User.id == user_id).first()

# Using parameterized queries
query, params = build_safe_select_query(
    table_name="users",
    columns=["id", "username", "email"],
    where_conditions={"id": user_id}
)
result = await execute_safe_query(engine, query, params)
```

## API Security

- **Rate limiting**: Implement rate limiting for API endpoints using the provided utilities (`security/api/rate_limiting.py`).
- **Input validation**: Validate all API inputs using Pydantic models with strict validation.
- **Output validation**: Validate all API outputs to prevent data leakage.
- **Security headers**: Use security headers for API responses, including:
  - Content-Security-Policy
  - X-Content-Type-Options: nosniff
  - X-Frame-Options: DENY
  - X-XSS-Protection: 1; mode=block
  - Strict-Transport-Security: max-age=31536000; includeSubDomains
  - Permissions-Policy: geolocation=(), camera=(), microphone=()
- **CORS**: Implement proper CORS configuration with specific allowed origins (never use wildcard `*`).
- **Authentication**: Require authentication for all sensitive endpoints.
- **Authorization**: Implement proper authorization checks for all endpoints.
- **API versioning**: Implement API versioning to handle breaking changes.
- **Request throttling**: Implement request throttling to prevent DoS attacks.
- **API documentation**: Document API security requirements and controls.

Example of proper API security:

```python
from security.api import rate_limit, add_security_headers, configure_secure_cors

# Rate limiting
@app.post("/api/resource", dependencies=[rate_limit(limit=10, window=60)])
async def create_resource(request: Request):
    # Process request

# Security headers
add_security_headers(app)

# CORS configuration
configure_secure_cors(app, allow_origins=["https://example.com"])
```

## Cryptography

- **Use the provided cryptography utilities**: Always use the built-in cryptography utilities (`security/encryption`) instead of implementing your own.
- **Key management**: Use the key management utilities (`security/encryption/key_management.py`) for managing cryptographic keys.
- **Secure random numbers**: Use `secrets` module for generating secure random numbers.
- **Modern algorithms**: Use modern cryptographic algorithms with appropriate key sizes.

Example of proper cryptography usage:

```python
from security.encryption import aes_encrypt, aes_decrypt, generate_aes_key

# Generate a key
key = generate_aes_key()

# Encrypt data
encrypted_data = aes_encrypt(plaintext, key)

# Decrypt data
plaintext = aes_decrypt(encrypted_data, key)
```

## Error Handling and Logging

- **Secure error handling**: Use the error handling utilities (`security/logging/error_handling.py`) to prevent information leakage.
- **Secure logging**: Use the secure logging utilities (`security/logging/secure_logging.py`) to mask sensitive data.
- **Consistent error messages**: Use consistent error messages that don't reveal sensitive information.
- **Log security events**: Log security-relevant events.

Example of proper error handling and logging:

```python
from security.logging import get_secure_logger, exception_handler, ErrorHandler

# Create a logger
logger = get_secure_logger("module_name")

# Log a message
logger.info("Processing request", {"user_id": user_id, "action": action})

# Handle exceptions
@exception_handler(Exception)
def process_request(request):
    # Process request

# Configure error handler
error_handler = ErrorHandler(app)
```

## File Operations

- **Validate file paths**: Validate file paths to prevent path traversal attacks.
- **Validate file types**: Validate file types to prevent malicious file uploads.
- **Secure file permissions**: Set secure file permissions.
- **Sanitize file names**: Sanitize file names to prevent path traversal attacks.

Example of proper file operations:

```python
from security.validation import validate_path, validate_filename

# Validate file path
if validate_path(file_path):
    # Process file
else:
    # Invalid file path

# Validate filename
if validate_filename(filename):
    # Process file
else:
    # Invalid filename
```

## Dependency Management

- **Keep dependencies up to date**: Regularly update dependencies to their latest versions.
- **Use a dependency management tool**: Use a tool like `pip-audit` or `safety` to check for vulnerabilities.
- **Pin dependency versions**: Pin dependency versions to prevent unexpected updates.
- **Minimize dependencies**: Minimize the number of dependencies to reduce the attack surface.

Example of proper dependency management:

```bash
# Check for vulnerabilities
safety check -r requirements.txt

# Update dependencies
pip install -U -r requirements.txt

# Pin dependency versions
pip freeze > requirements.txt
```

## Frontend Security

- **Content Security Policy**: Implement a Content Security Policy to prevent XSS attacks with strict directives:
  ```
  default-src 'self';
  script-src 'self' 'unsafe-inline';
  style-src 'self' 'unsafe-inline';
  img-src 'self' data:;
  connect-src 'self' https://api.example.com;
  frame-ancestors 'none';
  form-action 'self';
  ```
- **Input validation**: Validate all user inputs on the frontend using libraries like Zod or Yup.
- **Output encoding**: Use DOMPurify to sanitize HTML content before rendering.
- **CSRF protection**: Implement CSRF protection for all forms using the Double Submit Cookie pattern.
- **Secure cookies**: Use secure cookies with appropriate flags (HttpOnly, Secure, SameSite=strict).
- **Subresource Integrity**: Use SRI for all external scripts and stylesheets.
- **Trusted Types**: Implement Trusted Types to prevent DOM-based XSS attacks.
- **Frame protection**: Use X-Frame-Options and CSP frame-ancestors to prevent clickjacking.
- **Sanitize user-generated content**: Always sanitize user-generated content before rendering.
- **Use React's built-in protections**: Leverage React's built-in XSS protections by avoiding dangerouslySetInnerHTML.
- **Implement proper error boundaries**: Use error boundaries to prevent exposing sensitive information in error messages.

Example of proper frontend security:

```javascript
// React component with proper security practices
import React, { useState, useEffect } from 'react';
import DOMPurify from 'dompurify';
import { z } from 'zod';
import axios from 'axios';

// Input validation schema using Zod
const inputSchema = z.object({
  username: z.string().min(3).max(50).regex(/^[a-zA-Z0-9_]+$/),
  email: z.string().email(),
  message: z.string().max(1000)
});

// Secure form component
function SecureForm() {
  const [formData, setFormData] = useState({ username: '', email: '', message: '' });
  const [errors, setErrors] = useState({});
  const [csrfToken, setCsrfToken] = useState('');

  // Get CSRF token on component mount
  useEffect(() => {
    axios.get('/api/csrf-token')
      .then(response => setCsrfToken(response.data.token))
      .catch(error => console.error('Failed to fetch CSRF token:', error));
  }, []);

  const handleChange = (e) => {
    setFormData({ ...formData, [e.target.name]: e.target.value });
  };

  const handleSubmit = (e) => {
    e.preventDefault();

    // Validate input
    try {
      inputSchema.parse(formData);

      // Send data with CSRF token
      axios.post('/api/submit', formData, {
        headers: {
          'X-CSRF-Token': csrfToken
        }
      });

    } catch (error) {
      if (error instanceof z.ZodError) {
        // Convert Zod errors to a more usable format
        const fieldErrors = {};
        error.errors.forEach(err => {
          const field = err.path[0];
          fieldErrors[field] = err.message;
        });
        setErrors(fieldErrors);
      }
    }
  };

  // Safely display user content
  const renderSafeHTML = (content) => {
    return { __html: DOMPurify.sanitize(content) };
  };

  return (
    <form onSubmit={handleSubmit}>
      <div>
        <label htmlFor="username">Username:</label>
        <input
          type="text"
          id="username"
          name="username"
          value={formData.username}
          onChange={handleChange}
        />
        {errors.username && <p className="error">{errors.username}</p>}
      </div>

      <div>
        <label htmlFor="email">Email:</label>
        <input
          type="email"
          id="email"
          name="email"
          value={formData.email}
          onChange={handleChange}
        />
        {errors.email && <p className="error">{errors.email}</p>}
      </div>

      <div>
        <label htmlFor="message">Message:</label>
        <textarea
          id="message"
          name="message"
          value={formData.message}
          onChange={handleChange}
        />
        {errors.message && <p className="error">{errors.message}</p>}
      </div>

      {/* Only use dangerouslySetInnerHTML when absolutely necessary and with sanitization */}
      {formData.message && (
        <div className="preview">
          <h3>Preview:</h3>
          <div dangerouslySetInnerHTML={renderSafeHTML(formData.message)} />
        </div>
      )}

      <input type="hidden" name="csrf_token" value={csrfToken} />
      <button type="submit">Submit</button>
    </form>
  );
}

export default SecureForm;
```

## Security Testing

- **Automated security testing**: Implement automated security testing in the CI/CD pipeline.
- **Static analysis**: Use static analysis tools to detect security issues.
- **Dynamic analysis**: Use dynamic analysis tools to detect security issues.
- **Penetration testing**: Conduct regular penetration testing.
- **Code reviews**: Conduct security-focused code reviews.

Example of security testing:

```bash
# Static analysis
bandit -r .

# Dependency checking
safety check -r requirements.txt

# Run security tests
pytest tests/security
```

## Conclusion

Following these security guidelines will help ensure that the Bulo.Cloud Sentinel project remains secure. If you have any questions or concerns, please contact the security team.
