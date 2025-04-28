from fastapi import Depends, HTTPException, status, Request, Header
from fastapi.security import OAuth2PasswordBearer
# Replacing jose with PyJWT for security (CVE-2024-33664, CVE-2024-33663)
# from jose import JWTError, jwt
import jwt
from jwt.exceptions import PyJWTError as JWTError
from typing import Optional, List, Union
from pydantic import BaseModel
from datetime import datetime, timezone
from backend.infrastructure.config.config import settings
from security.auth.token_blacklist import is_token_blacklisted
from security.auth.cookie_handler import get_access_token_from_cookies
from security.logging.secure_logging import get_secure_logger

# Configure secure logger
logger = get_secure_logger("auth_dependencies")

oauth2_scheme = OAuth2PasswordBearer(tokenUrl="login", auto_error=False)

class TokenData(BaseModel):
    sub: str
    username: Optional[str] = None
    roles: List[str] = []
    jti: Optional[str] = None
    exp: Optional[float] = None
    iat: Optional[float] = None
    type: Optional[str] = None

async def get_token_from_request(
    request: Request,
    authorization: Optional[str] = Header(None)
) -> Optional[str]:
    """
    Get token from request (header or cookie).
    
    Args:
        request: The HTTP request
        authorization: Authorization header
        
    Returns:
        Token string or None
    """
    # Try to get token from authorization header
    if authorization and authorization.startswith("Bearer "):
        return authorization.replace("Bearer ", "")
    
    # Try to get token from cookie
    token = get_access_token_from_cookies(request)
    if token:
        return token
    
    return None

def verify_jwt_token(
    token: Optional[str] = Depends(oauth2_scheme),
    request: Request = None,
    extracted_token: Optional[str] = Depends(get_token_from_request)
) -> TokenData:
    """
    Verify JWT token from header or cookie.
    
    Args:
        token: Token from OAuth2 scheme (header)
        request: The HTTP request
        extracted_token: Token extracted from header or cookie
        
    Returns:
        TokenData object
    """
    # Use token from OAuth2 scheme or extracted token
    final_token = token or extracted_token
    
    if not final_token:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Not authenticated",
            headers={"WWW-Authenticate": "Bearer"},
        )
    credentials_exception = HTTPException(
        status_code=status.HTTP_401_UNAUTHORIZED,
        detail="Could not validate credentials",
        headers={"WWW-Authenticate": "Bearer"},
    )
    try:
        # Decode token with full validation
        payload = jwt.decode(
            final_token,
            settings.JWT_SECRET,
            algorithms=[settings.JWT_ALGORITHM],
            options={
                "verify_signature": True,
                "verify_exp": True,
                "verify_iat": True,
                "require": ["sub", "jti", "type"]
            }
        )

        # Extract and validate user ID
        user_id: str = payload.get("sub")
        if user_id is None:
            logger.warning("Token missing subject claim")
            raise credentials_exception

        # Extract token ID
        jti: str = payload.get("jti")
        if jti is None:
            logger.warning("Token missing jti claim")
            raise credentials_exception
            
        # Check token type
        token_type: str = payload.get("type")
        if token_type != "access":
            logger.warning(f"Invalid token type: {token_type}")
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Invalid token type",
                headers={"WWW-Authenticate": "Bearer"},
            )

        # Check if token is blacklisted
        if is_token_blacklisted(jti):
            logger.warning(f"Blacklisted token used: {jti}")
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Token has been revoked",
                headers={"WWW-Authenticate": "Bearer"},
            )

        # Extract username and roles
        username: str = payload.get("username")
        roles: List[str] = payload.get("roles", [])

        # Extract expiration and issued at times
        exp: float = payload.get("exp")
        iat: float = payload.get("iat")

        # Check if token was issued in the future (clock skew)
        if iat:
            current_time = datetime.now(timezone.utc).timestamp()
            if iat > current_time + 30:  # Allow 30 seconds of clock skew
                logger.warning(f"Token issued in the future: {iat}")
                raise HTTPException(
                    status_code=status.HTTP_401_UNAUTHORIZED,
                    detail="Token issued in the future",
                    headers={"WWW-Authenticate": "Bearer"},
                )

        # Create token data
        token_data = TokenData(
            sub=user_id,
            username=username,
            roles=roles,
            jti=jti,
            exp=exp,
            iat=iat,
            type=token_type
        )

        # Log access if request is provided
        if request:
            client_ip = request.client.host if request.client else "unknown"
            logger.debug(
                f"API access by user {username}",
                {
                    "username": username,
                    "user_id": user_id,
                    "path": request.url.path,
                    "method": request.method,
                    "ip_address": client_ip,
                    "user_agent": request.headers.get("user-agent", "unknown")
                }
            )

    except jwt.ExpiredSignatureError:
        logger.warning("Expired token used")
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Token has expired",
            headers={"WWW-Authenticate": "Bearer"},
        )
    except jwt.InvalidTokenError as e:
        logger.warning(f"Invalid token: {str(e)}")
        raise credentials_exception
    except JWTError as e:
        logger.warning(f"JWT error: {str(e)}")
        raise credentials_exception
    except Exception as e:
        logger.error(f"Unexpected error in token verification: {str(e)}")
        raise credentials_exception

    return token_data

def require_role(required_roles: List[str]):
    """
    Dependency for checking if user has any of the required roles.
    
    Args:
        required_roles: List of roles, any of which grants access
        
    Returns:
        Dependency function
    """
    def role_checker(token_data: TokenData = Depends(verify_jwt_token)):
        if not any(role in token_data.roles for role in required_roles):
            logger.warning(
                f"Insufficient permissions for user {token_data.username}",
                {
                    "username": token_data.username,
                    "user_id": token_data.sub,
                    "user_roles": token_data.roles,
                    "required_roles": required_roles
                }
            )
            raise HTTPException(
                status_code=status.HTTP_403_FORBIDDEN,
                detail="Insufficient permissions",
            )
        return token_data
    return role_checker

def require_all_roles(required_roles: List[str]):
    """
    Dependency for checking if user has all of the required roles.
    
    Args:
        required_roles: List of roles, all of which are required for access
        
    Returns:
        Dependency function
    """
    def role_checker(token_data: TokenData = Depends(verify_jwt_token)):
        if not all(role in token_data.roles for role in required_roles):
            logger.warning(
                f"Insufficient permissions for user {token_data.username}",
                {
                    "username": token_data.username,
                    "user_id": token_data.sub,
                    "user_roles": token_data.roles,
                    "required_roles": required_roles
                }
            )
            raise HTTPException(
                status_code=status.HTTP_403_FORBIDDEN,
                detail="Insufficient permissions",
            )
        return token_data
    return role_checker
