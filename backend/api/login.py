from fastapi import APIRouter, Depends, HTTPException, status, Request, Response

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
from fastapi.security import OAuth2PasswordRequestForm, OAuth2PasswordBearer
from pydantic import BaseModel
from typing import Optional
from datetime import datetime

from backend.application.services.auth_service import AuthService
from backend.domain.repositories.user_repository import IUserRepository
from backend.infrastructure.persistence.sqlalchemy_user_repository import SQLAlchemyUserRepository
from backend.db.session import get_db_session
from backend.api.dependencies import verify_jwt_token
from security.auth.token_blacklist import blacklist_token, blacklist_all_user_tokens
from security.auth.cookie_handler import set_jwt_cookies, unset_jwt_cookies
from security.logging.secure_logging import get_secure_logger

# Configure secure logger
logger = get_secure_logger("auth_api")

# Define token response model
class TokenResponse(BaseModel):
    access_token: str
    refresh_token: str
    token_type: str
    expires_in: int

# Define refresh token request model
class RefreshTokenRequest(BaseModel):
    refresh_token: str

# OAuth2 scheme for token extraction
oauth2_scheme = OAuth2PasswordBearer(tokenUrl="login")

router = APIRouter()

# Helper function to get auth service
async def get_auth_service():
    async with get_db_session() as session:
        user_repository = SQLAlchemyUserRepository(session)
        return AuthService(user_repository)

@router.post("/login", response_model=TokenResponse)
async def login(
    request: Request,
    form_data: OAuth2PasswordRequestForm = Depends(),
    auth_service: AuthService = Depends(get_auth_service)
):
    # Log login attempt (masking password)
    client_ip = request.client.host if request.client else "unknown"
    logger.info(
        f"Login attempt for user {form_data.username}",
        {
            "username": form_data.username,
            "ip_address": client_ip,
            "user_agent": request.headers.get("user-agent", "unknown")
        }
    )
    
    # Authenticate user
    user = await auth_service.authenticate_user(form_data.username, form_data.password)
    if not user:
        # Log failed login
        logger.warning(
            f"Failed login attempt for user {form_data.username}",
            {
                "username": form_data.username,
                "ip_address": client_ip,
                "user_agent": request.headers.get("user-agent", "unknown"),
                "reason": "Invalid credentials"
            }
        )
        
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Incorrect username or password",
            headers={"WWW-Authenticate": "Bearer"},
        )
    
    # Create token response
    token_response = auth_service.create_token_response(user)
    
    # Create response
    response = Response(content=token_response.model_dump_json(), media_type="application/json")
    
    # Set JWT cookies
    set_jwt_cookies(response, token_response.access_token, token_response.refresh_token)
    
    # Log successful login
    logger.info(
        f"Successful login for user {form_data.username}",
        {
            "username": form_data.username,
            "user_id": str(user.id),
            "ip_address": client_ip,
            "user_agent": request.headers.get("user-agent", "unknown")
        }
    )
    
    return response

@router.post("/refresh")
async def refresh_token(
    request: Request,
    refresh_request: Optional[RefreshTokenRequest] = None,
    auth_service: AuthService = Depends(get_auth_service)
):
    # Get refresh token from request body or cookies
    refresh_token = None
    if refresh_request:
        refresh_token = refresh_request.refresh_token
    else:
        from security.auth.cookie_handler import get_refresh_token_from_cookies
        refresh_token = get_refresh_token_from_cookies(request)
    
    if not refresh_token:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Refresh token not provided",
            headers={"WWW-Authenticate": "Bearer"},
        )
    
    # Refresh access token
    token_response = await auth_service.refresh_access_token(refresh_token)
    if not token_response:
        # Log failed refresh
        client_ip = request.client.host if request.client else "unknown"
        logger.warning(
            "Failed token refresh attempt",
            {
                "ip_address": client_ip,
                "user_agent": request.headers.get("user-agent", "unknown"),
                "reason": "Invalid refresh token"
            }
        )
        
        # Create response with unset cookies
        response = Response(
            content={"detail": "Invalid refresh token"},
            status_code=status.HTTP_401_UNAUTHORIZED,
            media_type="application/json"
        )
        unset_jwt_cookies(response)
        
        return response
    
    # Create response
    response = Response(content=token_response.model_dump_json(), media_type="application/json")
    
    # Set JWT cookies
    set_jwt_cookies(response, token_response.access_token, token_response.refresh_token)
    
    return response

@router.post("/logout")
async def logout(
    request: Request,
    token: Optional[str] = Depends(oauth2_scheme),
    token_data = Depends(verify_jwt_token)
):
    # Extract token ID (jti) from token data
    jti = getattr(token_data, "jti", None)
    if jti:
        # Get expiration time
        exp = getattr(token_data, "exp", datetime.now().timestamp() + 3600)
        
        # Blacklist token
        blacklist_token(jti, exp)
        
        # Log logout
        client_ip = request.client.host if request.client else "unknown"
        logger.info(
            f"User logged out",
            {
                "username": token_data.username,
                "user_id": token_data.sub,
                "ip_address": client_ip,
                "user_agent": request.headers.get("user-agent", "unknown")
            }
        )
    
    # Create response
    response = Response(
        content={"message": "Successfully logged out"},
        media_type="application/json"
    )
    
    # Unset JWT cookies
    unset_jwt_cookies(response)
    
    return response

@router.post("/logout-all")
async def logout_all(
    request: Request,
    token_data = Depends(verify_jwt_token)
):
    # Blacklist all tokens for the user
    user_id = token_data.sub
    count = blacklist_all_user_tokens(user_id)
    
    # Log logout all
    client_ip = request.client.host if request.client else "unknown"
    logger.info(
        f"User logged out from all devices",
        {
            "username": token_data.username,
            "user_id": user_id,
            "ip_address": client_ip,
            "user_agent": request.headers.get("user-agent", "unknown"),
            "tokens_revoked": count
        }
    )
    
    # Create response
    response = Response(
        content={"message": f"Successfully logged out from all devices ({count} sessions)"},
        media_type="application/json"
    )
    
    # Unset JWT cookies
    unset_jwt_cookies(response)
    
    return response

@router.get("/me")
async def read_users_me(token_data=Depends(verify_jwt_token)):
    return {
        "username": token_data.username,
        "user_id": token_data.sub,
        "roles": token_data.roles
    }
