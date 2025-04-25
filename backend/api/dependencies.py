from fastapi import Depends, HTTPException, status
from fastapi.security import OAuth2PasswordBearer
# Replacing jose with PyJWT for security (CVE-2024-33664, CVE-2024-33663)
# from jose import JWTError, jwt
import jwt
from jwt.exceptions import PyJWTError as JWTError
from typing import Optional
from pydantic import BaseModel
from datetime import datetime, timezone
from backend.infrastructure.config.config import settings

oauth2_scheme = OAuth2PasswordBearer(tokenUrl="login")

class TokenData(BaseModel):
    username: Optional[str] = None
    roles: Optional[list[str]] = []

def verify_jwt_token(token: str = Depends(oauth2_scheme)) -> TokenData:
    credentials_exception = HTTPException(
        status_code=status.HTTP_401_UNAUTHORIZED,
        detail="Could not validate credentials",
        headers={"WWW-Authenticate": "Bearer"},
    )
    try:
        # Decode token with full validation
        payload = jwt.decode(
            token,
            settings.jwt_secret,
            algorithms=[settings.jwt_algorithm],
            options={
                "verify_signature": True,
                "verify_exp": True,
                "verify_iat": True,
                "require": ["sub"]
            }
        )

        # Extract and validate username
        username: str = payload.get("sub")
        if username is None:
            raise credentials_exception

        # Extract roles
        roles: list[str] = payload.get("roles", [])

        # Check if token was issued in the future (clock skew)
        if "iat" in payload:
            iat_timestamp = payload["iat"]
            if isinstance(iat_timestamp, datetime):
                iat_timestamp = iat_timestamp.timestamp()
            current_time = datetime.now(timezone.utc).timestamp()
            if iat_timestamp > current_time + 30:  # Allow 30 seconds of clock skew
                raise credentials_exception

        # Create token data
        token_data = TokenData(username=username, roles=roles)

    except jwt.ExpiredSignatureError:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Token has expired",
            headers={"WWW-Authenticate": "Bearer"},
        )
    except jwt.InvalidTokenError:
        raise credentials_exception
    except JWTError:
        raise credentials_exception

    return token_data

def require_role(required_roles: list[str]):
    def role_checker(token_data: TokenData = Depends(verify_jwt_token)):
        if not any(role in token_data.roles for role in required_roles):
            raise HTTPException(
                status_code=status.HTTP_403_FORBIDDEN,
                detail="Insufficient permissions",
            )
        return token_data
    return role_checker
