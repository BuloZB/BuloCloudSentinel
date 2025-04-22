from fastapi import Depends, HTTPException, status
from fastapi.security import OAuth2PasswordBearer
# Replacing jose with PyJWT for security (CVE-2024-33664, CVE-2024-33663)
# from jose import JWTError, jwt
import jwt
from jwt.exceptions import PyJWTError as JWTError
from typing import Optional
from pydantic import BaseModel
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
        payload = jwt.decode(token, settings.jwt_secret, algorithms=[settings.jwt_algorithm])
        username: str = payload.get("sub")
        roles: list[str] = payload.get("roles", [])
        if username is None:
            raise credentials_exception
        token_data = TokenData(username=username, roles=roles)
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
