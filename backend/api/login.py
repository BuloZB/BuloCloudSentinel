from fastapi import APIRouter, Depends, HTTPException, status
from pydantic import BaseModel
from backend.api.dependencies import get_user_repository
from backend.domain.repositories.user_repository import IUserRepository
from backend.domain.services.user_service import UserService
from jose import JWTError, jwt
from backend.infrastructure.config.config import settings
from datetime import datetime, timedelta

router = APIRouter()

class LoginRequest(BaseModel):
    username: str
    password: str

class TokenResponse(BaseModel):
    access_token: str
    token_type: str = "bearer"

@router.post("/", response_model=TokenResponse)
async def login(request: LoginRequest, user_repo: IUserRepository = Depends(get_user_repository)):
    user_service = UserService(user_repo)
    user = await user_service.authenticate(request.username, request.password)
    if not user:
        raise HTTPException(status_code=status.HTTP_401_UNAUTHORIZED, detail="Invalid credentials")
    access_token = create_access_token(data={"sub": user.username})
    return {"access_token": access_token}

def create_access_token(data: dict, expires_delta: timedelta = None):
    to_encode = data.copy()
    expire = datetime.utcnow() + (expires_delta or timedelta(minutes=settings.jwt_expiration_minutes))
    to_encode.update({"exp": expire})
    encoded_jwt = jwt.encode(to_encode, settings.jwt_secret, algorithm=settings.jwt_algorithm)
    return encoded_jwt
