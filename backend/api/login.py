from fastapi import APIRouter, Depends, HTTPException, status
from pydantic import BaseModel
from backend.api.dependencies import get_user_repository
from backend.application.services.auth_service import AuthService
from backend.domain.repositories.user_repository import IUserRepository
from backend.infrastructure.config.config import settings
from datetime import timedelta

router = APIRouter()

class LoginRequest(BaseModel):
    username: str
    password: str

class TokenResponse(BaseModel):
    access_token: str
    token_type: str = "bearer"

@router.post("/", response_model=TokenResponse)
async def login(request: LoginRequest, user_repo: IUserRepository = Depends(get_user_repository)):
    auth_service = AuthService(user_repo)
    user = await auth_service.authenticate_user(request.username, request.password)
    if not user:
        raise HTTPException(status_code=status.HTTP_401_UNAUTHORIZED, detail="Invalid credentials")
    access_token = auth_service.create_access_token(data={"sub": user.username}, expires_delta=timedelta(minutes=settings.jwt_expiration_minutes))
    return {"access_token": access_token}
