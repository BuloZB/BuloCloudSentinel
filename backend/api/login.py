from fastapi import APIRouter, Depends, HTTPException, status
from fastapi.security import OAuth2PasswordRequestForm
from backend.application.services.auth_service import AuthService
from backend.api.dependencies import verify_jwt_token

router = APIRouter()

@router.post("/login")
async def login(form_data: OAuth2PasswordRequestForm = Depends()):
    auth_service = AuthService()
    user = auth_service.authenticate_user(form_data.username, form_data.password)
    if not user:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Incorrect username or password",
            headers={"WWW-Authenticate": "Bearer"},
        )
    access_token = auth_service.create_access_token(user)
    return {"access_token": access_token, "token_type": "bearer"}

@router.get("/me")
async def read_users_me(token_data=Depends(verify_jwt_token)):
    return {"username": token_data.username, "roles": token_data.roles}
