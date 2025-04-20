from fastapi import APIRouter, Depends, HTTPException, status
from fastapi.security import OAuth2PasswordRequestForm
from backend.api.dependencies import oauth2_scheme

router = APIRouter(
    prefix="/login",
    tags=["Authentication"]
)

@router.post("/")
async def login(form_data: OAuth2PasswordRequestForm = Depends()):
    # Placeholder for authentication logic against Keycloak
    if form_data.username == "admin" and form_data.password == "password":
        return {"access_token": "fake-jwt-token", "token_type": "bearer"}
    raise HTTPException(
        status_code=status.HTTP_401_UNAUTHORIZED,
        detail="Incorrect username or password",
        headers={"WWW-Authenticate": "Bearer"},
    )
