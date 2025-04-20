from fastapi import APIRouter, Depends
from backend.api.dependencies import get_current_user

router = APIRouter(
    prefix="/me",
    tags=["User Info"]
)

@router.get("/")
async def get_current_user_info(current_user: str = Depends(get_current_user)):
    # Placeholder for returning user info
    return {"username": current_user, "roles": ["user"]}
