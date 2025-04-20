from fastapi import APIRouter, Depends, HTTPException, status

router = APIRouter()

def get_current_user():
    # Placeholder for user authentication dependency
    return {"username": "admin"}

@router.get("/")
async def read_me(current_user: dict = Depends(get_current_user)):
    if not current_user:
        raise HTTPException(status_code=status.HTTP_401_UNAUTHORIZED, detail="Not authenticated")
    return {"username": current_user["username"]}
