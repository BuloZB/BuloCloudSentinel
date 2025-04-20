from fastapi import APIRouter

router = APIRouter()

@router.post("/")
async def setup():
    return {"message": "Setup endpoint placeholder"}
