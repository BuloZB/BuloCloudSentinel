from fastapi import APIRouter

router = APIRouter()

@router.get("/")
async def get_detections():
    # Placeholder for detections retrieval
    return {"detections": []}
