from fastapi import APIRouter, Depends
from backend.infrastructure.database import engine
from backend.infrastructure.persistence.models import Base

router = APIRouter()

@router.post("/")
async def setup():
    # Create database tables
    async with engine.begin() as conn:
        await conn.run_sync(Base.metadata.create_all)
    return {"message": "Database tables created successfully"}
